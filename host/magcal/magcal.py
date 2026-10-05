#!/usr/bin/env python3

"""OSMO/Bowie 磁力计标定工具（采集 / 拟合 / 上传）



三个子命令：



  collect  从板子读原始磁矢量 + 姿态，把一次翻滚存成本地 .npz

  fit      在本地跑四段式鲁棒椭球拟合，产出固件用的标定 blob

  upload   把 blob 推回板子，板子装进 RAM 并写进自己的 Flash



为什么要拆开：**转一次，之后所有算法迭代都在本地做**，不用再动板子。



拟合相对固件的四点增强

  1) IRLS / Huber 鲁棒加权      —— 抗离群点（转的过程中手靠到口袋、门开一下）

  2) SVD (lstsq) 而非正规方程    —— 不平方条件数，7000 样本下差别明显

  3) 姿态一致性剔除              —— 用相邻四元数预测磁场该怎么转，预测不了的丢

  4) 覆盖密度反加权              —— 24 个球面区块，稀疏区权重高、扎堆区权重低



固件的参数约定（必须严格对齐）

      u      = W * (raw - offset) / radius        |u| ~ 1

      m_out  = A * u * radius                     A = 磁->IMU 旋转

  offset/W/radius 描述椭球【形状】；A 和 dip 是板子自身属性，上传时由固件

  保留自己的（见 gloves.c 的 glove_mag_upload 合并逻辑）。

  W 归一化为 det(W) = 1。

"""



from __future__ import annotations



import argparse

import struct

import os
import sys

import time

from pathlib import Path



import numpy as np



# ---------------------------------------------------------------- 常量



MAG_UPLOAD_MAGIC = 0x4D414743  # "CGAM"

MAG_CAL_BLOB_VERSION = 4



MAG_BLOB_F_CAL1 = 0x01

MAG_BLOB_F_CAL2 = 0x02

MAG_BLOB_F_MAP = 0x04

MAG_BLOB_F_DIP = 0x08

MAG_BLOB_F_ALIGN = 0x10

MAG_BLOB_F_VERIFIED = 0x20

MAG_BLOB_F_MODEL_FULL = 0x40

MAG_BLOB_F_MODEL_DIAG = 0x80



BLOB_FMT = "<I"       # version

BLOB_FMT += "6f"      # offset[2][3]

BLOB_FMT += "18f"     # W[2][9]

BLOB_FMT += "2f"      # radius[2]

BLOB_FMT += "9f"      # map[9]

BLOB_FMT += "3f"      # map_c[3]

BLOB_FMT += "9f"      # align[9]

BLOB_FMT += "f"       # dip

BLOB_FMT += "I"       # flags

BLOB_SIZE = struct.calcsize(BLOB_FMT)

assert BLOB_SIZE == 200, BLOB_SIZE



HOST_ROOT = Path(__file__).resolve().parent.parent
GLOVE2ROBOT = Path(os.environ.get("OSMO_HOST_ROOT", str(HOST_ROOT)))





def fnv1a32(data: bytes) -> int:

    h = 2166136261

    for b in data:

        h ^= b

        h = (h * 16777619) & 0xFFFFFFFF

    return h





# ---------------------------------------------------------------- 分块几何

# 与固件 mf_cal_bin() 完全一致：场矢量在机体系的主导轴（6 个面）

# x 另两轴的象限（4）= 24 个球面区块。固件的达标线是占满 16 个。



def cap_index(d: np.ndarray) -> np.ndarray:
    """和固件 mf_cal_bin() 逐位一致的 24 区块编号。

    o1/o2 是【逐样本】由该样本的主导轴算出来的，必须用每行自己的轴去取分量。
    早期版本误把 o1 当成"按轴的下标表"来用 o1[k]，区块编号整体错乱，
    症状就是"明明转到了却显示没覆盖"。"""
    d = np.atleast_2d(np.asarray(d, dtype=float))
    n = len(d)
    if n == 0:
        return np.zeros(0, dtype=np.int64)
    a = np.abs(d)
    ax = np.argmax(a, axis=1)            # (n,) 每个样本自己的主导轴
    o1 = (ax + 1) % 3
    o2 = (ax + 2) % 3
    rows = np.arange(n)
    idx = 8 * ax
    idx = idx + 4 * (d[rows, ax] > 0.0)
    idx = idx + 2 * (d[rows, o1] > 0.0)
    idx = idx + 1 * (d[rows, o2] > 0.0)
    return idx.astype(np.int64)





def occupied_caps(d: np.ndarray) -> int:

    return int(len(np.unique(cap_index(d))))





# ---------------------------------------------------------------- 拟合



def _ellipsoid_from_theta(theta: np.ndarray):

    """[a,b,c,f,g,h,p,q,r] -> (Q, centre, k)  使 (m-b)^T (Q/k) (m-b) = 1"""

    a, b, c, f, g, h, p, q, r = theta

    Q = np.array([[a, h, g], [h, b, f], [g, f, c]], dtype=float)

    pv = np.array([p, q, r], dtype=float)

    try:

        centre = -np.linalg.solve(Q, pv)

    except np.linalg.LinAlgError:

        return None

    k = float(centre @ Q @ centre + 1.0)   # d = -1

    if not np.isfinite(k) or k <= 0:

        return None

    return Q, centre, k





def sym_sqrt(A: np.ndarray) -> np.ndarray:

    """对称正定矩阵的【对称】平方根 S, S S = A。



    必须和固件的 mf_m3_sqrt() 一致 —— 那是 Denman-Beavers 迭代，

    收敛到主对称平方根。用 Cholesky 会差一个旋转，W 的方向约定就变了，

    而固件的 A(磁->IMU) 是按它的约定估出来的，两者不匹配会让航向整体偏。"""

    w, V = np.linalg.eigh((A + A.T) * 0.5)

    w = np.maximum(w, 1e-300)

    return (V * np.sqrt(w)) @ V.T





def canonicalize(W: np.ndarray, b: np.ndarray, radius: float):

    """把任意等价的 (W, radius) 化到固件的规范形式。



    物理内容只取决于 Mn = W^T W / radius^2（椭球形状），W 本身可以差一个

    旋转。固件固定取 Mn 的对称平方根，这里做同样的转换。"""

    Mn = (W.T @ W) / (radius * radius)

    S = sym_sqrt(Mn)

    g = float(np.cbrt(np.linalg.det(S)))

    if not np.isfinite(g) or abs(g) < 1e-12:

        return None

    return S / g, b, 1.0 / g





def _params_from_theta(theta: np.ndarray):

    r = _ellipsoid_from_theta(theta)

    if r is None:

        return None

    Q, centre, k = r

    Mn = Q / k

    try:

        S = sym_sqrt(Mn)

    except np.linalg.LinAlgError:

        return None

    g = float(np.cbrt(np.linalg.det(S)))

    if not np.isfinite(g) or abs(g) < 1e-12:

        return None

    return S / g, centre, 1.0 / g





def linear_fit(m: np.ndarray, w: np.ndarray):

    """加权线性椭球拟合，用 SVD(lstsq) 求解 —— 不构造正规方程。"""

    x, y, z = m[:, 0], m[:, 1], m[:, 2]

    D = np.column_stack(

        [x * x, y * y, z * z, 2 * y * z, 2 * x * z, 2 * x * y, 2 * x, 2 * y, 2 * z]

    )

    sw = np.sqrt(np.maximum(w, 0.0))

    A = D * sw[:, None]

    b = sw.copy()

    theta, *_ = np.linalg.lstsq(A, b, rcond=None)

    return _params_from_theta(theta)





def refine(m: np.ndarray, w: np.ndarray, p0):

    """几何残差精修：最小化 |L(m-b)| - radius。



    L 用下三角参数化且固定 L[2][2] = 1 —— 这消除了 (W,radius) 的尺度简并，

    否则优化会沿一个零方向乱跑。"""

    from scipy.optimize import least_squares



    W0, b0, r0 = p0

    L0 = np.linalg.cholesky(W0 @ W0.T).T          # W = L^T? 统一取下三角

    L0 = np.tril(W0 / 1.0)

    if abs(L0[2, 2]) < 1e-9:

        L0 = np.tril(np.eye(3))

    L0 = L0 / L0[2, 2]

    x0 = np.concatenate([b0, [L0[0, 0], L0[1, 0], L0[1, 1], L0[2, 0], L0[2, 1]], [r0]])



    sw = np.sqrt(np.maximum(w, 0.0))



    def unpack(x):

        L = np.array([[x[3], 0.0, 0.0],

                      [x[4], x[5], 0.0],

                      [x[6], x[7], 1.0]])

        return L, x[0:3], x[8]



    def resid(x):

        L, b, r = unpack(x)

        return (np.linalg.norm((m - b) @ L.T, axis=1) - r) * sw



    try:

        sol = least_squares(resid, x0, method="trf", max_nfev=200)

    except Exception:

        return None

    L, b, r = unpack(sol.x)

    return canonicalize(L, b, r)





def huber_weights(res: np.ndarray, c: float = 1.5) -> np.ndarray:

    s = 1.4826 * np.median(np.abs(res - np.median(res)))  # 稳健 sigma

    if not np.isfinite(s) or s < 1e-12:

        s = float(np.std(res)) or 1e-12

    u = np.abs(res) / s

    w = np.ones_like(u)

    m = u > c

    w[m] = c / u[m]

    return w





def attitude_weights(m: np.ndarray, q: np.ndarray) -> np.ndarray:

    """姿态一致性：相邻样本之间，磁场在机体系里必须按四元数变化转过去。

    预测不了的样本就是干扰，按偏差给权。"""

    n = len(m)

    w = np.ones(n)

    if n < 3:

        return w

    # 机体系相对旋转 R_rel = R(q_i)^T R(q_{i-1})

    R = quat_to_mat(q)

    d = m / np.maximum(np.linalg.norm(m, axis=1, keepdims=True), 1e-12)

    Rrel = np.einsum("nji,njk->nik", R[1:], R[:-1])   # R_i^T R_{i-1}

    expected = np.einsum("nij,nj->ni", Rrel, d[:-1])

    err = np.arccos(np.clip(np.sum(expected * d[1:], axis=1), -1.0, 1.0))

    s = 1.4826 * np.median(np.abs(err - np.median(err)))

    if not np.isfinite(s) or s < 1e-9:

        s = max(float(np.std(err)), 1e-9)

    u = err / s

    wi = np.ones_like(u)

    k = u > 4.0

    wi[k] = 4.0 / u[k]

    w[1:] = wi

    return w





def coverage_weights(d: np.ndarray) -> np.ndarray:

    """覆盖密度反加权：稀疏区块权重高，扎堆区块权重低。"""

    idx = cap_index(d)

    cnt = np.bincount(idx, minlength=24).astype(float)

    target = len(idx) / 24.0

    w = np.ones(len(idx))

    nz = cnt[idx] > 0

    w[nz] = target / cnt[idx][nz]

    return np.clip(w, 0.05, 20.0)





def quat_to_mat(q: np.ndarray) -> np.ndarray:

    q = np.asarray(q, dtype=float)

    q = q / np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1e-12)

    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]

    R = np.empty((len(q), 3, 3))

    R[:, 0, 0] = 1 - 2 * (y * y + z * z)

    R[:, 0, 1] = 2 * (x * y - w * z)

    R[:, 0, 2] = 2 * (x * z + w * y)

    R[:, 1, 0] = 2 * (x * y + w * z)

    R[:, 1, 1] = 1 - 2 * (x * x + z * z)

    R[:, 1, 2] = 2 * (y * z - w * x)

    R[:, 2, 0] = 2 * (x * z - w * y)

    R[:, 2, 1] = 2 * (y * z + w * x)

    R[:, 2, 2] = 1 - 2 * (x * x + y * y)

    return R






def dip_weights(m, q, W, b, r, sig=3.0):
    """用【偏航无关】的 dip 一致性给权 —— 抓椭球残差抓不到的那类干扰。

    椭球残差只约束 |u| = 常数。如果干扰把磁场【方向】转了但模长没变，
    椭球残差看不出来，那几个脏样本会被当成好样本拟合进去。

    磁场在世界系的竖直分量（也就是 dip）是偏航无关的：不管 yaw_corr 怎么动，
    (R(q) u)_z 都必须恒定。所以拿它和姿态对照，就能独立地判出方向型干扰。

    只用竖直分量，不做世界系整体一致性 —— 因为采集时六轴 yaw 在自由漂
    （实测 14 分钟内平滑漂了 28 度），水平分量不可信。
    """
    if q is None or len(m) < 50:
        return np.ones(len(m))
    u = (m - b) @ W.T / max(r, 1e-9)
    R = quat_to_mat(q)
    z = np.einsum("nij,nj->ni", R, u)[:, 2]
    med = float(np.median(z))
    mad = 1.4826 * float(np.median(np.abs(z - med)))
    if not np.isfinite(mad) or mad < 1e-7:
        return np.ones(len(m))
    k = np.abs(z - med) / mad
    w = np.ones_like(k)
    bad = k > sig
    w[bad] = sig / k[bad]
    return np.clip(w, 0.0, 1.0)


def refine_world(m, q, p0, w=None, verbose=True):
    """世界系一致性精修 —— 本工具最强的一步。

    椭球拟合只用了 |u| = 常数 这一个约束。但真实物理还要求：
        磁场在世界系里必须【恒定】  R(q_i) * u_i = B   对所有 i
    这同时约束方向和大小，把姿态信息也用上了，因此比单纯椭球约束强得多。

    参数:  A (9)  使 u_i = A (m_i - b)
           b (3)  硬铁中心
           Bh(3)  世界系磁场（残差里归一化，消除整体尺度简并）
    残差:  R(q_i) A (m_i - b) - normalize(Bh)
    """
    from scipy.optimize import least_squares

    m = np.asarray(m, dtype=float)
    n = len(m)
    if q is None or n < 200:
        return None
    w = np.ones(n) if w is None else np.asarray(w, dtype=float)

    W0, b0, r0 = p0
    A0 = W0 / r0
    R = quat_to_mat(q)                      # (n,3,3) body->world
    u0 = (m - b0) @ A0.T
    B0 = (R @ u0[:, :, None])[:, :, 0].mean(axis=0)
    B0 /= max(np.linalg.norm(B0), 1e-9)

    def unpack(x):
        return x[0:9].reshape(3, 3), x[9:12], x[12:15]

    def resid(x):
        A, b, Bh = unpack(x)
        Bh = Bh / max(np.linalg.norm(Bh), 1e-9)
        u = (m - b) @ A.T
        pred = np.einsum("nij,nj->ni", R, u)
        return ((pred - Bh) * np.sqrt(w)[:, None]).ravel()

    x0 = np.concatenate([A0.reshape(-1), b0, B0])
    try:
        sol = least_squares(resid, x0, method="trf", loss="soft_l1",
                            f_scale=0.03, max_nfev=400)
    except Exception as exc:
        if verbose:
            print(f"  世界系精修失败: {exc}")
        return None

    A, b, Bh = unpack(sol.x)
    Bh = Bh / max(np.linalg.norm(Bh), 1e-9)
    u = (m - b) @ A.T
    pred = np.einsum("nij,nj->ni", R, u)
    ang = np.degrees(np.arccos(np.clip(np.sum(pred * Bh, axis=1), -1, 1)))
    mags = np.linalg.norm(u, axis=1)
    if verbose:
        print(f"  世界系精修: 角度残差中位 {np.median(ang):.3f} deg  "
              f"p95 {np.percentile(ang,95):.3f} deg")
        print(f"              |u| 均值 {mags.mean():.5f}  std {mags.std():.5f}")

    # 折回椭球参数: (m-b)^T (A^T A) (m-b) = |u|^2 ~ 1
    Mn = A.T @ A / (mags.mean() ** 2)
    S = sym_sqrt(Mn)
    g = float(np.cbrt(np.linalg.det(S)))
    if not np.isfinite(g) or abs(g) < 1e-12:
        return None
    return S / g, b, 1.0 / g

def robust_fit(m: np.ndarray, q: np.ndarray | None, iters: int = 12, verbose: bool = True):

    m = np.asarray(m, dtype=float)

    n = len(m)

    w_cov = np.ones(n)

    w_att = np.ones(n)



    w = np.ones(n)

    w_rob = np.ones(n)

    p = linear_fit(m, w)

    if p is None:

        raise RuntimeError("初始线性拟合失败")



    for it in range(iters):

        W, b, r = p

        d = (m - b) @ W.T

        res = np.linalg.norm(d, axis=1) - r

        if it == 0:

            w_cov = coverage_weights(d)

            if q is not None:

                w_att = attitude_weights(m, q)

        w_dip = dip_weights(m, q, W, b, r)

        w_rob = huber_weights(res)

        w = w_rob * w_cov * w_att * w_dip

        pn = refine(m, w, p)

        if pn is None:

            break

        Wn, bn, rn = pn

        if (np.allclose(Wn, W, rtol=1e-9, atol=1e-12)

                and np.allclose(bn, b, rtol=1e-9, atol=1e-9)):

            p = pn

            break

        p = pn

        if verbose:

            d = (m - bn) @ Wn.T

            res = np.linalg.norm(d, axis=1) - rn

            print(f"  iter {it+1:2d}  rms={np.sqrt(np.mean(res**2)):.6f}  "

                  f"p95={np.percentile(np.abs(res),95):.6f}  eff_w={w.mean():.3f}")



    W, b, r = p

    d = (m - b) @ W.T

    res = np.linalg.norm(d, axis=1) - r



    # 残差统计只看"被判为有效"的样本。把离群点算进去会让报告永远难看，

    # 而且看不出真实质量 —— 离群点的作用应该体现在 inlier 比例上。

    # 残差统一除以 radius -> 无量纲的"场半径"单位，和固件的 geo_q 同一量纲

    res = res / max(r, 1e-9)



    # inlier 只看纯鲁棒权重。乘上覆盖权重会把它带偏（稀疏区块权重能到 20，

    # 扎堆区块低到 0.05），那样统计出来的人数没有意义。

    keep = w_rob > 0.5

    if keep.sum() < 10:

        keep = w_rob >= np.percentile(w_rob, 50)

    rk = np.abs(res[keep])



    info = {

        "n": int(n),

        "n_inlier": int(keep.sum()),

        "bins": occupied_caps(d),

        "rms": float(np.sqrt(np.mean(res[keep] ** 2))),

        "p95": float(np.percentile(rk, 95)),

        "max": float(np.max(rk)),

        "rms_all": float(np.sqrt(np.mean(res ** 2))),

        "det_W": float(np.linalg.det(W)),

        "w_mean": float(w.mean()),

        "w_eff": float((w.sum() ** 2) / max((w ** 2).sum(), 1e-12)),

        "span_pct": [float((m[:, i].max() - m[:, i].min()) * np.linalg.norm(W[i]) / r * 100)

                     for i in range(3)],

    }

    return W, b, r, info, w





def make_blob(W, b, radius, flags=None):

    if flags is None:

        flags = MAG_BLOB_F_CAL1 | MAG_BLOB_F_VERIFIED | MAG_BLOB_F_MODEL_FULL

    off = np.zeros((2, 3))

    off[0] = b

    Wm = np.zeros((2, 9))

    Wm[0] = W.reshape(-1)

    Wm[1] = np.eye(3).reshape(-1)

    rad = np.array([radius, 0.0])

    mapp = np.eye(3).reshape(-1)

    mapc = np.zeros(3)

    align = np.eye(3).reshape(-1)

    vals = [MAG_CAL_BLOB_VERSION]

    vals += list(off.reshape(-1))

    vals += list(Wm.reshape(-1))

    vals += list(rad)

    vals += list(mapp)

    vals += list(mapc)

    vals += list(align)

    vals += [0.0]

    vals += [flags]

    return struct.pack(BLOB_FMT, *vals)





def parse_blob(buf: bytes):

    v = struct.unpack(BLOB_FMT, buf)

    return {

        "version": v[0],

        "offset": np.array(v[1:7]).reshape(2, 3),

        "W": np.array(v[7:25]).reshape(2, 9),

        "radius": np.array(v[25:27]),

        "map": np.array(v[27:36]),

        "map_c": np.array(v[36:39]),

        "align": np.array(v[39:48]),

        "dip": v[48],

        "flags": v[49],

    }





# ---------------------------------------------------------------- 串口



def _serial():

    import serial

    from serial.tools import list_ports

    try:

        from cobs import cobs

        from utils import bowiepb as bpb

    except Exception as exc:

        raise SystemExit(f"缺少依赖: {exc}")

    ports = [p.device for p in list_ports.comports() if (p.vid, p.pid) == (0x2833, 0xB015)]

    return serial, cobs, bpb, (ports[0] if ports else "COM3")





def cmd_collect(args) -> int:

    import serial  # noqa: F401

    import msvcrt

    from cobs import cobs

    from utils import bowiepb as bpb



    _s, _c, _b, port = _serial()

    port = args.port or port



    ser = serial.Serial(port, 115200, timeout=0.02)

    if args.duration and args.duration > 0:

        print(f"打开 {port}，采集 {args.duration:.0f} 秒")

    else:

        print(f"打开 {port}，【不限时】采集 —— 按 Enter 停止")

    print("目标: 覆盖 bins>=18/24, 三轴 span>=100%, 丢帧=0")

    print()



    buf = bytearray()

    m_ = []

    q_ = []

    t_m = []

    t_q = []

    t0 = time.time()

    last_ui = 0.0

    last_len = 0

    nframe = 0

    stop = False



    try:

        while not stop:

            if args.duration and args.duration > 0 and (time.time() - t0) >= args.duration:

                break

            if args.once:
                break
            if msvcrt.kbhit():

                msvcrt.getch()

                stop = True

                break



            chunk = ser.read(8192)

            if chunk:

                buf.extend(chunk)

                while True:

                    i = buf.find(0)

                    if i < 0:

                        break

                    pkt = bytes(buf[:i])

                    del buf[: i + 1]

                    if not pkt:

                        continue

                    try:

                        payload = cobs.decode(pkt)

                        msg = bpb.Data()

                        msg.parse(payload)

                    except Exception:

                        continue

                    nframe += 1

                    d = msg.to_dict()

                    g = d.get("mag")

                    if isinstance(g, dict):

                        t_m.append(float(g.get("seconds", 0))

                                   + float(g.get("nanoseconds", 0)) * 1e-9)

                        m_.append((g.get("x", 0.0), g.get("y", 0.0), g.get("z", 0.0)))

                    g = d.get("quat")

                    if isinstance(g, dict):

                        t_q.append(float(g.get("seconds", 0))

                                   + float(g.get("nanoseconds", 0)) * 1e-9)

                        q_.append((g.get("w", 1.0), g.get("x", 0.0),

                                   g.get("y", 0.0), g.get("z", 0.0)))



            # ---- 实时面板（不按时间截断，只让你决定何时停）----------------

            now = time.time()

            if now - last_ui >= 0.5 and len(m_) > 20:

                last_ui = now

                el = now - t0

                tm = np.asarray(t_m[-2000:])

                dt = np.diff(tm)

                dt = dt[dt > 0]

                rate = 1.0 / np.median(dt) if len(dt) else 0.0

                gaps = int(np.sum(dt > np.median(dt) * 1.5)) if len(dt) else 0



                mm = np.asarray(m_, dtype=float)

                b = mm - mm.mean(axis=0)

                caps = occupied_caps(b)          # 粗略覆盖（未标定方向，用去均值方向代理）

                span = np.ptp(mm, axis=0)

                srel = span / max(span.max(), 1e-9) * 100



                print(f"  t={el:6.1f}s  磁={len(m_):6d}  姿态={len(q_):6d}  "

                      f"{rate:5.1f}Hz  丢{gaps:<3d} "

                      f"覆盖={caps:2d}/24  跨度={srel[0]:3.0f}/{srel[1]:3.0f}/{srel[2]:3.0f}%",

                      end="\r")

    except KeyboardInterrupt:

        pass

    finally:

        ser.close()



    print()

    if len(m_) < 50 or len(q_) < 2:

        print(f"采集太少（磁 {len(m_)}，姿态 {len(q_)}）—— 上位机是否还占着串口？")

        return 1



    t_m = np.asarray(t_m)

    m_ = np.asarray(m_, dtype=float)

    t_q = np.asarray(t_q)

    q_ = np.asarray(q_, dtype=float)



    # ---- 采样率与丢帧实测 ------------------------------------------------

    # USB CDC 缓冲溢出会【静默】丢样本：拟合照样出结果，但那是错的。

    # 唯一可靠的判据是设备自己的时间戳间隔 —— 丢帧表现为间隔跳到 2x/3x。

    def rate_report(name, tt, unit="Hz"):

        if len(tt) < 10:

            print(f"  {name:8s} 样本太少，无法评估")

            return

        tt = np.sort(tt)

        d = np.diff(tt)

        d = d[d > 0]

        if len(d) < 5:

            print(f"  {name:8s} 时间戳异常")

            return

        med = float(np.median(d))

        span = float(tt[-1] - tt[0])

        rate = (len(tt) - 1) / span if span > 0 else 0.0

        gap = d > med * 1.5

        lost = int(np.sum(np.round(d[gap] / med) - 1)) if gap.any() else 0

        print(f"  {name:8s} {rate:7.1f} {unit}   dt 中位 {med*1000:6.2f}ms  "

              f"p95 {np.percentile(d,95)*1000:6.2f}ms  max {d.max()*1000:8.2f}ms")

        print(f"  {'':8s} 丢帧: {int(gap.sum())} 处，约 {lost} 样本 "

              f"({lost/max(len(tt),1)*100:.2f}%)")



    print()

    print("=========== 采样率实测 ===========")

    rate_report("磁力计", t_m)

    rate_report("四元数", t_q)



    # 按时间配对：每个磁样本取时间上最近的姿态

    order = np.argsort(t_q)

    t_q, q_ = t_q[order], q_[order]

    j = np.clip(np.searchsorted(t_q, t_m), 0, len(t_q) - 1)

    jm = np.clip(j - 1, 0, len(t_q) - 1)

    pick = np.where(np.abs(t_q[j] - t_m) <= np.abs(t_q[jm] - t_m), j, jm)

    dt = np.abs(t_q[pick] - t_m)

    keep = dt < 0.05

    print(f"  配对: {keep.sum()} / {len(t_m)}  (失配 {int((~keep).sum())})")



    out = Path(args.out)

    np.savez_compressed(out, mag=m_[keep], quat=q_[pick][keep],

                        t_mag=t_m[keep], t_quat=t_q[pick][keep], dt=dt[keep])

    print(f"已保存 -> {out}   ({int(keep.sum())} 对, {out.stat().st_size/1024:.0f} KB)")

    print()

    print("下一步:  python magcal.py fit " + str(out))

    return 0



# ---------------------------------------------------------------- 从录制转换



Q_SCALE = 16384.0     # 固件 bhi360.c: #define Q_SCALE 16384.0f





def _f(row, key):

    v = row.get(key)

    if v is None or v == "" or v == "None":

        return None

    try:

        x = float(v)

    except (TypeError, ValueError):

        return None

    return x if np.isfinite(x) else None





def _emit_npz(src, out_path, nrow, t_m, m_, t_q, q_):

    import csv as _csv  # noqa: F401

    print(f"读 {src}: {nrow} 行 -> 磁 {len(m_)}  姿态 {len(q_)}")

    if len(m_) < 50 or len(q_) < 2:

        print("样本太少。确认录制期间确实在收 mag/quat 包")

        return 1



    t_m = np.asarray(t_m, dtype=float)

    m_ = np.asarray(m_, dtype=float)

    t_q = np.asarray(t_q, dtype=float)

    q_ = np.asarray(q_, dtype=float)



    def rate_report(name, tt):

        tt = np.sort(tt)

        d = np.diff(tt)

        d = d[d > 0]

        if len(d) < 5:

            return

        med = float(np.median(d))

        rate = (len(tt) - 1) / max(float(tt[-1] - tt[0]), 1e-9)

        gap = d > med * 1.5

        lost = int(np.sum(np.round(d[gap] / med) - 1)) if gap.any() else 0

        print(f"  {name:8s} {rate:7.1f} Hz   dt 中位 {med*1000:6.2f}ms  "

              f"max {d.max()*1000:8.2f}ms   丢帧 {int(gap.sum())} 处 (约 {lost} 样本)")



    print("=========== 采样率实测 ===========")

    rate_report("磁力计", t_m)

    rate_report("四元数", t_q)



    order = np.argsort(t_q)

    t_q, q_ = t_q[order], q_[order]

    j = np.clip(np.searchsorted(t_q, t_m), 0, len(t_q) - 1)

    jm = np.clip(j - 1, 0, len(t_q) - 1)

    pick = np.where(np.abs(t_q[j] - t_m) <= np.abs(t_q[jm] - t_m), j, jm)

    dt = np.abs(t_q[pick] - t_m)

    keep = dt < 0.05

    print(f"  配对: {int(keep.sum())} / {len(t_m)}  (失配 {int((~keep).sum())})")



    out = Path(out_path)

    np.savez_compressed(out, mag=m_[keep], quat=q_[pick][keep],

                        t_mag=t_m[keep], t_quat=t_q[pick][keep], dt=dt[keep])

    print(f"已保存 -> {out}   ({int(keep.sum())} 对, {out.stat().st_size/1024:.0f} KB)")

    print()

    print(f"下一步:  python magcal.py fit {out}")

    return 0





def cmd_fromtrace(args) -> int:

    """把上位机录的文件转成拟合用的 npz（上位机可以一直开着）。



    自动识别两种格式：

      A) 【标定采集】的轻量 CSV —— kind,time,x,y,z,w

         （推荐：只写一个文件，I/O 最小，最不容易丢帧）

      B) 【追踪录制】的 analysis.csv —— mag_*/quat_* 分列

    """

    import csv



    src = Path(args.input)

    if src.is_dir():

        for cand in ("cal.csv", "analysis.csv"):

            if (src / cand).exists():

                src = src / cand

                break

        else:

            hits = sorted(src.glob("*.csv"))

            if not hits:

                print(f"{src} 里没有 CSV")

                return 1

            src = hits[0]

    if not src.exists():

        print(f"找不到 {src}")

        return 1



    t_m, m_, t_q, q_ = [], [], [], []

    nrow = 0

    with src.open("r", encoding="utf-8", errors="replace", newline="") as fh:

        rdr = csv.DictReader(fh)

        fields = [f.strip() for f in (rdr.fieldnames or [])]



        if "kind" in fields:

            print("格式: 标定采集（轻量 CSV）")

            for row in rdr:

                nrow += 1

                kind = (row.get("kind") or "").strip().upper()

                tv = _f(row, "time")

                x, y, z = _f(row, "x"), _f(row, "y"), _f(row, "z")

                if tv is None or None in (x, y, z):

                    continue

                if kind == "M":

                    t_m.append(tv)

                    m_.append((x, y, z))

                elif kind == "Q":

                    wv = _f(row, "w")

                    if wv is None:

                        continue

                    t_q.append(tv)

                    q_.append((wv, x, y, z))

        else:

            print("格式: 追踪录制 analysis.csv")

            for row in rdr:

                nrow += 1

                tm = _f(row, "mag_device_time_s")

                mx, my, mz = _f(row, "mag_x_raw"), _f(row, "mag_y_raw"), _f(row, "mag_z_raw")

                if tm is not None and None not in (mx, my, mz):

                    t_m.append(tm)

                    m_.append((mx, my, mz))



                tq = _f(row, "quat_device_time_s")

                if tq is None:

                    continue

                qn = [_f(row, k) for k in

                      ("quat_w_norm", "quat_x_norm", "quat_y_norm", "quat_z_norm")]

                if all(v is not None for v in qn):

                    qv = qn

                else:

                    qr = [_f(row, k) for k in

                          ("quat_w_raw", "quat_x_raw", "quat_y_raw", "quat_z_raw")]

                    if any(v is None for v in qr):

                        continue

                    qv = [v / Q_SCALE for v in qr]

                t_q.append(tq)

                q_.append(tuple(qv))



    return _emit_npz(src, args.out, nrow, t_m, m_, t_q, q_)



# ---------------------------------------------------------------- 实时监视

def cap_name(k: int) -> str:
    """把 0..23 的区块编号翻成人话，方便告诉你还缺哪个朝向。"""
    axis = "XYZ"
    ax = k // 8
    face = "+" if (k % 8) >= 4 else "-"
    o1 = (ax + 1) % 3
    o2 = (ax + 2) % 3
    s1 = "+" if (k % 8) % 4 >= 2 else "-"
    s2 = "+" if (k % 8) % 2 >= 1 else "-"
    return f"{face}{axis[ax]}({s1}{axis[o1]},{s2}{axis[o2]})"


def _newest_cal(d: Path):
    hits = [p for p in d.glob("cal_*.csv") if p.is_file()]
    if not hits:
        hits = [p for p in d.glob("*.csv") if p.is_file()]
    return max(hits, key=lambda p: p.stat().st_mtime) if hits else None


def cmd_watch(args) -> int:
    """一边采一边看覆盖度 —— 不用盲转。

    只【读】上位机正在写的 CSV，不碰串口，所以上位机照常开着。
    传一个【目录】给它时，会自动跟随该目录里最新的 cal_*.csv，
    所以你先把监视器开着，再在上位机点【开始标定采集】即可。

    覆盖度用去均值方向做代理；实测和最终拟合出的区块判定一致率 89%,
    覆盖【个数】在每个检查点都完全一致，足够用来决定什么时候停。
    """
    import msvcrt

    fh = None
    watch_dir = None
    src = Path(args.input)
    if src.is_dir():
        watch_dir = src
        src = None
    else:
        while not src.exists():
            time.sleep(0.2)

        fh = src.open("r", encoding="utf-8", errors="replace", newline="")
    print(f"监视目录 {watch_dir}" if watch_dir else f"监视 {src}")
    print("上位机里点【开始标定采集】就开始统计；点停止或按 Enter 结束。")
    print()

    t_m, m_, t_q = [], [], []
    first = True
    t0 = time.time()
    last_ui = 0.0
    last_pick = 0.0
    cur_mtime = 0.0

    try:
        while True:
            if watch_dir is not None:
                now = time.time()
                if now - last_pick >= 0.5:
                    last_pick = now
                    cand = _newest_cal(watch_dir)
                    if cand is not None and cand.stat().st_mtime > cur_mtime:
                        if fh is not None:
                            fh.close()
                        fh = cand.open("r", encoding="utf-8",
                                       errors="replace", newline="")
                        cur_mtime = cand.stat().st_mtime
                        src = cand
                        t_m.clear(); m_.clear(); t_q.clear()
                        first = True
                        t0 = time.time()
                        print(f"\n开始统计 {cand.name}                           ")
                if fh is None:
                    time.sleep(0.1)
                    continue

            line = fh.readline()
            if not line:
                if args.once:
                    break
                if msvcrt.kbhit():
                    msvcrt.getch()
                    break
                time.sleep(0.05)
                continue

            if first:
                first = False
                if line.lower().startswith("kind"):
                    continue
            parts = line.rstrip("\r\n").split(",")
            if len(parts) < 6:
                continue
            try:
                tv = float(parts[1])
            except ValueError:
                continue
            if parts[0] == "M":
                try:
                    m_.append((float(parts[2]), float(parts[3]), float(parts[4])))
                    t_m.append(tv)
                except ValueError:
                    pass
            elif parts[0] == "Q":
                t_q.append(tv)

            now = time.time()
            if now - last_ui >= 0.5 and len(m_) > 30:
                last_ui = now
                mm = np.asarray(m_, dtype=float)
                d = mm - mm.mean(axis=0)
                idx = cap_index(d)
                occ = len(np.unique(idx))
                dt = np.diff(np.asarray(t_m[-3000:]))
                dt = dt[dt > 0]
                rate = 1.0 / np.median(dt) if len(dt) else 0.0
                gaps = int(np.sum(dt > np.median(dt) * 1.5)) if len(dt) else 0
                sp = np.ptp(mm, axis=0)
                span = sp / max(sp.max(), 1e-9) * 100
                print("  t=%6.1f s  磁=%6d  姿态=%6d  %5.1f Hz  丢 %-3d  覆盖 = %2d/24  "
                      "跨度 = %3.0f/%3.0f/%3.0f%%"
                      % (now - t0, len(m_), len(t_q), rate, gaps,
                         occ, span[0], span[1], span[2]), end="\r")
    except KeyboardInterrupt:
        pass
    finally:
        if fh is not None:
            try:
                fh.close()
            except Exception:
                pass

    print()
    if len(m_) < 30:
        print("样本太少")
        return 1

    mm = np.asarray(m_, dtype=float)
    d = mm - mm.mean(axis=0)
    idx = cap_index(d)
    occ = len(np.unique(idx))
    print()
    print(f"=========== 本次覆盖 {occ}/24 ===========")
    miss = [cap_name(k) for k in range(24) if k not in set(idx.tolist())]
    if miss:
        print(f"  还缺 {len(miss)} 个朝向:")
        for k in range(0, len(miss), 4):
            print("    " + "  ".join(miss[k:k + 4]))
    else:
        print("  全覆盖")
    print()
    print(f"现在停止上位机的【标定采集】，然后:")
    print(f'  python magcal.py fromtrace "{src}" --out sweep2.npz')
    print( "  python magcal.py fit sweep2.npz")
    return 0


# ---------------------------------------------------------------- fit



def cmd_fit(args) -> int:

    z = np.load(args.input)

    m = z["mag"].astype(float)

    q = z["quat"].astype(float) if not args.no_attitude else None

    print(f"载入 {args.input}: {len(m)} 对样本")

    print(f"  原始跨度 x={np.ptp(m[:,0]):.0f} y={np.ptp(m[:,1]):.0f} z={np.ptp(m[:,2]):.0f} LSB")

    if q is not None:

        print(f"  覆盖(原始方向) = {occupied_caps(m)}/24")



    W, b, r, info, w = robust_fit(m, q, iters=args.iters)



    print()

    print("=========== 拟合结果 ===========")

    print(f"  样本            {info['n']}   有效 {info['n_inlier']} "

          f"({info['n_inlier']/info['n']*100:.1f}%)")

    print(f"  球面覆盖        {info['bins']}/24   (固件门槛 16)")

    print(f"  残差 rms        {info['rms']:.6f}  (只看有效样本；1.0 = 整个半径)")

    print(f"  残差 p95        {info['p95']:.6f}")

    print(f"  残差 max        {info['max']:.6f}")

    print(f"  det(W)          {info['det_W']:.6f}  (必须 = 1)")

    print(f"  有效样本数      {info['w_eff']:.0f} / {info['n']}")

    print(f"  三轴 span(%)    {info['span_pct'][0]:.0f} / {info['span_pct'][1]:.0f} / {info['span_pct'][2]:.0f}")

    print(f"  offset (硬铁)   [{b[0]:.2f} {b[1]:.2f} {b[2]:.2f}] LSB")

    print(f"  radius          {r:.2f}")

    print("  W (软铁, det=1)")

    for row in W:

        print(f"      [{row[0]: .6f} {row[1]: .6f} {row[2]: .6f}]")



    bad = []

    if info["bins"] < 16:

        bad.append(f"覆盖 {info['bins']}/24 < 16")

    if info["p95"] > 0.02:

        bad.append(f"残差 p95 {info['p95']:.4f} > 0.02 (=2% 场半径，环境有干扰？)")

    if abs(info["det_W"] - 1.0) > 1e-4:

        bad.append("det(W) 未归一")

    for i, s in enumerate(info["span_pct"]):

        if s < 100:

            bad.append(f"轴{i} span {s:.0f}% < 100%")

    if bad:

        print("\n  !! 未达标:")

        for x in bad:

            print(f"     - {x}")

    else:

        print("\n  ✓ 全部达标")



    blob = make_blob(W, b, r)

    outblob = Path(args.output or (Path(args.input).with_suffix(".blob")))

    outblob.write_bytes(blob)

    print(f"\nblob 已写出 -> {outblob}  ({len(blob)} 字节)")

    return 0





# ---------------------------------------------------------------- upload



def cmd_upload(args) -> int:

    import serial



    blob = Path(args.blob).read_bytes()

    if len(blob) != BLOB_SIZE:

        print(f"blob 大小不对: {len(blob)} != {BLOB_SIZE}")

        return 1

    print("blob 内容:")

    for k, v in parse_blob(blob).items():

        s = np.array2string(v, precision=4, suppress_small=True) if isinstance(v, np.ndarray) else v

        print(f"   {k:9s} {s}")



    _s, _c, _b, port = _serial()

    port = args.port or port

    ser = serial.Serial(port, 115200, timeout=0.1)

    frame = b"C" + struct.pack("<II", MAG_UPLOAD_MAGIC, len(blob)) + blob

    frame += struct.pack("<I", fnv1a32(blob))

    ser.write(frame)

    ser.flush()

    print(f"\n已发送 {len(frame)} 字节到 {port}，等待板子回应…")

    time.sleep(1.5)

    txt = ser.read(8192)

    ser.close()

    try:

        print(txt.decode("utf-8", "replace"))

    except Exception:

        print(repr(txt))

    return 0





def main() -> int:

    ap = argparse.ArgumentParser(description=__doc__,

                                 formatter_class=argparse.RawDescriptionHelpFormatter)

    sub = ap.add_subparsers(dest="cmd", required=True)



    c = sub.add_parser("collect", help="采集一次翻滚")

    c.add_argument("--port", default=None)

    c.add_argument("--out", required=True)

    c.add_argument("--duration", type=float, default=0.0, help="0=不限时，按 Enter 停止（默认）")

    c.set_defaults(func=cmd_collect)



    w = sub.add_parser("watch", help="一边采一边看覆盖度")
    w.add_argument("input", help="正在写入的标定采集 CSV")
    w.add_argument("--target", type=int, default=20)
    w.add_argument("--once", action="store_true", help="读完现有内容就退出（只出覆盖报告）")
    w.set_defaults(func=cmd_watch)

    tr = sub.add_parser("fromtrace", help="把上位机录制的 analysis.csv 转成 npz")

    tr.add_argument("input", help="analysis.csv 或其所在目录")

    tr.add_argument("--out", required=True)

    tr.set_defaults(func=cmd_fromtrace)



    f = sub.add_parser("fit", help="在本地拟合")

    f.add_argument("input")

    f.add_argument("--output", default=None)

    f.add_argument("--iters", type=int, default=12)

    f.add_argument("--no-attitude", action="store_true", help="跳过姿态一致性加权")

    f.set_defaults(func=cmd_fit)



    u = sub.add_parser("upload", help="把 blob 推回板子")

    u.add_argument("blob")

    u.add_argument("--port", default=None)

    u.set_defaults(func=cmd_upload)



    a = ap.parse_args()

    return a.func(a)





if __name__ == "__main__":

    sys.path.insert(0, str(GLOVE2ROBOT))

    raise SystemExit(main())




