#!/usr/bin/env python3
"""磁标定覆盖度【可视化】监视器。

显示一个球面，把它分成和固件完全一致的 24 个球面区块：
    6 个面（场矢量在机体系的主导轴 ±X/±Y/±Z）× 每面 4 个象限
已经采到样本的区块填绿色，没采到的留灰，当前场方向用红点标出。

只【读】上位机正在写的 CSV，不占串口，所以上位机照常开着。

用法:
    python magcal_view.pyw [CSV 或目录]
不给参数就自动跟随 ..\\recordings 里最新的 cal_*.csv。
鼠标拖动可以旋转球面。
"""

from __future__ import annotations

import math
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import magcal as mc  # noqa: E402

AXIS = "XYZ"


def cap_corners(k: int):
    """第 k 个球面区块的 4 个角（单位球面上的四边形）。"""
    ax = k // 8
    s_ax = 1.0 if (k % 8) >= 4 else -1.0
    o1 = (ax + 1) % 3
    o2 = (ax + 2) % 3
    s1 = 1.0 if (k % 8) % 4 >= 2 else -1.0
    s2 = 1.0 if (k % 8) % 2 >= 1 else -1.0

    def v(a, b, c):
        out = [0.0, 0.0, 0.0]
        out[ax] = a * s_ax
        out[o1] = b * s1
        out[o2] = c * s2
        n = math.sqrt(sum(t * t for t in out))
        return np.array([t / n for t in out])

    return [v(1, 0, 0), v(1, 1, 0), v(1, 1, 1), v(1, 0, 1)]


CAPS = [cap_corners(k) for k in range(24)]


class SphereView(tk.Canvas):
    def __init__(self, master, size=520):
        super().__init__(master, width=size, height=size, bg="#101418",
                         highlightthickness=0)
        self.size = size
        self.rx, self.ry = -0.45, 0.6
        self.counts = np.zeros(24, dtype=int)
        self.cur_dir = None
        self.bind("<Button-1>", self._drag_start)
        self.bind("<B1-Motion>", self._drag)

    def _drag_start(self, e):
        self._last = (e.x, e.y)

    def _drag(self, e):
        dx = e.x - self._last[0]
        dy = e.y - self._last[1]
        self._last = (e.x, e.y)
        self.ry += dx * 0.01
        self.rx += dy * 0.01
        self.redraw()

    def set_counts(self, counts):
        self.counts = counts
        self.redraw()

    def set_dir(self, d):
        self.cur_dir = d
        self.redraw()

    def _rot(self):
        cx, sx = math.cos(self.rx), math.sin(self.rx)
        cy, sy = math.cos(self.ry), math.sin(self.ry)
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        return Ry @ Rx

    def redraw(self):
        self.delete("all")
        R = self._rot()
        cx = cy = self.size / 2
        rad = self.size * 0.36

        def proj(v):
            w = R @ v
            return (cx + w[0] * rad, cy - w[2] * rad), w[1]

        # 球体轮廓
        self.create_oval(cx - rad, cy - rad, cx + rad, cy + rad,
                         outline="#2a3138", width=2)

        faces = []
        for k in range(24):
            pts = []
            zsum = 0.0
            for c in CAPS[k]:
                (px, py), depth = proj(c)
                pts.extend([px, py])
                zsum += depth
            faces.append((zsum / 4.0, k, pts))
        faces.sort(key=lambda t: t[0])

        for depth, k, pts in faces:
            n = self.counts[k]
            if depth < 0:                       # 背面
                fill = "#1d2a1d" if n else "#1a1e22"
                outline = "#243024" if n else "#232830"
            else:
                if n:
                    lv = min(1.0, n / 800.0)
                    g = int(120 + 120 * lv)
                    fill = f"#{30 + int(20*lv):02x}{g:02x}{50 + int(30*lv):02x}"
                else:
                    fill = "#2b3138"
                outline = "#39424c"
            self.create_polygon(*pts, fill=fill, outline=outline, width=1)

        # 坐标轴参考
        for i, (ax_v, label, col) in enumerate([
            (np.array([1.0, 0, 0]), "X", "#c8563c"),
            (np.array([0, 1.0, 0]), "Y", "#4fae52"),
            (np.array([0, 0, 1.0]), "Z", "#3f7fd0"),
        ]):
            (px, py), _ = proj(ax_v * 1.05)
            self.create_line(cx, cy, px, py, fill=col, width=2)
            self.create_text(px, py, text=label, fill=col,
                             font=("Arial", 11, "bold"))

        # 当前场方向
        if self.cur_dir is not None:
            (px, py), depth = proj(self.cur_dir)
            if depth >= 0:
                self.create_oval(px - 7, py - 7, px + 7, py + 7,
                                 fill="#ff4d4d", outline="#ffffff", width=2)


class App:
    def __init__(self, root, target, path):
        self.root = root
        self.target = target
        root.title("磁标定覆盖度监视")
        root.configure(bg="#101418")

        self.view = SphereView(root)
        self.view.pack(side=tk.LEFT, padx=10, pady=10)

        side = tk.Frame(root, bg="#101418")
        side.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 14), pady=14)

        self.lbl_cov = tk.Label(side, text="覆盖 --/24", fg="#7ee787", bg="#101418",
                                font=("Consolas", 22, "bold"), anchor="w")
        self.lbl_cov.pack(anchor="w")
        self.lbl_state = tk.Label(side, text="等待采集…", fg="#8b949e", bg="#101418",
                                  font=("Consolas", 11), anchor="w", justify="left")
        self.lbl_state.pack(anchor="w", pady=(6, 12))

        self.lbl_stat = tk.Label(side, text="", fg="#c9d1d9", bg="#101418",
                                 font=("Consolas", 10), anchor="w", justify="left")
        self.lbl_stat.pack(anchor="w")

        tk.Label(side, text="", bg="#101418").pack(pady=6)
        tk.Label(side, text="还缺的朝向:", fg="#8b949e", bg="#101418",
                 font=("Consolas", 10), anchor="w").pack(anchor="w")
        self.lbl_miss = tk.Label(side, text="", fg="#ffa657", bg="#101418",
                                 font=("Consolas", 10), anchor="w", justify="left")
        self.lbl_miss.pack(anchor="w")

        tk.Label(side, text="\n鼠标拖动可旋转球面", fg="#4a5560", bg="#101418",
                 font=("Consolas", 9)).pack(anchor="w")

        self.path = Path(path)
        self.watch_dir = self.path if self.path.is_dir() else None
        self.cur = None
        self.fh = None
        self.mtime = 0.0
        self.t_m, self.m_ = [], []
        self.t0 = time.time()
        self.frame = 0

        self.root.after(200, self.tick)

    def _pick(self):
        if self.watch_dir is None:
            return self.path if self.path.exists() else None
        return mc._newest_cal(self.watch_dir)

    def tick(self):
        try:
            self._tick()
        except Exception as exc:
            self.lbl_state.config(text=f"错误: {exc}", fg="#f85149")
        self.frame += 1
        self.root.after(150, self.tick)

    def _tick(self):
        cand = self._pick()
        if cand is not None and cand.stat().st_mtime > self.mtime:
            if self.fh is not None:
                self.fh.close()
            self.fh = cand.open("r", encoding="utf-8", errors="replace", newline="")
            self.mtime = cand.stat().st_mtime
            self.cur = cand
            self.t_m.clear()
            self.m_.clear()
            self.t0 = time.time()
            self.first = True
            self.lbl_state.config(text=f"采集中: {cand.name}", fg="#58a6ff")

        if self.fh is None:
            return

        while True:
            line = self.fh.readline()
            if not line:
                break
            if getattr(self, "first", True):
                self.first = False
                if line.lower().startswith("kind"):
                    continue
            p = line.rstrip("\r\n").split(",")
            if len(p) < 6 or p[0] != "M":
                continue
            try:
                self.t_m.append(float(p[1]))
                self.m_.append((float(p[2]), float(p[3]), float(p[4])))
            except ValueError:
                continue

        if len(self.m_) < 20:
            return

        mm = np.asarray(self.m_, dtype=float)
        d = mm - mm.mean(axis=0)
        nrm = np.linalg.norm(d, axis=1, keepdims=True)
        nrm[nrm < 1e-9] = 1.0
        dn = d / nrm
        idx = mc.cap_index(dn)
        counts = np.bincount(idx, minlength=24)
        occ = int((counts > 0).sum())

        self.view.set_counts(counts)
        self.view.set_dir(dn[-1])

        dt = np.diff(np.asarray(self.t_m[-3000:]))
        dt = dt[dt > 0]
        rate = 1.0 / float(np.median(dt)) if len(dt) else 0.0
        gaps = int(np.sum(dt > np.median(dt) * 1.5)) if len(dt) else 0
        sp = np.ptp(mm, axis=0)
        span = sp / max(sp.max(), 1e-9) * 100

        col = "#7ee787" if occ >= self.target else "#e3b341"
        self.lbl_cov.config(text=f"覆盖 {occ:2d}/24", fg=col)
        self.lbl_stat.config(
            text=(f"时间    {time.time()-self.t0:7.1f} s\n"
                  f"磁样本  {len(self.m_):7d}\n"
                  f"速率    {rate:7.1f} Hz\n"
                  f"丢帧    {gaps:7d} 处\n"
                  f"跨度    {span[0]:3.0f}/{span[1]:3.0f}/{span[2]:3.0f} %")
        )
        miss = [mc.cap_name(k) for k in range(24) if counts[k] == 0]
        self.lbl_miss.config(text="\n".join(miss) if miss else "无（已全覆盖）")

        if occ >= self.target:
            self.lbl_state.config(text=f"✓ 已达标 {occ}/24，可以停了", fg="#7ee787")


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else str(HERE.parent / "recordings")
    root = tk.Tk()
    App(root, 20, path)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
