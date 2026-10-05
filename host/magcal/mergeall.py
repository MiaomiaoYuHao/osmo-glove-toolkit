from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np
import magcal as mc


def load_pair(path: Path):
    t_m, m_, t_q, q_ = [], [], [], []
    with path.open(encoding="utf-8", errors="replace", newline="") as fh:
        reader = csv.reader(fh)
        next(reader, None)  # header
        for p in reader:
            if len(p) < 6:
                continue
            try:
                tv = float(p[1])
            except ValueError:
                continue
            if p[0] == "M":
                try:
                    m_.append((float(p[2]), float(p[3]), float(p[4])))
                    t_m.append(tv)
                except ValueError:
                    pass
            elif p[0] == "Q":
                try:
                    q_.append((float(p[5]), float(p[2]), float(p[3]), float(p[4])))
                    t_q.append(tv)
                except ValueError:
                    pass
    return np.asarray(t_m), np.asarray(m_, dtype=float), np.asarray(t_q), np.asarray(q_, dtype=float)


def main() -> int:
    here = Path(__file__).resolve().parent
    source = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else here.parent.parent / "recordings"
    output = Path(sys.argv[2]).expanduser() if len(sys.argv) > 2 else here / "allsweep.npz"
    files = sorted(source.glob("cal_*.csv")) if source.is_dir() else [source]
    print("找到", len(files), "次采集:")
    all_m, all_q = [], []
    for path in files:
        t_m, m_, t_q, q_ = load_pair(path)
        print(f"  {path.name:32s} 磁 {len(m_):7d}  姿态 {len(q_):7d}")
        if len(m_) < 50 or len(q_) < 2:
            continue
        order = np.argsort(t_q)
        t_q, q_ = t_q[order], q_[order]
        j = np.clip(np.searchsorted(t_q, t_m), 0, len(t_q) - 1)
        jm = np.clip(j - 1, 0, len(t_q) - 1)
        pick = np.where(np.abs(t_q[j] - t_m) <= np.abs(t_q[jm] - t_m), j, jm)
        keep = np.abs(t_q[pick] - t_m) < 0.05
        all_m.append(m_[keep])
        all_q.append(q_[pick][keep])
    if not all_m:
        print("没有可用于合并的标定 CSV")
        return 1
    merged_m = np.vstack(all_m)
    merged_q = np.vstack(all_q)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, mag=merged_m, quat=merged_q)
    print()
    print("合并后:", len(merged_m), "对样本")
    print("已保存", output)
    print("覆盖(代理):", len(np.unique(mc.cap_index(merged_m - merged_m.mean(axis=0)))), "/24")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
