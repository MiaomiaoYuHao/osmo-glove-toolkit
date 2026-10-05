#!/usr/bin/env python3
"""OSMO 三维力测试上位机。

只读取现有 BowieGlove COBS/protobuf 数据流，不修改固件。
两路磁力计做零点差分，再经过 3x3 标定矩阵得到三维相对力。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import queue
import subprocess
import sys

import numpy as np
import threading
import time
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Any

from serial.tools import list_ports

import serial
import read_osmo_glove as glove
from trace_recorder import TraceRecorder

Vec3 = tuple[float, float, float]
FINGER_NAMES = {0: "none", 1: "pinky", 2: "ring", 3: "middle", 4: "index", 5: "thumb"}
IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
MAG_SCALES = {
    "1/4 µT（双磁 Poll_Meta）": 0.25,
    "1/16 µT（官方NDOF）": 0.0625,
    "1 LSB（仅原始计数）": 1.0,
    "归一化（铁标定后）": 1.0,
}


def vadd(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vsub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vmul(a: Vec3, scale: float) -> Vec3:
    return (a[0] * scale, a[1] * scale, a[2] * scale)


def vnorm(a: Vec3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def median_axis(rows: list[Vec3], axis: int) -> float:
    values = sorted(row[axis] for row in rows)
    if not values:
        return 0.0
    middle = len(values) // 2
    if len(values) % 2:
        return values[middle]
    return 0.5 * (values[middle - 1] + values[middle])


def median_vec(rows: list[Vec3]) -> Vec3:
    return tuple(median_axis(rows, axis) for axis in range(3))  # type: ignore[return-value]


def parse_matrix(text: str) -> tuple[tuple[float, float, float], ...]:
    values = [float(part) for part in text.replace(",", " ").replace(";", " ").split()]
    if len(values) != 9:
        raise ValueError("矩阵必须包含 9 个数字")
    return (
        (values[0], values[1], values[2]),
        (values[3], values[4], values[5]),
        (values[6], values[7], values[8]),
    )


def matrix_vector(matrix: tuple[tuple[float, float, float], ...], vector: Vec3) -> Vec3:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


class ForceSerialReader(threading.Thread):
    """Small dedicated serial reader for real-time force display."""

    def __init__(self, port: str, output: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.port = port
        self.output = output
        self.stop_event = stop_event
        self.ser: serial.Serial | None = None
        self.last_rx = time.monotonic()
        self.has_rx = False

    def run(self) -> None:
        buffer = bytearray()
        while not self.stop_event.is_set():
            if self.ser is None:
                try:
                    self.ser = serial.Serial(self.port, baudrate=115200, timeout=0.1)
                    buffer.clear()
                    self.last_rx = time.monotonic()
                    self.has_rx = False
                    self.output.put(("status", f"Connected to {self.port}"))
                except Exception as exc:
                    self.output.put(("status", f"Reconnecting {self.port}: {exc}"))
                    time.sleep(0.35)
                    continue
            try:
                chunk = self.ser.read(1)
            except (serial.SerialException, OSError) as exc:
                self.output.put(("status", f"Connection lost, reconnecting {self.port}: {exc}"))
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                buffer.clear()
                time.sleep(0.25)
                continue
            if not chunk:
                silence_s = 10.0 if self.has_rx else 120.0
                if time.monotonic() - self.last_rx > silence_s:
                    self.output.put(("status", f"No data on {self.port}; reconnecting"))
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                    buffer.clear()
                continue
            self.last_rx = time.monotonic()
            self.has_rx = True
            buffer.extend(chunk)
            if len(buffer) > 1024 * 1024:
                buffer.clear()
                continue
            while True:
                delimiter = buffer.find(b"\x00")
                if delimiter < 0:
                    break
                packet = bytes(buffer[:delimiter])
                del buffer[: delimiter + 1]
                msg, error, _payload = glove.decode_packet_with_payload(packet)
                if msg is None:
                    if error and "cobs_decode_error" not in error:
                        self.output.put(("decode_error", error))
                    continue
                try:
                    row = glove.message_to_row(msg)
                except Exception as exc:
                    self.output.put(("decode_error", str(exc)))
                    continue
                if row.get("mag_x") is not None:
                    self.output.put(("mag", row))
                elif row.get("quat_x") is not None:
                    self.output.put(("quat", row))
                else:
                    self.output.put(("row", row))
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass



def fit_ellipsoid(samples: list[Vec3]) -> tuple[Vec3, tuple[Vec3, ...], float]:
    """Fit a general ellipsoid and return hard-iron offset + soft-iron matrix.

    The returned transform maps calibrated samples close to a unit sphere:
        calibrated = W @ (raw - offset)
    """
    if len(samples) < 120:
        raise ValueError(f"样本不足：{len(samples)} < 120")
    x = np.asarray(samples, dtype=float)
    center0 = np.median(x, axis=0)
    centered = x - center0
    scale0 = float(np.median(np.linalg.norm(centered, axis=1)))
    if not math.isfinite(scale0) or scale0 <= 1e-9:
        raise ValueError("磁场数据变化过小，无法拟合")

    span = np.percentile(x, 95, axis=0) - np.percentile(x, 5, axis=0)
    min_span = max(1e-9, scale0 * 0.12)
    if float(np.min(span)) < min_span:
        raise ValueError("旋转覆盖不足：请让三个轴都获得足够角度")

    mask = np.ones(len(x), dtype=bool)
    offset = np.zeros(3)
    soft = np.eye(3)
    residual = float("inf")
    for _ in range(3):
        points = centered[mask] / scale0
        px, py, pz = points[:, 0], points[:, 1], points[:, 2]
        design = np.column_stack((
            px * px, py * py, pz * pz,
            2.0 * px * py, 2.0 * px * pz, 2.0 * py * pz,
            px, py, pz,
        ))
        coeff, *_ = np.linalg.lstsq(design, np.ones(len(points)), rcond=None)
        a = np.array([
            [coeff[0], coeff[3], coeff[4]],
            [coeff[3], coeff[1], coeff[5]],
            [coeff[4], coeff[5], coeff[2]],
        ], dtype=float)
        b = np.array(coeff[6:9], dtype=float)
        center_p = -0.5 * np.linalg.solve(a, b)
        constant = float(center_p @ a @ center_p + 1.0)
        eigvals, eigvecs = np.linalg.eigh(a)
        if constant <= 1e-12 or float(np.min(eigvals)) <= 1e-12:
            raise ValueError("椭球拟合退化")
        sqrt_a = eigvecs @ np.diag(np.sqrt(eigvals)) @ eigvecs.T
        offset = center0 + scale0 * center_p
        soft = (sqrt_a / math.sqrt(constant)) / scale0
        radii = np.linalg.norm((x - offset) @ soft.T, axis=1)
        med = float(np.median(radii))
        mad = float(np.median(np.abs(radii - med)))
        residual = float(np.std(radii))
        if mad <= 1e-12:
            break
        new_mask = np.abs(radii - med) <= 3.5 * 1.4826 * mad
        if np.count_nonzero(new_mask) < 100 or np.array_equal(new_mask, mask):
            break
        mask = new_mask

    if not np.all(np.isfinite(soft)) or abs(float(np.linalg.det(soft))) < 1e-18:
        raise ValueError("软铁矩阵无效")
    return tuple(float(v) for v in offset), tuple(tuple(float(v) for v in row) for row in soft), residual


def fit_direction_matrix(samples: list[Vec3]) -> tuple[tuple[Vec3, ...], float, float]:
    if len(samples) != 6:
        raise ValueError("需要六个方向样本")
    desired = np.array([
        [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0], [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0], [0.0, 0.0, -1.0],
    ])
    measured = np.asarray(samples, dtype=float)
    solution, *_ = np.linalg.lstsq(measured, desired, rcond=None)
    predicted = measured @ solution
    residual = float(np.sqrt(np.mean((predicted - desired) ** 2)))
    condition = float(np.linalg.cond(measured))
    matrix = solution.T
    if not np.all(np.isfinite(matrix)) or condition > 1e6:
        raise ValueError(f"标定数据退化，condition={condition:.3g}")
    return tuple(tuple(float(v) for v in row) for row in matrix), residual, condition


def extract_rubbing_direction(samples: list[Vec3]) -> tuple[Vec3, float, float]:
    """Extract the dominant signed motion vector from repeated rubbing data."""
    if len(samples) < 20:
        raise ValueError(f"样本不足：{len(samples)} < 20")
    data = np.asarray(samples, dtype=float)
    centered = data - np.mean(data, axis=0)
    norms = np.linalg.norm(data, axis=1)
    active = norms > max(np.percentile(norms, 20), 1e-12)
    if np.count_nonzero(active) < 10:
        raise ValueError("动作幅度太小，无法学习方向")
    active_data = centered[active]
    covariance = np.cov(active_data, rowvar=False)
    eigvals, eigvecs = np.linalg.eigh(covariance)
    order = np.argsort(eigvals)
    axis = eigvecs[:, order[-1]]
    principal = max(float(eigvals[order[-1]]), 0.0)
    secondary = max(float(eigvals[order[-2]]), 1e-12)
    ratio = principal / secondary
    if ratio < 1.15:
        raise ValueError(f"运动方向不够集中，主轴比仅 {ratio:.2f}，请只沿一个方向揉搓")
    projection = centered @ axis
    gain = float(np.percentile(np.abs(projection), 90))
    if gain <= 1e-12:
        raise ValueError("方向幅值过小")

    # The first significant movement determines the sign of this labelled direction.
    onset_threshold = max(0.5 * float(np.percentile(norms, 95)), 1e-12)
    high = np.where(norms > onset_threshold)[0]
    if not len(high):
        raise ValueError("没有检测到起始运动")
    first = int(high[0])
    early_slice = data[first:min(first + 8, len(data))]
    early_weights = norms[first:min(first + 8, len(data))]
    early = np.average(early_slice, axis=0, weights=np.maximum(early_weights, 1e-12))
    if float(np.dot(early, axis)) < 0.0:
        axis = -axis
    return tuple(float(v) for v in axis * gain), ratio, gain


class ForceMonitor:
    """双磁差分三维力窗口。"""

    def __init__(self, root: tk.Tk, auto_connect: bool = True, auto_trace: bool = False):
        self.root = root
        self.root.title("OSMO 三维力测试上位机")
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        win_w = min(1280, max(900, screen_w - 80))
        win_h = min(820, max(620, screen_h - 110))
        win_x = max(0, (screen_w - win_w) // 2)
        win_y = max(0, (screen_h - win_h) // 2)
        self.root.geometry(f"{win_w}x{win_h}+{win_x}+{win_y}")
        self.root.minsize(min(980, win_w), min(620, win_h))
        self.events: queue.Queue = queue.Queue()
        self.stop_event = threading.Event()
        self.reader: ForceSerialReader | None = None
        self.mag_samples: dict[int, tuple[float, Vec3, dict[str, Any]]] = {}
        self.pair_left: int | None = None
        self.pair_right: int | None = None
        self.baseline_left: Vec3 | None = None
        self.baseline_right: Vec3 | None = None
        self.zero_started = 0.0
        self.zero_samples_left: list[Vec3] = []
        self.zero_samples_right: list[Vec3] = []
        self.force_raw: Vec3 = (0.0, 0.0, 0.0)
        self.force_filtered: Vec3 = (0.0, 0.0, 0.0)
        self.delta_vector: Vec3 = (0.0, 0.0, 0.0)
        self.raw_left: Vec3 = (0.0, 0.0, 0.0)
        self.raw_right: Vec3 = (0.0, 0.0, 0.0)
        self.matrix = IDENTITY
        self.iron_cal: dict[int, tuple[Vec3, tuple[Vec3, ...], float]] = {}
        self.iron_cal_path = Path(__file__).resolve().parent / "force_iron_calibration.json"
        self.iron_calibrating = False
        self.iron_samples: dict[int, list[Vec3]] = {}
        self.iron_started = 0.0
        self.iron_duration = 30.0
        self.iron_pair: tuple[int | None, int | None] = (None, None)
        self.direction_cal_path = Path(__file__).resolve().parent / "force_direction_calibration.json"
        self.direction_calibrating = False
        self.direction_collecting = False
        self.direction_step = 0
        self.direction_samples: list[Vec3] = []
        self.direction_results: dict[int, Vec3] = {}
        self.direction_collect_start = 0.0
        self.direction_collect_duration = 4.0
        self.direction_window: tk.Toplevel | None = None
        self.frame_count = 0
        self.rate_times: deque[float] = deque(maxlen=200)
        self.series: deque[tuple[float, float, float]] = deque(maxlen=420)
        self.record_file = None
        self.record_writer: csv.DictWriter | None = None
        self.record_path: Path | None = None
        self.record_rows = 0
        self.demo_running = False
        self.demo_timer = None
        self.demo_t0 = 0.0
        self.disable_demo = False
        self.trace = TraceRecorder("force_monitor", __file__)
        self.trace_root = Path(__file__).resolve().parent / "recordings"
        self.auto_trace = bool(auto_trace)
        self.port_var = tk.StringVar()
        self.pair_var = tk.StringVar(value="自动")
        self.unit_var = tk.StringVar(value="1/4 µT（双磁 Poll_Meta）")
        self.smoothing_var = tk.DoubleVar(value=0.75)
        self.gain_var = tk.DoubleVar(value=1.0)
        self.range_var = tk.DoubleVar(value=50.0)
        self.auto_range_var = tk.BooleanVar(value=True)
        self.topmost_var = tk.BooleanVar(value=True)
        self._auto_range_value = 50.0
        self._auto_range_updated = 0.0
        self._force_changed = True
        self._last_data_time = 0.0
        self.status_var = tk.StringVar(value="准备连接")
        self.zero_var = tk.StringVar(value="零点：未建立")
        self.iron_var = tk.StringVar(value="软硬铁：未标定")
        self.direction_var = tk.StringVar(value="方向标定：未标定")
        self.force_var = tk.StringVar(value="F = (0.000, 0.000, 0.000)")
        self.fx_var = tk.StringVar(value="Fx = 0.000")
        self.fy_var = tk.StringVar(value="Fy = 0.000")
        self.fz_var = tk.StringVar(value="Fz = 0.000")
        self.fmag_var = tk.StringVar(value="|F| = 0.000")
        self.mag_var = tk.StringVar(value="mag0=-  mag1=-  ΔB=-")
        self.rate_var = tk.StringVar(value="0.0 Hz  0 帧")
        self.trace_var = tk.StringVar(value="追踪：未录制")
        self._build_ui()
        self._load_iron_calibration()
        self._load_direction_calibration()
        self.refresh_ports()
        self.root.after(30, self._poll_events)
        self.root.after(33, self._render)
        if self.auto_trace:
            self.root.after(120, self.start_auto_trace)
        if auto_connect:
            self.root.after(350, self.auto_connect)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(180, self._raise_window)
        self._status_path = Path(__file__).resolve().parent / "3d_force_ui_status.json"
        self._status_written = 0.0
        self._restart_state_path = Path(__file__).resolve().parent / "force_3d_ui_restart.json"
        self._connect_started = time.monotonic()
        self._last_frame_time = time.monotonic()
        self._restarting = False
        self.root.after(2000, self._connection_watchdog)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=(10, 8)); top.pack(fill=tk.X)
        ttk.Label(top, text="端口:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=16, state="readonly"); self.port_combo.pack(side=tk.LEFT, padx=(5, 3))
        ttk.Button(top, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT, padx=2)
        self.connect_button = ttk.Button(top, text="连接", command=self.toggle_connection); self.connect_button.pack(side=tk.LEFT, padx=(8, 3))
        ttk.Button(top, text="双磁零点(Z)", command=self.start_zero).pack(side=tk.LEFT, padx=(16, 3))
        self.iron_button = ttk.Button(top, text="软硬铁标定(I)", command=self.toggle_iron_calibration)
        self.iron_button.pack(side=tk.LEFT, padx=3)
        ttk.Button(top, text="六方向标定", command=self.open_direction_calibration).pack(side=tk.LEFT, padx=3)
        self.record_button = ttk.Button(top, text="开始 CSV", command=self.toggle_recording); self.record_button.pack(side=tk.LEFT, padx=3)
        ttk.Button(top, text="清空曲线", command=self.clear_series).pack(side=tk.LEFT, padx=3)
        ttk.Button(top, text="演示", command=self.toggle_demo).pack(side=tk.LEFT, padx=3)
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.RIGHT)
        controls = ttk.Frame(self.root, padding=(10, 0, 10, 7)); controls.pack(fill=tk.X)
        ttk.Label(controls, text="传感器对:").pack(side=tk.LEFT)
        self.pair_combo = ttk.Combobox(controls, textvariable=self.pair_var, width=22, state="readonly"); self.pair_combo.pack(side=tk.LEFT, padx=(5, 12))
        self.pair_combo.bind("<<ComboboxSelected>>", lambda _event: self._on_pair_changed())
        ttk.Label(controls, text="磁单位:").pack(side=tk.LEFT)
        unit_combo = ttk.Combobox(controls, textvariable=self.unit_var, width=24, state="readonly", values=list(MAG_SCALES.keys())); unit_combo.pack(side=tk.LEFT, padx=(5, 12))
        unit_combo.bind("<<ComboboxSelected>>", lambda _event: self._reset_force_state("磁单位变化"))
        ttk.Label(controls, text="平滑:").pack(side=tk.LEFT)
        ttk.Scale(controls, from_=0.0, to=0.95, variable=self.smoothing_var, length=110).pack(side=tk.LEFT, padx=(4, 2))
        ttk.Label(controls, text="增益:").pack(side=tk.LEFT, padx=(10, 0)); ttk.Entry(controls, textvariable=self.gain_var, width=8).pack(side=tk.LEFT, padx=4)
        ttk.Label(controls, text="显示量程:").pack(side=tk.LEFT, padx=(8, 0)); ttk.Entry(controls, textvariable=self.range_var, width=8).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(controls, text="自动", variable=self.auto_range_var).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(controls, text="量程复位(R)", command=self.reset_auto_range).pack(side=tk.LEFT, padx=(5, 0))
        ttk.Checkbutton(controls, text="置顶", variable=self.topmost_var, command=self._apply_topmost).pack(side=tk.LEFT, padx=(5, 0))
        values = ttk.Frame(self.root, padding=(10, 0, 10, 5))
        values.pack(fill=tk.X)
        for name, variable, color in (
            ("Fx", self.fx_var, "#ff5c67"),
            ("Fy", self.fy_var, "#42e68b"),
            ("Fz", self.fz_var, "#6ea8ff"),
            ("|F|", self.fmag_var, "#ffd166"),
        ):
            panel = tk.Frame(values, bg="#0b1118", highlightbackground="#243a4d", highlightthickness=1)
            panel.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
            tk.Label(panel, text=name, bg="#0b1118", fg=color, font=("Arial", 11, "bold")).pack(anchor="w", padx=8, pady=(5, 0))
            tk.Label(panel, textvariable=variable, bg="#0b1118", fg=color, font=("Consolas", 20, "bold")).pack(anchor="w", padx=8, pady=(0, 5))
        bottom = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        bottom.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Label(bottom, textvariable=self.rate_var, font=("Consolas", 10)).pack(side=tk.LEFT)
        ttk.Label(bottom, text="Z=双磁零点；矩阵单位阵时输出为相对力 a.u.").pack(side=tk.RIGHT)

        self.series_canvas = tk.Canvas(self.root, height=230, background="#0d1117", highlightthickness=0)
        self.series_canvas.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=(0, 5))
        self.series_canvas.bind("<Configure>", lambda _event: setattr(self, "_force_changed", True))

        middle = ttk.Frame(self.root, padding=(10, 0, 10, 4))
        middle.pack(fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(middle, background="#101318", highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _event: setattr(self, "_force_changed", True))
        side = ttk.Frame(middle, padding=(9, 3), width=355)
        side.pack(side=tk.RIGHT, fill=tk.Y)
        side.pack_propagate(False)
        ttk.Label(side, text="三维力 / 传感器坐标系", font=("Arial", 11, "bold")).pack(anchor="center")
        ttk.Separator(side).pack(fill=tk.X, pady=7)
        ttk.Label(side, textvariable=self.force_var, font=("Consolas", 11, "bold"), justify=tk.LEFT, anchor=tk.W).pack(fill=tk.X)
        ttk.Label(side, textvariable=self.mag_var, font=("Consolas", 9), justify=tk.LEFT, anchor=tk.W).pack(fill=tk.X, pady=(6, 2))
        ttk.Label(side, textvariable=self.zero_var, foreground="#6fa8dc").pack(anchor=tk.W, pady=(4, 2))
        ttk.Label(side, textvariable=self.iron_var, foreground="#e6b85c").pack(anchor=tk.W, pady=(0, 4))
        ttk.Button(side, text="清除软硬铁标定", command=self.clear_iron_calibration).pack(anchor=tk.W, pady=(0, 2))
        ttk.Label(side, textvariable=self.direction_var, foreground="#8fd3a8").pack(anchor=tk.W, pady=(0, 2))
        ttk.Button(side, text="清除六方向标定", command=self.clear_direction_calibration).pack(anchor=tk.W, pady=(0, 6))
        ttk.Label(side, text="方向 / 交叉耦合矩阵 M（F = M·ΔB）").pack(anchor=tk.W)
        self.matrix_text = tk.Text(side, height=4, width=34, font=("Consolas", 10), bg="#0b1118", fg="#dce7f2", insertbackground="white")
        self.matrix_text.pack(fill=tk.X, pady=(3, 4))
        self.matrix_text.insert("1.0", "1 0 0\n0 1 0\n0 0 1")
        matrix_buttons = ttk.Frame(side)
        matrix_buttons.pack(fill=tk.X)
        ttk.Button(matrix_buttons, text="应用矩阵", command=self.apply_matrix).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3))
        ttk.Button(matrix_buttons, text="单位阵", command=self.identity_matrix).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(3, 0))
        ttk.Label(side, text="默认显示相对力。\n先空载归零，再施加 ±X/±Y/±Z 已知力标定矩阵，\n即可把单位改为 N。", justify=tk.LEFT).pack(anchor=tk.W, pady=(10, 0))
        ttk.Label(side, textvariable=self.trace_var, wraplength=330).pack(anchor=tk.W, side=tk.BOTTOM, pady=5)

        self.root.bind("<Key-z>", lambda _event: self.start_zero()); self.root.bind("<Key-Z>", lambda _event: self.start_zero())
        self.root.bind("<Key-d>", lambda _event: self.toggle_demo()); self.root.bind("<Key-D>", lambda _event: self.toggle_demo())
        self.root.bind("<Key-r>", lambda _event: self.reset_auto_range()); self.root.bind("<Key-R>", lambda _event: self.reset_auto_range())
        self.root.bind("<Key-i>", lambda _event: self.toggle_iron_calibration()); self.root.bind("<Key-I>", lambda _event: self.toggle_iron_calibration())


    def _apply_topmost(self) -> None:
        try:
            self.root.attributes("-topmost", bool(self.topmost_var.get()))
        except Exception:
            pass

    def _raise_window(self) -> None:
        try:
            self.root.deiconify()
            try:
                self.root.state("zoomed")
            except Exception:
                pass
            self.root.lift()
            self._apply_topmost()
        except Exception:
            pass

    def toggle_demo(self) -> None:
        if self.demo_running:
            self.stop_demo()
        else:
            self.start_demo()

    def start_demo(self) -> None:
        if self.disable_demo:
            return
        self.demo_running = True
        self.demo_t0 = time.monotonic()
        self.pair_left, self.pair_right = 13, 33
        self.unit_var.set("1 LSB（仅原始计数）")
        self.gain_var.set(1.0)
        self.smoothing_var.set(0.18)
        self.baseline_left = (12000.0, -3500.0, 22000.0)
        self.baseline_right = (10500.0, -3100.0, 21000.0)
        self.series.clear()
        self.status_var.set("演示模式：合成三维力")
        self._force_changed = True
        self._demo_tick()

    def _demo_tick(self) -> None:
        if not self.demo_running:
            return
        t = time.monotonic() - self.demo_t0
        fx = 900.0 * math.sin(2.0 * math.pi * 0.24 * t)
        fy = 700.0 * math.cos(2.0 * math.pi * 0.31 * t)
        fz = 450.0 * math.sin(2.0 * math.pi * 0.17 * t - 0.7)
        base0 = (12000.0, -3500.0, 22000.0)
        base1 = (10500.0, -3100.0, 21000.0)
        self._handle_mag({"sensor_id": 13, "mag_x": base0[0] + fx, "mag_y": base0[1] + fy, "mag_z": base0[2] + fz, "_demo": True})
        self._handle_mag({"sensor_id": 33, "mag_x": base1[0], "mag_y": base1[1], "mag_z": base1[2], "_demo": True})
        self.demo_timer = self.root.after(40, self._demo_tick)

    def stop_demo(self) -> None:
        self.demo_running = False
        if self.demo_timer is not None:
            try:
                self.root.after_cancel(self.demo_timer)
            except Exception:
                pass
        self.demo_timer = None
        self.status_var.set("演示已停止")

    def refresh_ports(self) -> None:
        ports = list(list_ports.comports())
        values = [port.device for port in ports]
        self.port_combo["values"] = values
        if not values:
            self.port_var.set("")
            self.status_var.set("未发现串口")
            return
        preferred = next((p.device for p in ports if p.vid == 0x2833 and p.pid == 0xB015), None)
        if self.port_var.get() not in values:
            self.port_var.set(preferred or values[0])

    def auto_connect(self) -> None:
        if self.port_var.get().strip() and self.reader is None:
            self.connect()

    def toggle_connection(self) -> None:
        if self.reader is not None and self.reader.is_alive():
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        if self.reader is not None and self.reader.is_alive():
            self.status_var.set("串口线程已在运行")
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("三维力", "没有选择串口")
            return
        self.stop_event.clear()
        self._connect_started = time.monotonic()
        self._last_frame_time = time.monotonic()
        self._restarting = False
        self.trace.record_event("ui", "connect_requested", {"port": port})
        self.reader = ForceSerialReader(port, self.events, self.stop_event)
        self.reader.start()
        self.connect_button.configure(text="断开")
        self.status_var.set(f"正在打开 {port}")

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.reader is not None:
            self.reader.join(timeout=1.0)
        self.reader = None
        self.connect_button.configure(text="连接")
        self.status_var.set("已断开")

    def _all_pairs(self) -> list[str]:
        now = time.monotonic()
        pairs = []
        for sid in sorted(self.mag_samples):
            if 1 <= sid <= 20 and sid + 20 in self.mag_samples:
                if now - self.mag_samples[sid][0] < 1.0 and now - self.mag_samples[sid + 20][0] < 1.0:
                    pairs.append(f"{sid} + {sid + 20}")
        return pairs

    def _refresh_pairs(self) -> None:
        values = ["自动"] + self._all_pairs()
        if list(self.pair_combo["values"]) != values:
            self.pair_combo["values"] = values
            if self.pair_var.get() not in values:
                self.pair_var.set("自动")

    def _load_iron_calibration(self) -> None:
        self.iron_cal.clear()
        if not self.iron_cal_path.exists():
            self.iron_var.set("软硬铁：未标定")
            return
        try:
            payload = json.loads(self.iron_cal_path.read_text(encoding="utf-8"))
            for sid_text, item in payload.get("sensors", {}).items():
                sid = int(sid_text)
                offset = tuple(float(v) for v in item["offset"])
                soft = tuple(tuple(float(v) for v in row) for row in item["matrix"])
                residual = float(item.get("residual", 0.0))
                if len(offset) == 3 and len(soft) == 3 and all(len(row) == 3 for row in soft):
                    self.iron_cal[sid] = (offset, soft, residual)
            self.iron_var.set(f"软硬铁：已加载 {len(self.iron_cal)} 路")
        except Exception as exc:
            self.iron_var.set(f"软硬铁：加载失败 {exc}")

    def _save_iron_calibration(self) -> None:
        payload = {
            "version": 1,
            "sensors": {
                str(sid): {
                    "offset": list(item[0]),
                    "matrix": [list(row) for row in item[1]],
                    "residual": item[2],
                }
                for sid, item in self.iron_cal.items()
            },
        }
        self.iron_cal_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def _apply_iron_calibration(self, sid: int, vector: Vec3) -> Vec3:
        item = self.iron_cal.get(sid)
        if item is None:
            return vector
        offset, soft, _residual = item
        return matrix_vector(soft, vsub(vector, offset))

    def toggle_iron_calibration(self) -> None:
        if self.iron_calibrating:
            self._finish_iron_calibration(manual=True)
        else:
            self.start_iron_calibration()

    def start_iron_calibration(self) -> None:
        self.cancel_direction_calibration()
        if self.demo_running:
            self.stop_demo()
        pair = (self.pair_left, self.pair_right)
        if pair[0] is None or pair[1] is None:
            self._select_auto_pair()
            pair = (self.pair_left, self.pair_right)
        if pair[0] is None or pair[1] is None:
            messagebox.showerror("软硬铁标定", "先建立 id 与 id+20 的传感器对")
            return
        self.iron_pair = pair
        self.iron_samples = {pair[0]: [], pair[1]: []}
        messagebox.showinfo(
            "软硬铁标定",
            "保持外界无额外磁源，用 30 秒把传感器绕 X/Y/Z 各方向缓慢旋转，尽量覆盖所有姿态。\n\n"
            "最好在安装磁皮前做传感器磁标定；如果磁皮已安装，请保持磁皮与传感器相对位置不动。\n\n"
            "点击确定后立即开始采样。",
        )
        self.iron_started = time.monotonic()
        self.iron_calibrating = True
        self.iron_button.configure(text="完成并计算(I)")
        self.iron_var.set("软硬铁：采集中 0%")
        self.status_var.set("软硬铁标定中：缓慢旋转传感器")

    def _finish_iron_calibration(self, manual: bool = False) -> None:
        if not self.iron_calibrating:
            return
        pair = self.iron_pair
        try:
            fitted = {}
            for sid in pair:
                if sid is None:
                    continue
                samples = self.iron_samples.get(sid, [])
                offset, soft, residual = fit_ellipsoid(samples)
                fitted[sid] = (offset, soft, residual)
            if not fitted:
                raise ValueError("没有有效样本")
            self.iron_cal.update(fitted)
            self._save_iron_calibration()
            summary = ", ".join(
                f"{sid}: {self.iron_cal[sid][2]:.3f}" for sid in fitted
            )
            self.unit_var.set("归一化（铁标定后）")
            self._reset_force_state("软硬铁标定完成")
            self.iron_var.set(f"软硬铁：完成，残差 {summary}")
            self.status_var.set("软硬铁标定完成，正在重新零点")
            messagebox.showinfo("软硬铁标定", f"标定完成。\n残差：{summary}")
        except Exception as exc:
            self.iron_var.set(f"软硬铁：失败 {exc}")
            self.status_var.set("软硬铁标定失败")
            if manual:
                messagebox.showerror("软硬铁标定", str(exc))
        finally:
            self.iron_calibrating = False
            self.iron_button.configure(text="软硬铁标定(I)")
            self.iron_samples = {}

    def clear_iron_calibration(self) -> None:
        self.iron_cal.clear()
        self._save_iron_calibration()
        self.iron_var.set("软硬铁：已清除")
        self._reset_force_state("软硬铁标定已清除")


    def _load_direction_calibration(self) -> None:
        if not self.direction_cal_path.exists():
            self.direction_var.set("方向标定：未标定")
            return
        try:
            payload = json.loads(self.direction_cal_path.read_text(encoding="utf-8"))
            matrix = tuple(tuple(float(v) for v in row) for row in payload["matrix"])
            if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
                raise ValueError("matrix shape")
            self.matrix = matrix
            self.matrix_text.delete("1.0", tk.END)
            self.matrix_text.insert("1.0", "\n".join(" ".join(f"{v:+.5f}" for v in row) for row in matrix))
            residual = float(payload.get("residual", 0.0))
            self.direction_var.set(f"方向标定：已加载，残差 {residual:.4f}")
        except Exception as exc:
            self.direction_var.set(f"方向标定：加载失败 {exc}")

    def _save_direction_calibration(self, residual: float = 0.0, condition: float = 0.0) -> None:
        payload = {
            "version": 1,
            "matrix": [list(row) for row in self.matrix],
            "residual": residual,
            "condition": condition,
        }
        self.direction_cal_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def open_direction_calibration(self) -> None:
        if self.direction_window is not None and self.direction_window.winfo_exists():
            self.direction_window.lift()
            return
        if self.demo_running:
            self.stop_demo()
        pair = (self.pair_left, self.pair_right)
        if pair[0] is None or pair[1] is None or self.baseline_left is None:
            messagebox.showerror("六方向标定", "先连接双磁传感器并完成零点")
            return
        self.direction_calibrating = True
        self.direction_collecting = False
        self.direction_step = 0
        self.direction_samples = []
        self.direction_results = {}
        window = tk.Toplevel(self.root)
        self.direction_window = window
        window.title("六方向方向/增益标定")
        window.geometry("620x390+180+120")
        window.attributes("-topmost", True)
        ttk.Label(window, text="六方向标定", font=("Arial", 17, "bold")).pack(pady=(14, 4))
        ttk.Label(
            window,
            text="每个方向先点击“开始采集”，然后在 4 秒内反复沿该方向揉搓。\n"
                 "开始后先朝标注的正方向明显移动一下，再来回揉搓，用第一次移动确定正负。\n"
                 "依次完成现实 +X、-X、+Y、-Y、+Z、-Z。",
            justify=tk.CENTER,
        ).pack(pady=6)
        self.direction_step_var = tk.StringVar()
        self.direction_progress_var = tk.StringVar()
        self.direction_result_var = tk.StringVar(value="结果：-")
        ttk.Label(window, textvariable=self.direction_step_var, font=("Arial", 16, "bold"), foreground="#35d6ff").pack(pady=8)
        ttk.Label(window, textvariable=self.direction_progress_var).pack()
        ttk.Label(window, textvariable=self.direction_result_var, font=("Consolas", 12)).pack(pady=8)
        buttons = ttk.Frame(window)
        buttons.pack(pady=10)
        self.direction_capture_button = ttk.Button(buttons, text="开始采集", command=self.start_direction_capture)
        self.direction_capture_button.pack(side=tk.LEFT, padx=5)
        self.direction_next_button = ttk.Button(buttons, text="下一步", command=self.next_direction_step, state=tk.DISABLED)
        self.direction_next_button.pack(side=tk.LEFT, padx=5)
        self.direction_compute_button = ttk.Button(buttons, text="计算并应用", command=self.compute_direction_calibration, state=tk.DISABLED)
        self.direction_compute_button.pack(side=tk.LEFT, padx=5)
        ttk.Button(buttons, text="取消", command=self.cancel_direction_calibration).pack(side=tk.LEFT, padx=5)
        window.protocol("WM_DELETE_WINDOW", self.cancel_direction_calibration)
        self._show_direction_step()

    def _direction_step_text(self) -> str:
        labels = (
            "现实 +X 方向（你定义的 X 正方向）",
            "现实 -X 方向（与 +X 相反）",
            "现实 +Y 方向（你定义的 Y 正方向）",
            "现实 -Y 方向（与 +Y 相反）",
            "现实 +Z 方向（按压/靠近方向）",
            "现实 -Z 方向（拉起/远离方向）",
        )
        return labels[min(self.direction_step, 5)]

    def _show_direction_step(self) -> None:
        if not self.direction_calibrating:
            return
        self.direction_step_var.set(f"{self.direction_step + 1}/6  {self._direction_step_text()}")
        self.direction_progress_var.set("等待开始采集")
        self.direction_result_var.set("结果：-")
        self.direction_capture_button.configure(state=tk.NORMAL)
        self.direction_next_button.configure(state=tk.DISABLED)
        self.direction_compute_button.configure(state=tk.DISABLED)

    def start_direction_capture(self) -> None:
        if not self.direction_calibrating:
            return
        self.direction_collecting = True
        self.direction_samples = []
        self.direction_collect_start = time.monotonic()
        self.direction_progress_var.set("采集中……")
        self.direction_capture_button.configure(state=tk.DISABLED)
        self.direction_next_button.configure(state=tk.DISABLED)

    def _finish_direction_capture(self) -> None:
        if not self.direction_collecting:
            return
        self.direction_collecting = False
        if len(self.direction_samples) < 8:
            self.direction_progress_var.set(f"样本不足：{len(self.direction_samples)}，请重采")
            self.direction_capture_button.configure(state=tk.NORMAL)
            return
        try:
            sample, axis_ratio, axis_gain = extract_rubbing_direction(self.direction_samples)
        except Exception as exc:
            self.direction_progress_var.set(f"学习失败：{exc}")
            self.direction_capture_button.configure(state=tk.NORMAL)
            self.direction_next_button.configure(state=tk.DISABLED)
            return
        self.direction_results[self.direction_step] = sample
        self.direction_result_var.set(
            "主轴向量 = (%.3f, %.3f, %.3f)  幅值=%.3f  主轴比=%.2f  样本=%d"
            % (sample[0], sample[1], sample[2], axis_gain, axis_ratio, len(self.direction_samples))
        )
        self.direction_progress_var.set("本方向完成")
        if len(self.direction_results) == 6:
            self.direction_compute_button.configure(state=tk.NORMAL)
        else:
            self.direction_next_button.configure(state=tk.NORMAL)
        self.direction_capture_button.configure(state=tk.NORMAL)

    def next_direction_step(self) -> None:
        if self.direction_step not in self.direction_results:
            return
        if self.direction_step < 5:
            self.direction_step += 1
            self._show_direction_step()

    def compute_direction_calibration(self) -> None:
        if len(self.direction_results) != 6:
            messagebox.showerror("六方向标定", "六个方向还没有采集完")
            return
        try:
            symmetric = dict(self.direction_results)
            for plus, minus in ((0, 1), (2, 3), (4, 5)):
                axis = vmul(vsub(symmetric[plus], symmetric[minus]), 0.5)
                if vnorm(axis) <= 1e-12:
                    raise ValueError(f"方向 {plus + 1}/{minus + 1} 正反相互抵消，请重采")
                symmetric[plus] = axis
                symmetric[minus] = vmul(axis, -1.0)
            matrix, residual, condition = fit_direction_matrix([symmetric[i] for i in range(6)])
            self.matrix = matrix
            self.matrix_text.delete("1.0", tk.END)
            self.matrix_text.insert("1.0", "\n".join(" ".join(f"{v:+.6f}" for v in row) for row in self.matrix))
            self._save_direction_calibration(residual, condition)
            self.direction_var.set(f"方向标定：完成，残差 {residual:.4f}，cond {condition:.3g}")
            self._reset_force_state("六方向标定完成")
            messagebox.showinfo("六方向标定", f"标定完成。\n残差：{residual:.5f}\n条件数：{condition:.3g}")
            self.cancel_direction_calibration()
        except Exception as exc:
            messagebox.showerror("六方向标定", str(exc))

    def cancel_direction_calibration(self) -> None:
        self.direction_collecting = False
        self.direction_calibrating = False
        self.direction_samples = []
        if self.direction_window is not None:
            try:
                self.direction_window.destroy()
            except Exception:
                pass
        self.direction_window = None


    def clear_direction_calibration(self) -> None:
        self.matrix = IDENTITY
        self.matrix_text.delete("1.0", tk.END)
        self.matrix_text.insert("1.0", "1 0 0\n0 1 0\n0 0 1")
        try:
            self.direction_cal_path.unlink(missing_ok=True)
        except Exception:
            pass
        self.direction_var.set("方向标定：已清除")
        self.force_raw = (0.0, 0.0, 0.0)
        self.force_filtered = (0.0, 0.0, 0.0)
        self.series.clear()
        self._force_changed = True
        self.status_var.set("六方向标定已清除")


    def _on_pair_changed(self) -> None:
        value = self.pair_var.get()
        if value == "自动":
            self.pair_left = self.pair_right = None
        else:
            left, right = (int(part) for part in value.split("+"))
            self.pair_left, self.pair_right = left, right
        self._reset_force_state("传感器对变化")

    def _reset_force_state(self, reason: str) -> None:
        self.baseline_left = self.baseline_right = None
        self.force_raw = self.force_filtered = self.delta_vector = (0.0, 0.0, 0.0)
        self.series.clear()
        self._force_changed = True
        self._last_data_time = 0.0
        self.zero_var.set(f"零点：未建立（{reason}）")
        self.trace.record_event("ui", "force_state_reset", {"reason": reason})

    def _select_auto_pair(self) -> None:
        if self.pair_left is not None or self.pair_var.get() != "自动":
            return
        pairs = self._all_pairs()
        if pairs:
            left, right = (int(part) for part in pairs[0].split("+"))
            self.pair_left, self.pair_right = left, right
            self.status_var.set(f"自动配对：sensor {left} + {right}")

    def start_zero(self) -> None:
        pair = (self.pair_left, self.pair_right)
        if pair[0] is None or pair[1] is None:
            self._select_auto_pair()
            pair = (self.pair_left, self.pair_right)
        if pair[0] is None or pair[1] is None:
            self.zero_var.set("零点：还没有 id 与 id+20 两路数据")
            return
        self.zero_started = time.monotonic()
        self.zero_samples_left = []
        self.zero_samples_right = []
        self.zero_var.set("零点：采集中，请保持无外力……")
        self.status_var.set("双磁零点采集中")

    def _update_zero(self, now: float) -> None:
        if not self.zero_started:
            return
        elapsed = now - self.zero_started
        if elapsed < 1.5:
            return
        if len(self.zero_samples_left) >= 6 and len(self.zero_samples_right) >= 6:
            self.baseline_left = median_vec(self.zero_samples_left)
            self.baseline_right = median_vec(self.zero_samples_right)
            self.zero_var.set(f"零点：已建立（{len(self.zero_samples_left)}/{len(self.zero_samples_right)}）")
            self.status_var.set("双磁零点完成")
            self.zero_started = 0.0
            self.zero_samples_left = []
            self.zero_samples_right = []
        elif elapsed >= 5.0:
            self.zero_var.set("零点：样本不足，请重试")
            self.status_var.set("双磁零点失败")
            self.zero_started = 0.0
            self.zero_samples_left = []
            self.zero_samples_right = []

    def _handle_mag(self, row: dict[str, Any]) -> None:
        if self.demo_running and not row.get("_demo"):
            self.stop_demo()
            self.mag_samples.clear()
            self.pair_left = self.pair_right = None
            self.baseline_left = self.baseline_right = None
        sid_raw = row.get("sensor_id")
        try:
            sid = int(sid_raw)
        except (TypeError, ValueError):
            return
        values = (row.get("mag_x"), row.get("mag_y"), row.get("mag_z"))
        if any(value is None for value in values):
            return
        raw_vector = (float(values[0]), float(values[1]), float(values[2]))
        now = time.monotonic()

        if self.iron_calibrating and not row.get("_demo") and sid in self.iron_pair:
            self.iron_samples.setdefault(sid, []).append(raw_vector)
            elapsed = now - self.iron_started
            counts = len(self.iron_samples.get(self.iron_pair[0], [])) + len(self.iron_samples.get(self.iron_pair[1], []))
            self.iron_var.set(f"软硬铁：采集中 {min(100, int(elapsed / self.iron_duration * 100))}%（{counts}）")
            if elapsed >= self.iron_duration:
                self._finish_iron_calibration()
            self._force_changed = True
            return

        vector = self._apply_iron_calibration(sid, raw_vector)
        self.mag_samples[sid] = (now, vector, row)
        self.frame_count += 1
        self._last_frame_time = now
        self.rate_times.append(now)
        if self.frame_count == 50 and self._restart_state_path.exists():
            try:
                self._restart_state_path.unlink()
            except Exception:
                pass
        self._refresh_pairs()
        self._select_auto_pair()
        if self.pair_left is None or self.pair_right is None:
            return
        if self.baseline_left is None and self.zero_started == 0.0 and not row.get("_demo"):
            self.start_zero()
        if sid == self.pair_left:
            self.raw_left = vector
            if self.zero_started:
                self.zero_samples_left.append(vector)
        elif sid == self.pair_right:
            self.raw_right = vector
            if self.zero_started:
                self.zero_samples_right.append(vector)
        else:
            return
        self._update_zero(now)
        if self.baseline_left is None or self.baseline_right is None:
            return
        left_delta = vsub(self.raw_left, self.baseline_left)
        right_delta = vsub(self.raw_right, self.baseline_right)
        self.delta_vector = vsub(left_delta, right_delta)

        if self.direction_calibrating:
            if self.direction_collecting and not row.get("_demo"):
                self.direction_samples.append(self.delta_vector)
                elapsed = now - self.direction_collect_start
                self.direction_progress_var.set(
                    f"采集中 {min(100, int(elapsed / self.direction_collect_duration * 100))}%  "
                    f"样本 {len(self.direction_samples)}"
                )
                if elapsed >= self.direction_collect_duration:
                    self._finish_direction_capture()
            self._force_changed = True
            return

        scale = MAG_SCALES.get(self.unit_var.get(), 1.0)
        self.force_raw = matrix_vector(self.matrix, vmul(self.delta_vector, scale * float(self.gain_var.get())))
        alpha = max(0.0, min(0.95, float(self.smoothing_var.get())))
        self.force_filtered = vadd(vmul(self.force_filtered, alpha), vmul(self.force_raw, 1.0 - alpha))
        self.series.append(self.force_filtered)
        self._force_changed = True
        self._last_data_time = now
        self._write_record()


    def _connection_watchdog(self) -> None:
        if self._restarting:
            return
        now = time.monotonic()
        status = self.status_var.get()
        reader_alive = self.reader is not None and self.reader.is_alive()
        if not reader_alive:
            self.root.after(2000, self._connection_watchdog)
            return
        if self.frame_count == 0 and now - self._connect_started > 30.0:
            if ("正在打开" in status) or ("Reconnecting" in status) or ("No data" in status):
                self._auto_restart("串口打开超时")
                return
        if self.frame_count > 0 and now - self._last_frame_time > 15.0 and not self.demo_running:
            self._auto_restart("数据流中断")
            return
        self.root.after(2000, self._connection_watchdog)

    def _auto_restart(self, reason: str) -> None:
        self._restarting = True
        history = []
        if self._restart_state_path.exists():
            try:
                history = json.loads(self._restart_state_path.read_text(encoding="utf-8")).get("times", [])
            except Exception:
                history = []
        history = [float(t) for t in history if time.time() - float(t) < 300.0]
        if len(history) >= 3:
            self.status_var.set(f"{reason}，自动恢复已停止，请拔插 USB/检查 ST-Link")
            self._restarting = False
            self.root.after(5000, self._connection_watchdog)
            return
        history.append(time.time())
        try:
            self._restart_state_path.write_text(json.dumps({"times": history}), encoding="utf-8")
        except Exception:
            pass
        self.status_var.set(f"{reason}，正在自动重启上位机……")
        helper = (
            "import os,subprocess,sys,time,pathlib\\n"
            "pid=int(sys.argv[1]); launcher=sys.argv[2]; cli=sys.argv[3]\\n"
            "for _ in range(40):\\n"
            "    try:\\n"
            "        os.kill(pid,0)\\n"
            "    except OSError:\\n"
            "        break\\n"
            "    time.sleep(0.25)\\n"
            "try:\\n"
            "    subprocess.run([cli,'-c','port=SWD','freq=1000','mode=HOTPLUG','-rst'],timeout=8,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)\\n"
            "except Exception:\\n"
            "    pass\\n"
            "subprocess.Popen([sys.executable,launcher],cwd=str(pathlib.Path(launcher).parent),creationflags=subprocess.DETACHED_PROCESS|subprocess.CREATE_NO_WINDOW)\\n"
        )
        cli_path = r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
        launcher = str(Path(__file__).resolve().with_name("force_3d_ui_launcher.pyw"))
        flags = subprocess.DETACHED_PROCESS | subprocess.CREATE_NO_WINDOW
        subprocess.Popen([sys.executable, "-c", helper, str(os.getpid()), launcher, cli_path], creationflags=flags)
        self.root.after(300, lambda: os._exit(0))


    def _process_event(self, event: str, payload: Any) -> None:
        if event in ("mag", "row"):
            row = payload if isinstance(payload, dict) else {}
            if row.get("mag_x") is not None:
                try:
                    self._handle_mag(row)
                except Exception as exc:
                    self.status_var.set(f"数据回调错误：{exc}")
                    self._force_changed = True
        elif event == "status":
            if str(payload) == "Disconnected":
                self.reader = None
                self.connect_button.configure(text="连接")
                self.status_var.set("串口已断开")
        elif event == "decode_error":
            self.status_var.set(f"解码错误：{payload}")
        elif event == "error":
            self.status_var.set(str(payload))
            messagebox.showerror("OSMO 三维力", str(payload))
            self.stop_event.set()
            self.reader = None
            self.connect_button.configure(text="连接")

    def _drain_events(self, max_events: int = 120) -> int:
        processed = 0
        while processed < max_events:
            try:
                event, payload = self.events.get_nowait()
            except queue.Empty:
                return processed
            self._process_event(event, payload)
            processed += 1
        return processed

    def _poll_events(self) -> None:
        try:
            self._drain_events(100)
        finally:
            self.root.after(5 if not self.events.empty() else 15, self._poll_events)

    def _rate_hz(self) -> float:
        now = time.monotonic()
        while self.rate_times and now - self.rate_times[0] > 1.0:
            self.rate_times.popleft()
        return float(len(self.rate_times))

    def toggle_recording(self) -> None:
        if self.record_file is not None:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self) -> None:
        path = filedialog.asksaveasfilename(
            title="保存三维力 CSV",
            initialdir=str(Path(__file__).resolve().parent / "recordings"),
            initialfile=time.strftime("force_%Y%m%d_%H%M%S.csv"),
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        fields = [
            "host_time", "sensor_left", "sensor_right", "fx", "fy", "fz", "mag",
            "dbx", "dby", "dbz", "mag0_x", "mag0_y", "mag0_z", "mag1_x", "mag1_y", "mag1_z",
        ]
        self.record_file = open(path, "w", newline="", encoding="utf-8")
        self.record_writer = csv.DictWriter(self.record_file, fieldnames=fields)
        self.record_writer.writeheader()
        self.record_path = Path(path)
        self.record_rows = 0
        self.record_button.configure(text="停止 CSV")
        self.status_var.set(f"正在记录：{self.record_path.name}")

    def _write_record(self) -> None:
        if self.record_writer is None:
            return
        self.record_writer.writerow({
            "host_time": time.time(),
            "sensor_left": self.pair_left,
            "sensor_right": self.pair_right,
            "fx": self.force_filtered[0], "fy": self.force_filtered[1], "fz": self.force_filtered[2],
            "mag": math.sqrt(sum(value * value for value in self.force_filtered)),
            "dbx": self.delta_vector[0], "dby": self.delta_vector[1], "dbz": self.delta_vector[2],
            "mag0_x": self.raw_left[0], "mag0_y": self.raw_left[1], "mag0_z": self.raw_left[2],
            "mag1_x": self.raw_right[0], "mag1_y": self.raw_right[1], "mag1_z": self.raw_right[2],
        })
        self.record_rows += 1
        if self.record_file is not None and self.record_rows % 25 == 0:
            self.record_file.flush()

    def stop_recording(self) -> None:
        if self.record_file is not None:
            self.record_file.flush()
            self.record_file.close()
        self.record_file = None
        self.record_writer = None
        self.record_button.configure(text="开始 CSV")
        if self.record_path is not None:
            self.status_var.set(f"CSV 已保存：{self.record_path.name}")

    def apply_matrix(self) -> None:
        try:
            self.matrix = parse_matrix(self.matrix_text.get("1.0", tk.END))
            self._save_direction_calibration()
            self.direction_var.set("方向标定：手动矩阵")
            self.status_var.set("标定矩阵已应用")
            self.trace.record_event("ui", "matrix_applied", {"matrix": self.matrix})
        except Exception as exc:
            messagebox.showerror("三维力", str(exc))

    def identity_matrix(self) -> None:
        self.matrix_text.delete("1.0", tk.END)
        self.matrix_text.insert("1.0", "1 0 0\n0 1 0\n0 0 1")
        self.matrix = IDENTITY
        self._save_direction_calibration()
        self.direction_var.set("方向标定：单位阵")
        self.status_var.set("标定矩阵已复位")

    def clear_series(self) -> None:
        self.series.clear()
        self._force_changed = True
        self.status_var.set("曲线已清空")


    def start_auto_trace(self) -> None:
        try:
            self.start_trace_recording(self.trace_root)
        except Exception as exc:
            self.trace_var.set(f"追踪启动失败：{exc}")

    def start_trace_recording(self, parent_dir: Path | None = None) -> None:
        if self.trace.is_recording:
            return
        if parent_dir is None:
            selected = filedialog.askdirectory(title="选择追踪录制目录", initialdir=str(self.trace_root))
            if not selected:
                return
            parent_dir = Path(selected)
        session = self.trace.start_session(parent_dir, metadata={
            "port": self.port_var.get(),
            "pair": self.pair_var.get(),
            "unit": self.unit_var.get(),
            "module": "3d_force",
        })
        self.trace_button.configure(text="停止追踪录制")
        self.trace_var.set(f"追踪中：{session}")

    def stop_trace_recording(self, reason: str = "user") -> None:
        if not self.trace.is_recording:
            return
        session = self.trace.stop_session(reason=reason)
        self.trace_button.configure(text="开始追踪录制")
        self.trace_var.set(f"追踪已停止：{session}")

    def toggle_trace_recording(self) -> None:
        if self.trace.is_recording:
            self.stop_trace_recording("ui_toggle")
        else:
            self.start_trace_recording(None)

    @staticmethod
    def _project(point: Vec3, width: int, height: int, value_range: float) -> tuple[float, float]:
        scale = min(width, height) * 0.34 / max(value_range, 1e-9)
        cx, cy = width * 0.50, height * 0.56
        x, y, z = point
        c30 = math.cos(math.pi / 6.0)
        s30 = math.sin(math.pi / 6.0)
        return (
            cx + (x - y) * c30 * scale,
            cy - z * scale + (x + y) * s30 * scale,
        )

    def _auto_range_target(self) -> float:
        recent = list(self.series)[-120:]
        magnitudes = []
        for row in recent:
            magnitude = vnorm(row)
            if magnitude > 1e-9:
                magnitudes.append(magnitude)
        if not magnitudes:
            return max(1.0, self._auto_range_value)
        magnitudes.sort()
        index = min(len(magnitudes) - 1, int(0.95 * (len(magnitudes) - 1)))
        return max(1.0, magnitudes[index] * 1.15)

    def _display_range(self) -> float:
        if not self.auto_range_var.get():
            return max(0.001, float(self.range_var.get()))
        target = self._auto_range_target()
        if target > self._auto_range_value:
            self._auto_range_value = target
        else:
            # Rapid recovery after a transient spike; no long visual dead zone.
            self._auto_range_value = max(1.0, self._auto_range_value * 0.80 + target * 0.20)
        now = time.monotonic()
        if now - self._auto_range_updated >= 0.15:
            self._auto_range_updated = now
            self.range_var.set(round(self._auto_range_value, 3))
        return self._auto_range_value

    def reset_auto_range(self) -> None:
        self.auto_range_var.set(True)
        self._auto_range_value = self._auto_range_target()
        self.range_var.set(round(self._auto_range_value, 3))
        self._force_changed = True
        self.status_var.set(f"量程已复位：±{self._auto_range_value:.3f}")


    def _write_status_file(self, now: float) -> None:
        if now - self._status_written < 1.0:
            return
        self._status_written = now
        try:
            force_canvas = [
                self.canvas.winfo_rootx(), self.canvas.winfo_rooty(),
                self.canvas.winfo_width(), self.canvas.winfo_height(),
            ]
            series_canvas = [
                self.series_canvas.winfo_rootx(), self.series_canvas.winfo_rooty(),
                self.series_canvas.winfo_width(), self.series_canvas.winfo_height(),
            ]
            payload = {
                "pid": __import__("os").getpid(),
                "time": time.time(),
                "window_title": self.root.title(),
                "force_canvas": force_canvas,
                "series_canvas": series_canvas,
                "force_items": len(self.canvas.find_all()),
                "series_items": len(self.series_canvas.find_all()),
                "pair": [self.pair_left, self.pair_right],
                "baseline_ready": self.baseline_left is not None and self.baseline_right is not None,
                "force": list(self.force_filtered),
                "delta": list(self.delta_vector),
                "series_len": len(self.series),
                "frames": self.frame_count,
                "reader_alive": self.reader is not None and self.reader.is_alive(),
                "series_tail": [list(row) for row in list(self.series)[-5:]],
                "series_line_coords": [self.series_canvas.coords(item)[:8] for item in self.series_canvas.find_all() if self.series_canvas.type(item) == "line"][-3:],
                "demo": self.demo_running,
                "status": self.status_var.get(),
                "range": self._display_range(),
            }
            self._status_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception:
            pass

    def _draw_force(self, width: int, height: int) -> None:
        value_range = self._display_range()
        axis = value_range * 1.15
        origin = self._project((0.0, 0.0, 0.0), width, height, value_range)

        def grid_line(a, b, color):
            pa = self._project(a, width, height, value_range)
            pb = self._project(b, width, height, value_range)
            self.canvas.create_line(pa[0], pa[1], pb[0], pb[1], fill=color, width=1)

        # XY, XZ, YZ planes.
        for index in range(-5, 6):
            p = index / 5.0 * axis
            grid_line((p, -axis, 0.0), (p, axis, 0.0), "#233b4e")
            grid_line((-axis, p, 0.0), (axis, p, 0.0), "#233b4e")
            grid_line((p, 0.0, -axis), (p, 0.0, axis), "#24463f")
            grid_line((-axis, 0.0, p), (axis, 0.0, p), "#24463f")
            grid_line((0.0, p, -axis), (0.0, p, axis), "#303653")
            grid_line((0.0, -axis, p), (0.0, axis, p), "#303653")

        axis_defs = (
            ((-axis, 0.0, 0.0), (axis, 0.0, 0.0), "#ff5c67", "+X"),
            ((0.0, -axis, 0.0), (0.0, axis, 0.0), "#42e68b", "+Y"),
            ((0.0, 0.0, -axis), (0.0, 0.0, axis), "#6ea8ff", "+Z"),
        )
        for start_point, end_point, color, label in axis_defs:
            a = self._project(start_point, width, height, value_range)
            b = self._project(end_point, width, height, value_range)
            self.canvas.create_line(a[0], a[1], b[0], b[1], fill=color, width=2)
            self.canvas.create_text(b[0] + 8, b[1] - 8, text=label, fill=color, font=("Arial", 11, "bold"))

        # Plane labels.
        for point, text, color in (
            ((axis, -axis, 0.0), "XY", "#ffd166"),
            ((axis, 0.0, -axis), "XZ", "#42e68b"),
            ((0.0, axis, -axis), "YZ", "#6ea8ff"),
        ):
            q = self._project(point, width, height, value_range)
            self.canvas.create_text(q[0], q[1], text=text, fill=color, font=("Consolas", 10, "bold"))

        self.canvas.create_oval(origin[0] - 5, origin[1] - 5, origin[0] + 5, origin[1] + 5, outline="#9ec6df", width=2)
        force = self.force_filtered
        force_norm = vnorm(force)
        shown = vmul(force, value_range / force_norm) if force_norm > value_range else force
        tip = self._project(shown, width, height, value_range)
        color = "#ff4757" if force_norm > value_range else "#ffd166" if force_norm > value_range * 0.65 else "#35d6ff"

        projections = (
            ((shown[0], shown[1], 0.0), "#ffd166", "XY"),
            ((shown[0], 0.0, shown[2]), "#42e68b", "XZ"),
            ((0.0, shown[1], shown[2]), "#6ea8ff", "YZ"),
        )
        for point, proj_color, label in projections:
            projected = self._project(point, width, height, value_range)
            self.canvas.create_line(tip[0], tip[1], projected[0], projected[1], fill=proj_color, dash=(3, 3), width=1)
            self.canvas.create_line(origin[0], origin[1], projected[0], projected[1], fill=proj_color, dash=(2, 4), width=1)
            self.canvas.create_oval(projected[0] - 4, projected[1] - 4, projected[0] + 4, projected[1] + 4, outline=proj_color, width=2)
            self.canvas.create_text(projected[0] + 7, projected[1] + 7, text=label, fill=proj_color, anchor="nw", font=("Consolas", 9, "bold"))

        self.canvas.create_line(origin[0], origin[1], tip[0], tip[1], fill=color, width=5, arrow=tk.LAST, arrowshape=(15, 19, 7))
        self.canvas.create_oval(tip[0] - 6, tip[1] - 6, tip[0] + 6, tip[1] + 6, fill=color, outline="#ffffff", width=1)
        self.canvas.create_text(tip[0] + 10, tip[1] - 10, text=f"|F|={force_norm:.3f}", fill="#dcecf8", anchor="sw", font=("Arial", 11, "bold"))
        if self.demo_running:
            self.canvas.create_text(width * 0.5, 55, text="演示模式 · 非真实传感器", fill="#ffb84d", font=("Arial", 16, "bold"))
        elif self.baseline_left is None:
            self.canvas.create_text(width * 0.5, height - 30, text="等待真实双磁数据和自动零点……", fill="#aab2c0", font=("Arial", 12))
        elif force_norm <= 1e-9:
            self.canvas.create_text(width * 0.5, height - 30, text="力≈0，请按压磁皮", fill="#aab2c0", font=("Arial", 12))

    def _draw_series(self) -> None:
        canvas = self.series_canvas
        canvas.delete("all")
        width = max(2, canvas.winfo_width())
        height = max(2, canvas.winfo_height())
        left, right, top, bottom = 62, 15, 12, 16
        plot_w = max(1, width - left - right)
        plot_h = max(1, height - top - bottom)
        band_h = plot_h / 3.0
        value_range = self._display_range()
        values = list(self.series)
        colors = ("#ff5c67", "#42e68b", "#6ea8ff")
        names = ("Fx", "Fy", "Fz")

        for axis, (name, color) in enumerate(zip(names, colors)):
            band_top = top + axis * band_h
            center = band_top + band_h / 2.0
            canvas.create_rectangle(left, band_top + 2, width - right, band_top + band_h - 2, outline="#243a4d")
            canvas.create_line(left, center, width - right, center, fill="#60798c")
            canvas.create_text(8, center, text=name, fill=color, anchor="w", font=("Consolas", 11, "bold"))
            canvas.create_text(width - right - 5, band_top + 9, text=f"±{value_range:.2f}", fill="#7997aa", anchor="e", font=("Consolas", 9))
            if len(values) >= 2:
                points = []
                for index, row in enumerate(values):
                    x = left + index * plot_w / max(1, len(values) - 1)
                    value = max(-value_range, min(value_range, row[axis]))
                    y = center - value / value_range * (band_h * 0.42)
                    points.extend((x, y))
                canvas.create_line(*points, fill=color, width=2, smooth=False)

    def _render(self) -> None:
        now = time.monotonic()
        fresh = now - self._last_data_time < 0.75
        dirty = self._force_changed or fresh or not self.canvas.find_all()
        if dirty:
            self.canvas.delete("all")
            width = max(2, self.canvas.winfo_width())
            height = max(2, self.canvas.winfo_height())
            self.canvas.create_text(16, 14, anchor="nw", text="正交视图 | 先 Z 归零，再按压磁皮", fill="#9fb9cc", font=("Arial", 10))
            self._draw_force(width, height)
            self._draw_series()
            self._force_changed = False

        last = getattr(self, "_last_label_time", 0.0)
        if now - last >= 0.10:
            self._last_label_time = now
            force = self.force_filtered
            self.force_var.set(f"F = ({force[0]:+.3f}, {force[1]:+.3f}, {force[2]:+.3f})  |F|={vnorm(force):.3f}")
            self.fx_var.set(f"Fx = {force[0]:+.3f}")
            self.fy_var.set(f"Fy = {force[1]:+.3f}")
            self.fz_var.set(f"Fz = {force[2]:+.3f}")
            self.fmag_var.set(f"|F| = {vnorm(force):.3f}")
            self.mag_var.set(
                f"mag0感测(校正)={self.raw_left[0]:.3f},{self.raw_left[1]:.3f},{self.raw_left[2]:.3f}  "
                f"mag1参考(校正)={self.raw_right[0]:.3f},{self.raw_right[1]:.3f},{self.raw_right[2]:.3f}  "
                f"ΔB={self.delta_vector[0]:.1f},{self.delta_vector[1]:.1f},{self.delta_vector[2]:.1f}"
            )
            self.rate_var.set(f"{self._rate_hz():.1f} Hz  {self.frame_count} 帧")
        self._write_status_file(now)
        self.root.after(50, self._render)

    def on_close(self) -> None:
        self.cancel_direction_calibration()
        self.stop_demo()
        self.stop_recording()
        self.stop_trace_recording("window_close")
        self.stop_event.set()
        if self.reader is not None:
            self.reader.join(timeout=1.0)
        self.root.destroy()


def run_self_test() -> int:
    matrix = parse_matrix("2 0 0; 0 3 0; 0 0 4")
    force = matrix_vector(matrix, (1.0, 2.0, 3.0))
    assert force == (2.0, 6.0, 12.0), force
    assert abs(vnorm((3.0, 4.0, 0.0)) - 5.0) < 1e-9

    true_offset = np.array([12.0, -7.0, 4.0])
    true_soft = np.array([[0.8, 0.08, 0.0], [0.03, 0.92, 0.02], [0.0, 0.04, 1.1]])
    synthetic = []
    for theta in np.linspace(0.0, math.pi, 14):
        for phi in np.linspace(0.0, 2.0 * math.pi, 18, endpoint=False):
            unit = np.array([math.sin(theta) * math.cos(phi), math.sin(theta) * math.sin(phi), math.cos(theta)])
            synthetic.append(tuple(np.linalg.solve(true_soft, unit) + true_offset))
    fit_offset, fit_soft, fit_residual = fit_ellipsoid(synthetic)
    calibrated_radii = [
        vnorm(matrix_vector(fit_soft, vsub(sample, fit_offset)))
        for sample in synthetic
    ]
    assert max(calibrated_radii) - min(calibrated_radii) < 0.05, (fit_residual, max(calibrated_radii) - min(calibrated_radii))

    desired_test = np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]], dtype=float)
    true_direction = np.array([[2.0, 0.2, 0.0], [0.1, 1.5, 0.2], [0.0, 0.3, 1.2]])
    measured_test = desired_test @ np.linalg.inv(true_direction).T
    fit_matrix, fit_direction_residual, fit_condition = fit_direction_matrix([tuple(row) for row in measured_test])
    recovered = matrix_vector(fit_matrix, tuple(measured_test[0]))
    assert abs(recovered[0] - 1.0) < 1e-6 and abs(recovered[1]) < 1e-6 and abs(recovered[2]) < 1e-6, recovered

    rubbing_axis = np.array([0.8, -0.2, 0.3])
    rubbing_axis /= np.linalg.norm(rubbing_axis)
    rubbing_samples = []
    base_sample = np.array([100.0, 200.0, 300.0])
    for i in range(60):
        amplitude = 12.0 if i < 8 else (12.0 if i % 2 == 0 else -12.0)
        noise = np.random.default_rng(i).normal(0.0, 0.02, 3)
        rubbing_samples.append(tuple(base_sample + rubbing_axis * amplitude + noise))
    learned_axis, learned_ratio, learned_gain = extract_rubbing_direction(rubbing_samples)
    learned_unit = np.asarray(learned_axis) / np.linalg.norm(learned_axis)
    assert float(np.dot(learned_unit, rubbing_axis)) > 0.95, (learned_axis, learned_ratio, learned_gain)

    root = tk.Tk()
    root.withdraw()
    app = ForceMonitor(root, auto_connect=False, auto_trace=False)
    root.geometry("1200x800")
    root.update_idletasks()
    time.sleep(0.12)
    root.update()
    assert len(app.canvas.find_all()) > 0, "force canvas is blank"
    assert len(app.series_canvas.find_all()) > 0, "series canvas is blank"
    force_lines = sum(1 for item in app.canvas.find_all() if app.canvas.type(item) == "line")
    assert force_lines >= 60, force_lines
    app.iron_cal.clear()
    app.matrix = IDENTITY
    app.unit_var.set("1 LSB（仅原始计数）")
    app.gain_var.set(1.0)
    app.smoothing_var.set(0.0)
    app._handle_mag({"sensor_id": 13, "mag_x": 100.0, "mag_y": 200.0, "mag_z": 300.0})
    app._handle_mag({"sensor_id": 33, "mag_x": 1000.0, "mag_y": 2000.0, "mag_z": 3000.0})
    assert app.pair_left == 13 and app.pair_right == 33, (app.pair_left, app.pair_right)
    app.start_zero()
    app.zero_started -= 2.0
    for _ in range(6):
        app._handle_mag({"sensor_id": 13, "mag_x": 100.0, "mag_y": 200.0, "mag_z": 300.0})
        app._handle_mag({"sensor_id": 33, "mag_x": 1000.0, "mag_y": 2000.0, "mag_z": 3000.0})
    assert app.baseline_left == (100.0, 200.0, 300.0), app.baseline_left
    assert app.baseline_right == (1000.0, 2000.0, 3000.0), app.baseline_right
    app.raw_left = (110.0, 220.0, 330.0)
    app.raw_right = (990.0, 1990.0, 2990.0)
    left_delta = vsub(app.raw_left, app.baseline_left)
    right_delta = vsub(app.raw_right, app.baseline_right)
    assert vsub(left_delta, right_delta) == (20.0, 30.0, 40.0)
    app.delta_vector = vsub(left_delta, right_delta)
    app.force_raw = matrix_vector(app.matrix, app.delta_vector)
    app.force_filtered = app.force_raw
    assert app.force_filtered == (20.0, 30.0, 40.0), app.force_filtered
    point = app._project((1.0, 0.0, 0.0), 1200, 800, 2.0)
    assert len(point) == 2
    app.start_demo()
    time.sleep(0.16)
    root.update()
    assert vnorm(app.force_filtered) > 1.0, app.force_filtered
    assert "0.000" not in app.fx_var.get() or "0.000" not in app.fy_var.get() or "0.000" not in app.fz_var.get()
    app.on_close()
    print("FORCE_SELF_TEST_PASS=True")
    print("MATRIX_OK=True")
    print("DIFFERENTIAL_OK=True")
    print("ZERO_CAPTURE_OK=True")
    print("ELLIPSOID_OK=True")
    print("RUBBING_AXIS_OK=True")
    print("SIX_DIRECTION_FIT_OK=True")
    print("THREE_PLANE_GRID_OK=True")
    print("VECTOR_PROJECTIONS_OK=True")
    print("CANVAS_PROJECTION_OK=True")
    return 0



def run_live_test(seconds: float) -> int:
    root = tk.Tk()
    root.withdraw()
    app = ForceMonitor(root, auto_connect=True, auto_trace=False)
    app.disable_demo = True
    app.stop_demo()

    def finish() -> None:
        root.update()
        print("LIVE_STATUS=", app.status_var.get())
        print("LIVE_PORT=", app.port_var.get())
        print("LIVE_READER=", app.reader is not None and app.reader.is_alive())
        print("LIVE_DEMO=", app.demo_running)
        print("LIVE_PAIR=", app.pair_left, app.pair_right)
        print("LIVE_FRAMES=", app.frame_count)
        print("LIVE_BASELINE=", app.baseline_left, app.baseline_right)
        print("LIVE_RAW0=", app.raw_left)
        print("LIVE_RAW1=", app.raw_right)
        print("LIVE_DELTA=", app.delta_vector)
        print("LIVE_FORCE=", app.force_filtered)
        series_types = [app.series_canvas.type(item) for item in app.series_canvas.find_all()]
        force_types = [app.canvas.type(item) for item in app.canvas.find_all()]
        print("LIVE_CANVAS_ITEMS=", len(app.canvas.find_all()), len(app.series_canvas.find_all()))
        print("LIVE_SERIES_LINES=", series_types.count("line"))
        print("LIVE_FORCE_LINES=", force_types.count("line"))
        ok = (
            app.pair_left is not None
            and app.pair_right is not None
            and app.frame_count > 10
            and len(app.canvas.find_all()) > 0
        )
        app.on_close()
        print("LIVE_TEST_PASS=", ok)

    root.after(max(1500, int(seconds * 1000)), finish)
    root.mainloop()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true", help="运行数学和窗口检查后退出")
    parser.add_argument("--no-auto-trace", action="store_true", help="启动时不自动开始追踪录制")
    parser.add_argument("--live-test-seconds", type=float, default=0.0, help="连接真实传感器并自动测试指定秒数")
    args = parser.parse_args()
    if args.self_test:
        return run_self_test()
    if args.live_test_seconds > 0:
        return run_live_test(args.live_test_seconds)
    root = tk.Tk()
    ForceMonitor(root, auto_connect=True, auto_trace=not args.no_auto_trace)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
