#!/usr/bin/env python3
"""OSMO attitude preview with a quaternion-driven cube.

Host-side only: decodes the existing COBS/protobuf firmware stream.
Press Space to calibrate the current pose as the cube's zero/reference pose.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import deque
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Any

import serial
from serial.tools import list_ports

import read_osmo_glove as glove
import replay_protocol as rp
from firmware_profile import latest_known_firmware
from runtime_metrics import collect_runtime_metrics
from serial_trace_reader import TracedSerialReader as SerialReader
from trace_recorder import TraceRecorder

Quat = tuple[float, float, float, float]


def q_normalize(q: Quat) -> Quat:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n <= 1e-12:
        return 1.0, 0.0, 0.0, 0.0
    return w / n, x / n, y / n, z / n


def q_conjugate(q: Quat) -> Quat:
    w, x, y, z = q
    return w, -x, -y, -z


def q_multiply(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return q_normalize((
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ))


def q_rotate(q: Quat, v: tuple[float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = q_normalize(q)
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def q_yaw_deg(q: Quat) -> float:
    w, x, y, z = q_normalize(q)
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def q_from_euler_deg(rx: float, ry: float, rz: float) -> Quat:
    cx, sx = math.cos(math.radians(rx) / 2.0), math.sin(math.radians(rx) / 2.0)
    cy, sy = math.cos(math.radians(ry) / 2.0), math.sin(math.radians(ry) / 2.0)
    cz, sz = math.cos(math.radians(rz) / 2.0), math.sin(math.radians(rz) / 2.0)
    return q_multiply(q_multiply((cx, sx, 0.0, 0.0), (cy, 0.0, sy, 0.0)), (cz, 0.0, 0.0, sz))


IDENTITY: Quat = (1.0, 0.0, 0.0, 0.0)



class PoseMonitor:
    VERTICES = [
        (-1.0, -1.0, -1.0), (1.0, -1.0, -1.0),
        (1.0, 1.0, -1.0), (-1.0, 1.0, -1.0),
        (-1.0, -1.0, 1.0), (1.0, -1.0, 1.0),
        (1.0, 1.0, 1.0), (-1.0, 1.0, 1.0),
    ]
    FACES = [
        ((0, 1, 2, 3), "#3979b8"),
        ((4, 7, 6, 5), "#4d9bd6"),
        ((0, 4, 5, 1), "#39a66f"),
        ((3, 2, 6, 7), "#55c98b"),
        ((0, 3, 7, 4), "#c65d5d"),
        ((1, 5, 6, 2), "#e27b72"),
    ]
    MAG_GATE_BITS = {
        0x01: "calibration_missing",
        0x02: "reference_dip_missing",
        0x04: "norm_gate",
        0x08: "dip_gate",
        0x10: "step_gate",
        0x20: "gradient_gate",
        0x40: "drift_or_diag_hold_gate",
        0x80: "hard_event_or_candidate_gate",
    }
    MAG_STATUS_NAMES = {
        0: "NO_CAL", 1: "CAL", 2: "ACQ", 3: "LOCKED", 4: "DEGRADED", 5: "COASTING",
    }
    MAG_FEATURE_INFO = {
        0x01: ("标定状态", "尚未建立或尚未完成磁标定；绝对方向暂不可用。", "info"),
        0x02: ("地磁参考未建立", "缺少可信的地磁 dip/reference，磁修正暂时不能进入闭环。", "info"),
        0x04: ("磁场大小异常", "|B| 偏离稳定地磁范围，疑似磁体、铁磁物或局部磁场源靠近。", "disturbance"),
        0x08: ("磁倾角或方向异常", "磁矢量方向与地磁模型不一致，疑似局部磁场扭曲。", "disturbance"),
        0x10: ("方向与运动不符", "姿态运动能解释的磁场变化与实测不一致，疑似瞬时磁干扰。", "disturbance"),
        0x20: ("空间梯度异常", "多磁点变化不平行或空间梯度过大，疑似局部磁源或结构形变。", "disturbance"),
        0x40: ("磁航向漂移或修正保持", "航向漂移、修正速度或保持门控触发，修正可能暂停或冻结。", "warning"),
        0x80: ("强磁事件或候选参考", "检测到强磁扰动，或已生成候选新参考，等待稳定确认。", "strong"),
    }
    MAG_PAUSE_GATE_LABELS = {
        "磁标定",
        "参考磁 dip",
        "磁场大小门限",
        "磁倾角门限",
        "单拍方向阶跃",
        "空间梯度",
        "多链路一致性",
        "航向漂移",
        "诊断保持",
        "强磁事件",
        "拒绝状态",
        "候选参考",
        "航向有效",
        "磁修正权重",
        "修正保持",
    }
    # 每个门控负责的干扰特征，一句话总结。窗口只显示触发的门控，
    # 且只显示这一句，不附带数值/阈值/其它行。
    MAG_GATE_SUMMARY = {
        # --- 姿态精度包的 8 位诊断掩码 ---
        "标定状态": "磁标定未完成",
        "地磁参考未建立": "地磁参考未建立",
        "磁场大小异常": "磁场强度超范围",
        "磁倾角或方向异常": "磁倾角超范围",
        "方向与运动不符": "磁场方向与姿态不符",
        "空间梯度异常": "磁场空间梯度异常",
        "磁航向漂移或修正保持": "磁航向漂移",
        "强磁事件或候选参考": "强磁事件",
        # --- 固件磁诊断行标签（暂停门控白名单） ---
        "磁标定": "磁标定未完成",
        "参考磁 dip": "地磁参考未建立",
        "磁场大小门限": "磁场强度超范围",
        "磁倾角门限": "磁倾角超范围",
        "单拍方向阶跃": "磁场方向与姿态不符",
        "空间梯度": "磁场空间梯度异常",
        "多链路一致性": "多链路磁场不一致",
        "航向漂移": "磁航向漂移",
        "诊断保持": "诊断保持中",
        "强磁事件": "强磁事件",
        "拒绝状态": "修正被拒绝",
        "候选参考": "候选参考待确认",
        "航向有效": "航向无效",
        "磁修正权重": "修正权重为 0",
        "修正保持": "修正被保持",
    }

    # 磁修正面板数据新鲜度阈值（秒）。
    # 姿态精度流每个四元数帧刷新；固件磁诊断流在诊断窗口打开时约 1 Hz 轮询。
    # 超过阈值即视为陈旧：面板必须清空/标注，而不是把最后一帧冻结在界面上。
    MAG_ACC_FRESH_S = 1.0
    MAG_DIAG_FRESH_S = 2.0
    EDGES = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]

    def __init__(self, root: tk.Tk, auto_connect: bool = True, auto_trace: bool = False):
        self.root = root
        self.root.title("OSMO 姿态测试上位机")
        self.root.geometry("1100x740")
        self.root.minsize(760, 560)

        self.events: queue.Queue = queue.Queue()
        self.stop_event = threading.Event()
        self.reader: SerialReader | None = None
        self.current_q: Quat = IDENTITY
        self.reference_q: Quat = IDENTITY
        self.current_sensor_id: int | None = None
        self.current_accuracy: float | None = None
        self.frame_count = 0
        self.sensor_ids: set[int] = set()
        self.mag_by_sensor: dict[int, tuple[float, float, float]] = {}
        self.mag_history: dict[int, deque[tuple[float, float, float]]] = {}
        self.mag_world_history: dict[int, deque[tuple[float, float, float]]] = {}
        self.mag_common_history: dict[int, deque[tuple[float, float, float]]] = {}
        self.mag_diff_history: dict[int, deque[tuple[float, float, float]]] = {}
        self.mag_diff_baseline: dict[int, tuple[float, float, float]] = {}
        self.mag_norm_history: dict[int, deque[float]] = {}
        self.mag_lp_norm: dict[int, float] = {}
        self.mag_lp_norm_prev: dict[int, float] = {}
        self.mag_lp_diff: dict[int, float] = {}
        self.mag_lp_diff_prev: dict[int, float] = {}
        self.mag_metric_time: dict[int, float] = {}
        self.mag_ref_vec: dict[int, tuple[float, float, float]] = {}
        self.mag_pointer_zero: float | None = None
        self.mag_pointer_heading_lp: float | None = None
        self.mag_pointer_last_heading: float | None = None
        self.mag_pointer_stable_since: float | None = None
        self.mag_pointer_rebase_needed = False
        self.mag_state: dict[int, str] = {}
        self.mag_state_since: dict[int, float] = {}
        self.last_quat_for_motion: Quat | None = None
        self.last_quat_time: float | None = None
        self.motion_rates: deque[float] = deque(maxlen=9)
        self.quat_motion_deg_s = 0.0
        self.rate_times: list[float] = []
        self.pipeline_path = Path(__file__).resolve().parent / "diagnostics" / "pipeline_live.json"
        self.pipeline_counts: dict[str, int] = {"quat": 0, "mag": 0, "decode_error": 0, "bad_quat": 0, "bad_mag": 0, "duplicate_index": 0}
        self.pipeline_quat_times: deque[float] = deque(maxlen=600)
        self.pipeline_mag_times: deque[float] = deque(maxlen=600)
        self.pipeline_last_index: dict[tuple[int, str], int] = {}
        self.pipeline_last_flush = time.monotonic()
        self.pipeline_anomalies: deque[dict[str, Any]] = deque(maxlen=80)
        self.pipeline_last_state: dict[str, Any] = {}
        self.trace = TraceRecorder("pose_monitor", __file__)
        self.trace_root = Path(__file__).resolve().parent / "recordings"
        self.auto_trace = bool(auto_trace)
        self.firmware_diag_latest: dict[str, Any] = {}
        self.mag_diag_live_fields: dict[str, float] = {}
        self.mag_diag_live_rows: list[dict[str, Any]] = []
        self.mag_config_params: dict[str, float] = {}
        self.mag_config_rows: dict[int, dict[str, Any]] = {}
        self.mag_config_meta: dict[str, Any] = {}
        self.mag_live_last_update = 0.0
        # 磁修正面板：数据新鲜度追踪（修复"面板不实时/文字与窗口对不上"）
        self.mag_accuracy_mono = 0.0
        self.mag_diag_mono = 0.0
        self.last_mag_accuracy: tuple[int, float, int, int] | None = None
        self.mag_diag_poll_interval_s = 0.9  # 诊断窗口打开时的 P 轮询周期
        self._mag_panel_last_tick = 0.0
        self.firmware_diag_history: dict[str, deque[dict[str, Any]]] = {
            "gq": deque(maxlen=7200),
            "gy": deque(maxlen=7200),
            "mag_detail": deque(maxlen=7200),
            "mag_status": deque(maxlen=7200),
            "sinfo": deque(maxlen=256),
            "events": deque(maxlen=2048),
        }

        self.port_var = tk.StringVar()
        self.sensor_var = tk.StringVar(value="自动")
        self.axis_mode_var = tk.StringVar(value="Z往上（BHI360）")
        self.mag_scale_var = tk.StringVar(value="1/4 µT（双磁 Poll_Meta）")
        self.status_var = tk.StringVar(value="Ready")
        self.frame_var = tk.StringVar(value="Quat frames: 0")
        self.quat_var = tk.StringVar(value="quat: identity")
        self.calibration_var = tk.StringVar(value="未校准")
        self.mag_quality_var = tk.StringVar(value="地磁监测：等待磁数据（不参与姿态）")
        self.mag_detail_var = tk.StringVar(value="|B|=-  noise=-  Δdiff=-")
        self.fusion_status_var = tk.StringVar(value="磁修正状态：等待数据")
        self.yaw_diag_var = tk.StringVar(value="Yaw诊断：等待磁数据")
        self.trace_status_var = tk.StringVar(value="追踪：未录制")
        self.firmware_status_var = tk.StringVar(value="固件诊断：等待文本（FIX20生产版可能关闭GQ/GY）")
        self.mag_correction_var = tk.StringVar(value="磁修正：等待数据")
        self.mag_gate_header_var = tk.StringVar(value="")
        self.mag_pause_active_labels: list[str] = []
        self.mag_recovery_var = tk.StringVar(value="")
        # 方向残差门控强度（0..100%），由滑块实时下发到固件。
        self.mag_dir_strength_var = tk.DoubleVar(value=90.0)
        self.mag_dir_strength_readout_var = tk.StringVar(value="")
        self.mag_dir_eff_var = tk.StringVar(value="方向门强度：等待固件数据")
        self.mag_dir_strength_last_sent: int | None = None
        self.mag_diag_window: tk.Toplevel | None = None
        self.mag_feature_text: tk.Text | None = None
        self.firmware_status_poll_var = tk.BooleanVar(value=True)
        self.firmware_status_poll_interval_s = 5.0
        self.last_firmware_status_poll = 0.0
        self.last_mag_gate_mask: int | None = None
        self.last_mag_gate_since_ns: dict[int, int] = {}
        self.last_mag_status_code: int | None = None
        self.last_mag_cal_result: int | None = None
        self.last_firmware_gate_values: dict[str, object] = {}
        self.mag_feature_states: dict[int, dict[str, Any]] = {
            bit: {"active": False, "count": 0, "last_enter": None, "last_exit": None, "duration_ms": None}
            for bit in self.MAG_FEATURE_INFO
        }
        self.mag_disturbance_active = False
        self.mag_disturbance_started_ns: int | None = None
        self.mag_disturbance_had_strong = False
        self.mag_correction_effective = False
        self.mag_correction_effective_since_ns: int | None = None
        self.mag_recovery_candidate_since_ns: int | None = None
        self.mag_recovery_notice_until = 0.0
        self.mag_last_feature_ui_update = 0.0
        self.last_pose_ui_update = 0.0
        self.last_yaw_diag_update = 0.0
        self.last_compass_render = 0.0

        self._build_ui()
        self.refresh_ports()
        self.root.after(50, self._poll_events)
        self.root.after(250, self._render)
        self.root.bind("<space>", self._on_space)
        self.root.bind("<KeyPress-x>", self._demo_x)
        self.root.bind("<KeyPress-y>", self._demo_y)
        self.root.bind("<KeyPress-z>", self._demo_z)
        if self.auto_trace:
            self.root.after(120, self.start_auto_trace)
        if auto_connect:
            self.root.after(300, self.auto_connect)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=(10, 8))
        top.pack(fill=tk.X)
        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=14, state="normal")
        self.port_combo.pack(side=tk.LEFT, padx=(5, 3))
        ttk.Button(top, text="刷新", command=self.refresh_ports, takefocus=False).pack(side=tk.LEFT, padx=2)
        self.connect_button = ttk.Button(top, text="连接", command=self.toggle_connection, takefocus=False)
        self.connect_button.pack(side=tk.LEFT, padx=(8, 3))
        ttk.Button(top, text="空格校准", command=self.calibrate, takefocus=False).pack(side=tk.LEFT, padx=(12, 3))
        ttk.Button(top, text="磁干扰诊断", command=self.open_mag_diag_window, takefocus=False).pack(side=tk.LEFT, padx=(4, 3))
        ttk.Button(top, text="3D力窗口", command=self.open_force_window, takefocus=False).pack(side=tk.LEFT, padx=(4, 3))
        ttk.Label(top, text="传感器:").pack(side=tk.LEFT, padx=(16, 2))
        self.sensor_combo = ttk.Combobox(top, textvariable=self.sensor_var, width=12, state="readonly")
        self.sensor_combo.pack(side=tk.LEFT)
        ttk.Label(top, text="显示轴:").pack(side=tk.LEFT, padx=(14, 2))
        self.axis_combo = ttk.Combobox(
            top,
            textvariable=self.axis_mode_var,
            width=18,
            state="readonly",
            values=["Z往上（BHI360）", "Y往上（原始）", "X往上"],
        )
        self.axis_combo.pack(side=tk.LEFT)
        self.sensor_combo.bind("<<ComboboxSelected>>", lambda _event: self._record_ui_change("sensor_filter"))
        self.axis_combo.bind("<<ComboboxSelected>>", lambda _event: self._record_ui_change("axis_mode"))

        trace_bar = ttk.Frame(self.root, padding=(10, 0, 10, 4))
        trace_bar.pack(fill=tk.X)
        self.trace_button = ttk.Button(trace_bar, text="开始追踪录制", command=self.toggle_trace_recording, takefocus=False)
        self.trace_button.pack(side=tk.LEFT)
        self.mark_button = ttk.Button(trace_bar, text="参考位", command=self.mark_reference_pose, takefocus=False)
        self.mark_button.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(trace_bar, text="开始", command=lambda: self.record_operator_marker("motion_start"), takefocus=False).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(trace_bar, text="停止", command=lambda: self.record_operator_marker("motion_stop"), takefocus=False).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Label(trace_bar, textvariable=self.trace_status_var, anchor="w").pack(side=tk.LEFT, padx=(10, 0), fill=tk.X, expand=True)
        ttk.Checkbutton(trace_bar, text="P状态5秒", variable=self.firmware_status_poll_var, command=lambda: self._record_ui_change("p_status_poll", self.firmware_status_poll_var.get())).pack(side=tk.RIGHT)
        self.root.bind("<Control-m>", lambda _event: self.mark_reference_pose())

        info = ttk.Frame(self.root, padding=(10, 0, 10, 4))
        info.pack(fill=tk.X)
        ttk.Label(info, textvariable=self.status_var, width=48, anchor="w").pack(side=tk.LEFT)
        ttk.Label(info, textvariable=self.calibration_var, width=30, anchor="e").pack(side=tk.RIGHT)

        diag = ttk.Frame(self.root, padding=(10, 0, 10, 4))
        diag.pack(fill=tk.X)
        ttk.Label(
            diag,
            textvariable=self.fusion_status_var,
            font=("Arial", 10),
            justify=tk.LEFT,
            anchor=tk.W,
        ).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(
            diag,
            textvariable=self.yaw_diag_var,
            font=("Consolas", 10, "bold"),
            justify=tk.RIGHT,
            anchor=tk.E,
        ).pack(side=tk.RIGHT)

        fw_diag = ttk.Frame(self.root, padding=(10, 0, 10, 4))
        fw_diag.pack(fill=tk.X)
        ttk.Label(
            fw_diag,
            textvariable=self.firmware_status_var,
            font=("Consolas", 9),
            anchor=tk.W,
            justify=tk.LEFT,
        ).pack(side=tk.LEFT, fill=tk.X, expand=True)

        middle = ttk.Frame(self.root, padding=(10, 0, 10, 6))
        middle.pack(fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(middle, background="#101318", highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", lambda _event: self.canvas.focus_set())

        compass_panel = ttk.Frame(middle, padding=(8, 4), width=360)
        compass_panel.pack(side=tk.RIGHT, fill=tk.Y)
        compass_panel.pack_propagate(False)
        ttk.Label(compass_panel, text="世界地磁北 / 表盘随Yaw旋转").pack(anchor="center")
        self.compass_canvas = tk.Canvas(
            compass_panel,
            width=330,
            height=270,
            background="#101318",
            highlightthickness=0,
        )
        self.compass_canvas.pack(fill=tk.X)

        button_grid = ttk.Frame(compass_panel)
        button_grid.pack(fill=tk.X, pady=(4, 2))
        buttons = [
            ("开始磁标定(H)", self.hard_iron_start),
            ("立即结算(J)", self.hard_iron_finish),
            ("清除磁标定(K)", self.hard_iron_clear),
            ("重采磁基线(R)", self.rebaseline_mag),
            ("恢复可信标定(G)", self.restore_mag_calibration),
            ("磁差分归零", self.zero_mag_difference),
        ]
        for index, (text, command) in enumerate(buttons):
            ttk.Button(button_grid, text=text, command=command, takefocus=False).grid(
                row=index // 2, column=index % 2, sticky="ew", padx=2, pady=2
            )
        button_grid.columnconfigure(0, weight=1)
        button_grid.columnconfigure(1, weight=1)

        ttk.Label(compass_panel, text="磁单位:").pack(pady=(2, 0))
        self.mag_scale_combo = ttk.Combobox(
            compass_panel,
            textvariable=self.mag_scale_var,
            width=30,
            state="readonly",
            values=["1/4 µT（双磁 Poll_Meta）", "1/16 µT（官方NDOF）", "1 LSB（仅原始计数）"],
        )
        self.mag_scale_combo.pack()
        self.mag_scale_combo.bind("<<ComboboxSelected>>", lambda _event: self._record_ui_change("mag_scale"))
        ttk.Label(
            compass_panel,
            textvariable=self.mag_quality_var,
            font=("Arial", 11, "bold"),
            width=38,
            wraplength=330,
            justify=tk.CENTER,
            anchor=tk.CENTER,
        ).pack(pady=(4, 0))
        ttk.Label(
            compass_panel,
            textvariable=self.mag_detail_var,
            font=("Consolas", 9),
            width=38,
            justify=tk.CENTER,
            anchor=tk.CENTER,
        ).pack(pady=(2, 0))
        bottom = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        bottom.pack(fill=tk.X)
        ttk.Label(bottom, textvariable=self.quat_var, font=("Consolas", 10), width=95, anchor="w").pack(side=tk.LEFT)
        ttk.Label(bottom, text="空格：把当前姿态设为初始位；X/Y/Z：旋转演示").pack(side=tk.RIGHT)
        ttk.Label(bottom, textvariable=self.frame_var, width=28, anchor="e").pack(side=tk.RIGHT, padx=(0, 18))

    def _record_ui_change(self, name: str, value: Any | None = None) -> None:
        payload = {
            "name": name,
            "port": self.port_var.get(),
            "sensor_filter": self.sensor_var.get(),
            "axis_mode": self.axis_mode_var.get(),
            "mag_scale": self.mag_scale_var.get(),
            "reference_quaternion": list(self.reference_q),
            "calibration": self.calibration_var.get(),
        }
        if value is not None:
            payload["value"] = value
        self.trace.record_event("ui", "configuration_changed", payload)

    def record_operator_marker(self, marker: str) -> None:
        if not self.trace.is_recording:
            self.status_var.set("追踪未录制，无法记录操作标记")
            return
        self.trace.record_event("operator", marker, {
            "monotonic_ns": time.monotonic_ns(),
            "wall_time": time.time(),
            "reference_quaternion_wxyz": list(self.reference_q),
            "current_quaternion_wxyz": list(self.current_q),
            "current_sensor_id": self.current_sensor_id,
            "current_accuracy": self.current_accuracy,
            "mag_pointer_zero_deg": self.mag_pointer_zero,
            "mag_pointer_heading_lp_deg": self.mag_pointer_heading_lp,
            "mag_state": dict(self.mag_state),
        })
        self.status_var.set(f"已记录操作标记：{marker}")

    def mark_reference_pose(self) -> None:
        self.record_operator_marker("reference_pose")

    def start_auto_trace(self) -> None:
        if self.trace.is_recording:
            return
        self.trace_root.mkdir(parents=True, exist_ok=True)
        self.start_trace_recording(self.trace_root)

    def start_trace_recording(self, parent_dir: Path | None = None) -> None:
        if self.trace.is_recording:
            return
        if parent_dir is None:
            self.trace_root.mkdir(parents=True, exist_ok=True)
            selected = filedialog.askdirectory(
                title="选择追踪记录目录",
                initialdir=str(self.trace_root),
                mustexist=False,
            )
            if not selected:
                return
            parent_dir = Path(selected)
        metadata = {
            "port": self.port_var.get(),
            "sensor_filter": self.sensor_var.get(),
            "axis_mode": self.axis_mode_var.get(),
            "mag_scale": self.mag_scale_var.get(),
            "reference_quaternion": list(self.reference_q),
            "calibration": self.calibration_var.get(),
            "window_geometry": self.root.geometry(),
            "reader_connected_at_start": self.reader is not None and self.reader.is_alive(),
            "latest_known_firmware": latest_known_firmware(),
        }
        try:
            session = self.trace.start_session(parent_dir, metadata=metadata)
        except Exception as exc:
            self.trace_status_var.set(f"追踪启动失败：{exc}")
            messagebox.showerror("OSMO Pose", f"追踪启动失败：{exc}")
            return
        self.trace_button.configure(text="停止追踪录制")
        self.trace_status_var.set(f"追踪中：{session}")
        self.status_var.set(f"追踪记录：{session}")
        # Ask the firmware for a data-only calibration + fusion-state snapshot.
        # Old firmware ignores 'D'; new firmware answers with numeric frames.
        self._replay_snapshot_sent = self._send_cmd(b"D", "replay_snapshot")

    def stop_trace_recording(self, reason: str = "user") -> None:
        if not self.trace.is_recording:
            return
        self._drain_events()
        session = self.trace.stop_session(reason=reason)
        self.trace_button.configure(text="开始追踪录制")
        self.trace_status_var.set(f"追踪已停止：{session}")
        self.status_var.set(f"追踪已保存：{session}")

    def toggle_trace_recording(self) -> None:
        if self.trace.is_recording:
            self.stop_trace_recording("ui_toggle")
        else:
            self.start_trace_recording(None)
    def refresh_ports(self) -> None:
        ports = list(list_ports.comports())
        values = [p.device for p in ports]
        self.port_combo["values"] = values
        if not values:
            if not self.port_var.get().strip(): self.port_var.set("COM3")
            self.status_var.set("No ports listed; retrying COM3")
            self.root.after(1000, self.refresh_ports)
            return
        preferred = next((p.device for p in ports if p.vid == 0x2833 and p.pid == 0xB015), None)
        current = self.port_var.get()
        if current in values:
            return
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
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("OSMO Pose", "没有可用串口。")
            return
        self.stop_event.clear()
        self.mag_diag_live_fields.clear()
        self.mag_diag_live_rows.clear()
        self.mag_config_params.clear()
        self.mag_config_rows.clear()
        self.trace.record_event("ui", "connect_requested", {"port": port})
        self.reader = SerialReader(port, self.events, self.stop_event, self.trace)
        self.reader.start()
        self.connect_button.configure(text="断开")
        self.status_var.set(f"Opening {port} ...")
        self.canvas.focus_set()
        self.root.after(1200, lambda: self._send_cmd(b"P", "mag_diag_connect"))
        # If auto-trace started before the serial port was connected, the first
        # D command was recorded as not_connected.  Retry it once the reader is up.
        if self.trace.is_recording and not getattr(self, "_replay_snapshot_sent", False):
            self.root.after(1500, self._retry_replay_snapshot)

    def _retry_replay_snapshot(self) -> None:
        if (self.trace.is_recording and self.reader is not None and
                self.reader.ser is not None and
                not getattr(self, "_replay_snapshot_sent", False)):
            self._replay_snapshot_sent = self._send_cmd(b"D", "replay_snapshot_after_connect")

    def disconnect(self) -> None:
        self.trace.record_event("ui", "disconnect_requested", {"port": self.port_var.get()})
        self.stop_event.set()
        if self.reader is not None:
            self.reader.join(timeout=1.0)
        self.reader = None
        self.connect_button.configure(text="连接")
        self.status_var.set("Disconnected")
        self.canvas.focus_set()

    def _on_space(self, _event: tk.Event | None = None) -> str:
        self.calibrate()
        return "break"

    def _set_demo(self, q: Quat) -> None:
        self.current_q = q_normalize(q)
        self.current_sensor_id = -1
        self.current_accuracy = None
        self.calibration_var.set("旋转演示模式")

    def _demo_x(self, _event: tk.Event | None = None) -> None:
        self._set_demo(q_from_euler_deg(45.0, 0.0, 0.0))

    def _demo_y(self, _event: tk.Event | None = None) -> None:
        self._set_demo(q_from_euler_deg(0.0, 45.0, 0.0))

    def _demo_z(self, _event: tk.Event | None = None) -> None:
        self._set_demo(q_from_euler_deg(0.0, 0.0, 45.0))

    def calibrate(self) -> None:
        if self.current_sensor_id is None:
            self.status_var.set("还没有收到四元数，无法校准")
            return
        self.reference_q = self.current_q
        self.calibration_var.set("已校准：当前姿态为初始位")
        self.trace.record_event("calibration", "reference_set", {
            "sensor_id": self.current_sensor_id,
            "reference_quaternion": list(self.reference_q),
        })
        self.status_var.set("校准完成")
        self.canvas.focus_set()

    def zero_mag_difference(self) -> None:
        sid = sid_override if sid_override is not None else self.current_sensor_id
        if sid is None or sid < 1 or sid > 20:
            candidates = sorted(k for k in self.mag_by_sensor if 1 <= k <= 20)
            sid = candidates[0] if candidates else None
        primary = self.mag_by_sensor.get(sid) if sid is not None else None
        secondary = self.mag_by_sensor.get(sid + 20) if sid is not None else None
        if primary is None or secondary is None:
            self.status_var.set("没有双磁数据，无法归零磁差分")
            return
        baseline = tuple(primary[i] - secondary[i] for i in range(3))
        self.mag_diff_baseline[sid] = baseline
        self.mag_diff_history.setdefault(sid, deque(maxlen=80)).clear()
        self.trace.record_event("calibration", "mag_difference_zeroed", {
            "sensor_id": sid,
            "primary": list(primary),
            "secondary": list(secondary),
            "baseline": list(baseline),
        })
        self.status_var.set("磁场差分基线已归零")

    def rebaseline_mag(self) -> None:
        if not self._send_cmd(b"R", "rebaseline_mag"):
            return
        self.trace.record_event("calibration", "mag_state_cleared_for_rebaseline", {})
        self.mag_history.clear()
        self.mag_world_history.clear()
        self.mag_common_history.clear()
        self.mag_diff_history.clear()
        self.mag_diff_baseline.clear()
        self.mag_norm_history.clear()
        self.mag_lp_norm.clear()
        self.mag_lp_norm_prev.clear()
        self.mag_lp_diff.clear()
        self.mag_lp_diff_prev.clear()
        self.mag_metric_time.clear()
        self.mag_ref_vec.clear()
        self.mag_state.clear()
        self.mag_state_since.clear()
        self.mag_pointer_zero = None
        self.mag_pointer_heading_lp = None
        self.mag_pointer_last_heading = None
        self.mag_pointer_stable_since = None
        self.mag_pointer_rebase_needed = False
        self.status_var.set("已请求固件重新采磁基线")

    def _clear_mag_state(self) -> None:
        self.mag_history.clear()
        self.mag_world_history.clear()
        self.mag_common_history.clear()
        self.mag_diff_history.clear()
        self.mag_diff_baseline.clear()
        self.mag_norm_history.clear()
        self.mag_lp_norm.clear()
        self.mag_lp_norm_prev.clear()
        self.mag_lp_diff.clear()
        self.mag_lp_diff_prev.clear()
        self.mag_metric_time.clear()
        self.mag_ref_vec.clear()
        self.mag_state.clear()
        self.mag_state_since.clear()
        self.mag_pointer_zero = None
        self.mag_pointer_heading_lp = None
        self.mag_pointer_last_heading = None
        self.mag_pointer_stable_since = None
        self.mag_pointer_rebase_needed = False

    def _send_cmd(self, cmd: bytes, intent: str) -> bool:
        if self.reader is None or self.reader.ser is None:
            self.trace.record_command(cmd, intent, "not_connected")
            self.status_var.set("串口未连接")
            return False
        try:
            self.reader.ser.write(cmd)
            self.reader.ser.flush()
            self.trace.record_command(cmd, intent, "sent")
            return True
        except Exception as exc:
            self.trace.record_command(cmd, intent, "failed", f"{type(exc).__name__}: {exc}")
            self.status_var.set(f"命令发送失败: {exc}")
            return False

    def hard_iron_start(self) -> None:
        if self._send_cmd(b"H", "hard_iron_start"):
            self._clear_mag_state()
            self.status_var.set("硬铁校准中：会自动求解，请缓慢绕 X/Y/Z 各方向转完整圈")

    def hard_iron_clear(self) -> None:
        if self._send_cmd(b"K", "hard_iron_clear"):
            self._clear_mag_state()
            self.status_var.set("已清除硬铁校准")

    def hard_iron_finish(self) -> None:
        if self._send_cmd(b"J", "hard_iron_finish"):
            self._clear_mag_state()
            self.status_var.set("已请求立即结算...")

    def restore_mag_calibration(self) -> None:
        if self._send_cmd(b"G", "restore_mag_calibration"):
            self._clear_mag_state()
            self.status_var.set("已请求恢复上次可信磁标定")

    def _update_yaw_diag(self, sid: int | None = None) -> None:
        if sid is None or sid < 1 or sid > 20:
            candidates = sorted(k for k in self.mag_by_sensor if 1 <= k <= 20)
            sid = candidates[0] if candidates else None
        if sid is None:
            return
        mag = self.mag_by_sensor.get(sid)
        if mag is None:
            return
        now = time.monotonic()
        if now - self.last_yaw_diag_update < 0.05:
            return
        self.last_yaw_diag_update = now
        world = q_rotate(self.current_q, mag)
        mag_yaw = math.degrees(math.atan2(world[1], world[0]))
        out_yaw = q_yaw_deg(self.current_q)
        err = wrap_deg(mag_yaw - out_yaw)
        self.yaw_diag_var.set(
            f"输出Yaw={out_yaw:7.1f}°  磁航向={mag_yaw:7.1f}°  差={err:6.1f}°"
        )

    def _update_mag_pointer_reference(self, heading_deg: float, now: float) -> None:
        """Keep the compass needle zeroed to the currently accepted datum.

        The firmware deliberately re-baselines after a stable field change.
        When that new datum is accepted and settles, the UI must move the
        needle zero with it instead of leaving a permanent visual offset.
        """
        packed = float(self.current_accuracy or 0.0)
        packed_int = int(math.floor(packed))
        diag = packed_int // 100
        status = packed_int % 10
        weight = max(0.0, min(1.0, (packed - packed_int) / 0.99))
        fault = bool(diag & (0x40 | 0x80))
        clean = (not fault) and (status == 3) and (weight >= 0.95)

        if self.mag_pointer_heading_lp is None:
            self.mag_pointer_heading_lp = heading_deg
        else:
            self.mag_pointer_heading_lp = wrap_deg(
                self.mag_pointer_heading_lp
                + 0.20 * wrap_deg(heading_deg - self.mag_pointer_heading_lp)
            )
        smooth_heading = self.mag_pointer_heading_lp

        if self.mag_pointer_zero is None:
            self.mag_pointer_zero = smooth_heading
            self.mag_pointer_last_heading = smooth_heading
            self.mag_pointer_stable_since = now
            self.mag_pointer_rebase_needed = False
            return

        # Do not move the needle zero while the device is actually moving.
        if self.quat_motion_deg_s > 3.0:
            self.mag_pointer_last_heading = smooth_heading
            self.mag_pointer_stable_since = None
            return

        if not clean:
            self.mag_pointer_last_heading = smooth_heading
            self.mag_pointer_stable_since = None
            if fault:
                self.mag_pointer_rebase_needed = True
            return

        delta = abs(wrap_deg(smooth_heading - self.mag_pointer_zero))
        if self.mag_pointer_rebase_needed or delta > 5.0:
            if self.mag_pointer_stable_since is None:
                self.mag_pointer_stable_since = now
            elif now - self.mag_pointer_stable_since >= 0.5:
                self.mag_pointer_zero = smooth_heading
                self.mag_pointer_rebase_needed = False
                self.mag_pointer_stable_since = None
        else:
            self.mag_pointer_stable_since = None

        self.mag_pointer_last_heading = smooth_heading
    def set_quaternion(self, x: float, y: float, z: float, w: float, sensor_id: int = -1) -> None:
        self.current_q = q_normalize((w, x, y, z))
        self.current_sensor_id = sensor_id
        self.current_accuracy = None

    def _pipeline_anomaly(self, kind: str, detail: str) -> None:
        anomaly = {
            "host_time": time.time(),
            "kind": kind,
            "detail": detail,
        }
        self.pipeline_anomalies.append(anomaly)
        self.trace.record_event("anomaly", kind, {"detail": detail})

    @staticmethod
    def _row_trace_meta(row: dict[str, Any]) -> dict[str, Any]:
        meta = row.get("_trace") or {}
        return dict(meta) if isinstance(meta, dict) else {}

    @staticmethod
    def _compact_json(value: Any) -> str:
        try:
            return json.dumps(
                value, ensure_ascii=False, separators=(",", ":"),
                allow_nan=False, default=str,
            )
        except Exception:
            return json.dumps(str(value), ensure_ascii=False)

    def _analysis_base(self, row: dict[str, Any]) -> dict[str, Any]:
        meta = self._row_trace_meta(row)
        packet = row.get("packet") or {}
        mag = packet.get("mag") or {} if isinstance(packet, dict) else {}
        quat = packet.get("quat") or {} if isinstance(packet, dict) else {}
        rx_mono_ns = meta.get("rx_monotonic_ns")
        now_mono_ns = time.monotonic_ns()
        latency_anchor_ns = meta.get("replay_monotonic_ns", rx_mono_ns)
        latency_ms = (
            (now_mono_ns - int(latency_anchor_ns)) / 1e6
            if latency_anchor_ns is not None else None
        )
        qx = row.get("quat_x")
        qy = row.get("quat_y")
        qz = row.get("quat_z")
        qw = row.get("quat_w")
        return {
            "trace_id": meta.get("trace_id"),
            "connection_id": meta.get("connection_id"),
            "sequence": meta.get("sequence"),
            "rx_wall_time_ns": meta.get("rx_wall_time_ns"),
            "rx_wall_time": (
                int(meta["rx_wall_time_ns"]) / 1e9
                if meta.get("rx_wall_time_ns") is not None else row.get("host_time")
            ),
            "rx_monotonic_ns": rx_mono_ns,
            "rx_monotonic_s": (
                int(rx_mono_ns) / 1e9 if rx_mono_ns is not None else None
            ),
            "processing_monotonic_ns": now_mono_ns,
            "processing_latency_ms": latency_ms,
            "record_kind": row.get("kind"),
            "packet_len": meta.get("packet_len"),
            "decode_ok": True,
            "decode_error": None,
            "source": "serial",
            "index": row.get("index"),
            "finger": row.get("finger"),
            "finger_id": row.get("finger_id"),
            "link": row.get("link"),
            "sensor_id": row.get("sensor_id"),
            "kind": row.get("kind"),
            "mag_seconds": mag.get("seconds"),
            "mag_nanoseconds": mag.get("nanoseconds"),
            "mag_device_time_s": self._packet_time(mag),
            "quat_seconds": quat.get("seconds"),
            "quat_nanoseconds": quat.get("nanoseconds"),
            "quat_device_time_s": self._packet_time(quat),
            "mag_x_raw": row.get("mag_x"),
            "mag_y_raw": row.get("mag_y"),
            "mag_z_raw": row.get("mag_z"),
            "quat_x_raw": row.get("quat_x"),
            "quat_y_raw": row.get("quat_y"),
            "quat_z_raw": row.get("quat_z"),
            "quat_w_raw": row.get("quat_w"),
            "quat_accuracy_raw": row.get("quat_accuracy"),
            "quat_norm": (
                math.sqrt(float(qx) ** 2 + float(qy) ** 2 + float(qz) ** 2 + float(qw) ** 2)
                if None not in (qx, qy, qz, qw) else None
            ),
            "queue_depth": self.events.qsize(),
            "ui_axis_mode": self.axis_mode_var.get(),
            "ui_sensor_filter": self.sensor_var.get(),
            "ui_mag_scale": self.mag_scale_var.get(),
        }

    @staticmethod
    def _packet_time(values: dict[str, Any]) -> float | None:
        seconds = values.get("seconds")
        nanoseconds = values.get("nanoseconds")
        if seconds is None and nanoseconds is None:
            return None
        try:
            return float(seconds or 0) + float(nanoseconds or 0) * 1e-9
        except (TypeError, ValueError):
            return None

    def _record_analysis(self, row: dict[str, Any], **values: Any) -> None:
        if not self.trace.is_recording:
            return
        payload = self._analysis_base(row)
        details = values.pop("details", None)
        payload.update(values)
        if details is not None:
            payload["details_json"] = self._compact_json(details)
        self.trace.record_analysis(payload)

    def _record_stage(self, stage: str, row: dict[str, Any], **values: Any) -> None:
        if not self.trace.is_recording:
            return
        meta = self._row_trace_meta(row)
        payload = {
            "trace_id": meta.get("trace_id"),
            "connection_id": meta.get("connection_id"),
            "sequence": meta.get("sequence"),
            "rx_wall_time_ns": meta.get("rx_wall_time_ns"),
            "rx_monotonic_ns": meta.get("rx_monotonic_ns"),
            "source_kind": row.get("kind"),
            "sensor_id": row.get("sensor_id"),
            "index": row.get("index"),
        }
        payload.update(values)
        self.trace.record_stage(stage, payload)

    def _pipeline_flush(self, now: float) -> None:
        if now - self.pipeline_last_flush < 1.0:
            return
        self.pipeline_last_flush = now
        cutoff = now - 1.0
        while self.pipeline_quat_times and self.pipeline_quat_times[0] < cutoff:
            self.pipeline_quat_times.popleft()
        while self.pipeline_mag_times and self.pipeline_mag_times[0] < cutoff:
            self.pipeline_mag_times.popleft()
        relative_q = self._relative_q()
        render_width = max(2, self.canvas.winfo_width())
        render_height = max(2, self.canvas.winfo_height())
        render_state = {
            "current_quaternion_wxyz": list(self.current_q),
            "relative_quaternion_wxyz": list(relative_q),
            "current_yaw_deg": q_yaw_deg(self.current_q),
            "relative_yaw_deg": q_yaw_deg(relative_q),
            "axis_mode": self.axis_mode_var.get(),
            "sensor_filter": self.sensor_var.get(),
            "canvas_width": render_width,
            "canvas_height": render_height,
            "cube_screen_points": self.cube_screen_points(render_width, render_height),
            "mag_pointer_zero_deg": self.mag_pointer_zero,
            "mag_pointer_heading_lp_deg": self.mag_pointer_heading_lp,
            "mag_pointer_rebase_needed": self.mag_pointer_rebase_needed,
        }
        runtime_metrics = collect_runtime_metrics(self.trace.session_dir)
        snapshot = {
            "host_time": time.time(),
            "connected": self.reader is not None and self.reader.ser is not None,
            "port": self.port_var.get(),
            "counts": dict(self.pipeline_counts),
            "quat_hz": len(self.pipeline_quat_times),
            "mag_hz": len(self.pipeline_mag_times),
            "queue_depth": self.events.qsize(),
            "last_state": dict(self.pipeline_last_state),
            "anomalies": list(self.pipeline_anomalies)[-20:],
            "trace": self.trace.stats(),
            "firmware_diagnostic": self.firmware_diag_latest,
            "render": render_state,
            "runtime": runtime_metrics,
        }
        poll_interval = self.firmware_status_poll_interval_s
        window = self.mag_diag_window
        if window is not None:
            try:
                if window.winfo_exists():
                    # 诊断窗口打开时把 P 轮询提到 1 Hz，门控数值才是准实时的；
                    # 窗口关掉就回到 5 s，避免长期占用 CDC 带宽。
                    poll_interval = self.mag_diag_poll_interval_s
            except Exception:
                poll_interval = self.firmware_status_poll_interval_s
        if (self.firmware_status_poll_var.get() and
                self.reader is not None and self.reader.ser is not None and
                now - self.last_firmware_status_poll >= poll_interval):
            if self._send_cmd(b"P", "auto_mag_status_poll"):
                self.last_firmware_status_poll = now
            # 顺带重发一次方向残差门控强度：板子复位/换固件后无需人工再拨。
            try:
                self._send_dir_strength(int(round(float(self.mag_dir_strength_var.get()))), force=True)
            except Exception:
                pass

        if self.trace.is_recording:
            self.trace.record_system({
                "event": "periodic_snapshot",
                "connected": snapshot["connected"],
                "port": snapshot["port"],
                "counts": snapshot["counts"],
                "quat_hz": snapshot["quat_hz"],
                "mag_hz": snapshot["mag_hz"],
                "event_queue_depth": snapshot["queue_depth"],
                "last_state": snapshot["last_state"],
                "anomalies": snapshot["anomalies"],
                "firmware_diagnostic": snapshot["firmware_diagnostic"],
                "render": snapshot["render"],
                "runtime": snapshot["runtime"],
            })
            trace_info = snapshot["trace"]
            session_text = str(trace_info.get("session_dir"))
            self.trace_status_var.set(
                f"追踪中：{session_text}  "
                f"写入={trace_info.get("written", 0)}  "
                f"队列={trace_info.get("queue_depth", 0)}  "
                f"丢弃={trace_info.get("dropped", 0)}"
            )
        for error in self.trace.pop_errors():
            message = str(error.get("error", error))
            self.pipeline_anomalies.append({
                "host_time": time.time(),
                "kind": "trace_recorder",
                "detail": message,
            })
        try:
            tmp = self.pipeline_path.with_suffix(".tmp")
            tmp.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2), encoding="utf-8")
            tmp.replace(self.pipeline_path)
        except Exception:
            pass
    def _handle_quat(self, row: dict[str, Any]) -> None:
        try:
            sid = int(row.get("sensor_id") or 0)
        except Exception:
            sid = 0
        self.pipeline_counts["quat"] += 1
        self.pipeline_quat_times.append(time.monotonic())
        self.sensor_ids.add(sid)

        if self.sensor_var.get() != "自动":
            try:
                if int(self.sensor_var.get()) != sid:
                    self._record_stage("quat_rejected", row, reason="sensor_filter")
                    self._record_analysis(
                        row,
                        accepted=False,
                        drop_reason="sensor_filter",
                        details={"selected_sensor": self.sensor_var.get()},
                    )
                    return
            except ValueError:
                pass

        raw_w = row.get("quat_w")
        raw_x = row.get("quat_x")
        raw_y = row.get("quat_y")
        raw_z = row.get("quat_z")
        try:
            q_raw = (
                float(raw_w or 0.0),
                float(raw_x or 0.0),
                float(raw_y or 0.0),
                float(raw_z or 0.0),
            )
        except (TypeError, ValueError) as exc:
            self.pipeline_counts["bad_quat"] += 1
            self._pipeline_anomaly("bad_quat", f"sensor={sid} values={q_raw if 'q_raw' in locals() else None} error={exc}")
            self._record_stage("quat_rejected", row, reason="bad_values", error=str(exc))
            self._record_analysis(
                row,
                accepted=False,
                drop_reason="bad_values",
                details={"error": str(exc)},
            )
            return

        q_norm_value = math.sqrt(sum(v * v for v in q_raw))
        if q_norm_value <= 1e-12 or not all(math.isfinite(v) for v in q_raw):
            self.pipeline_counts["bad_quat"] += 1
            self._pipeline_anomaly(
                "bad_quat",
                f"sensor={sid} norm={q_norm_value} raw={q_raw}",
            )
            self._record_stage("quat_rejected", row, reason="bad_quaternion", raw=q_raw, norm=q_norm_value)
            self._record_analysis(
                row,
                accepted=False,
                drop_reason="bad_quaternion",
                quat_norm=q_norm_value,
                details={"raw_quaternion_wxyz": q_raw},
            )
            return

        self.current_q = q_normalize(q_raw)
        self.current_sensor_id = sid
        self._update_yaw_diag(sid)
        acc_value = row.get("quat_accuracy")
        self.current_accuracy = float(acc_value) if acc_value is not None else 0.0

        delta_deg: float | None = None
        dt: float | None = None
        trace_meta = self._row_trace_meta(row)
        rx_monotonic_ns = trace_meta.get("rx_monotonic_ns")
        now_motion = int(rx_monotonic_ns) / 1e9 if rx_monotonic_ns is not None else time.monotonic()
        if self.last_quat_for_motion is not None and self.last_quat_time is not None:
            dot = abs(sum(a * b for a, b in zip(self.current_q, self.last_quat_for_motion)))
            dot = max(-1.0, min(1.0, dot))
            delta_deg = math.degrees(2.0 * math.acos(dot))
            dt = now_motion - self.last_quat_time
            # Ignore duplicate/same-burst frames. Otherwise a tiny dt turns a
            # quantization-level change into a huge false angular rate.
            if dt >= 0.002:
                self.motion_rates.append(delta_deg / dt)
                ordered = sorted(self.motion_rates)
                self.quat_motion_deg_s = ordered[len(ordered) // 2]
        self.last_quat_for_motion = self.current_q
        self.last_quat_time = now_motion
        self.frame_count += 1
        now = time.monotonic()
        self.rate_times.append(now)
        while self.rate_times and now - self.rate_times[0] > 1.0:
            self.rate_times.pop(0)
        update_pose_ui = now - self.last_pose_ui_update >= 0.05
        if update_pose_ui:
            self.last_pose_ui_update = now
            self.frame_var.set(f"Quat frames: {self.frame_count}    {len(self.rate_times)} Hz")

        values = sorted(self.sensor_ids)
        choices = ["自动"] + [str(v) for v in values]
        if list(self.sensor_combo["values"]) != choices:
            self.sensor_combo["values"] = choices

        q = self.current_q
        relative_q = self._relative_q()
        yaw_deg = q_yaw_deg(q)
        relative_yaw_deg = q_yaw_deg(relative_q)
        packed = float(self.current_accuracy or 0.0)
        packed_int = int(math.floor(packed))
        diag = packed_int // 100
        cal_result = (packed_int // 10) % 10
        status_code = packed_int % 10
        weight = max(0.0, min(1.0, (packed - packed_int) / 0.99))

        fusion_text = {
            0: "未生效（未标定）",
            1: "未生效（标定采集中）",
            2: "未生效（航向获取中）",
            3: "已生效（LOCKED）",
            4: "部分生效（DEGRADED）",
            5: "暂停（COASTING）",
        }.get(status_code, f"未知状态{status_code}")

        cal_text = {
            0: "尚未生成结果（自动尝试中）",
            1: "成功",
            2: "样本不足（继续自动尝试）",
            3: "转动范围不足（继续自动尝试）",
            4: "求解失败（继续自动尝试）",
            5: "残差过大（继续自动尝试）",
            6: "增益异常（继续自动尝试）",
            7: "已清除",
        }.get(cal_result, f"未知标定{cal_result}")

        diag_rows = [
            (0x01, "未标定"),
            (0x02, "参考磁场/dip未建立"),
            (0x04, "磁场大小门控"),
            (0x08, "磁倾角门控"),
            (0x10, "磁场变化率过大"),
            (0x20, "空间梯度门控"),
            (0x40, "磁矢量漂移/状态门控"),
            (0x80, "重新建立参考中"),
        ]
        diag_lines = ["诊断清单："]
        for mask, label in diag_rows:
            diag_lines.append(f"  [{'!' if (diag & mask) else ' '}] {label}")
        known_mask = sum(mask for mask, _ in diag_rows)
        unknown = diag & ~known_mask
        if unknown:
            diag_lines.append(f"  [!] 未知诊断位 0x{unknown:02X}")
        diag_text = "\n".join(diag_lines)
        self._track_accuracy_gates(row, diag, status_code, cal_result, weight, packed)

        if update_pose_ui:
            self.quat_var.set(
                f"quat: x={q[1]: .5f} y={q[2]: .5f} z={q[3]: .5f} w={q[0]: .5f}  sensor={sid}"
            )
            self.fusion_status_var.set(
                f"磁修正：{fusion_text}  权重：{weight*100:.0f}%  标定：{cal_text}"
            )

        self._record_stage(
            "quat_processing",
            row,
            accepted=True,
            input={"raw_quaternion_wxyz": q_raw, "raw_norm": q_norm_value},
            normalized_quaternion_wxyz=q,
            reference_quaternion_wxyz=self.reference_q,
            relative_quaternion_wxyz=relative_q,
            yaw_deg=yaw_deg,
            relative_yaw_deg=relative_yaw_deg,
            motion={
                "delta_deg": delta_deg,
                "dt_s": dt,
                "rate_deg_s": delta_deg / dt if delta_deg is not None and dt and dt > 0 else None,
                "median_rate_deg_s": self.quat_motion_deg_s,
            },
            accuracy={
                "packed": packed,
                "status_code": status_code,
                "cal_result": cal_result,
                "diagnostic_bits": diag,
                "weight": weight,
                "fault": bool(diag & (0x40 | 0x80)),
                "status_text": fusion_text,
                "calibration_text": cal_text,
                "unknown_bits": unknown,
            },
            final={
                "sensor_id": sid,
                "quaternion_wxyz": q,
                "relative_quaternion_wxyz": relative_q,
                "yaw_deg": yaw_deg,
                "relative_yaw_deg": relative_yaw_deg,
            },
        )
        self._record_analysis(
            row,
            accepted=True,
            quat_norm=q_norm_value,
            quat_w_norm=q[0],
            quat_x_norm=q[1],
            quat_y_norm=q[2],
            quat_z_norm=q[3],
            ref_quat_w=self.reference_q[0],
            ref_quat_x=self.reference_q[1],
            ref_quat_y=self.reference_q[2],
            ref_quat_z=self.reference_q[3],
            relative_quat_w=relative_q[0],
            relative_quat_x=relative_q[1],
            relative_quat_y=relative_q[2],
            relative_quat_z=relative_q[3],
            yaw_deg=yaw_deg,
            relative_yaw_deg=relative_yaw_deg,
            motion_delta_deg=delta_deg,
            motion_dt_s=dt,
            motion_rate_deg_s=delta_deg / dt if delta_deg is not None and dt and dt > 0 else None,
            motion_median_deg_s=self.quat_motion_deg_s,
            acc_status=status_code,
            acc_cal_result=cal_result,
            acc_diag=diag,
            acc_weight=weight,
            acc_fault=bool(diag & (0x40 | 0x80)),
            acc_status_text=fusion_text,
            acc_cal_text=cal_text,
            details={"diagnostic_text": diag_lines, "unknown_bits": unknown},
        )

    def _handle_mag(self, row: dict[str, Any]) -> None:
        self.pipeline_counts["mag"] += 1
        self.pipeline_mag_times.append(time.monotonic())
        try:
            sid = int(row.get("sensor_id") or 0)
            vec = (
                float(row.get("mag_x") or 0.0),
                float(row.get("mag_y") or 0.0),
                float(row.get("mag_z") or 0.0),
            )
        except Exception as exc:
            self.pipeline_counts["bad_mag"] += 1
            self._pipeline_anomaly("bad_mag", str(exc))
            self._record_stage("mag_rejected", row, reason="bad_values", error=str(exc))
            self._record_analysis(row, accepted=False, drop_reason="bad_values")
            return
        if not all(math.isfinite(v) for v in vec):
            self.pipeline_counts["bad_mag"] += 1
            self._pipeline_anomaly("bad_mag", f"sensor={sid} vector={vec}")
            self._record_stage("mag_rejected", row, reason="non_finite", vector=vec)
            self._record_analysis(row, accepted=False, drop_reason="non_finite")
            return
        self.mag_by_sensor[sid] = vec
        self._update_yaw_diag(sid)
        history = self.mag_history.setdefault(sid, deque(maxlen=80))
        history.append(vec)
        world_vec: tuple[float, float, float] | None = None
        if self.current_sensor_id is not None:
            world_vec = q_rotate(self.current_q, vec)
            world_history = self.mag_world_history.setdefault(sid, deque(maxlen=80))
            world_history.append(world_vec)

        secondary: tuple[float, float, float] | None = None
        common: tuple[float, float, float] | None = None
        diff: tuple[float, float, float] | None = None
        raw_mag: float | None = None
        raw_diff: float | None = None
        prev_lp_mag: float | None = None
        curr_lp_mag: float | None = None
        prev_lp_diff: float | None = None
        curr_lp_diff: float | None = None
        dt: float | None = None
        alpha: float | None = None
        state_info: dict[str, Any] = {}
        baseline_set = False

        # Primary magnetometer IDs are 1..20; the paired differential channel
        # is transmitted as primary_id + 20. Fuse common mode and monitor the
        # dynamic part of the difference.
        if 1 <= sid <= 20:
            secondary = self.mag_by_sensor.get(sid + 20)
            if secondary is not None:
                common = tuple((vec[i] + secondary[i]) * 0.5 for i in range(3))
                diff = tuple(vec[i] - secondary[i] for i in range(3))
                common_hist = self.mag_common_history.setdefault(sid, deque(maxlen=80))
                diff_hist = self.mag_diff_history.setdefault(sid, deque(maxlen=80))
                norm_hist = self.mag_norm_history.setdefault(sid, deque(maxlen=80))
                common_hist.append(common)
                diff_hist.append(diff)
                raw_mag = self._norm3(common)
                raw_diff = self._norm3(diff)
                norm_hist.append(raw_mag)

                now_metric = time.monotonic()
                prev_time = self.mag_metric_time.get(sid)
                dt = 0.02 if prev_time is None else max(now_metric - prev_time, 1e-3)
                alpha = 1.0 - math.exp(-dt / 0.25)
                prev_lp_mag = self.mag_lp_norm.get(sid, raw_mag)
                prev_lp_diff = self.mag_lp_diff.get(sid, raw_diff)
                self.mag_lp_norm_prev[sid] = prev_lp_mag
                self.mag_lp_diff_prev[sid] = prev_lp_diff
                curr_lp_mag = alpha * raw_mag + (1.0 - alpha) * prev_lp_mag
                curr_lp_diff = alpha * raw_diff + (1.0 - alpha) * prev_lp_diff
                self.mag_lp_norm[sid] = curr_lp_mag
                self.mag_lp_diff[sid] = curr_lp_diff
                self.mag_metric_time[sid] = now_metric
                state_info = self._update_mag_state(sid, common, now_metric)

                if sid not in self.mag_diff_baseline and len(diff_hist) >= 20 and self.quat_motion_deg_s < 10.0:
                    self.mag_diff_baseline[sid] = tuple(
                        sum(v[i] for v in diff_hist) / len(diff_hist) for i in range(3)
                    )
                    baseline_set = True
            else:
                # Official NDOF mode exposes one raw magnetometer channel.
                # Run magnitude and stationary-vector detection without a
                # second BMM350 channel.
                norm_hist = self.mag_norm_history.setdefault(sid, deque(maxlen=80))
                raw_mag = self._norm3(vec)
                norm_hist.append(raw_mag)

                now_metric = time.monotonic()
                prev_time = self.mag_metric_time.get(sid)
                dt = 0.02 if prev_time is None else max(now_metric - prev_time, 1e-3)
                alpha = 1.0 - math.exp(-dt / 0.25)
                prev_lp_mag = self.mag_lp_norm.get(sid, raw_mag)
                self.mag_lp_norm_prev[sid] = prev_lp_mag
                curr_lp_mag = alpha * raw_mag + (1.0 - alpha) * prev_lp_mag
                self.mag_lp_norm[sid] = curr_lp_mag
                self.mag_metric_time[sid] = now_metric
                state_info = self._update_mag_state(sid, vec, now_metric)

        self._record_mag_trace(
            row,
            sid,
            vec,
            world_vec=world_vec,
            secondary=secondary,
            common=common,
            diff=diff,
            raw_mag=raw_mag,
            raw_diff=raw_diff,
            prev_lp_mag=prev_lp_mag,
            curr_lp_mag=curr_lp_mag,
            prev_lp_diff=prev_lp_diff,
            curr_lp_diff=curr_lp_diff,
            dt=dt,
            alpha=alpha,
            state_info=state_info,
            baseline_set=baseline_set,
        )

    def _record_mag_trace(
        self,
        row: dict[str, Any],
        sid: int,
        vec: tuple[float, float, float],
        *,
        world_vec: tuple[float, float, float] | None,
        secondary: tuple[float, float, float] | None,
        common: tuple[float, float, float] | None,
        diff: tuple[float, float, float] | None,
        raw_mag: float | None,
        raw_diff: float | None,
        prev_lp_mag: float | None,
        curr_lp_mag: float | None,
        prev_lp_diff: float | None,
        curr_lp_diff: float | None,
        dt: float | None,
        alpha: float | None,
        state_info: dict[str, Any],
        baseline_set: bool,
    ) -> None:
        scale_label = self.mag_scale_var.get()
        if scale_label.startswith("1/4"):
            unit_scale = 0.25
        elif scale_label.startswith("1/16"):
            unit_scale = 1.0 / 16.0
        else:
            unit_scale = 1.0
        scaled_vec = tuple(v * unit_scale for v in vec)
        quality = self._mag_quality(sid) if 1 <= sid <= 20 else (
            "辅助/第二磁通道（仅监测）", "#8c98a8", None, None, None, None, None
        )
        quality_status, quality_color, quality_common, magnitude, noise, diff_dynamic, jump = quality

        mag_yaw_deg: float | None = None
        output_yaw_deg: float | None = None
        yaw_error_deg: float | None = None
        if 1 <= sid <= 20:
            world_diag = q_rotate(self.current_q, vec)
            mag_yaw_deg = math.degrees(math.atan2(world_diag[1], world_diag[0]))
            output_yaw_deg = q_yaw_deg(self.current_q)
            yaw_error_deg = wrap_deg(mag_yaw_deg - output_yaw_deg)

        diff_baseline = self.mag_diff_baseline.get(sid) if 1 <= sid <= 20 else None
        diff_dynamic_vec = None
        if diff is not None and diff_baseline is not None:
            diff_dynamic_vec = tuple(diff[i] - diff_baseline[i] for i in range(3))
        elif diff is not None:
            diff_dynamic_vec = diff

        self._record_stage(
            "mag_processing",
            row,
            accepted=True,
            raw={"vector": vec, "norm": self._norm3(vec), "unit_scale": unit_scale, "scaled_vector": scaled_vec},
            world={"vector": world_vec},
            pairing={
                "pair_id": sid if 1 <= sid <= 20 else sid - 20,
                "role": "primary" if 1 <= sid <= 20 else "secondary_or_auxiliary",
                "secondary_sensor_id": sid + 20 if 1 <= sid <= 20 and secondary is not None else None,
                "secondary_seen": secondary is not None,
                "secondary_vector": secondary,
            },
            common={"vector": common},
            difference={
                "vector": diff,
                "baseline": diff_baseline,
                "baseline_set_now": baseline_set,
                "dynamic_vector": diff_dynamic_vec,
                "dynamic_norm": self._norm3(diff_dynamic_vec) if diff_dynamic_vec is not None else None,
            },
            low_pass={
                "dt_s": dt,
                "alpha": alpha,
                "norm_prev": prev_lp_mag,
                "norm": curr_lp_mag,
                "difference_norm_prev": prev_lp_diff,
                "difference_norm": curr_lp_diff,
                "raw_norm": raw_mag,
                "raw_difference_norm": raw_diff,
            },
            state_machine=state_info,
            quality={
                "status": quality_status,
                "color": quality_color,
                "common_scaled": quality_common,
                "magnitude_ut": magnitude,
                "noise_ut": noise,
                "difference_dynamic": diff_dynamic,
                "jump_ut": jump,
            },
            yaw_diagnostics={"mag_yaw_deg": mag_yaw_deg, "output_yaw_deg": output_yaw_deg, "error_deg": yaw_error_deg},
        )
        ref = state_info.get("reference")
        if ref is None:
            ref = (None, None, None)
        self._record_analysis(
            row,
            accepted=True,
            mag_scale=scale_label,
            mag_unit_scale=unit_scale,
            mag_x=scaled_vec[0],
            mag_y=scaled_vec[1],
            mag_z=scaled_vec[2],
            mag_world_x=world_vec[0] if world_vec is not None else None,
            mag_world_y=world_vec[1] if world_vec is not None else None,
            mag_world_z=world_vec[2] if world_vec is not None else None,
            paired_sensor_id=sid + 20 if 1 <= sid <= 20 and secondary is not None else None,
            paired_seen=secondary is not None,
            mag_secondary_x=secondary[0] if secondary is not None else None,
            mag_secondary_y=secondary[1] if secondary is not None else None,
            mag_secondary_z=secondary[2] if secondary is not None else None,
            mag_common_x=common[0] if common is not None else None,
            mag_common_y=common[1] if common is not None else None,
            mag_common_z=common[2] if common is not None else None,
            mag_diff_x=diff[0] if diff is not None else None,
            mag_diff_y=diff[1] if diff is not None else None,
            mag_diff_z=diff[2] if diff is not None else None,
            mag_diff_baseline_x=diff_baseline[0] if diff_baseline is not None else None,
            mag_diff_baseline_y=diff_baseline[1] if diff_baseline is not None else None,
            mag_diff_baseline_z=diff_baseline[2] if diff_baseline is not None else None,
            mag_diff_dynamic_x=diff_dynamic_vec[0] if diff_dynamic_vec is not None else None,
            mag_diff_dynamic_y=diff_dynamic_vec[1] if diff_dynamic_vec is not None else None,
            mag_diff_dynamic_z=diff_dynamic_vec[2] if diff_dynamic_vec is not None else None,
            mag_diff_norm_raw=raw_diff,
            mag_diff_norm_lp_prev=prev_lp_diff,
            mag_diff_norm_lp=curr_lp_diff,
            mag_norm_raw=raw_mag,
            mag_norm_lp_prev=prev_lp_mag,
            mag_norm_lp=curr_lp_mag,
            mag_dt_s=dt,
            mag_alpha=alpha,
            mag_state_before=state_info.get("state_before"),
            mag_state_after=state_info.get("state_after"),
            mag_state_since=state_info.get("state_since"),
            mag_decision=state_info.get("decision"),
            mag_ref_x=ref[0],
            mag_ref_y=ref[1],
            mag_ref_z=ref[2],
            mag_ref_norm=state_info.get("reference_norm"),
            mag_current_norm=state_info.get("current_norm"),
            mag_dot=state_info.get("dot"),
            mag_angle_deg=state_info.get("angle_deg"),
            mag_dmag_rel=state_info.get("dmag_rel"),
            mag_baseline_set=baseline_set,
            mag_quality=quality_status,
            mag_magnitude_ut=magnitude,
            mag_noise_ut=noise,
            mag_jump_ut=jump,
            mag_yaw_deg=mag_yaw_deg,
            output_yaw_deg=output_yaw_deg,
            yaw_error_deg=yaw_error_deg,
            pointer_zero_deg=self.mag_pointer_zero,
            pointer_heading_lp_deg=self.mag_pointer_heading_lp,
            pointer_heading_delta_deg=(
                wrap_deg(self.mag_pointer_heading_lp - self.mag_pointer_zero)
                if self.mag_pointer_heading_lp is not None and self.mag_pointer_zero is not None else None
            ),
            pointer_rebase_needed=self.mag_pointer_rebase_needed,
            pointer_stable_since=self.mag_pointer_stable_since,
            details={"diff_dynamic": diff_dynamic_vec, "quality_color": quality_color, "state_machine": state_info},
        )
    def _update_mag_state(self, sid: int, common: tuple[float, float, float], now: float) -> dict[str, Any]:
        """Two-mode detector: vector check at rest, magnitude check in motion."""
        state = self.mag_state.get(sid, "REST")
        since = self.mag_state_since.get(sid, now)
        info: dict[str, Any] = {
            "state_before": state,
            "state_since": since,
            "motion_rate_deg_s": self.quat_motion_deg_s,
            "reference": None,
            "reference_norm": None,
            "current_norm": self._norm3(common),
            "dot": None,
            "angle_deg": None,
            "dmag_rel": None,
        }

        # In motion, direction is expected to change; drop the old vector
        # reference and use only magnitude/differential diagnostics.
        if self.quat_motion_deg_s > 3.0:
            self.mag_state[sid] = "MOTION"
            self.mag_state_since[sid] = now
            self.mag_ref_vec.pop(sid, None)
            info.update({"state_after": "MOTION", "state_since": now, "decision": "motion_detected_clear_reference"})
            return info

        ref = self.mag_ref_vec.get(sid)
        info["reference"] = ref
        if ref is None:
            self.mag_ref_vec[sid] = common
            self.mag_state[sid] = "REST"
            self.mag_state_since[sid] = now
            info.update({
                "state_after": "REST",
                "state_since": now,
                "decision": "initialize_reference",
                "reference": common,
                "reference_norm": self._norm3(common),
            })
            return info

        n_ref = self._norm3(ref)
        n_cur = self._norm3(common)
        info.update({"reference_norm": n_ref, "current_norm": n_cur})
        if n_ref < 1.0 or n_cur < 1.0:
            self.mag_state[sid] = "BAD"
            self.mag_state_since[sid] = now
            info.update({"state_after": "BAD", "state_since": now, "decision": "bad_low_magnitude"})
            return info

        dot = sum(ref[i] * common[i] for i in range(3)) / (n_ref * n_cur)
        dot = max(-1.0, min(1.0, dot))
        angle = math.degrees(math.acos(dot))
        dmag_rel = abs(n_cur - n_ref) / n_ref
        info.update({"dot": dot, "angle_deg": angle, "dmag_rel": dmag_rel})

        if angle <= 5.0 and dmag_rel <= 0.02:
            if state in ("SUSPECT", "BAD"):
                if now - since >= 0.5:
                    self.mag_ref_vec[sid] = common
                    self.mag_state[sid] = "REST"
                    self.mag_state_since[sid] = now
                    info.update({
                        "state_after": "REST",
                        "state_since": now,
                        "decision": "recover_rest",
                        "reference": common,
                        "reference_norm": n_cur,
                    })
                else:
                    info.update({"state_after": state, "decision": "wait_recovery_hold"})
            else:
                alpha = 0.01
                updated_ref = tuple(alpha * common[i] + (1.0 - alpha) * ref[i] for i in range(3))
                self.mag_ref_vec[sid] = updated_ref
                self.mag_state[sid] = "REST"
                self.mag_state_since[sid] = since
                info.update({
                    "state_after": "REST",
                    "decision": "stable_track_reference",
                    "reference": updated_ref,
                    "reference_norm": self._norm3(updated_ref),
                })
            return info

        if angle > 20.0 or dmag_rel > 0.08:
            self.mag_state[sid] = "BAD"
            self.mag_state_since[sid] = now
            info.update({
                "state_after": "BAD",
                "state_since": now,
                "decision": "bad_vector_or_magnitude",
            })
        elif state == "SUSPECT" and now - since >= 0.10:
            self.mag_state[sid] = "BAD"
            self.mag_state_since[sid] = now
            info.update({
                "state_after": "BAD",
                "state_since": now,
                "decision": "suspect_timeout_bad",
            })
        elif state != "BAD":
            self.mag_state[sid] = "SUSPECT"
            self.mag_state_since[sid] = now
            info.update({
                "state_after": "SUSPECT",
                "state_since": now,
                "decision": "suspect_vector_change",
            })
        else:
            info.update({"state_after": "BAD", "decision": "stay_bad"})
        return info

    @staticmethod
    def _norm3(v: tuple[float, float, float]) -> float:
        return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

    def _mag_quality(self, sid_override: int | None = None) -> tuple[str, str, tuple[float, float, float] | None, float | None, float | None, float | None, float | None]:
        """Simple industry-style earth-field range monitor.

        Pose never depends on this result.  Bosch RV already performs the
        magnetic fusion internally; this panel only shows whether the
        corrected field magnitude looks like normal earth field.
        """
        sid = sid_override if sid_override is not None else self.current_sensor_id
        if sid is None or sid < 1 or sid > 20:
            candidates = sorted(k for k in self.mag_by_sensor if 1 <= k <= 20)
            sid = candidates[0] if candidates else None

        primary = self.mag_by_sensor.get(sid) if sid is not None else None
        if primary is None:
            return "无磁数据 / 指南针不可用", "#8c98a8", None, None, None, None, None

        secondary = self.mag_by_sensor.get(sid + 20) if sid is not None else None
        if secondary is not None:
            common = primary
            diff = tuple(primary[i] - secondary[i] for i in range(3))
            diff_dynamic = self._norm3(diff)
        else:
            common = primary
            diff_dynamic = None

        scale_label = self.mag_scale_var.get()
        if scale_label.startswith("1/4"):
            unit_scale = 0.25
        elif scale_label.startswith("1/16"):
            unit_scale = 1.0 / 16.0
        else:
            unit_scale = 1.0
        common = tuple(v * unit_scale for v in common)
        magnitude = self._norm3(common)

        norm_history = list(self.mag_norm_history.get(sid, deque()))[-80:]
        mag_noise = 0.0
        if len(norm_history) >= 4:
            mean_mag = sum(norm_history) / len(norm_history)
            mag_noise = math.sqrt(sum((v - mean_mag) ** 2 for v in norm_history) / len(norm_history)) * unit_scale

        lp_mag = self.mag_lp_norm.get(sid, magnitude / unit_scale) * unit_scale
        lp_mag_prev = self.mag_lp_norm_prev.get(sid, magnitude / unit_scale) * unit_scale
        jump = abs(lp_mag - lp_mag_prev)

        status, color = "磁场大小监测（仅显示）", "#8c98a8"

        return status, color, common, magnitude, mag_noise, diff_dynamic, jump
    def _emit_gate_event(
        self,
        *,
        source: str,
        phase: str,
        gate: str,
        meta: dict[str, Any],
        old_value: Any = None,
        new_value: Any = None,
        duration_ms: float | None = None,
        context: dict[str, Any] | None = None,
    ) -> None:
        self.trace.record_gate_event({
            "source": source,
            "phase": phase,
            "gate": gate,
            "trace_id": meta.get("trace_id"),
            "connection_id": meta.get("connection_id"),
            "rx_wall_time_ns": meta.get("rx_wall_time_ns"),
            "rx_monotonic_ns": meta.get("rx_monotonic_ns"),
            "old_value": old_value,
            "new_value": new_value,
            "duration_ms": duration_ms,
            "context": context or {},
        })

    def open_force_window(self) -> None:
        """Open the standalone 3D force monitor in another process."""
        script = Path(__file__).resolve().with_name("force_3d_ui_launcher.pyw")
        self.trace.record_event("ui", "open_force_window", {"script": str(script)})
        subprocess.Popen([sys.executable, str(script), "--no-auto-trace"], cwd=str(script.parent))

    def open_mag_diag_window(self) -> None:
        if self.mag_diag_window is not None and self.mag_diag_window.winfo_exists():
            self.mag_diag_window.deiconify()
            self.mag_diag_window.lift()
            self._refresh_mag_feature_text(force=True)
            return
        window = tk.Toplevel(self.root)
        self.mag_diag_window = window
        window.title("FIX30 磁修正门限 / 磁干扰诊断")
        window.geometry("860x760")
        window.minsize(700, 560)
        ttk.Label(
            window,
            textvariable=self.mag_gate_header_var,
            font=("Microsoft YaHei UI", 12),
            wraplength=480,
            justify=tk.LEFT,
            anchor=tk.W,
        ).pack(fill=tk.X, padx=12, pady=(12, 4))
        ttk.Label(
            window,
            textvariable=self.mag_recovery_var,
            font=("Microsoft YaHei UI", 16, "bold"),
            foreground="#39a66f",
            wraplength=520,
            justify=tk.LEFT,
            anchor=tk.W,
        ).pack(fill=tk.X, padx=12, pady=(0, 8))
        strength_row = ttk.Frame(window, padding=(12, 0, 12, 6))
        strength_row.pack(fill=tk.X)
        ttk.Label(strength_row, text="方向门控强度", font=("Microsoft YaHei UI", 11)).pack(side=tk.LEFT)
        self.mag_dir_strength_scale = ttk.Scale(
            strength_row, from_=0.0, to=95.0, orient=tk.HORIZONTAL,
            variable=self.mag_dir_strength_var, command=self._on_dir_strength_change,
        )
        self.mag_dir_strength_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(10, 10))
        ttk.Label(
            strength_row, textvariable=self.mag_dir_strength_readout_var,
            font=("Microsoft YaHei UI", 10), width=18, anchor=tk.W,
        ).pack(side=tk.LEFT)
        ttk.Label(
            window, textvariable=self.mag_dir_eff_var,
            font=("Microsoft YaHei UI", 11), foreground="#f0c75e",
            anchor=tk.W, justify=tk.LEFT,
        ).pack(fill=tk.X, padx=12, pady=(2, 6))
        text_frame = ttk.Frame(window, padding=(10, 0, 10, 10))
        text_frame.pack(fill=tk.BOTH, expand=True)
        feature_text = tk.Text(
            text_frame,
            wrap=tk.WORD,
            state=tk.DISABLED,
            font=("Microsoft YaHei UI", 12),
            spacing1=10,
            spacing3=10,
            padx=12,
            pady=12,
            background="#11161d",
            foreground="#dce5f0",
            insertbackground="#ffffff",
        )
        scrollbar = ttk.Scrollbar(text_frame, orient=tk.VERTICAL, command=feature_text.yview)
        feature_text.configure(yscrollcommand=scrollbar.set)
        feature_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        feature_text.tag_configure("active", foreground="#ff7070")
        feature_text.tag_configure("strong", foreground="#ff9f43")
        feature_text.tag_configure("warning", foreground="#f0c75e")
        feature_text.tag_configure("info", foreground="#8fb8e8")
        self.mag_feature_text = feature_text
        try:
            self._on_dir_strength_change(str(self.mag_dir_strength_var.get()))
        except Exception:
            pass
        window.protocol("WM_DELETE_WINDOW", self.close_mag_diag_window)
        self._refresh_mag_feature_text(force=True)

    def close_mag_diag_window(self) -> None:
        window = self.mag_diag_window
        self.mag_feature_text = None
        self.mag_diag_window = None
        if window is not None:
            try:
                window.destroy()
            except Exception:
                pass
    def _set_mag_feature_state(self, bit: int, active: bool, rx_ns: int, duration_ms: float | None = None) -> None:
        state = self.mag_feature_states.setdefault(
            bit,
            {"active": False, "count": 0, "last_enter": None, "last_exit": None, "duration_ms": None},
        )
        changed = state["active"] != active
        state["active"] = active
        if active:
            state["count"] += 1
            state["last_enter"] = rx_ns
        else:
            state["last_exit"] = rx_ns
            state["duration_ms"] = duration_ms
        if changed:
            self._refresh_mag_feature_text(force=True)

    def _mag_acc_fresh(self) -> bool:
        """姿态精度流是否新鲜（每帧四元数刷新，是面板的主判据）。"""
        return (time.monotonic() - float(getattr(self, "mag_accuracy_mono", 0.0))) <= self.MAG_ACC_FRESH_S

    def _mag_diag_fresh(self) -> bool:
        """固件磁诊断流是否新鲜（P 文本行 / YAW-YF 二进制帧）。"""
        return (time.monotonic() - float(getattr(self, "mag_diag_mono", 0.0))) <= self.MAG_DIAG_FRESH_S

    def _expire_mag_states(self) -> None:
        """诊断流一停就作废它带来的暂停标签。

        这是原实现最致命的缺陷：``mag_pause_active_labels`` 只要曾经非空就再也
        没人清，于是 ``pause_active`` 永远为真、``effective`` 永远为假，
        "磁干扰响应中"会永久粘在面板上。诊断数据过期即丢弃标签。
        """
        if not self._mag_diag_fresh() and self.mag_pause_active_labels:
            self.mag_pause_active_labels = []

    def _effective_mag_pause_labels(self) -> list[str]:
        """当前真正生效的暂停门控标签（实时层 + 新鲜的细节层）。"""
        labels: list[str] = []
        if self._mag_acc_fresh():
            for bit, (label, _description, _severity) in self.MAG_FEATURE_INFO.items():
                if self.mag_feature_states.get(bit, {}).get("active") and label not in labels:
                    labels.append(label)
        if self._mag_diag_fresh():
            for label in self.mag_pause_active_labels:
                if label not in labels:
                    labels.append(label)
        return labels

    @staticmethod
    def _dir_strength_text(pct: int) -> str:
        if pct <= 0:
            return "关闭"
        thr = 3.0 + (1.0 - pct / 100.0) * (20.0 - 3.0)
        return f"{pct}%  门限 {thr:.1f}°"

    def _send_dir_strength(self, pct: int, force: bool = False) -> None:
        pct = max(0, min(95, int(pct)))
        if not force and pct == self.mag_dir_strength_last_sent:
            return
        self.mag_dir_strength_last_sent = pct
        self._send_cmd(("T DIR %d\n" % pct).encode("ascii"), "set_dir_strength")

    def _on_dir_strength_change(self, value: str) -> None:
        try:
            pct = int(round(float(value)))
        except (TypeError, ValueError):
            return
        self._set_var_if_changed(self.mag_dir_strength_readout_var, self._dir_strength_text(pct))
        self._send_dir_strength(pct)

    def _refresh_mag_feature_text(self, force: bool = False) -> None:
        widget = self.mag_feature_text
        if widget is None or not widget.winfo_exists():
            return
        now = time.monotonic()
        if not force and now - self.mag_last_feature_ui_update < 0.2:
            return
        self.mag_last_feature_ui_update = now

        self._expire_mag_states()
        labels = self._effective_mag_pause_labels()

        widget.configure(state=tk.NORMAL)
        widget.delete("1.0", tk.END)
        shown: set[str] = set()
        for label in labels:
            summary = self.MAG_GATE_SUMMARY.get(label, label)
            if summary in shown:
                continue
            shown.add(summary)
            widget.insert(tk.END, f"{summary}\n", "active")
        widget.configure(state=tk.DISABLED)

    def _refresh_mag_panel_tick(self) -> None:
        """周期重算面板：没有新数据时也要走到"陈旧"，不能停在最后一帧。"""
        window = self.mag_diag_window
        if window is None:
            return
        try:
            if not window.winfo_exists():
                return
        except Exception:
            return
        self._refresh_mag_correction_state()
        # tick 必须强制重建文本：否则会被 0.2 s 节流挡住，过期标签清不掉。
        self._refresh_mag_feature_text(force=True)

    @staticmethod
    def _set_var_if_changed(var: tk.StringVar, value: str) -> None:
        value = str(value)
        if var.get() != value:
            var.set(value)
    def _refresh_mag_correction_state(
        self,
        status_code: int | None = None,
        weight: float | None = None,
        mask: int | None = None,
        rx_ns: int | None = None,
    ) -> None:
        """实时刷新"磁修正"面板状态。

        旧实现的问题：状态文字只由姿态精度流驱动，而暂停标签只由固件磁诊断流
        （默认 5 秒一次的 P 文本轮询）驱动，两条流互相看不见、且谁都不会过期。
        结果就是"面板不实时"：一边说已生效/已恢复，另一边还挂着上一帧的门控；
        或者反过来显示"磁干扰响应中"而结果窗口已经空了。

        现在改为：
          * 实时层 = 姿态精度包里的 8 位诊断掩码，每帧刷新，作为主判据；
          * 细节层 = 固件磁诊断标签，只在新鲜时并入，过期立刻丢弃；
          * 允许无参调用 —— 周期 tick 用最近一次精度包重算，所以数据停住时
            面板会明确走到"数据陈旧"，而不是把最后一帧永久冻结。
        """
        disturbance_bits = 0x04 | 0x08 | 0x10 | 0x20 | 0x40 | 0x80

        if status_code is None or weight is None or mask is None:
            last = self.last_mag_accuracy
            if last is None:
                self._set_var_if_changed(self.mag_correction_var, "磁修正：等待数据")
                self._set_var_if_changed(self.mag_gate_header_var, "磁修正：等待数据")
                return
            status_code, weight, mask, rx_ns = last
        else:
            rx_ns = int(rx_ns or 0)
            self.last_mag_accuracy = (int(status_code), float(weight), int(mask), rx_ns)
            self.mag_accuracy_mono = time.monotonic()

        self._expire_mag_states()
        acc_fresh = self._mag_acc_fresh()
        diag_fresh = self._mag_diag_fresh()
        pause_labels = self._effective_mag_pause_labels()

        mask_active = bool(int(mask) & disturbance_bits) and acc_fresh
        strong = (bool(int(mask) & 0x80) and acc_fresh) or ("强磁事件" in pause_labels)
        active = mask_active or bool(pause_labels)
        effective = (int(status_code) == 3) and (float(weight) >= 0.95) and not active
        status_name = self.MAG_STATUS_NAMES.get(int(status_code), str(status_code))

        if not acc_fresh and not diag_fresh:
            text = "磁修正：数据陈旧"
        elif pause_labels:
            text = "磁修正：暂停（" + "、".join(pause_labels) + "）"
        elif active:
            text = "磁修正：暂停（门控触发）"
        elif effective:
            text = "磁修正：已生效"
        elif int(status_code) == 5:
            text = "磁修正：暂停"
        elif int(status_code) == 4:
            text = "磁修正：降级"
        elif int(status_code) == 2:
            text = "磁修正：获取航向"
        else:
            text = f"磁修正：尚未生效（{status_name}）"
        self._set_var_if_changed(self.mag_correction_var, text)
        # 窗口顶部原来被写死成空串；现在让它可以显示同一份实时状态。
        self._set_var_if_changed(self.mag_gate_header_var, text)

        now_ns = time.monotonic_ns()
        if active:
            if not self.mag_disturbance_active:
                self.mag_disturbance_started_ns = rx_ns or now_ns
            self.mag_disturbance_active = True
            if strong:
                self.mag_disturbance_had_strong = True
            self.mag_recovery_candidate_since_ns = None
            self._set_var_if_changed(self.mag_recovery_var, "磁干扰响应中")
        else:
            if self.mag_disturbance_active:
                self.mag_disturbance_active = False
                self.mag_recovery_candidate_since_ns = rx_ns or now_ns
            if (
                not self.mag_disturbance_active
                and self.mag_recovery_candidate_since_ns is not None
                and effective
                and rx_ns
            ):
                stable_s = (rx_ns - self.mag_recovery_candidate_since_ns) / 1e9
                if stable_s >= 1.0:
                    if self.mag_disturbance_had_strong:
                        self._set_var_if_changed(self.mag_recovery_var, "强磁干扰已恢复")
                    else:
                        self._set_var_if_changed(self.mag_recovery_var, "磁干扰已退出")
                    self.trace.record_gate_event({
                        "source": "magnetic_recovery",
                        "phase": "recovered",
                        "gate": "disturbance_recovered",
                        "rx_monotonic_ns": rx_ns or now_ns,
                        "duration_ms": stable_s * 1000.0,
                        "context": {
                            "had_strong_event": self.mag_disturbance_had_strong,
                            "status": status_name,
                            "weight": float(weight),
                        },
                    })
                    self.mag_recovery_notice_until = time.time() + 12.0
                    self.mag_recovery_candidate_since_ns = None
                    self.mag_disturbance_had_strong = False
            if not acc_fresh and not diag_fresh:
                self._set_var_if_changed(self.mag_recovery_var, "磁数据中断")

        self.mag_correction_effective = effective
    def _track_accuracy_gates(
        self,
        row: dict[str, Any],
        mask: int,
        status_code: int,
        cal_result: int,
        weight: float,
        accuracy_raw: float,
    ) -> None:
        meta = self._row_trace_meta(row)
        rx_ns = int(meta.get("rx_monotonic_ns") or 0)
        # 先标记"姿态精度流刚刚刷新"，再做门控状态更新：否则改动某一位时
        # _set_mag_feature_state 触发的强制重画会因时间戳还是 0 而被判成陈旧、
        # 导致结果窗口比状态文字慢一拍。
        self.mag_accuracy_mono = time.monotonic()
        context = {
            "accuracy_raw": accuracy_raw,
            "diagnostic_mask": mask,
            "status_code": status_code,
            "status": self.MAG_STATUS_NAMES.get(status_code, str(status_code)),
            "cal_result": cal_result,
            "weight": weight,
        }
        if self.last_mag_gate_mask is None:
            if mask == 0:
                self._emit_gate_event(
                    source="accuracy", phase="initial_clear", gate="all_mag_gates",
                    meta=meta, old_value=None, new_value=0, context=context,
                )
            else:
                for bit, name in self.MAG_GATE_BITS.items():
                    if mask & bit:
                        self.last_mag_gate_since_ns[bit] = rx_ns
                        self._set_mag_feature_state(bit, True, rx_ns)
                        self._emit_gate_event(
                            source="accuracy", phase="initial_active", gate=name,
                            meta=meta, old_value=None, new_value=1, context=context,
                        )
        else:
            for bit, name in self.MAG_GATE_BITS.items():
                was = bool(self.last_mag_gate_mask & bit)
                now = bool(mask & bit)
                if was == now:
                    continue
                if now:
                    self.last_mag_gate_since_ns[bit] = rx_ns
                    self._set_mag_feature_state(bit, True, rx_ns)
                    self._emit_gate_event(
                        source="accuracy", phase="enter", gate=name,
                        meta=meta, old_value=0, new_value=1, context=context,
                    )
                else:
                    started = self.last_mag_gate_since_ns.pop(bit, None)
                    duration_ms = (rx_ns - started) / 1e6 if started and rx_ns else None
                    self._set_mag_feature_state(bit, False, rx_ns, duration_ms)
                    self._emit_gate_event(
                        source="accuracy", phase="exit", gate=name,
                        meta=meta, old_value=1, new_value=0,
                        duration_ms=duration_ms, context=context,
                    )
        if self.last_mag_status_code is None:
            self._emit_gate_event(
                source="accuracy", phase="initial", gate="mag_status",
                meta=meta, old_value=None,
                new_value=self.MAG_STATUS_NAMES.get(status_code, status_code),
                context=context,
            )
        elif status_code != self.last_mag_status_code:
            self._emit_gate_event(
                source="accuracy", phase="change", gate="mag_status",
                meta=meta,
                old_value=self.MAG_STATUS_NAMES.get(self.last_mag_status_code, self.last_mag_status_code),
                new_value=self.MAG_STATUS_NAMES.get(status_code, status_code),
                context=context,
            )
        if self.last_mag_cal_result is None:
            self._emit_gate_event(
                source="accuracy", phase="initial", gate="mag_cal_result",
                meta=meta, old_value=None, new_value=cal_result, context=context,
            )
        elif cal_result != self.last_mag_cal_result:
            self._emit_gate_event(
                source="accuracy", phase="change", gate="mag_cal_result",
                meta=meta, old_value=self.last_mag_cal_result,
                new_value=cal_result, context=context,
            )
        self.last_mag_gate_mask = mask
        self.last_mag_status_code = status_code
        self.last_mag_cal_result = cal_result
        self._refresh_mag_correction_state(status_code, weight, mask, rx_ns)
        self._refresh_mag_feature_text()

    def _track_firmware_gate_value(
        self,
        *,
        key: str,
        value: Any,
        source: str,
        payload: dict[str, Any],
        context: dict[str, Any] | None = None,
    ) -> None:
        meta = {
            "trace_id": None,
            "connection_id": payload.get("connection_id"),
            "rx_wall_time_ns": payload.get("rx_wall_time_ns"),
            "rx_monotonic_ns": payload.get("rx_monotonic_ns"),
        }
        previous = self.last_firmware_gate_values.get(key)
        if key not in self.last_firmware_gate_values:
            self._emit_gate_event(
                source=source, phase="initial", gate=key, meta=meta,
                old_value=None, new_value=value, context=context,
            )
        elif previous != value:
            self._emit_gate_event(
                source=source, phase="change", gate=key, meta=meta,
                old_value=previous, new_value=value, context=context,
            )
        self.last_firmware_gate_values[key] = value
        self._refresh_mag_feature_text()
    def _handle_firmware_line(self, payload: dict[str, Any]) -> None:
        line = str(payload.get("line") or "")
        parsed = payload.get("parsed") if isinstance(payload.get("parsed"), dict) else {"type": "text", "raw": line, "text": line}
        parsed_type = str(parsed.get("type") or "text")
        entry = {
            "line": line,
            "parsed": parsed,
            "connection_id": payload.get("connection_id"),
            "rx_wall_time_ns": payload.get("rx_wall_time_ns"),
            "rx_monotonic_ns": payload.get("rx_monotonic_ns"),
        }
        self.firmware_diag_latest = entry
        history_key = {
            "gq": "gq",
            "gy": "gy",
            "mag_detail": "mag_detail",
            "mag_status_row": "mag_status",
            "sinfo": "sinfo",
            "sinfo_error": "sinfo",
        }.get(parsed_type, "events")
        self.firmware_diag_history[history_key].append(entry)

        fields = parsed.get("fields") or {}
        if parsed_type == "mag_status_row" and fields:
            self._merge_mag_values(fields, prefix="p_")
            self._refresh_mag_live_gate_rows()
        if parsed_type == "gy":
            self._track_firmware_gate_value(
                key="gy_corrected_valid", value=bool(fields.get("corrected_valid_bool")),
                source="GY", payload=payload, context={"fields": fields},
            )
            self._track_firmware_gate_value(
                key="gy_motion_hold_active", value=bool(int(fields.get("motion_hold") or 0)),
                source="GY", payload=payload, context={"fields": fields},
            )
        elif parsed_type == "mag_detail":
            for key in ("learning_state", "revision", "fixes", "learning_opens"):
                self._track_firmware_gate_value(
                    key=f"mag_{key}", value=fields.get(key), source="MAG",
                    payload=payload, context={"fields": fields},
                )
        elif parsed_type == "mag_status_row":
            for key in ("status", "rejecting", "hard_event", "candidate_valid", "dip_valid", "model", "align", "learning_state"):
                self._track_firmware_gate_value(
                    key=f"p_{key}", value=fields.get(key), source="P",
                    payload=payload, context={"fields": fields},
                )
        if parsed_type == "gq":
            self.firmware_status_var.set(
                f"FW GQ f{fields.get('finger')} l{fields.get('link')} id{fields.get('sensor_id')} "
                f"acc{fields.get('accuracy_raw')} "
                f"q=({fields.get('quat_x'):.6f},{fields.get('quat_y'):.6f},"
                f"{fields.get('quat_z'):.6f},{fields.get('quat_w'):.6f}) "
                f"yawc={fields.get('yaw_corr_deg'):+.2f}°"
            )
        elif parsed_type == "gy":
            self.firmware_status_var.set(
                f"FW GY f{fields.get('finger')} l{fields.get('link')} "
                f"P={fields.get('passthrough')} C={fields.get('corrected')} "
                f"CV{fields.get('corrected_valid')} bias={fields.get('bias')} "
                f"ACT={fields.get('activity'):.1f} MH={fields.get('motion_hold')}"
            )
        elif parsed_type == "mag_detail":
            self.firmware_status_var.set(
                f"FW MAG f{fields.get('finger')} l{fields.get('link')} "
                f"RAW={fields.get('raw')} CAL={fields.get('cal')} OFF={fields.get('offset')} "
                f"REV{fields.get('revision')} FIX{fields.get('fixes')} "
                f"LRN{fields.get('learning_state')} OPENS{fields.get('learning_opens')} "
                f"datum={fields.get('datum_deg'):+.2f}° yawc={fields.get('yaw_corr_deg'):+.2f}°"
            )
        elif parsed_type == "mag_status_row":
            self.firmware_status_var.set(
                f"FW P link={fields.get('link')} status={fields.get('status')} "
                f"trust={fields.get('trust_pct')} model={fields.get('model')} "
                f"resid={fields.get('resid_pct')} align={fields.get('align')} "
                f"grad={fields.get('grad')}/{fields.get('grad_threshold')} "
                f"drift={fields.get('drift_mdeg_s')}mdeg/s "
                f"datum={fields.get('datum_deg')}° yawc={fields.get('yaw_corr_deg')}°"
            )
        elif parsed_type in ("sinfo", "sinfo_error"):
            self.firmware_status_var.set(f"FW SINFO {line}")
        elif parsed_type == "mag_event":
            self.firmware_status_var.set(f"FW {parsed.get('severity', 'event')}: {parsed.get('message', line)}")
        else:
            compact = line if len(line) <= 160 else line[:157] + "..."
            self.firmware_status_var.set(f"FW TEXT: {compact}")
    def _merge_mag_values(self, values: dict[str, Any], prefix: str = "") -> None:
        for key, value in values.items():
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(numeric):
                self.mag_diag_live_fields[f"{prefix}{key}"] = numeric

    def _handle_replay_row(self, row: dict[str, Any]) -> None:
        try:
            frame = rp.decode_replay_row(row)
        except Exception:
            return
        if frame is None:
            return
        if frame.kind in ("YF", "YAW", "YAW2", "HEALTH"):
            try:
                fields = rp.decode_diag_frame(frame)
            except Exception:
                return
            self._merge_mag_values(fields)
            self._refresh_mag_live_gate_rows()
        elif frame.kind == "CONFIG":
            if frame.index is None:
                return
            key = (int(frame.link or 0), int(frame.index) & 0xFFFFFFFF)
            self.mag_config_rows[key] = row
            try:
                blobs = rp.assemble_blobs(list(self.mag_config_rows.values()))
            except Exception:
                return
            for (kind, _link), blob in blobs.items():
                if kind != "CONFIG":
                    continue
                try:
                    config = rp.parse_config(blob)
                except Exception:
                    continue
                self.mag_config_meta = config
                self.mag_config_params = dict(config.get("params") or {})
                break

    def _refresh_mag_live_gate_rows(self) -> None:
        fields = self.mag_diag_live_fields
        params = self.mag_config_params
        has_p = all(key in fields for key in ("p_dip_valid", "p_e_norm_x1000", "p_e_dip_x1000", "p_e_step_x1000", "p_weight_pct"))
        has_yaw = all(key in fields for key in ("clean", "trust", "hold_s", "dir_res_deg", "yaw_valid"))
        if not (has_p and has_yaw):
            return
        now = time.monotonic()
        if self.mag_diag_live_rows and (now - self.mag_live_last_update) < 0.10:
            return
        self.mag_live_last_update = now

        def pick(names: tuple[str, ...], default: float | None = None, scale: float = 1.0) -> float | None:
            for name in names:
                if name not in fields:
                    continue
                try:
                    value = float(fields[name]) / scale
                except (TypeError, ValueError):
                    continue
                if math.isfinite(value):
                    return value
            return default

        def threshold(name: str, default: float) -> float:
            value = params.get(name, default)
            try:
                value = float(value)
            except (TypeError, ValueError):
                return default
            return value if math.isfinite(value) else default

        rows: list[dict[str, Any]] = []

        def add(
            label: str,
            current: str,
            limit: str,
            active: bool,
            severity: str = "warning",
            detail: str = "",
        ) -> None:
            rows.append({
                "label": label,
                "current": current,
                "limit": limit,
                "active": bool(active),
                "severity": severity,
                "detail": detail,
            })

        cal_raw = pick(("p_cal_valid", "cal_valid"), None)
        if cal_raw is None:
            cal_valid = (pick(("p_radius1",), 0.0) > 1.0) and (pick(("p_revision",), 0.0) > 0.0)
        else:
            cal_valid = cal_raw > 0.5
        ref_valid = (pick(("p_ref_dip_valid", "p_dip_valid", "ref_dip_valid"), 0.0) or 0.0) > 0.5
        e_norm = pick(("p_e_norm_x1000",), None, 1000.0)
        if e_norm is None:
            e_norm = pick(("e_norm",), 0.0)
        e_dip = pick(("p_e_dip_x1000",), None, 1000.0)
        if e_dip is None:
            e_dip = pick(("e_dip",), 0.0)
        e_step = pick(("p_e_step_x1000",), None, 1000.0)
        if e_step is None:
            e_step = pick(("e_step",), 0.0)
        e_drift = pick(("p_e_drift_x1000",), None, 1000.0)
        e_grad = pick(("p_e_grad_x1000",), None, 1000.0)
        if e_grad is None:
            grad = pick(("grad",), 0.0)
            grad_thr = pick(("grad_thr",), 0.0)
            e_grad = (grad / grad_thr) if grad_thr and grad_thr > 0.0 else None
        e_fleet = pick(("p_e_fleet_x1000",), None, 1000.0)
        relax = pick(("relax",), 1.0)
        weight = pick(("p_weight_pct",), None, 100.0)
        if weight is None:
            weight = pick(("weight",), 0.0)
        clean = pick(("clean",), 0.0)
        trust = pick(("trust",), 0.0)
        hold_s = pick(("hold_s",), 0.0)
        large = pick(("large",), 0.0)
        snap = pick(("snap",), 0.0)
        nis = pick(("nis",), 0.0)
        innov = pick(("innov_deg",), 0.0)
        dir_res = pick(("dir_res_deg",), 0.0)
        omega_dps = pick(("omega_dps",), 0.0)
        dirlock_ms = pick(("dirlock_ms",), 0.0)
        diag_hold = pick(("diag_hold",), 0.0)
        hard_event = pick(("p_hard_event", "hard_event"), 0.0)
        rejecting = pick(("p_rejecting", "rejecting"), 0.0)
        candidate = pick(("p_candidate_valid", "cand_valid"), 0.0)
        yaw_valid = pick(("yaw_valid",), 1.0)
        coast = pick(("coast",), 0.0)
        stuck_s = pick(("p_stuck_s",), 0.0)
        fe_obs = pick(("p_fe_obs_x1000",), None, 1000.0)
        if fe_obs is None:
            fe_obs = pick(("fe_obs",), 0.0)
        fe_res = pick(("p_fe_res_x1000",), None, 1000.0)
        if fe_res is None:
            fe_res = pick(("fe_res",), 0.0)

        yaw_innov_max = threshold("yaw_innov_max", 20.0)
        yaw_nis_max = threshold("yaw_nis_max", 9.0)
        dir_res_limit = threshold("dir_res_deg", 3.0)
        omega_limit = threshold("slew_omega_max_rad_s", 9.0) * 57.2957795
        fe_obs_limit = threshold("fe_obs_max", 0.8)
        fe_res_limit = threshold("fe_res_max", 0.25)

        add("磁标定", "有效" if cal_valid else "未建立", "必须有效", not cal_valid, "strong")
        add("参考磁 dip", "已建立" if ref_valid else "未建立", "必须建立", not ref_valid, "strong")
        add("磁场大小门限", f"{e_norm:.3f}", "1.000", e_norm >= 1.0, "disturbance")
        add("磁倾角门限", f"{e_dip:.3f}", "1.000", e_dip >= 1.0, "disturbance")
        add("单拍方向阶跃", f"{e_step:.3f}", f"{relax:.2f}", e_step >= relax, "disturbance", f"欠账越大，门限放宽到 {relax:.2f}")
        if e_grad is None:
            add("空间梯度", "N/A", "1.000", False, "info")
        else:
            add("空间梯度", f"{e_grad:.3f}", "1.000", e_grad >= 1.0, "disturbance")
        if e_fleet is None:
            add("多链路一致性", "单链路/未启用", "1.000", False, "info")
        else:
            add("多链路一致性", f"{e_fleet:.3f}", "1.000", e_fleet >= 1.0, "disturbance")
        if e_drift is None:
            add("航向漂移", "N/A", "1.000", stuck_s > 0.0 or weight <= 0.0, "warning", f"stuck={stuck_s:.1f}s")
        else:
            add("航向漂移", f"{e_drift:.3f}", "1.000", e_drift >= 1.0, "warning")
        add("诊断保持", f"{diag_hold:.0f}", "> 0", diag_hold > 0.0, "warning")
        add("强磁事件", "是" if hard_event > 0.5 else "否", "否", hard_event > 0.5, "strong")
        add("拒绝状态", "是" if rejecting > 0.5 else "否", "否", rejecting > 0.5, "strong")
        add("候选参考", "是" if candidate > 0.5 else "否", "否", candidate > 0.5, "strong")
        add("航向有效", "是" if yaw_valid > 0.5 else "否", "是", yaw_valid <= 0.5, "warning")
        add("磁修正权重", f"{weight:.1f}%", "> 0%", weight <= 0.0, "warning")
        if clean <= 0.5:
            add("clean 判定", "不通过", "通过", True, "warning", f"trust={trust:.2f}")
        add("方向一致性", f"{dir_res:.3f}°", f"{dir_res_limit:.3f}°", dir_res >= dir_res_limit, "disturbance")
        add("角速度上限", f"{omega_dps:.1f}°/s", f"{omega_limit:.1f}°/s", omega_dps > omega_limit, "warning")
        add("Yaw 新息", f"{innov:.2f}°", f"{yaw_innov_max:.2f}°", innov > yaw_innov_max, "warning")
        add("Yaw NIS", f"{nis:.2f}", f"{yaw_nis_max:.2f}", nis > yaw_nis_max, "warning")
        add("修正保持", f"{hold_s:.3f}s", "0 s", hold_s > 0.0, "warning")
        add("快速修正路径", "是" if large > 0.5 else "否", "否", large > 0.5, "warning")
        add("全速回正档", "是" if snap > 0.5 else "否", "否", snap > 0.5, "warning")
        add("方向锁定倒计时", f"{dirlock_ms:.0f}ms", "0 ms", dirlock_ms > 0.0, "warning")
        add("漂移外推", f"{coast:.3f}s", "0 s", coast > 0.0, "warning")
        add("FE 可观测性", f"{fe_obs:.4f}", f"{fe_obs_limit:.4f}", fe_obs > fe_obs_limit, "warning")
        add("FE 残差", f"{fe_res:.4f}", f"{fe_res_limit:.4f}", fe_res > fe_res_limit, "warning")

        dir_eff = pick(("dir_strength_eff",), None)
        if dir_eff is not None:
            try:
                up = int(round(float(self.mag_dir_strength_var.get())))
            except Exception:
                up = 0
            self._set_var_if_changed(
                self.mag_dir_eff_var,
                f"方向门强度：生效 {dir_eff * 100.0:.0f}%　静止上限 {up}%",
            )
        self.mag_diag_live_rows = rows
        self.mag_diag_mono = time.monotonic()
        pause_labels = [
            row["label"] for row in rows
            if row["active"] and row["label"] in self.MAG_PAUSE_GATE_LABELS
        ]
        self.mag_pause_active_labels = pause_labels
        # 诊断流一到就同步重算面板。旧代码只更新窗口、不重算状态文字，
        # 于是"文字说已生效、窗口还挂着旧门控"这类不同步会一直存在。
        self._refresh_mag_correction_state()
        self._refresh_mag_feature_text(force=True)

    def _process_event(self, event: str, payload: Any) -> None:
        if event == "firmware_line":
            self._handle_firmware_line(payload if isinstance(payload, dict) else {"line": str(payload)})
        elif event == "replay":
            row = payload if isinstance(payload, dict) else {}
            self._handle_replay_row(row)
        elif event in ("quat", "mag", "row"):
            row = payload if isinstance(payload, dict) else {}
            meta = self._row_trace_meta(row)
            latency_ms = None
            if meta.get("rx_monotonic_ns") is not None:
                latency_ms = (time.monotonic_ns() - int(meta["rx_monotonic_ns"])) / 1e6
            index = row.get("index")
            sid = row.get("sensor_id")
            key = (int(sid or 0), event)
            if index is not None:
                try:
                    index_int = int(index)
                    previous = self.pipeline_last_index.get(key)
                    if previous == index_int:
                        self.pipeline_counts["duplicate_index"] += 1
                        self._pipeline_anomaly(
                            "duplicate_index",
                            f"kind={event} sensor={sid} index={index_int}",
                        )
                    self.pipeline_last_index[key] = index_int
                except (TypeError, ValueError):
                    pass
            self.pipeline_last_state = {
                "trace_id": meta.get("trace_id"),
                "kind": event,
                "sensor_id": sid,
                "index": index,
                "queue_latency_ms": latency_ms,
                "frame_count": self.frame_count,
                "current_sensor_id": self.current_sensor_id,
                "current_accuracy": self.current_accuracy,
                "current_quaternion_wxyz": list(self.current_q),
                "reference_quaternion_wxyz": list(self.reference_q),
                "quat_motion_deg_s": self.quat_motion_deg_s,
                "mag_state": dict(self.mag_state),
                "mag_diff_baseline": {
                    str(k): list(v) for k, v in self.mag_diff_baseline.items()
                },
            }
            if event == "quat":
                self._handle_quat(row)
            elif event == "mag":
                self._handle_mag(row)
            else:
                self._record_stage("unclassified_row", row, accepted=False)
                self._record_analysis(row, accepted=False, drop_reason="unclassified_packet")
            self._pipeline_flush(time.monotonic())
        elif event == "status":
            self.status_var.set(str(payload))
            if str(payload) == "Disconnected":
                self.reader = None
                self.connect_button.configure(text="连接")
        elif event == "decode_error":
            self.pipeline_counts["decode_error"] += 1
            detail = payload if isinstance(payload, dict) else {"error": str(payload)}
            self._pipeline_anomaly("decode_error", str(detail))
        elif event == "error":
            self.status_var.set(str(payload))
            self.trace.record_event("system", "fatal_error", {"error": str(payload)})
            messagebox.showerror("OSMO Pose", str(payload))
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

    def _poll_events(self) -> None:
        self._drain_events(60)
        next_delay = 5 if not self.events.empty() else 20
        self.root.after(next_delay, self._poll_events)

    def _relative_q(self) -> Quat:
        return q_multiply(q_conjugate(self.reference_q), self.current_q)

    def _display_vector(self, v: tuple[float, float, float]) -> tuple[float, float, float]:
        """Map the BHI360 earth frame (Z-up) into the screen frame (Y-up).

        The important part is world Z -> screen Y.  Without this permutation,
        physical yaw appears as rotation around the screen depth axis.
        """
        x, y, z = v
        mode = self.axis_mode_var.get()
        if mode.startswith("Z"):
            return x, z, -y
        if mode.startswith("X"):
            return -z, x, y
        return x, y, z

    def _project(self, point: tuple[float, float, float], width: int, height: int) -> tuple[float, float, float]:
        # Orthographic projection: no perspective scale, so opposite sides stay
        # parallel and the cube does not show a smaller rear face at rest.
        x, y, z = point
        size = min(width, height) * 0.24
        return width * 0.5 + x * size, height * 0.5 - y * size, z

    def rotated_vertices(self) -> list[tuple[float, float, float]]:
        q = self._relative_q()
        return [self._display_vector(q_rotate(q, v)) for v in self.VERTICES]

    def cube_screen_points(self, width: int, height: int) -> list[tuple[float, float, float]]:
        return [self._project(v, width, height) for v in self.rotated_vertices()]

    def _draw_axes(self, width: int, height: int, q: Quat) -> None:
        origin = self._project(self._display_vector(q_rotate(q, (0.0, 0.0, 0.0))), width, height)
        axes = [
            ((1.55, 0.0, 0.0), "#ff5b5b", "X"),
            ((0.0, 1.55, 0.0), "#55e67b", "Y"),
            ((0.0, 0.0, 1.55), "#5b8cff", "Z"),
        ]
        for vec, color, label in axes:
            end = self._project(self._display_vector(q_rotate(q, vec)), width, height)
            self.canvas.create_line(origin[0], origin[1], end[0], end[1], fill=color, width=3)
            self.canvas.create_text(end[0] + 10, end[1] - 10, text=label, fill=color, font=("Arial", 12, "bold"))

    def _draw_compass(self) -> None:
        now = time.monotonic()
        if now - self.last_compass_render < 0.05:
            return
        self.last_compass_render = now
        c = self.compass_canvas
        c.delete("all")
        width = 240
        height = 240
        cx, cy, radius = width / 2, height / 2, 86

        quality, color, primary, magnitude, noise, delta, jump = self._mag_quality()
        self.mag_quality_var.set(quality)

        device_yaw = q_yaw_deg(self.current_q) % 360.0
        dial_rot = -device_yaw

        def polar(angle_deg: float, r: float) -> tuple[float, float]:
            rad = math.radians(angle_deg + dial_rot)
            return cx + math.sin(rad) * r, cy - math.cos(rad) * r

        c.create_oval(
            cx - radius, cy - radius, cx + radius, cy + radius,
            outline=color, width=4, fill="#0d1117",
        )
        for angle in range(0, 360, 15):
            inner = radius - (10 if angle % 90 == 0 else 5)
            outer = radius - 1
            x1, y1 = polar(angle, inner)
            x2, y2 = polar(angle, outer)
            c.create_line(x1, y1, x2, y2, fill="#8993a3", width=1)
        for angle, text, text_color in ((0, "N", "#ef6461"), (90, "E", "#e6ebf2"),
                                         (180, "S", "#e6ebf2"), (270, "W", "#e6ebf2")):
            x, y = polar(angle, radius - 14)
            c.create_text(x, y, text=text, fill=text_color, font=("Arial", 11, "bold"))
        c.create_polygon(cx, cy - radius - 7, cx - 5, cy - radius + 1, cx + 5, cy - radius + 1,
                         fill="#f0c75e", outline="")

        if primary is not None and self.current_sensor_id is not None:
            world_field = q_rotate(self.current_q, primary)
            heading = math.atan2(world_field[1], world_field[0])
            heading_deg = math.degrees(heading)
            self._update_mag_pointer_reference(heading_deg, time.monotonic())
            zero_deg = self.mag_pointer_zero if self.mag_pointer_zero is not None else heading_deg
            display_heading = math.radians(wrap_deg(heading_deg - zero_deg))
            dx = math.sin(display_heading)
            dy = -math.cos(display_heading)
            c.create_line(
                cx - dx * radius * 0.30, cy - dy * radius * 0.30,
                cx + dx * radius * 0.78, cy + dy * radius * 0.78,
                fill="#ef6461", width=4, arrow=tk.LAST,
            )
            c.create_line(
                cx + dx * radius * 0.18, cy + dy * radius * 0.18,
                cx - dx * radius * 0.60, cy - dy * radius * 0.60,
                fill="#e6ebf2", width=2,
            )
            c.create_text(cx, cy + 25,
                          text=f"MagΔ {wrap_deg(heading_deg - zero_deg):+.0f}°",
                          fill="#cdd5e0", font=("Consolas", 9))
        else:
            c.create_text(cx, cy, text="NO MAG", fill="#8c98a8", font=("Arial", 13, "bold"))

        c.create_text(cx, cy - 27, text=f"Yaw {device_yaw:5.1f}°",
                      fill="#f0c75e", font=("Consolas", 10, "bold"))

        if magnitude is None:
            self.mag_detail_var.set("|B|=-  noise=-  Δdiff=-")
        else:
            delta_text = "-" if delta is None else f"{delta:.1f}"
            self.mag_detail_var.set(
                f"|B|={magnitude:.1f}  noise|B|={noise:.2f}\n"
                f"Δdiff={delta_text}  Δ|B|={jump:.1f}"
            )

    def _render(self) -> None:
        _now = time.monotonic()
        if _now - self._mag_panel_last_tick >= 0.25:
            self._mag_panel_last_tick = _now
            self._refresh_mag_panel_tick()
        width = max(2, self.canvas.winfo_width())
        height = max(2, self.canvas.winfo_height())
        self.canvas.delete("all")
        self.canvas.create_text(
            18, 18, anchor="nw",
            text="Orthographic view  |  Space = calibrate  |  rotate sensor  |  X/Y/Z keys = demo",
            fill="#aab2c0", font=("Arial", 10),
        )

        q = self._relative_q()
        vertices = self.rotated_vertices()
        points = [self._project(v, width, height) for v in vertices]

        faces = []
        for indices, color in self.FACES:
            depth = sum(points[i][2] for i in indices) / len(indices)
            faces.append((depth, indices, color))
        faces.sort(key=lambda item: item[0])
        for _depth, indices, color in faces:
            coords = []
            for i in indices:
                coords.extend((points[i][0], points[i][1]))
            self.canvas.create_polygon(coords, fill=color, outline="#080a0d", width=2)

        for a, b in self.EDGES:
            self.canvas.create_line(points[a][0], points[a][1], points[b][0], points[b][1], fill="#e8edf5", width=2)
        self._draw_axes(width, height, q)
        self._draw_compass()

        if self.current_sensor_id is None:
            self.canvas.create_text(width * 0.5, height - 32, text="等待传感器四元数...", fill="#aab2c0", font=("Arial", 12))
        self.root.after(33, self._render)

    def on_close(self) -> None:
        self.trace.record_event("ui", "window_close_requested", {})
        self.stop_event.set()
        if self.reader is not None:
            self.reader.join(timeout=1.0)
        self.reader = None
        self.stop_trace_recording("window_close")
        self.root.destroy()


def run_self_test() -> int:
    qx = q_from_euler_deg(90.0, 0.0, 0.0)
    qy = q_from_euler_deg(0.0, 90.0, 0.0)
    qz = q_from_euler_deg(0.0, 0.0, 90.0)
    vx = q_rotate(qx, (0.0, 1.0, 0.0))
    vy = q_rotate(qy, (0.0, 0.0, 1.0))
    vz = q_rotate(qz, (1.0, 0.0, 0.0))
    assert abs(vx[2] - 1.0) < 1e-6, vx
    assert abs(vy[0] - 1.0) < 1e-6, vy
    assert abs(vz[1] - 1.0) < 1e-6, vz

    root = tk.Tk()
    root.withdraw()
    app = PoseMonitor(root, auto_connect=False)
    app.set_quaternion(0.0, 0.0, 0.0, 1.0, sensor_id=1)
    app.calibrate()
    p0 = app.cube_screen_points(800, 600)

    app.set_quaternion(math.sin(math.radians(45)), 0.0, 0.0, math.cos(math.radians(45)), sensor_id=1)
    p1 = app.cube_screen_points(800, 600)
    app.set_quaternion(0.0, math.sin(math.radians(45)), 0.0, math.cos(math.radians(45)), sensor_id=1)
    p2 = app.cube_screen_points(800, 600)
    app.set_quaternion(0.0, 0.0, math.sin(math.radians(45)), math.cos(math.radians(45)), sensor_id=1)
    p3 = app.cube_screen_points(800, 600)

    def changed(a, b):
        return any(abs(x[0] - y[0]) > 1.0 or abs(x[1] - y[1]) > 1.0 for x, y in zip(a, b))

    assert changed(p0, p1)
    assert changed(p1, p2)
    assert changed(p2, p3)

    # BHI360 earth frame is Z-up. A physical yaw (rotation around world Z)
    # must become a screen rotation around the vertical screen axis (Y).
    yaw_vec = app._display_vector(q_rotate(qz, (1.0, 0.0, 0.0)))
    assert abs(yaw_vec[0]) < 1e-6 and abs(yaw_vec[1]) < 1e-6
    assert abs(yaw_vec[2] + 1.0) < 1e-6

    app.on_close()
    print("POSE_SELF_TEST_PASS=True")
    print("X_AXIS_ROTATION_OK=True")
    print("Y_AXIS_ROTATION_OK=True")
    print("Z_AXIS_ROTATION_OK=True")
    print("YAW_MAPS_TO_SCREEN_VERTICAL_AXIS=True")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true", help="run math/rendering checks and exit")
    parser.add_argument("--no-auto-trace", action="store_true", help="do not start a trace session on launch")
    args = parser.parse_args()
    if args.self_test:
        return run_self_test()
    root = tk.Tk()
    PoseMonitor(root, auto_connect=True, auto_trace=not args.no_auto_trace)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

































