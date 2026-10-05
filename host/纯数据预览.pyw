#!/usr/bin/env python3
"""OSMO 纯数据预览 - 解码 COBS/protobuf 数据流的只读上位机。"""

from __future__ import annotations

import csv
import queue
import threading
import time
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports

import read_osmo_glove as glove
from firmware_profile import latest_known_firmware
from serial_trace_reader import TracedSerialReader as SerialReader
from trace_recorder import TraceRecorder


CSV_FIELDS = [
    "host_time",
    "index",
    "finger",
    "link",
    "sensor_id",
    "kind",
    "mag_x",
    "mag_y",
    "mag_z",
    "quat_x",
    "quat_y",
    "quat_z",
    "quat_w",
    "quat_accuracy",
]



class MonitorApp:
    def __init__(self, root: tk.Tk, auto_connect: bool = True, auto_trace: bool = False):
        self.root = root
        self.root.title("OSMO 纯数据预览")
        self.root.geometry("1100x720")
        self.root.minsize(820, 520)

        self.events: queue.Queue = queue.Queue()
        self.stop_event = threading.Event()
        self.reader: SerialReader | None = None
        self.rate_times: deque[float] = deque()
        self.total_frames = 0

        self.record_file = None
        self.record_writer = None
        self.record_path: Path | None = None
        self.trace = TraceRecorder("data_preview", __file__)
        self.trace_root = Path(__file__).resolve().parent / "recordings"
        self.auto_trace = bool(auto_trace)

        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Ready")
        self.stats_var = tk.StringVar(value="Frames: 0    Rate: 0.0 Hz")
        self.latest_var = tk.StringVar(value="Latest MAG: -    Latest QUAT: -")
        self.show_mag_var = tk.BooleanVar(value=True)
        self.show_quat_var = tk.BooleanVar(value=True)
        self.show_meta_var = tk.BooleanVar(value=True)
        self.trace_status_var = tk.StringVar(value="追踪：未录制")
        self.firmware_status_var = tk.StringVar(value="固件诊断：等待文本（FIX20生产版可能关闭GQ/GY）")
        self.firmware_diag_latest: dict = {}
        self.firmware_lines: deque[dict] = deque(maxlen=5000)

        self._build_ui()
        self.refresh_ports()
        self.root.after(50, self._poll_events)
        if self.auto_trace:
            self.root.after(120, self.start_auto_trace)
        if auto_connect:
            self.root.after(300, self.auto_connect)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=(10, 8))
        top.pack(fill=tk.X)

        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=18, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT, padx=2)
        self.connect_button = ttk.Button(top, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=(10, 4))

        ttk.Checkbutton(top, text="MAG", variable=self.show_mag_var).pack(side=tk.LEFT, padx=(18, 2))
        ttk.Checkbutton(top, text="QUAT", variable=self.show_quat_var).pack(side=tk.LEFT, padx=2)
        ttk.Checkbutton(top, text="META", variable=self.show_meta_var).pack(side=tk.LEFT, padx=2)

        actions = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        actions.pack(fill=tk.X)
        ttk.Button(actions, text="Clear", command=self.clear_text).pack(side=tk.LEFT)
        ttk.Button(actions, text="Save text...", command=self.save_text).pack(side=tk.LEFT, padx=6)
        self.record_button = ttk.Button(actions, text="Start CSV", command=self.toggle_recording)
        self.record_button.pack(side=tk.LEFT, padx=6)
        self.trace_button = ttk.Button(actions, text="开始追踪录制", command=self.toggle_trace_recording)
        self.trace_button.pack(side=tk.LEFT, padx=6)
        ttk.Label(actions, textvariable=self.status_var).pack(side=tk.RIGHT)

        info = ttk.Frame(self.root, padding=(10, 0, 10, 6))
        info.pack(fill=tk.X)
        ttk.Label(info, textvariable=self.stats_var).pack(side=tk.LEFT)
        ttk.Label(info, textvariable=self.trace_status_var).pack(side=tk.RIGHT, padx=(12, 0))
        ttk.Label(info, textvariable=self.latest_var).pack(side=tk.RIGHT)

        fw_info = ttk.Frame(self.root, padding=(10, 0, 10, 6))
        fw_info.pack(fill=tk.X)
        ttk.Label(fw_info, textvariable=self.firmware_status_var, anchor="w").pack(fill=tk.X)

        text_frame = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        text_frame.pack(fill=tk.BOTH, expand=True)
        self.text = tk.Text(
            text_frame,
            state=tk.DISABLED,
            wrap=tk.NONE,
            font=("Consolas", 10),
            background="#111111",
            foreground="#e8e8e8",
            insertbackground="#ffffff",
        )
        self.text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar = ttk.Scrollbar(text_frame, orient=tk.VERTICAL, command=self.text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.text.configure(yscrollcommand=scrollbar.set)

        bottom = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        bottom.pack(fill=tk.X)
        ttk.Label(
            bottom,
            text="Read-only host app. It decodes the existing COBS/protobuf firmware; no firmware modification is required.",
        ).pack(side=tk.LEFT)

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
        try:
            session = self.trace.start_session(parent_dir, metadata={
                "port": self.port_var.get(),
                "show_mag": self.show_mag_var.get(),
                "show_quat": self.show_quat_var.get(),
                "show_meta": self.show_meta_var.get(),
                "latest_known_firmware": latest_known_firmware(),
            })
        except Exception as exc:
            self.trace_status_var.set(f"追踪启动失败：{exc}")
            messagebox.showerror("OSMO Monitor", f"追踪启动失败：{exc}")
            return
        self.trace_button.configure(text="停止追踪录制")
        self.trace_status_var.set(f"追踪中：{session}")
        self.status_var.set(f"追踪记录：{session}")

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
        values = [port.device for port in ports]
        self.port_combo["values"] = values
        if not values:
            self.port_var.set("")
            self.status_var.set("No serial ports found")
            return

        preferred = next(
            (p.device for p in ports if p.vid == 0x2833 and p.pid == 0xB015),
            None,
        )
        current = self.port_var.get()
        if current in values:
            return
        self.port_var.set(preferred or values[0])

    def auto_connect(self) -> None:
        port = self.port_var.get().strip()
        if port and self.reader is None:
            self.connect()

    def toggle_connection(self) -> None:
        if self.reader is not None and self.reader.is_alive():
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("OSMO Monitor", "No COM port selected.")
            return

        self.stop_event.clear()
        self.trace.record_event("ui", "connect_requested", {"port": port})
        self.reader = SerialReader(port, self.events, self.stop_event, self.trace, typed_output=False)
        self.reader.start()
        self.connect_button.configure(text="Disconnect")
        self.status_var.set(f"Opening {port} ...")

    def disconnect(self) -> None:
        self.trace.record_event("ui", "disconnect_requested", {"port": self.port_var.get()})
        self.stop_event.set()
        if self.reader is not None:
            self.reader.join(timeout=1.0)
        self.reader = None
        self.connect_button.configure(text="Connect")
        self.status_var.set("Disconnected")

    def _append_line(self, line: str) -> None:
        self.text.configure(state=tk.NORMAL)
        self.text.insert(tk.END, line + "\n")
        self.text.see(tk.END)
        self.text.configure(state=tk.DISABLED)

    def _row_line(self, row: dict) -> str:
        stamp = time.strftime("%H:%M:%S", time.localtime(float(row["host_time"])))
        kind = row.get("kind", "UNKNOWN")
        common = (
            f"{stamp} {kind:<8} index={row.get('index')} "
            f"finger={row.get('finger')} link={row.get('link')} sensor_id={row.get('sensor_id')}"
        )
        if row.get("mag_x") is not None:
            return f"{common} x={row.get('mag_x')} y={row.get('mag_y')} z={row.get('mag_z')}"
        if row.get("quat_x") is not None:
            return (
                f"{common} x={row.get('quat_x')} y={row.get('quat_y')} "
                f"z={row.get('quat_z')} w={row.get('quat_w')} accuracy={row.get('quat_accuracy')}"
            )
        return common

    def _record_row_trace(self, row: dict) -> None:
        meta = row.get("_trace") or {}
        packet = row.get("packet") or {}
        mag = packet.get("mag") or {} if isinstance(packet, dict) else {}
        quat = packet.get("quat") or {} if isinstance(packet, dict) else {}
        latency_ms = None
        if meta.get("rx_monotonic_ns") is not None:
            latency_ms = (time.monotonic_ns() - int(meta["rx_monotonic_ns"])) / 1e6
        self.trace.record_stage("preview_row", {
            "trace_id": meta.get("trace_id"),
            "connection_id": meta.get("connection_id"),
            "sequence": meta.get("sequence"),
            "rx_wall_time_ns": meta.get("rx_wall_time_ns"),
            "rx_monotonic_ns": meta.get("rx_monotonic_ns"),
            "kind": row.get("kind"),
            "index": row.get("index"),
            "sensor_id": row.get("sensor_id"),
            "displayed": self._should_display(row),
            "formatted_line": self._row_line(row),
            "queue_depth": self.events.qsize(),
        })
        self.trace.record_analysis({
            "trace_id": meta.get("trace_id"),
            "connection_id": meta.get("connection_id"),
            "sequence": meta.get("sequence"),
            "rx_wall_time_ns": meta.get("rx_wall_time_ns"),
            "rx_wall_time": row.get("host_time"),
            "rx_monotonic_ns": meta.get("rx_monotonic_ns"),
            "processing_monotonic_ns": time.monotonic_ns(),
            "processing_latency_ms": latency_ms,
            "record_kind": row.get("kind"),
            "packet_len": meta.get("packet_len"),
            "decode_ok": True,
            "source": "serial",
            "index": row.get("index"),
            "finger": row.get("finger"),
            "finger_id": row.get("finger_id"),
            "link": row.get("link"),
            "sensor_id": row.get("sensor_id"),
            "kind": row.get("kind"),
            "mag_seconds": mag.get("seconds"),
            "mag_nanoseconds": mag.get("nanoseconds"),
            "quat_seconds": quat.get("seconds"),
            "quat_nanoseconds": quat.get("nanoseconds"),
            "mag_x_raw": row.get("mag_x"),
            "mag_y_raw": row.get("mag_y"),
            "mag_z_raw": row.get("mag_z"),
            "quat_x_raw": row.get("quat_x"),
            "quat_y_raw": row.get("quat_y"),
            "quat_z_raw": row.get("quat_z"),
            "quat_w_raw": row.get("quat_w"),
            "quat_accuracy_raw": row.get("quat_accuracy"),
            "accepted": True,
            "queue_depth": self.events.qsize(),
        })
    def _should_display(self, row: dict) -> bool:
        kind = row.get("kind")
        if kind == "QUAT":
            return self.show_quat_var.get()
        if kind == "MAG":
            return self.show_mag_var.get()
        if kind == "MAG_META":
            return self.show_meta_var.get()
        return True

    def _handle_row(self, row: dict) -> None:
        now = time.monotonic()
        self.rate_times.append(now)
        while self.rate_times and now - self.rate_times[0] > 1.0:
            self.rate_times.popleft()
        self.total_frames += 1

        self.stats_var.set(f"Frames: {self.total_frames}    Rate: {len(self.rate_times):.1f} Hz")

        if row.get("kind") == "MAG" and row.get("sensor_id") == 1:
            self.latest_var.set(
                f"Latest MAG: ({row.get('mag_x')}, {row.get('mag_y')}, {row.get('mag_z')})"
            )
        elif row.get("kind") == "QUAT":
            self.latest_var.set(
                f"Latest QUAT: ({row.get('quat_x'):.4f}, {row.get('quat_y'):.4f}, "
                f"{row.get('quat_z'):.4f}, {row.get('quat_w'):.4f})"
            )

        if self._should_display(row):
            self._append_line(self._row_line(row))

        self._record_row_trace(row)

        if self.record_writer is not None:
            self.record_writer.writerow({field: row.get(field, "") for field in CSV_FIELDS})
            self.record_file.flush()

    def _handle_firmware_line(self, payload: dict) -> None:
        line = str(payload.get("line") or "")
        parsed = payload.get("parsed") if isinstance(payload.get("parsed"), dict) else {"type": "text", "raw": line}
        parsed_type = str(parsed.get("type") or "text")
        entry = {
            "line": line,
            "parsed": parsed,
            "connection_id": payload.get("connection_id"),
            "rx_wall_time_ns": payload.get("rx_wall_time_ns"),
            "rx_monotonic_ns": payload.get("rx_monotonic_ns"),
        }
        self.firmware_diag_latest = entry
        self.firmware_lines.append(entry)
        label = {
            "gq": "GQ",
            "gy": "GY",
            "mag_detail": "MAG",
            "mag_status_row": "P",
            "mag_status_header": "P-HEADER",
            "sinfo": "SINFO",
            "sinfo_error": "SINFO-ERR",
            "mag_event": "MAG-EVENT",
        }.get(parsed_type, "TEXT")
        self.firmware_status_var.set(f"FW {label}: {line}")
        if parsed_type != "mag_status_header":
            self._append_line(f"FW {line}")
    def _process_event(self, event: str, payload: object) -> None:
        if event == "firmware_line":
            self._handle_firmware_line(payload if isinstance(payload, dict) else {"line": str(payload)})
        elif event == "row":
            self._handle_row(payload)
        elif event == "status":
            self.status_var.set(str(payload))
            if str(payload) == "Disconnected":
                self.reader = None
                self.connect_button.configure(text="Connect")
        elif event == "decode_error":
            detail = payload if isinstance(payload, dict) else {"error": str(payload)}
            self.trace.record_event("anomaly", "decode_error", detail)
            self.status_var.set(f"Decode error: {detail.get('error')}")
        elif event == "error":
            self.status_var.set(str(payload))
            self.trace.record_event("system", "fatal_error", {"error": str(payload)})
            messagebox.showerror("OSMO Monitor", str(payload))
            self.stop_event.set()
            self.reader = None
            self.connect_button.configure(text="Connect")

    def _drain_events(self) -> int:
        processed = 0
        while True:
            try:
                event, payload = self.events.get_nowait()
            except queue.Empty:
                return processed
            self._process_event(event, payload)
            processed += 1

    def _poll_events(self) -> None:
        self._drain_events()

        if self.trace.is_recording:
            trace_info = self.trace.stats()
            self.trace_status_var.set(
                f"追踪：写入={trace_info.get('written', 0)}  "
                f"队列={trace_info.get('queue_depth', 0)}  "
                f"丢弃={trace_info.get('dropped', 0)}"
            )
        for error in self.trace.pop_errors():
            self.trace.record_event("anomaly", "trace_recorder", error)
        self.root.after(50, self._poll_events)

    def clear_text(self) -> None:
        self.text.configure(state=tk.NORMAL)
        self.text.delete("1.0", tk.END)
        self.text.configure(state=tk.DISABLED)

    def save_text(self) -> None:
        content = self.text.get("1.0", tk.END).strip()
        if not content:
            return
        path = filedialog.asksaveasfilename(
            title="Save text",
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            initialfile="osmo_data.txt",
        )
        if not path:
            return
        Path(path).write_text(content + "\n", encoding="utf-8")
        self.status_var.set(f"Saved: {path}")

    def toggle_recording(self) -> None:
        self._record_row_trace(row)

        if self.record_writer is not None:
            self.record_file.close()
            self.record_file = None
            self.record_writer = None
            self.record_button.configure(text="Start CSV")
            self.status_var.set(f"CSV stopped: {self.record_path}")
            return

        path = filedialog.asksaveasfilename(
            title="Start CSV recording",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile=time.strftime("osmo_%Y%m%d_%H%M%S.csv"),
        )
        if not path:
            return
        self.record_path = Path(path)
        self.record_file = self.record_path.open("w", newline="", encoding="utf-8")
        self.record_writer = csv.DictWriter(self.record_file, fieldnames=CSV_FIELDS, extrasaction="ignore")
        self.record_writer.writeheader()
        self.record_file.flush()
        self.record_button.configure(text="Stop CSV")
        self.status_var.set(f"CSV recording: {self.record_path}")

    def on_close(self) -> None:
        self.trace.record_event("ui", "window_close_requested", {})
        self.stop_event.set()
        if self.reader is not None:
            self.reader.join(timeout=1.0)
        self.reader = None
        self.stop_trace_recording("window_close")
        if self.record_file is not None:
            self.record_file.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    MonitorApp(root, auto_trace=True)
    root.mainloop()


if __name__ == "__main__":
    main()
