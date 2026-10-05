#!/usr/bin/env python3
"""Serial COBS reader with complete raw-frame tracing."""

from __future__ import annotations

import queue
import threading
import time
from typing import Any

import serial
from serial.tools import list_ports

import read_osmo_glove as glove
from firmware_diagnostics import parse_firmware_line
from replay_protocol import kind_for_sensor
from trace_recorder import TraceRecorder


class TracedSerialReader(threading.Thread):
    """Read COBS frames and preserve raw bytes plus decode/system context."""

    def __init__(
        self,
        port: str,
        output: queue.Queue,
        stop_event: threading.Event,
        trace: TraceRecorder,
        typed_output: bool = True,
    ):
        super().__init__(daemon=True)
        self.port = port
        self.output = output
        self.stop_event = stop_event
        self.trace = trace
        self.typed_output = bool(typed_output)
        self.ser: serial.Serial | None = None
        self.connection_id = 0
        self.sequence = 0
        self._serial_trace_buffer = bytearray()
        self._serial_trace_first_ns: int | None = None
        self._serial_trace_last_ns: int | None = None
        self._text_pending = bytearray()
        self._text_saw_cr = False

    def _emit_firmware_line(self, line_bytes: bytes) -> None:
        if not line_bytes:
            return
        line = line_bytes.decode("ascii", "replace").strip()
        if not line:
            return
        rx_wall_time_ns = time.time_ns()
        rx_monotonic_ns = time.monotonic_ns()
        parsed = parse_firmware_line(line)
        self.trace.record_firmware_line(
            connection_id=self.connection_id,
            line=line,
            parsed=parsed,
            rx_wall_time_ns=rx_wall_time_ns,
            rx_monotonic_ns=rx_monotonic_ns,
        )
        self.output.put(("firmware_line", {
            "line": line,
            "parsed": parsed,
            "connection_id": self.connection_id,
            "rx_wall_time_ns": rx_wall_time_ns,
            "rx_monotonic_ns": rx_monotonic_ns,
        }))

    def _feed_text(self, chunk: bytes) -> None:
        for value in chunk:
            if value == 13:
                self._text_saw_cr = True
                continue
            if value == 10:
                if self._text_saw_cr:
                    self._emit_firmware_line(bytes(self._text_pending))
                self._text_pending.clear()
                self._text_saw_cr = False
            elif value == 9 or 32 <= value <= 126:
                if self._text_saw_cr:
                    self._text_pending.clear()
                    self._text_saw_cr = False
                self._text_pending.append(value)
                if len(self._text_pending) > 4096:
                    self._text_pending.clear()
            else:
                self._text_pending.clear()
                self._text_saw_cr = False

    def _flush_text(self) -> None:
        if self._text_pending:
            self._emit_firmware_line(bytes(self._text_pending))
        self._text_pending.clear()
        self._text_saw_cr = False

    def _trace_accumulate(self, chunk: bytes) -> None:
        if not chunk:
            return
        now_ns = time.monotonic_ns()
        if not self._serial_trace_buffer:
            self._serial_trace_first_ns = now_ns
        self._serial_trace_last_ns = now_ns
        self._serial_trace_buffer.extend(chunk)

    def _trace_flush(self, force: bool = False) -> None:
        if not self._serial_trace_buffer:
            return
        if not force and len(self._serial_trace_buffer) < 4096:
            return
        self.trace.record_serial_bytes(
            connection_id=self.connection_id,
            data=bytes(self._serial_trace_buffer),
            first_rx_monotonic_ns=self._serial_trace_first_ns,
            last_rx_monotonic_ns=self._serial_trace_last_ns,
        )
        self._serial_trace_buffer.clear()
        self._serial_trace_first_ns = None
        self._serial_trace_last_ns = None

    def _port_metadata(self) -> dict[str, Any]:
        try:
            for port in list_ports.comports():
                if port.device == self.port:
                    return {
                        "device": port.device,
                        "description": port.description,
                        "hwid": port.hwid,
                        "vid": port.vid,
                        "pid": port.pid,
                        "serial_number": port.serial_number,
                        "manufacturer": port.manufacturer,
                        "product": port.product,
                        "interface": port.interface,
                    }
        except Exception as exc:
            return {"device": self.port, "metadata_error": str(exc)}
        return {"device": self.port}

    def _record_serial_remainder(self, buffer: bytearray, reason: str) -> None:
        if not buffer:
            return
        raw = bytes(buffer)
        self.trace.record_event("serial", "unterminated_buffer", {
            "connection_id": self.connection_id,
            "reason": reason,
            "length": len(raw),
            "sha256": hashlib.sha256(raw).hexdigest(),
            "hex": raw.hex(),
        })
    def _record_decode_error(
        self,
        trace_id: int,
        sequence: int,
        error: str,
        packet: bytes,
    ) -> None:
        self.output.put(("decode_error", {
            "trace_id": trace_id,
            "sequence": sequence,
            "error": error,
            "packet_len": len(packet),
        }))

    def run(self) -> None:
        buffer = bytearray()
        while not self.stop_event.is_set():
            if self.ser is None:
                self.trace.record_event("serial", "connect_attempt", self._port_metadata())
                try:
                    self.ser = serial.Serial(self.port, baudrate=115200, timeout=0.2)
                    self.ser.reset_input_buffer()
                    buffer.clear()
                    self.connection_id += 1
                    self.sequence = 0
                    self.trace.record_system({
                        "event": "serial_connected",
                        "connection_id": self.connection_id,
                        "port": self.port,
                        "port_metadata": self._port_metadata(),
                        "serial": {
                            "baudrate": getattr(self.ser, "baudrate", None),
                            "bytesize": getattr(self.ser, "bytesize", None),
                            "parity": str(getattr(self.ser, "parity", None)),
                            "stopbits": str(getattr(self.ser, "stopbits", None)),
                            "timeout": getattr(self.ser, "timeout", None),
                            "dsrdtr": getattr(self.ser, "dsrdtr", None),
                            "rtscts": getattr(self.ser, "rtscts", None),
                            "xonxoff": getattr(self.ser, "xonxoff", None),
                        },
                    })
                    self.output.put(("status", f"Connected to {self.port}"))
                except Exception as exc:
                    self.trace.record_event("serial", "connect_failed", {
                        "port": self.port,
                        "error": f"{type(exc).__name__}: {exc}",
                    })
                    if not self.stop_event.is_set():
                        self.output.put(("status", f"Reconnecting {self.port}: {exc}"))
                        time.sleep(0.4)
                    continue

            try:
                # Read one byte with the configured timeout. Reading
                # in_waiting bytes can raise "device reports readiness to
                # read but returned no data" on Windows USB CDC.
                chunk = self.ser.read(1)
            except (serial.SerialException, OSError) as exc:
                self.trace.record_system({
                    "event": "serial_disconnected",
                    "connection_id": self.connection_id,
                    "port": self.port,
                    "error": f"{type(exc).__name__}: {exc}",
                    "buffer_bytes": len(buffer),
                })
                if not self.stop_event.is_set():
                    self.output.put(("status", f"Connection lost, reconnecting {self.port}... ({exc})"))
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self._flush_text()
                self._trace_flush(force=True)
                buffer.clear()
                time.sleep(0.2)
                continue

            if not chunk:
                continue
            buffer.extend(chunk)
            self._feed_text(chunk)
            self._trace_accumulate(chunk)
            self._trace_flush(force=False)

            if len(buffer) > 1024 * 1024:
                trace_id = self.trace.next_trace_id()
                self.sequence += 1
                raw = bytes(buffer)
                rx_monotonic_ns = time.monotonic_ns()
                self.trace.record_frame(
                    trace_id=trace_id,
                    connection_id=self.connection_id,
                    sequence=self.sequence,
                    rx_wall_time_ns=time.time_ns(),
                    rx_monotonic_ns=rx_monotonic_ns,
                    raw_packet=raw,
                    decoded=None,
                    decode_error="receive_buffer_overflow",
                    kind="OVERSIZE",
                    read_overflow=True,
                )
                self._trace_flush(force=True)
                buffer.clear()
                self._record_decode_error(
                    trace_id, self.sequence, "receive_buffer_overflow", raw
                )
                continue

            while True:
                delimiter = buffer.find(b"\x00")
                if delimiter < 0:
                    break
                packet = bytes(buffer[:delimiter])
                del buffer[: delimiter + 1]
                trace_id = self.trace.next_trace_id()
                self.sequence += 1
                rx_monotonic_ns = time.monotonic_ns()
                rx_wall_time_ns = time.time_ns()
                msg, decode_error, protobuf_payload = glove.decode_packet_with_payload(packet)
                if msg is None:
                    self.trace.record_frame(
                        trace_id=trace_id,
                        connection_id=self.connection_id,
                        sequence=self.sequence,
                        rx_wall_time_ns=rx_wall_time_ns,
                        rx_monotonic_ns=rx_monotonic_ns,
                        raw_packet=packet,
                        decoded=None,
                        protobuf_payload=protobuf_payload,
                        decode_error=decode_error or "decode_failed",
                        kind="DECODE_ERROR",
                    )
                    self._record_decode_error(
                        trace_id,
                        self.sequence,
                        decode_error or "decode_failed",
                        packet,
                    )
                    self._trace_flush(force=True)
                    continue

                try:
                    row = glove.message_to_row(msg)
                except Exception as exc:
                    error = f"row_conversion_error: {type(exc).__name__}: {exc}"
                    self.trace.record_frame(
                        trace_id=trace_id,
                        connection_id=self.connection_id,
                        sequence=self.sequence,
                        rx_wall_time_ns=rx_wall_time_ns,
                        rx_monotonic_ns=rx_monotonic_ns,
                        raw_packet=packet,
                        decoded=None,
                        protobuf_payload=protobuf_payload,
                        decode_error=error,
                        kind="ROW_ERROR",
                    )
                    self._record_decode_error(trace_id, self.sequence, error, packet)
                    self._trace_flush(force=True)
                    continue

                replay_kind = kind_for_sensor(row.get("sensor_id"))
                if replay_kind is not None:
                    kind = replay_kind
                elif row.get("mag_x") is not None:
                    kind = "MAG_META" if int(row.get("sensor_id") or 0) > 20 else "MAG"
                elif row.get("quat_x") is not None:
                    kind = "QUAT"
                else:
                    kind = "UNKNOWN"
                row["kind"] = kind
                row["_trace"] = {
                    "trace_id": trace_id,
                    "connection_id": self.connection_id,
                    "sequence": self.sequence,
                    "rx_wall_time_ns": rx_wall_time_ns,
                    "rx_monotonic_ns": rx_monotonic_ns,
                    "packet_len": len(packet),
                }
                self.trace.record_frame(
                    trace_id=trace_id,
                    connection_id=self.connection_id,
                    sequence=self.sequence,
                    rx_wall_time_ns=rx_wall_time_ns,
                    rx_monotonic_ns=rx_monotonic_ns,
                    raw_packet=packet,
                    decoded=row.get("packet"),
                    protobuf_payload=protobuf_payload,
                    decode_error=None,
                    kind=kind,
                )
                if not self.typed_output:
                    self.output.put(("row", row))
                elif replay_kind is not None:
                    # Snapshot/raw-input frames are recorded and parsed offline.
                    # They must not drive the live pose or magnetometer widgets.
                    self.output.put(("replay", row))
                elif kind == "QUAT":
                    self.output.put(("quat", row))
                elif kind in ("MAG", "MAG_META"):
                    self.output.put(("mag", row))
                else:
                    self.output.put(("row", row))
                self._trace_flush(force=True)

        self._flush_text()
        self._trace_flush(force=True)
        if self.ser is not None:
            self.trace.record_system({
                "event": "serial_closed",
                "connection_id": self.connection_id,
                "port": self.port,
                "buffer_bytes": len(buffer),
            })
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.output.put(("status", "Disconnected"))


__all__ = ["TracedSerialReader"]