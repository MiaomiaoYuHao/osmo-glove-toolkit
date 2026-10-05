#!/usr/bin/env python3
"""Lightweight runtime metrics for trace sessions, using stdlib first."""

from __future__ import annotations

import ctypes
import os
import platform
import shutil
import threading
import time
from pathlib import Path
from typing import Any


class _ProcessMemoryCounters(ctypes.Structure):
    _fields_ = [
        ("cb", ctypes.c_ulong),
        ("PageFaultCount", ctypes.c_ulong),
        ("PeakWorkingSetSize", ctypes.c_size_t),
        ("WorkingSetSize", ctypes.c_size_t),
        ("QuarantinePeakPagedPoolUsage", ctypes.c_size_t),
        ("QuarantinePagedPoolUsage", ctypes.c_size_t),
        ("QuarantinePeakNonPagedPoolUsage", ctypes.c_size_t),
        ("QuarantineNonPagedPoolUsage", ctypes.c_size_t),
        ("PagefileUsage", ctypes.c_size_t),
        ("PeakPagefileUsage", ctypes.c_size_t),
    ]


def _windows_memory() -> dict[str, Any] | None:
    if os.name != "nt":
        return None
    try:
        process = ctypes.windll.kernel32.GetCurrentProcess()
        counters = _ProcessMemoryCounters()
        counters.cb = ctypes.sizeof(counters)
        ok = ctypes.windll.psapi.GetProcessMemoryInfo(
            process, ctypes.byref(counters), counters.cb
        )
        if not ok:
            return None
        return {
            "working_set_bytes": int(counters.WorkingSetSize),
            "peak_working_set_bytes": int(counters.PeakWorkingSetSize),
            "pagefile_bytes": int(counters.PagefileUsage),
            "peak_pagefile_bytes": int(counters.PeakPagefileUsage),
            "page_fault_count": int(counters.PageFaultCount),
        }
    except Exception:
        return None


def collect_runtime_metrics(session_dir: str | Path | None = None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "wall_time": time.time(),
        "monotonic_ns": time.monotonic_ns(),
        "pid": os.getpid(),
        "process_time_s": time.process_time(),
        "thread_count": threading.active_count(),
        "thread_names": [thread.name for thread in threading.enumerate()],
        "python": platform.python_version(),
        "platform": platform.platform(),
    }
    memory = _windows_memory()
    if memory is None:
        try:
            import resource
            usage = resource.getrusage(resource.RUSAGE_SELF)
            memory = {
                "max_rss_bytes": int(usage.ru_maxrss) * 1024,
                "user_time_s": usage.ru_utime,
                "system_time_s": usage.ru_stime,
            }
        except Exception:
            memory = None
    result["memory"] = memory
    try:
        import psutil
        process = psutil.Process(os.getpid())
        result["psutil"] = {
            "cpu_percent": process.cpu_percent(interval=None),
            "memory_rss_bytes": process.memory_info().rss,
            "memory_vms_bytes": process.memory_info().vms,
            "num_threads": process.num_threads(),
        }
    except Exception:
        result["psutil"] = None
    if session_dir is not None:
        try:
            usage = shutil.disk_usage(Path(session_dir))
            result["disk"] = {
                "total_bytes": usage.total,
                "used_bytes": usage.used,
                "free_bytes": usage.free,
            }
        except Exception as exc:
            result["disk"] = {"error": str(exc)}
    return result


__all__ = ["collect_runtime_metrics"]