#!/usr/bin/env python3
"""ASCII-safe launcher for the THost 3D force UI."""
from __future__ import annotations

import ctypes
import msvcrt
import os
import runpy
import sys
import time
import traceback
from pathlib import Path

BASE = Path(__file__).resolve().parent
APP = BASE / "3D力测试上位机.pyw"
LOG = BASE / "3d_force_ui_error.log"

def activate_existing() -> bool:
    hwnd = ctypes.windll.user32.FindWindowW(None, "OSMO 三维力测试上位机")
    if hwnd:
        ctypes.windll.user32.ShowWindow(hwnd, 5)
        ctypes.windll.user32.SetForegroundWindow(hwnd)
        return True
    return False

lock_file = open(BASE / "force_3d_ui.lock", "a+", encoding="utf-8")
try:
    msvcrt.locking(lock_file.fileno(), msvcrt.LK_NBLCK, 1)
except OSError:
    for _ in range(20):
        if activate_existing():
            raise SystemExit(0)
        time.sleep(0.25)
    ctypes.windll.user32.MessageBoxW(0, "三维力上位机正在启动，请稍候。", "OSMO 三维力", 0)
    raise SystemExit(0)

try:
    hwnd = ctypes.windll.user32.FindWindowW(None, "OSMO 三维力测试上位机")
    if hwnd:
        ctypes.windll.user32.ShowWindow(hwnd, 5)
        ctypes.windll.user32.SetForegroundWindow(hwnd)
        raise SystemExit(0)
    os.chdir(BASE)
    sys.argv = [str(APP), "--no-auto-trace"]
    runpy.run_path(str(APP), run_name="__main__")
except SystemExit:
    raise
except BaseException as exc:
    text = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
    LOG.write_text(text, encoding="utf-8")
    try:
        import tkinter as tk
        from tkinter import messagebox
        root = tk.Tk()
        root.withdraw()
        messagebox.showerror("OSMO 三维力启动失败", f"{exc}\n\n详情已写入：\n{LOG}")
        root.destroy()
    except Exception:
        pass
    raise
