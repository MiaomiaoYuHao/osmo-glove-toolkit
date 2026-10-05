# THost 整理清单

本仓库的 `host/` 来自原工作目录 `C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost`。原目录同时包含源码、固件构建树、现场录制和大量诊断输出；公开仓库按“可安装、可运行、可复现、体积合理”整理，而不是原样搬运。

## 已纳入

| 原位置 | 仓库位置 | 说明 |
|---|---|---|
| `THost\3D力测试上位机.pyw` | `host/3D力测试上位机.pyw` | 3D 磁力上位机 |
| `THost\姿态测试上位机.pyw` | `host/姿态测试上位机.pyw` | 姿态、磁诊断与 Yaw 上位机 |
| `THost\纯数据预览.pyw` | `host/纯数据预览.pyw` | MAG / QUAT / META 原始预览 |
| `THost\*.py` | `host/*.py` | 协议、追踪、回放、诊断与回归工具 |
| `THost\*.md` | `host/docs/*.md` | 使用说明、追踪录制和 FIX 记录 |
| `THost\magcal\` | `host/magcal/` | 核心标定工具；历史实验脚本放在 `lab/` |
| `THost\启动_*.bat` 的功能 | `run_*.bat`、`host/launchers/*.bat` | 改为相对路径并自动寻找 `pyw` / `pythonw` |
| `THost\firmware\` | `firmware/releases/` | 去重后的 HEX/BIN 发布镜像与校验和 |

## 未直接纳入

| 内容 | 原因 |
|---|---|
| `THost\recordings\` | 约 2.7 GB 现场录制，属于运行数据而非源码 |
| `THost\diagnostics\` | 约 225 MB 测试输出和临时 trace |
| `THost\firmware\*\FROZEN_*` 构建副本 | 约 223 MB；可复现源码和正式发布镜像已单独整理 |
| `*.bak*`、`_tmp_*` | 历史备份与一次性临时脚本 |
| `__pycache__\`、`*.exe` | Python 缓存和 Windows 本机编译产物 |
| `*.npz`、`*.bin`、`*.blob`、`*.csv` | 标定采集结果；运行 `magcal.py` 后可重新生成 |

## 验证入口

```powershell
python "host\3D力测试上位机.pyw" --self-test
python "host\姿态测试上位机.pyw" --self-test
python host\magcal\test_fit.py
```

历史实验脚本的记录见 `host/magcal/lab/README.md`。
