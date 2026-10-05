# Bowie REPLAYBIN 数据-only 协议（v2）

主机解析的唯一权威定义在：

`C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\replay_protocol.py`

固件只发送数值和 `sensor_id`，不在每个帧里重复字段名。字段名、顺序、缩放和分组全部由上面的 Python schema 对齐。

## 合成 sensor_id

| ID | 名称 | 内容 |
|---:|---|---|
| 240 | RAW_QUAT | `mag_fusion_on_quat()` 之前的 GAMERV，五元组 `x,y,z,w,accuracy` |
| 241 | RAW_MAG | 标定/FIR 之前的原始磁矢量 `x,y,z` |
| 242 | RAW_GYRO | 实际送入 `mag_fusion_on_gyro()` 的 corrected gyro |
| 243 | RAW_ACC | corrected accelerometer |
| 244 | RAW_MAG2 | 第二磁力计原始值，硬件有才发 |
| 245 | CAL_BLOB | 200B 的 `mag_cal_blob_t` 快照 |
| 246 | FUSION_STATE | `sizeof(mag_fusion_t)` 字节的完整运行状态快照 |
| 247 | YF | 快速 yaw 状态，6 个 float/帧，100 Hz |
| 248 | YAW | yaw 控制状态，6 个 float/帧，10 Hz |
| 249 | YAW2 | 门控/场估计状态，6 个 float/帧，10 Hz |
| 250 | CONFIG | 固件参数、结构尺寸、offset、layout hash、build CRC |
| 251 | EVENT | 标定、学习、hard event、FIFO/传感器错误、clock sync 等事件 |
| 252 | HEALTH | 1 Hz 计数器和传输健康信息 |

## 快照传输

245/246/250 使用同一套分块格式：

- 首帧：`index = 0xFFFFFFFF`
  - `mag.seconds = length`
  - `mag.nanoseconds = crc`
  - `quat.seconds = version`
  - `quat.nanoseconds = kind`
- 数据帧：`index = chunk`
  - `mag.seconds, mag.nanoseconds, quat.seconds, quat.nanoseconds`
  - 依次是 4 个小端 32-bit word，也就是 16 字节原始数据。

YAW/YAW2 的 `t_ms` 使用最近一次 BHI 采样的设备时间（由 BHI tick 换算），因此可以直接和离线 `replay_engine_output.csv` 的 `ts_ticks` 对齐。

## 事件格式

`REPLAY_EVENT_ID` 每帧携带：

- `index = event_seq`
- `mag.seconds = event_code`
- `mag.nanoseconds = HAL_GetTick()`
- `quat.seconds = a`
- `quat.nanoseconds = b`

`clock_sync`（code 27）中 `a/b` 是最近一次 BHI 64-bit timestamp 的低/高 32 位。

## 操作流程

1. 烧入 REPLAYBIN。
2. 上位机开始录制；录制开始会自动发送 `D` 请求快照。
3. 录制结束。
4. 提取：

```powershell
python THost\extract_replay.py <录制目录>
```

输出：

- `replay_raw_inputs.csv`
- `replay_yf.csv`
- `replay_yaw.csv`
- `replay_yaw2.csv`
- `replay_health.csv`
- `replay_events.csv`
- `snapshots/CAL_BLOB_link*.bin`
- `snapshots/FUSION_STATE_link*.bin`
- `snapshots/CONFIG_link*.bin`

5. 离线重跑固件算法：

```powershell
python THost\replay_firmware.py <录制目录>
```

输出：

- `replay_firmware/replay_engine_output.csv`

这个 CSV 由同一份 `mag_fusion.c` 和录制时的完整 `mag_fusion_t` 快照生成，可以直接和录制期间的 YAW/YF 输出对齐比较。

6. 对比录制输出和离线输出：

```powershell
python THost\compare_replay.py <replay_firmware 工作目录>
```

输出：

- `replay_compare.csv`
- `replay_compare.json`（yaw/datum 误差 max、mean、p95）

7. 检查输入完整性：

```powershell
python THost\verify_replay.py <replay_firmware 工作目录>
```

输出 `replay_integrity.json`，会检查各 RAW 流的采样数、时间戳缺口、重复时间戳，并汇总 HEALTH 中的 CDC/FIFO/sensor 错误。时钟偏移与漂移记录在 `clock_map.json`。
