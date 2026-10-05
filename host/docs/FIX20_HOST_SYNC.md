# FIX20 上位机同步说明

## FIX20 相对 FIX14 的固件变化

- 主融合 gyro 输入从 `GYRO_PASS` 改为 `BHY2_SENSOR_ID_GYRO` corrected gyro。
- corrected gyro 启动 bias 初始化为 0。
- still-only residual bias LPF 保留。
- 磁标定、门控和 yaw 逻辑不变。
- CDC protobuf 协议不变。
- `BHI_DIAG_PRINT=0`，生产快照通常不输出 FIX14 的 `GQ/GY` 文本。

## 上位机同步结果

### 不变部分

- 每个 quaternion protobuf 帧继续解码。
- `accuracy` 的 status、cal_result、8 位门控 mask、weight 继续逐帧记录。
- `gate_events.jsonl` 继续逐帧检测门控 enter/exit/initial_active。
- `P` 状态表继续每 5 秒自动轮询。
- 原始串口流、原始 COBS 帧、protobuf 和算法中间态继续完整录制。

### FIX20 变化

- 上位机不会假设存在 `GQ/GY`；缺失时不会误判为错误。
- UI 标签改为通用固件诊断，不再写死 FIX14。
- trace manifest 加入最新已知固件画像：
  - name: `FIX20`
  - hex SHA-256: `F2C0A1D0F9E5A76989A68294D2A2F615294A189227001A2E6C75FCB4D664EC51`
  - gyro source: corrected `BHY2_SENSOR_ID_GYRO`
  - BHI diagnostic text: disabled
- P 表 parser 不受影响，完整 40 列继续保存到 `firmware_diagnostics.jsonl`。

## FIX20 当前仍不可从主机直接观测

corrected gyro 已用于固件融合，但不会作为 protobuf 或文本直接发送给主机。主机只能看到最终 quaternion、accuracy 门控位和 P 表状态。

若需要逐帧验证 corrected gyro 本身，需要固件增加以下之一：

- 新的 protobuf 诊断字段
- 高频 corrected gyro 文本流
- 可运行时打开/关闭的 gyro 诊断输出

在上位机侧，不需要改协议解析；如果固件增加字段，只需扩展 `read_osmo_glove.py` / `firmware_diagnostics.py`。