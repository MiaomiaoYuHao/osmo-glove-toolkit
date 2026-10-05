# FIX14 Host 信息覆盖清单

> FIX20 同步说明见 `FIX20_HOST_SYNC.md`。FIX20 协议未变，但生产快照关闭 BHI_DIAG_PRINT。

## 已逐帧实时覆盖

来源：每个 quaternion protobuf 帧的 `accuracy` 字段。

- `status`：NO_CAL / CAL / ACQ / LOCKED / DEGRADED / COASTING
- `cal_result`
- 8 位 diagnostic mask
- `weight`
- 每个 mask bit 的 `initial_active` / `enter` / `exit` 和 `duration_ms`

输出：

- `analysis.csv`
- `gate_events.jsonl`
- `system.jsonl` 最新状态

## 已按 FIX14 文本实时覆盖

### GQ，1 Hz

- raw quaternion x/y/z/w
- accuracy
- yawc
- 换算后的 quaternion
- 接收时间

### GY，1 Hz

- passthrough gyro P
- corrected gyro C
- CV corrected-valid
- internal gyro bias B
- gyro activity
- motion hold

### MAG，固件开启 MAG_DIAG_PRINT 时

- RAW x/y/z
- CAL x/y/z
- OFF x/y/z
- revision
- fixes
- learning opens
- learning state
- datum
- yawc

### P 磁状态表，默认 5 秒轮询

- status、trust、radius1、radius2、model、resid、align
- grad、grad threshold、drift、dip、stuck、fixes
- FE n / obs / residual / d / B / A
- geo / ellipsoid quality
- e_step
- e_drift
- e_lp
- e_norm
- e_dip
- e_grad
- e_fleet
- weight
- rejecting
- hard event
- candidate valid
- candidate seconds
- good seconds
- e_inst
- datum
- yawc
- revision
- learning state
- learning opens

输出：

- `firmware_diagnostics.jsonl`
- `system.jsonl`
- `gate_events.jsonl` 中的布尔/状态变化

## 当前只能得到快照，不能每帧得到

以下值由 `P` 表提供，当前是 5 秒左右的快照，不是每个 quaternion 帧：

- `e_norm`
- `e_dip`
- `e_step`
- `e_drift`
- `e_grad`
- `e_fleet`
- `e_inst`
- `e_lp`
- `grad_now`
- `grad_threshold`
- `drift_rate`
- `ref_dip`
- `stuck_s`
- `candidate_s`
- `good_s`
- `weight`
- `yaw_corr`
- `datum`

说明：这些量对应的最终门控结果已经通过 100 Hz 级 `accuracy` mask 获得；缺的是“门控判决前的连续输入值”。

## 当前完全没有传输给主机

以下字段存在于 `mag_fusion_t`，但 FIX14 的 protobuf、GQ/GY/MAG/P 都没有输出：

### Yaw Kalman

- `yaw_P`
- `yaw_trust`
- `yaw_large_active`
- `yaw_blend`
- `yaw_blend_valid`
- `mag_datum_offset`

### Rebaseline / candidate / safety

- `safe_valid`
- `safe_yaw_valid`
- `safe_norm`
- `safe_dip`
- `safe_yaw_corr`
- `safe_datum`
- `cand_norm`
- `cand_dip`
- `head_rebase_s`
- `yaw_hold_s`
- `drift_fired_s`

### Drift / coasting

- `drift_valid`
- `drift_anchored`
- `drift_n`
- `drift_anchor`
- `drift_acc_s`
- `drift_bad_s`
- `coast_acc`

### Learning window

- `learn_hold_s`
- `learn_cool_s`
- `learn_adopted`

### Field estimator / alignment

- `A[9]` 完整矩阵
- `align_valid`
- `field_valid`
- `have_last`
- `fe.d_run`
- `fe.since`
- `map_valid`
- `map_res`
- `map_since`
- `have2`
- `stale2_s`
- `u1_valid`
- `u2_valid`
- `seq_used`
- `w_now`
- `w_lp`
- `w_prev`
- `w_prev2`
- `w_anchor`
- `sigma`
- `sigma_n`
- `grad_mu`
- `grad_var`
- `grad_n`
- `ref_norm`
- `ref_norm0`
- `ref_dip0`
- `ref_dip_acc`
- `ref_dip_n`
- `dir_res`
- `dir_lock_s`
- `m_dir_valid`
- `fleet_idx`
- `fleet_joined`
- `q_stab`
- `q_stab_valid`
- `gyro_valid`

### Calibration

- 完整 `W[9]` soft-iron matrix
- 完整 map matrix `map[9]`
- `map_c[3]`
- 完整 alignment matrix `align[9]`
- calibration bins 覆盖情况
- `bins_req`
- `restarts`
- `window_s`
- `next_try`
- `seen`
- `active`
- `frozen`

## 建议的固件扩展

如果测试需要完整离线复现融合内部状态，建议新增一个高频诊断帧，而不是继续依赖文本：

- 10-20 Hz
- 固定字段、定长或 protobuf submessage
- 至少包含 yaw、yawc、weight、yaw_P、yaw_trust、e_norm、e_dip、e_step、e_drift、e_grad、e_inst、e_lp、ref_dip、grad_now、grad_threshold、candidate/rebaseline 状态
- 单独位图扩展为逐门控 enter/hold/exit
- 同时输出 A/W/map 的摘要哈希和 revision

在固件加入这些字段前，主机无法凭现有输出重建它们，只能记录当前快照和最终门控位。