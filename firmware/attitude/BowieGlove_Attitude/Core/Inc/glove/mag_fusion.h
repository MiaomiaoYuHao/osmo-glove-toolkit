/*
 * mag_fusion.h
 *
 * Absolute, drift-free, magnetically-hardened attitude for BHI360 + 1..2x BMM350.
 *
 * ============================== ARCHITECTURE ==============================
 *
 * The BHI360 gives a 6-axis Game Rotation Vector: roll/pitch are gravity
 * locked and immune to magnetic disturbance, yaw free-runs.  This module adds
 * exactly ONE degree of freedom on top:
 *
 *      q_out = q_z(yaw_corr) * q_gamerv
 *
 * Because the correction is a pure rotation about the world vertical, a
 * magnetic disturbance can never corrupt roll/pitch - by construction.
 *
 * Signal chain, in order:
 *
 *   raw LSB
 *     -> BODY prefilter          3-tap [1 2 1]/4, zero at fs/2   (rev 2)
 *     -> calibration             m = A * W * (raw - b),  |m| ~ radius
 *          W  symmetric soft iron   from the ellipsoid fit
 *          b  hard iron             from the ellipsoid fit
 *          A  ROTATION mag->IMU     from the dip-invariance solve (rev 2)
 *     -> WORLD frame              w = R_gamerv * u
 *     -> WORLD boxcar             MAG_WORLD_AVG taps, kills mains lines (rev 2)
 *     -> gates                    all sigma-normalised against measured noise
 *     -> 1-D Kalman on yaw        self-tuning bandwidth, NIS gating   (rev 2)
 *
 * WHY FILTER IN THE WORLD FRAME.  The world-frame magnetic vector is constant
 * no matter how the board moves, so it can be averaged over 100 ms with no
 * motion-dependent distortion and no group delay that matters.  Filtering the
 * BODY vector instead - which is what rev 1 did inside the consistency gate -
 * introduces a lag proportional to rotation rate and therefore a false
 * "magnetic inconsistency" every time the hand moves quickly.
 *
 * ========================= WHAT REV 2 CHANGED =============================
 *
 *  1. Anti-alias / mains rejection.  A magnetically noisy environment puts a
 *     50/60 Hz line right at or near fs/2 when the magnetometer runs at
 *     100 Hz.  Nothing downstream can remove it and a sample-to-sample
 *     derivative DOUBLES it.  Two filters now sit in front of everything.
 *  2. Mag-to-IMU rotation A.  An ellipsoid fit only ever constrains |m|, and
 *     |R m| = |m| for any rotation R, so the fit says NOTHING about the
 *     rotation between the magnetometer axes and the IMU axes.  Rev 1 assumed
 *     A = I.  Any real mismatch makes the measured dip swing with attitude,
 *     which is exactly "some orientations trip the gate".  A is now estimated
 *     online from the fact that the vertical field component is the same in
 *     every attitude.
 *  3. Every gate is normalised by the MEASURED noise of this hardware instead
 *     of a hand-picked constant, so a threshold can no longer sit below the
 *     noise floor and reject continuously.
 *  4. The gradient threshold learner no longer feeds on its own output.  In
 *     rev 1 the learning window was gated by e (which contains e_grad), so it
 *     only ever saw the lower half of the distribution and ratcheted itself
 *     shut.
 *  5. The weight and the event machine run off a low-passed error, so a
 *     single noisy sample can no longer drop the lock.
 *  6. The body-offset estimator runs CONTINUOUSLY with exponential forgetting
 *     instead of only after a fault, so a magnetised board is absorbed during
 *     normal motion and never reaches the stuck state at all.
 *  7. The output stabiliser is applied to the 6-axis attitude BEFORE the yaw
 *     correction, not after.  Rev 1 low-passed its own correction with a
 *     100 s time constant while the board was still, so recovery was
 *     invisible for a minute and a half.
 *  8. Calibration: cross-validated model selection, spherical-cap coverage
 *     instead of axis spans, like-for-like residual comparison when deciding
 *     whether to adopt a refit, and a time-bounded window so the fit tracks
 *     thermal drift.
 *  9. Optional cross-link consensus: 20 links on one hand share one earth
 *     field.  |B| and dip are yaw-invariant, so they are directly comparable
 *     across links, which gives a disturbance detector with a 15 cm baseline
 *     instead of the 8 mm between the two parts on one board.
 *
 * Output frame: +Z up (from GAMERV), +X along horizontal magnetic north
 * rotated by MAG_DECLINATION_DEG.
 *
 * No HAL, no Bosch API, no allocation - it builds and unit-tests on a host.
 */

#ifndef INC_GLOVE_MAG_FUSION_H_
#define INC_GLOVE_MAG_FUSION_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/*  Front-end filtering                                                     */
/* ======================================================================== */

/* Body-frame prefilter, applied to the raw LSB of BOTH magnetometers before
 * anything else touches them.  3 taps = [1 2 1]/4, which is an exact zero at
 * fs/2 and -20 dB at fs/2.5.  Set to 1 to disable, 2 for [1 1]/2.
 * This is the cheap, always-correct half of the anti-alias fix; the real fix
 * is to lower the magnetometer ODR so the BMM350 averages internally. */
#ifndef MAG_BODY_FIR
#define MAG_BODY_FIR            3u
#endif

/* World-frame boxcar length, in samples.  A boxcar of length L has exact
 * zeros at every multiple of fs/L, so L = 10 at 100 Hz nulls 10/20/30/40/50 Hz
 * - that covers 50 Hz mains AND 60 Hz mains aliased to 40 Hz.  The world
 * vector is constant under body motion, so the 50 ms group delay costs
 * nothing except a 50 ms lag in following a genuine environment change.
 * Must be <= MAG_WORLD_AVG_MAX. */
#ifndef MAG_WORLD_AVG
#define MAG_WORLD_AVG           10u
#endif
#define MAG_WORLD_AVG_MAX       16u

/* ======================================================================== */
/*  Yaw estimator (1-D Kalman)                                              */
/* ======================================================================== */

/* Process noise: how fast the 6-axis yaw is allowed to wander, rad^2 per
 * second.  (0.1 deg/s)^2 ~ 3e-6.  Larger = faster lock, noisier hold. */
#ifndef MAG_YAW_Q
#define MAG_YAW_Q               3.0e-6f
#endif
/* Floor on the measurement variance, rad^2, so a pathologically optimistic
 * noise estimate cannot make the filter chase every sample. */
#ifndef MAG_YAW_R_MIN
#define MAG_YAW_R_MIN           4.0e-6f     /* (0.115 deg)^2 */
#endif
/* Initial/reset variance, rad^2.  Large = the first measurement is taken
 * almost at face value. */
#ifndef MAG_YAW_P0
#define MAG_YAW_P0              1.0f
#endif
/* Ceiling on P so a long coast cannot make the next measurement a step. */
#ifndef MAG_YAW_P_MAX
#define MAG_YAW_P_MAX           0.05f       /* (12.8 deg)^2 */
#endif
/* Normalised innovation squared above which a sample is an outlier.  9 = 3
 * sigma.  This replaces the fixed 30 deg innovation limit of rev 1. */
#ifndef MAG_YAW_NIS_MAX
#define MAG_YAW_NIS_MAX         9.0f
#endif
#ifndef MAG_YAW_INNOV_MAX
#define MAG_YAW_INNOV_MAX       0.34906585f  /* 20 deg - 见 MAG_YAW_SNAP_DEG */
#endif
/*
 * Rate at which the heading converges on a LARGE but clean measurement - the
 * gyro-drift case.  Fast enough to walk off 150 deg in about eight seconds,
 * slow enough that it reads as a correction rather than a jump, and bounded
 * so one bad sample moves the heading by 0.2 deg and no more.
 */
#ifndef MAG_YAW_SLEW_MAX
#define MAG_YAW_SLEW_MAX        1.60f   /* rad/s, ~92 deg/s after confirmation */
#endif
/* Stricter trust gate for the large-innovation gyro-drift correction.  The
 * normal disturbance gates remain FIX11-like; this extra gate prevents the
 * drift path from stealing the leading edge of a magnet disturbance. */
#ifndef MAG_YAW_SLEW_TRUST_MAX
#define MAG_YAW_SLEW_TRUST_MAX  0.50f
#endif
#ifndef MAG_YAW_SLEW_INST_MAX
#define MAG_YAW_SLEW_INST_MAX   1.50f
#endif
#ifndef MAG_YAW_SLEW_OMEGA_MAX
#define MAG_YAW_SLEW_OMEGA_MAX  9.42477796f /* rad/s, ~540 deg/s */
#endif
/* Separate timer for the new drift correction.  It must not share state with
 * the FIX11 event/recovery machine. */
/* 大修正路径的冻结上限。原来是 0.5s，后按用户要求收到 0.2s；
 * 现按用户要求【彻底关闭】该门限：置 0 表示进入大修正路径后不再"先冻结
 * 再步进"，每一拍都直接按 slew 限速推进一步。
 * 干扰防护仍然由 clean 谓词（物理门 + dir_lock + 角速度上限）负责，只有
 * MAG_YAW_*_MAX 的限速还在，冻结等待没有了。 */
#ifndef MAG_YAW_HOLD_S
#define MAG_YAW_HOLD_S          0.0f
#endif
/* Fast trusted-response path.  The fast/slow parameters are blended by a
 * continuous trust state instead of being selected by a per-sample boolean.
 * This removes audible/visible stick-slip when e_inst, e_lp or dir_lock_s
 * crosses its threshold. */
#ifndef MAG_YAW_HOLD_FAST_S
#define MAG_YAW_HOLD_FAST_S     0.2f
#endif
#ifndef MAG_YAW_SLEW_FAST_MAX
#define MAG_YAW_SLEW_FAST_MAX   3.2f   /* rad/s, ~183 deg/s */
#endif
#ifndef MAG_YAW_FAST_INST_MAX
#define MAG_YAW_FAST_INST_MAX   1.0f
#endif
/* ---- 欠账自适应松弛（FIX20 增加）--------------------------------------
 * 输出 yaw 相对磁航向欠账越大，允许磁修正越容易介入。
 *
 * 为什么需要：clean 谓词里 e_step 是【航向域】的量 —— 世界系磁场是拿
 * 【已修正】的四元数算的，所以 yaw_corr 一动 w 就跟着转，e_step 会把
 * 【修正动作本身】误当成干扰。于是死锁：欠账大 -> e_step 涨 -> clean
 * 不成立 -> 不许修 -> 欠账还在。
 *
 * 做法（三档）：
 *     欠账 <= MAG_YAW_RELAX_LO_DEG(5 度)      -> relax = 1.0，和原版完全一致
 *     欠账 =  MAG_YAW_RELAX_HI_DEG(15 度)     -> relax = 1.5
 *     欠账 >= MAG_YAW_SNAP_DEG(20 度)         -> 全速回零（见 .c 里的 snap 分支）
 *   5~15 度之间线性过渡。
 *
 * 【物理门永不放松】磁场大小 / 倾角 / 双磁梯度 / 整手一致性这四道门才是
 * 真干扰会动的量；放松它们等于拿抗干扰能力换响应速度，不做。
 * ---------------------------------------------------------------------- */
#ifndef MAG_YAW_RELAX_LO_DEG
#define MAG_YAW_RELAX_LO_DEG    5.0f
#endif
#ifndef MAG_YAW_RELAX_HI_DEG
#define MAG_YAW_RELAX_HI_DEG    15.0f
#endif
#ifndef MAG_YAW_RELAX_MAX
#define MAG_YAW_RELAX_MAX       1.5f
#endif
/* 欠账超过这个角度就不再限速，全速把 yaw 拉回磁航向。 */
/* yaw 大修正路径诊断的打印频率（Hz）。0 = 关闭。
 *
 * 一次修正事件只持续几十到几百毫秒，5 秒一次的 P 表抓不到。这里单独按固定
 * 频率打 YAW 一行（不打整张 P 表）：
 *   整张 P 表 ~600 字节，10 Hz = 6 KB/s，会和 15 KB/s 的数据流抢带宽；
 *   YAW 一行  ~150 字节，10 Hz = 1.5 KB/s，安全。
 * 默认 10 Hz，即 0.1 秒分辨率。 */
#ifndef MAG_YAW_TEXT_DIAG
#define MAG_YAW_TEXT_DIAG       0    /* 1 = 旧 YAW/YAW2/YF 文本诊断 */
#endif
#ifndef MAG_YAW_DIAG_HZ
#define MAG_YAW_DIAG_HZ         10   /* 仅 MAG_YAW_TEXT_DIAG=1 时有效 */
#endif
#ifndef MAG_YAW_BIN_DIAG_HZ
#define MAG_YAW_BIN_DIAG_HZ     10   /* YAW/YAW2 二进制数据流频率 */
#endif
#ifndef MAG_YAW_BIN_FAST_HZ
#define MAG_YAW_BIN_FAST_HZ     100  /* YF 二进制数据流频率 */
#endif
/* 紧凑快速行的频率（Hz）。0 = 关闭。
 *
 * 100 Hz 打完整 YAW+YAW2（896 B/帧）会产生 87.5 KB/s 文本，而数据流只有
 * 10 KB/s —— 链路上 90% 是文本，上位机来不及处理就会滞后。
 * 所以拆成两档：
 *   YF   紧凑行，只打快速变化的 14 个量，100 Hz，~9 KB/s
 *   YAW/YAW2 完整 73 个量，MAG_YAW_DIAG_HZ(10 Hz)，~9 KB/s
 * 合计 ~18 KB/s，既有 100 Hz 动态分辨率又不淹链路。 */
#ifndef MAG_YAW_FAST_HZ
#define MAG_YAW_FAST_HZ         100
#endif
#ifndef MAG_YAW_SNAP_DEG
#define MAG_YAW_SNAP_DEG        20.0f
#endif
#ifndef MAG_YAW_FAST_LP_MAX
#define MAG_YAW_FAST_LP_MAX     0.5f
#endif
/* Trust rises slowly so a short clean burst cannot unlock full response, but
 * falls faster so a real disturbance withdraws the fast path promptly. */
#ifndef MAG_YAW_TRUST_RISE_S
#define MAG_YAW_TRUST_RISE_S    0.15f
#endif
#ifndef MAG_YAW_TRUST_FALL_S
#define MAG_YAW_TRUST_FALL_S    0.05f
#endif
/* Hysteresis on the large-innovation path.  Exit happens only when the
 * innovation has genuinely settled, preventing the controller from toggling
 * between rate-limited slew and the Kalman branch around 30 degrees. */
#ifndef MAG_YAW_NIS_EXIT
#define MAG_YAW_NIS_EXIT        4.0f
#endif
#ifndef MAG_YAW_INNOV_EXIT
#define MAG_YAW_INNOV_EXIT      0.035f  /* ~2 deg */
#endif
/* Direction-only magnetic disturbance detector.  The body magnetic vector is
 * predicted from the previous sample and the quaternion change; a direction
 * residual that cannot be explained by attitude change latches a disturbance. */
/*
 * 方向残差门：编译期总开关（运行时强度另见 MAG_DIR_STRENGTH 说明）。
 *   1 = 启用（默认）。实际是否动作、动作多硬，由运行时可调的"门控强度"决定：
 *       强度 0% = 等效关闭；100% = 原行为（3.0° 门限 + e=4.0 硬事件）。
 *   0 = 编译期彻底移除该门（连 dir_lock_s 都不再置位）。
 */
#ifndef MAG_DIR_GATE_ENABLE
#define MAG_DIR_GATE_ENABLE     1
#endif
/* ---- 方向门强度【运动自适应】----------------------------------------
 * 方向残差门的误报主要来自运动（磁样本与四元数时间戳错配 + 标定残差随
 * 姿态变）。所以：越静止，门越强（回到用户设定值，上限 95%）；越剧烈，
 * 门越弱（最低 MAG_DIRMOT_MIN_STRENGTH），把"抗磁"和"及时修复"折中。
 * 强度同时作用在门限与 e 的顶高幅度上（见 mag_fusion_dir_thr_deg / dir_e）。 */
#ifndef MAG_DIRMOT_LO_DPS
#define MAG_DIRMOT_LO_DPS       10.0f   /* <= 这个角速度算"静止"           */
#endif
#ifndef MAG_DIRMOT_HI_DPS
#define MAG_DIRMOT_HI_DPS       120.0f  /* >= 这个角速度算"剧烈"           */
#endif
#ifndef MAG_DIRMOT_MIN_STRENGTH
#define MAG_DIRMOT_MIN_STRENGTH 0.20f   /* 剧烈时的强度下限                */
#endif
#ifndef MAG_DIRMOT_MAX_STRENGTH
#define MAG_DIRMOT_MAX_STRENGTH 0.95f   /* 静止时的强度上限（用户要求 95%）*/
#endif
#ifndef MAG_DIRMOT_TAU_FALL_S
#define MAG_DIRMOT_TAU_FALL_S   0.03f   /* 变剧烈：强度下降时间常数（快）  */
#endif
#ifndef MAG_DIRMOT_TAU_RISE_S
#define MAG_DIRMOT_TAU_RISE_S   0.30f   /* 变安静：强度恢复时间常数（慢）  */
#endif
#ifndef MAG_DIR_RES_RAD
#define MAG_DIR_RES_RAD         0.0523598776f  /* 3.0 deg，强度 100% 时的门限 */
#endif
/* 强度 0% 时门限放宽到多少度（1%..100% 之间线性）。 */
#ifndef MAG_DIR_THR_MAX_DEG
#define MAG_DIR_THR_MAX_DEG     20.0f
#endif
/* 方向残差门【强度】运行时可调（0.0 .. 1.0），由上位机滑块下发：
 *   0.00 = 关闭：不置 dir_lock_s、不改 e
 *   1.00 = 原行为：门限 3.0°，命中后 e 顶到 MAG_EVENT_ENTER+1 = 4.0（硬事件）
 *   0.90 = 固件默认：门限 4.7°，命中后 e = 3.7（仍 ≥ EVENT_ENTER，仍进硬事件）
 *   中间 = 门限按强度放宽（1.0→3°，0+→MAG_DIR_THR_MAX_DEG），
 *          且 e 的顶高幅度按 (1 + 3s) 缩放：
 *          0.67 → e=3.0（刚好 EVENT_ENTER）
 *          0.50 → e=2.5（刚好 REJECT_AT，权重归零）
 *          0.33 → e=2.0（只软降权，不清零）
 * dir_res 本身始终照算、照上报。 */
void  mag_fusion_set_dir_strength(float s);
float mag_fusion_dir_strength(void);
float mag_fusion_dir_thr_deg(void);
#ifndef MAG_DIR_LOCK_S
#define MAG_DIR_LOCK_S          0.5f
#endif
#ifndef MAG_DIR_OMEGA_MAX
#define MAG_DIR_OMEGA_MAX       9.42477796f /* ~540 deg/s */
#endif
#ifndef MAG_OMEGA_DERATE
#define MAG_OMEGA_DERATE        4.0f    /* rad/s at which the gain halves     */
#endif
#ifndef MAG_YAW_BLEND_TAU_S
#define MAG_YAW_BLEND_TAU_S      1.00f   /* output-only datum handoff          */
#endif

/* ======================================================================== */
/*  Noise estimation                                                        */
/* ======================================================================== */

/* The per-sample noise of the world-frame unit vector is measured from its
 * own second difference, which is blind to any smooth signal.  Tracked
 * asymmetrically - fast down, slow up - so it settles on the floor rather
 * than on a running mean that a disturbance could inflate. */
#ifndef MAG_NOISE_FALL
#define MAG_NOISE_FALL          0.02f
#endif
#ifndef MAG_NOISE_RISE
#define MAG_NOISE_RISE          0.002f
#endif
#ifndef MAG_NOISE_MIN
#define MAG_NOISE_MIN           0.0015f /* 0.15% of field, ~0.09 deg          */
#endif
#ifndef MAG_NOISE_MAX
#define MAG_NOISE_MAX           0.25f
#endif
#ifndef MAG_NOISE_WARMUP
#define MAG_NOISE_WARMUP        100u
#endif

/* ======================================================================== */
/*  Disturbance gates - all expressed as "k sigma OR a physical floor"      */
/* ======================================================================== */

#ifndef MAG_NORM_TOL
#define MAG_NORM_TOL            0.15f   /* ||m|-r|/r before derating          */
#endif
#ifndef MAG_NORM_SIGMA
#define MAG_NORM_SIGMA          9.0f
#endif
#ifndef MAG_DIP_TOL_RAD
#define MAG_DIP_TOL_RAD         0.35f   /* dip error, rad (~11 deg)           */
#endif
#ifndef MAG_DIP_SIGMA
#define MAG_DIP_SIGMA           9.0f
#endif
/* Fast world-frame innovation: |w - w_slow| against noise.  Catches steps.
 * Relaxed with body rate because mag and quat samples are not simultaneous. */
#ifndef MAG_STEP_SIGMA
#define MAG_STEP_SIGMA          9.5f
#endif
#ifndef MAG_STEP_FLOOR
#define MAG_STEP_FLOOR          0.04f
#endif
#ifndef MAG_STEP_OMEGA_K
#define MAG_STEP_OMEGA_K        0.10f   /* extra tolerance per rad/s          */
#endif

/* Residual soft-iron/model error is orientation dependent.  A fallback
 * diag/sphere fit therefore looks like a slowly moving world vector and can
 * falsely trip the step/drift gates.  Add a bounded tolerance proportional to
 * the measured calibration residual. */
#ifndef MAG_CAL_GATE_K
#define MAG_CAL_GATE_K          3.0f
#endif
#ifndef MAG_CAL_GATE_MIN
#define MAG_CAL_GATE_MIN        0.060f
#endif
#ifndef MAG_CAL_GATE_MAX
#define MAG_CAL_GATE_MAX        0.180f
#endif
/* Slow windowed drift of the FILTERED world vector against a frozen anchor.
 * This is the detector that catches a magnet creeping in: it is tight because
 * the world vector should not move at all. */
#ifndef MAG_DRIFT_WINDOW_S
#define MAG_DRIFT_WINDOW_S      0.50f
#endif
#ifndef MAG_DRIFT_SIGMA
#define MAG_DRIFT_SIGMA         9.0f
#endif
#ifndef MAG_DRIFT_FLOOR_RAD
#define MAG_DRIFT_FLOOR_RAD     0.020f  /* 1.15 deg - never tighter           */
#endif
#ifndef MAG_DRIFT_OMEGA_K
#define MAG_DRIFT_OMEGA_K       0.05f
#endif
#ifndef MAG_DRIFT_RUN
#define MAG_DRIFT_RUN           2u      /* consecutive bad windows            */
#endif
/*
 * Hard bound on how long gate 4 may stay fired against one anchor.  A
 * fired gate suppresses the weight, and the weight is the only thing that
 * could bring the field back to the anchor, so without this the gate is a
 * latch: stale anchor, angle growing every window, no way out except the
 * 90 s stuck-recal and the yaw step that comes with it.
 */
#ifndef MAG_DRIFT_REANCHOR_S
#define MAG_DRIFT_REANCHOR_S    6.0f
#endif
#ifndef MAG_DIAG_HOLD
#define MAG_DIAG_HOLD           100u    /* keep the host diagnostic lit       */
#endif

/* Spatial gradient between the two parts.  The threshold is LEARNED because
 * the useful value depends on how far apart they sit, which this code cannot
 * know.  Unlike rev 1 the learner is NOT gated by its own output. */
#ifndef MAG_GRAD_SIGMA
#define MAG_GRAD_SIGMA          6.0f
#endif
#ifndef MAG_GRAD_FLOOR
#define MAG_GRAD_FLOOR          0.006f
#endif
#ifndef MAG_GRAD_CEIL
#define MAG_GRAD_CEIL           0.150f
#endif
#ifndef MAG_GRAD_FALL
#define MAG_GRAD_FALL           0.02f
#endif
#ifndef MAG_GRAD_RISE
#define MAG_GRAD_RISE           0.0005f
#endif
#ifndef MAG_GRAD_WARMUP
#define MAG_GRAD_WARMUP         200u
#endif
/* The gate can never be tighter than what the inter-sensor map can actually
 * achieve, times this factor.  Without it a poor affine fit makes the gate
 * fire forever. */
#ifndef MAG_GRAD_RES_K
#define MAG_GRAD_RES_K          3.0f
#endif

/* Cross-link consensus (optional, see mag_fusion_join_fleet).  A link whose
 * dip disagrees with the median of the rest of the hand is looking at a local
 * source: nothing else on the glove sees it. */
#ifndef MAG_FLEET_DIP_TOL
#define MAG_FLEET_DIP_TOL       0.12f   /* rad, ~7 deg                        */
#endif
#ifndef MAG_FLEET_NORM_TOL
#define MAG_FLEET_NORM_TOL      0.12f
#endif
#ifndef MAG_FLEET_MIN_PEERS
#define MAG_FLEET_MIN_PEERS     4u
#endif

/* ======================================================================== */
/*  Weighting and the event machine                                         */
/* ======================================================================== */

/* The instantaneous max-of-gates is low-passed before it is allowed to change
 * the weight.  THIS is what stops a single noisy sample from dropping the
 * lock; rev 1 used the raw per-sample value everywhere. */
#ifndef MAG_E_TAU_S
#define MAG_E_TAU_S             0.15f
#endif
/* An instantaneous error this large bypasses the low pass - a real step
 * should be caught in one sample, not in 150 ms. */
#ifndef MAG_E_INSTANT
#define MAG_E_INSTANT           6.0f
#endif
#ifndef MAG_REJECT_AT
#define MAG_REJECT_AT           2.5f    /* normalised error -> weight 0       */
#endif
#ifndef MAG_EVENT_ENTER
#define MAG_EVENT_ENTER         3.0f    /* latch a disturbance above this     */
#endif
#ifndef MAG_SAFE_MAX_ERR
#define MAG_SAFE_MAX_ERR        0.60f   /* refresh the pre-disturbance snapshot */
#endif
#ifndef MAG_RECOVER_S
#define MAG_RECOVER_S           0.20f   /* clean seconds needed to re-trust   */
#endif
#ifndef MAG_HORIZ_MIN
#define MAG_HORIZ_MIN           0.10f   /* min horizontal component / radius  */
#endif

/* ======================================================================== */
/*  Learning window                                                         */
/* ======================================================================== */
/*
 * Everything that sits in the MEASUREMENT path - cal[].offset, cal[].W,
 * cal[].radius and the mag-to-IMU rotation A - is frozen once a model has
 * been committed.  The published vector is A W (raw - offset), so anything
 * that moves those three moves the reported field even though the sensor
 * read exactly the same number.
 *
 * Continuous background learning is wrong for this application, and the
 * failure is not subtle: the estimators only advance while the board turns,
 * because both the ellipsoid bins and the field estimator's attitude
 * decimation are driven by attitude change.  So the model creeps during
 * vigorous motion and is frozen at rest - which is indistinguishable from
 * gyro drift at the output, and is exactly what it was mistaken for.
 *
 * The policy here is: LOCKED by default; open a bounded window only when
 * the model is demonstrably wrong; adopt at most one update per window.
 */
#ifndef MAG_RELEARN_ENTER
#define MAG_RELEARN_ENTER       2.00f   /* physical inconsistency to arm      */
#endif
#ifndef MAG_RELEARN_EXIT
#define MAG_RELEARN_EXIT        1.00f   /* below this the anomaly is over     */
#endif
#ifndef MAG_RELEARN_CONFIRM_S
#define MAG_RELEARN_CONFIRM_S   8.0f    /* it must PERSIST this long          */
#endif
#ifndef MAG_RELEARN_WINDOW_S
#define MAG_RELEARN_WINDOW_S    30.0f   /* hard bound on an open window       */
#endif
/*
 * Minimum LOCKED time between windows.
 *
 * Without this the machine loops: the window times out with the anomaly
 * still present, LOCKED re-arms on that same anomaly, and it reopens - which
 * is continuous learning with extra steps.  The backstop matters because the
 * case where a window fails to resolve the anomaly is precisely the case
 * where relearning is NOT the answer: a magnet that travels with the glove,
 * a cracked solder joint, a sensor running out of range.  Refitting against
 * any of those repeatedly is the worst thing the module can do.
 *
 * With the defaults the measurement path can change at most once per ~2.5
 * minutes, and only while a physical inconsistency has persisted for 8 s.
 */
#ifndef MAG_RELEARN_COOLDOWN_S
#define MAG_RELEARN_COOLDOWN_S  120.0f
#endif
/*
 * A relearn refit replaces the committed model outright, so it must clear a
 * higher coverage bar than the bootstrap fit does.  MAG_CAL_BINS_SPH (6 of
 * 24) is a quarter of the sphere - enough to bootstrap from nothing, nowhere
 * near enough to justify discarding a model that has been working.
 */
#ifndef MAG_RELEARN_BINS
#define MAG_RELEARN_BINS        14u
#endif

#define MAG_LEARN_BOOTSTRAP     0u      /* no committed model yet - wide open */
#define MAG_LEARN_LOCKED        1u      /* normal operation - nothing moves   */
#define MAG_LEARN_ARMED         2u      /* anomaly seen, waiting on it        */
#define MAG_LEARN_OPEN          3u      /* window open, one adoption allowed  */
#ifndef MAG_STALE_S
#define MAG_STALE_S             0.50f
#endif

/* --- rebaselining onto a genuinely new environment --------------------- */
#ifndef MAG_REBASE_ERR_MAX
#define MAG_REBASE_ERR_MAX      6.0f
#endif
#ifndef MAG_REBASE_CAND_TOL
#define MAG_REBASE_CAND_TOL     0.70f
#endif
#ifndef MAG_REBASE_S
#define MAG_REBASE_S            4.0f    /* single magnetometer                */
#endif
#ifndef MAG_REBASE_DUAL_S
#define MAG_REBASE_DUAL_S       1.0f    /* both parts agree -> uniform field  */
#endif
#ifndef MAG_HEAD_REBASE_S
#define MAG_HEAD_REBASE_S       1.0f    /* clean heading confirmation          */
#endif

/* --- coasting drift compensation -------------------------------------- */
#ifndef MAG_COAST_WIN_S
#define MAG_COAST_WIN_S         5.0f
#endif
#ifndef MAG_COAST_TAU_S
#define MAG_COAST_TAU_S         20.0f
#endif
#ifndef MAG_COAST_MAX_RAD_S
#define MAG_COAST_MAX_RAD_S     0.0087f /* +/- 0.5 deg/s                      */
#endif
#ifndef MAG_COAST_WARMUP
#define MAG_COAST_WARMUP        3u
#endif
#ifndef MAG_COAST_OMEGA_MAX
#define MAG_COAST_OMEGA_MAX     2.0f
#endif
#ifndef MAG_COAST_MAX_RAD
#define MAG_COAST_MAX_RAD       0.26f   /* bound the open-loop extrapolation  */
#endif
/* A drift window no longer has to be perfect end to end - it only has to be
 * clean on average.  Requiring 500 consecutive flawless samples, as rev 1
 * did, is unreachable on real hardware, so the drift estimate never armed. */
#ifndef MAG_COAST_BAD_FRAC
#define MAG_COAST_BAD_FRAC      0.10f
#endif

/* ======================================================================== */
/*  Mag-to-IMU alignment / earth field / body offset estimator              */
/* ======================================================================== */
/*
 *      A * m_i = R_i^T * B + d
 *
 * A  3x3 rotation, magnetometer axes -> IMU axes      (rev 2, was assumed I)
 * B  earth field in the world frame, constant
 * d  body-fixed offset: board magnetisation, residual hard iron
 * R_i from the 6-axis solution, which no magnet can touch
 *
 * Solved by alternation: given A the (B,d) problem is the 6x6 linear system
 * rev 1 already had; given (B,d) the A problem is a Procrustes fit.  Three
 * rounds converge.  The accumulators carry exponential forgetting so samples
 * taken while a magnet was present age out instead of poisoning the window
 * forever, which is what left rev 1 unable to recover.
 *
 * dip comes out of B and is therefore YAW-FREE and averaged over every
 * attitude the hand has visited - rather than rev 1's mean over the first two
 * seconds in whatever single pose the glove happened to start in.
 */
#ifndef MAG_FE_ENABLE
#define MAG_FE_ENABLE           1
#endif
#ifndef MAG_FE_STEP_RAD
#define MAG_FE_STEP_RAD         0.05f   /* attitude decimation, ~3 deg        */
#endif
#ifndef MAG_FE_TAU_S
#define MAG_FE_TAU_S            45.0f   /* forgetting time constant           */
#endif
#ifndef MAG_FE_MIN_N
#define MAG_FE_MIN_N            40.0f   /* effective attitudes before solving */
#endif
#ifndef MAG_FE_OBS_MAX
#define MAG_FE_OBS_MAX          0.80f   /* 1.0 = the board never turned       */
#endif
#ifndef MAG_FE_RES_MAX
#define MAG_FE_RES_MAX          0.25f   /* solve residual, field radii        */
#endif
#ifndef MAG_FE_FIELD_TOL
#define MAG_FE_FIELD_TOL        0.25f   /* |B| must land near 1.0             */
#endif
#ifndef MAG_FE_EVERY
#define MAG_FE_EVERY            25u     /* accepted samples between solves    */
#endif
/* Alignment adoption */
#ifndef MAG_FE_A_RES_MAX
#define MAG_FE_A_RES_MAX        0.05f   /* A is in the measurement path: strict */
#endif
#ifndef MAG_FE_D_RES_MAX
#define MAG_FE_D_RES_MAX        0.20f   /* body-offset solve may be noisier      */
#endif
#ifndef MAG_FE_A_STEP_MAX
#define MAG_FE_A_STEP_MAX       0.05f   /* max rotation adopted at once, rad  */
#endif
#ifndef MAG_FE_A_MAX
#define MAG_FE_A_MAX            1.80f   /* allow a 90 deg axis swap, refuse more */
#endif
/* Body-offset (magnetisation) adoption */
#ifndef MAG_FE_D_MIN
#define MAG_FE_D_MIN            0.040f  /* fold offsets above this, in radii  */
#endif
#ifndef MAG_FE_D_MAX
#define MAG_FE_D_MAX            2.50f
#endif
#ifndef MAG_STUCK_RECAL_S
#define MAG_STUCK_RECAL_S       90.0f   /* last resort: reset magnetic datum  */
#endif

/* ======================================================================== */
/*  Inter-magnetometer map (affine, was a pure rotation in rev 1)           */
/* ======================================================================== */
/*
 * u1 ~ Maff * u2 + caff.  The two parts sit in different soft-iron
 * environments - different neighbours, different distance to the shield - so
 * their difference is NOT a pure rotation.  Forcing one, as rev 1 did, leaves
 * an attitude-dependent residual that either blocks alignment entirely or
 * pushes the learned gradient threshold to its ceiling, in both cases
 * throwing away the second magnetometer.
 */
#ifndef MAG_MAP_STEP
#define MAG_MAP_STEP            0.06f
#endif
#ifndef MAG_MAP_MIN_N
#define MAG_MAP_MIN_N           120.0f
#endif
#ifndef MAG_MAP_TAU_S
#define MAG_MAP_TAU_S           120.0f
#endif
#ifndef MAG_MAP_EVERY
#define MAG_MAP_EVERY           64u
#endif
#ifndef MAG_MAP_RES_MAX
#define MAG_MAP_RES_MAX         0.08f
#endif
#ifndef MAG_DUAL_AVERAGE
#define MAG_DUAL_AVERAGE        1
#endif

/* ======================================================================== */
/*  Reference field tracking                                                */
/* ======================================================================== */
#ifndef MAG_REF_TAU_S
#define MAG_REF_TAU_S           60.0f
#endif
#ifndef MAG_REF_DRIFT_MAX
#define MAG_REF_DRIFT_MAX       0.20f
#endif
#ifndef MAG_REF_DIP_SLACK
#define MAG_REF_DIP_SLACK       0.50f
#endif
#ifndef MAG_DECLINATION_DEG
#define MAG_DECLINATION_DEG     0.0f
#endif
#ifndef MAG_YAW_DATUM_DEG
#define MAG_YAW_DATUM_DEG       0.0f
#endif

/* ======================================================================== */
/*  Calibration                                                             */
/* ======================================================================== */

/* Hold a second normal matrix and cross-validate on it.  This is the only
 * honest test of whether a richer model generalises or has simply memorised
 * the patch of sphere it was shown.  Costs 220 bytes per sensor. */
#ifndef MAG_CAL_CV
#define MAG_CAL_CV              1
#endif
#ifndef MAG_CAL_AUTO
#define MAG_CAL_AUTO            1
#endif
#ifndef MAG_CAL_SPHERE_MIN
#define MAG_CAL_SPHERE_MIN      60u
#endif
#ifndef MAG_CAL_DIAG_MIN
#define MAG_CAL_DIAG_MIN        150u
#endif
#ifndef MAG_CAL_FULL_MIN
#define MAG_CAL_FULL_MIN        300u
#endif
#ifndef MAG_CAL_WARMUP
#define MAG_CAL_WARMUP          60u
#endif
#ifndef MAG_CAL_STEP
#define MAG_CAL_STEP            0.04f
#endif
/* Spherical-cap coverage.  24 bins over the sphere (6 faces x 4 quadrants).
 * Counting occupied bins is a direct measure of whether the parameters are
 * observable; rev 1's per-axis min/max span is not - a single large circular
 * sweep fills the spans while leaving whole directions unvisited. */
#ifndef MAG_CAL_BINS_SPH
#define MAG_CAL_BINS_SPH        6u
#endif
#ifndef MAG_CAL_BINS_DIAG
#define MAG_CAL_BINS_DIAG       11u
#endif
#ifndef MAG_CAL_BINS_FULL
#define MAG_CAL_BINS_FULL       16u
#endif
#ifndef MAG_CAL_COVERAGE
#define MAG_CAL_COVERAGE        1.00f   /* per-axis span, in radii            */
#endif
#ifndef MAG_CAL_FULL_GAIN
#define MAG_CAL_FULL_GAIN       0.90f   /* CV gain the full model must show   */
#endif
#ifndef MAG_CAL_DIAG_GAIN
#define MAG_CAL_DIAG_GAIN       0.92f
#endif
#ifndef MAG_CAL_FULL_RIDGE
#define MAG_CAL_FULL_RIDGE      0.010f
#endif
#ifndef MAG_CAL_MAX_CROSS
#define MAG_CAL_MAX_CROSS       0.40f
#endif
#ifndef MAG_CAL_MAX_RESIDUAL
#define MAG_CAL_MAX_RESIDUAL    0.08f
#endif
#ifndef MAG_CAL_GAIN_MIN
#define MAG_CAL_GAIN_MIN        0.65f
#endif
#ifndef MAG_CAL_GAIN_MAX
#define MAG_CAL_GAIN_MAX        1.55f
#endif
#ifndef MAG_CAL_RETRY_EVERY
#define MAG_CAL_RETRY_EVERY     128u
#endif
#ifndef MAG_CAL_REFINE
#define MAG_CAL_REFINE          1
#endif
#ifndef MAG_CAL_REFINE_EVERY
#define MAG_CAL_REFINE_EVERY    150u
#endif
/* A refit is adopted only if it beats the COMMITTED parameters scored on the
 * SAME data.  Rev 1 compared the new residual against the residual the old
 * parameters had scored on a DIFFERENT window, which is not a comparison at
 * all and could lock the calibration onto the first lucky fit forever. */
#ifndef MAG_CAL_REFINE_GAIN
#define MAG_CAL_REFINE_GAIN     0.97f
#endif
#ifndef MAG_CAL_REFINE_MAX_N
#define MAG_CAL_REFINE_MAX_N    4000u
#endif
/* ...and the window also expires on TIME, so the fit follows the thermal
 * drift of a magnetometer strapped to a warm hand.  Rev 1's sample-count-only
 * window could span hours if the hand was mostly still. */
#ifndef MAG_CAL_WINDOW_S
#define MAG_CAL_WINDOW_S        300.0f
#endif

/* ======================================================================== */
/*  Static output stabiliser                                                */
/* ======================================================================== */
#ifndef MAG_STAB_STATIC_TAU_S
#define MAG_STAB_STATIC_TAU_S   4.0f
#endif
#ifndef MAG_STAB_MOTION_TAU_S
#define MAG_STAB_MOTION_TAU_S   0.004f
#endif
#ifndef MAG_STAB_BREAKOUT_RAD
#define MAG_STAB_BREAKOUT_RAD   0.02f
#endif
#ifndef MAG_STAB_HOLD
#define MAG_STAB_HOLD           12u
#endif
#ifndef MAG_STAB_GYRO_MOVE_RAW
#define MAG_STAB_GYRO_MOVE_RAW  20.0f
#endif
#ifndef MAG_STAB_GYRO_LPF
#define MAG_STAB_GYRO_LPF       0.25f
#endif
#ifndef MAG_STAB_GYRO_BIAS_LPF
#define MAG_STAB_GYRO_BIAS_LPF  0.005f
#endif

/* ======================================================================== */
/*  Status / result codes                                                   */
/* ======================================================================== */

#define MAG_ST_NO_CAL           0u
#define MAG_ST_CALIBRATING      1u
#define MAG_ST_ACQUIRING        2u
#define MAG_ST_LOCKED           3u
#define MAG_ST_DEGRADED         4u
#define MAG_ST_COASTING         5u

#define MAG_CAL_NONE            0u
#define MAG_CAL_OK              1u
#define MAG_CAL_ERR_SAMPLES     2u
#define MAG_CAL_ERR_COVERAGE    3u
#define MAG_CAL_ERR_SOLVE       4u
#define MAG_CAL_ERR_RESIDUAL    5u
#define MAG_CAL_ERR_GAIN        6u
#define MAG_CAL_CLEARED         7u

/* ======================================================================== */
/*  State                                                                   */
/* ======================================================================== */

#define MF_NB                   10u
#define MF_NSYM                 ((MF_NB * (MF_NB + 1u)) / 2u)   /* 55 */

#if MAG_CAL_CV
#define MF_NSET                 2u
#else
#define MF_NSET                 1u
#endif

#define MAG_CAL_BLOB_VERSION    4u

#define MAG_BLOB_F_CAL1         0x01u
#define MAG_BLOB_F_CAL2         0x02u
#define MAG_BLOB_F_MAP          0x04u
#define MAG_BLOB_F_DIP          0x08u
#define MAG_BLOB_F_ALIGN        0x10u
#define MAG_BLOB_F_VERIFIED     0x20u
#define MAG_BLOB_F_MODEL_FULL   0x40u
#define MAG_BLOB_F_MODEL_DIAG   0x80u

/* Persistable calibration for one link. */
typedef struct mag_cal_blob {
    uint32_t version;
    float    offset[2][3];      /* hard-iron centre, raw LSB, per sensor      */
    float    W[2][9];           /* soft-iron matrix, row major, det == 1      */
    float    radius[2];
    float    map[9];            /* u1 ~ map * u2 + map_c                      */
    float    map_c[3];
    float    align[9];          /* mag -> IMU rotation for the primary        */
    float    dip;
    uint32_t flags;
} mag_cal_blob_t;

/* Calibration state for ONE magnetometer. */
typedef struct mag_cal {
    float    offset[3];
    float    W[9];
    float    radius;
    float    quality;           /* algebraic residual of the committed fit    */
    float    geo_q;             /* running |(|m|-r)|/r of the committed fit   */
    uint8_t  valid;
    uint8_t  result;
    uint8_t  model;             /* 0 sphere, 1 diagonal, 2 full ellipsoid     */

    float    M[MF_NSET][MF_NSYM];
    float    c0[3];
    float    s0;
    float    lo[3], hi[3];
    float    last[3];
    float    fir[MAG_BODY_FIR > 1u ? (MAG_BODY_FIR - 1u) : 1u][3];
    uint32_t bins;              /* occupied spherical caps, bitmask           */
    uint8_t  bins_req;          /* caps required to commit; 0 = MAG_CAL_BINS_SPH */
    uint32_t n;
    uint32_t seen;
    uint32_t next_try;
    uint32_t revision;
    float    window_s;
    uint8_t  set;               /* which normal matrix the next sample joins  */
    uint8_t  active;            /* 0 idle, 1 explicit sweep, 2 background     */
    uint8_t  frozen;
    uint8_t  restarts;
    uint8_t  fir_n;
} mag_cal_t;

/* Joint mag-to-IMU rotation / earth field / body-offset estimator. */
typedef struct mag_field_est {
    float    n;                 /* forgetting-weighted count                  */
    float    SR[9];             /* sum R_i                                    */
    float    T[27];             /* T[j*9+k*3+l] = sum R_i[j][k] * m_i[l]      */
    float    Sm[3];             /* sum m_i        (body, pre-alignment)       */
    float    Smm;               /* sum |m_i|^2                                */
    float    last_q[4];
    float    B[3];              /* last solved world field                    */
    float    d[3];              /* last solved body offset                    */
    float    res;               /* last solve residual                        */
    float    obs;               /* last observability metric                  */
    uint32_t since;
    uint16_t fixes;             /* body offsets folded in                     */
    uint8_t  d_run;             /* consecutive strong body-offset solves      */
    uint8_t  have_last;
    uint8_t  field_valid;       /* B (and therefore dip) is trustworthy       */
    uint8_t  align_valid;       /* A has been adopted at least once           */
} mag_field_est_t;


#define MAG_LOCAL_BINS          8u

typedef struct mag_local_bin {
    float    d[3];              /* validated body-frame offset for this face */
    float    spread;            /* max deviation of accepted estimates       */
    uint8_t  valid;
    uint8_t  committed;
    uint8_t  count;
} mag_local_bin_t;

typedef struct mag_fusion {
    mag_cal_t cal[2];

    /* ---- mag -> IMU rotation, primary sensor ---- */
    float    A[9];
    mag_field_est_t fe;

    /* ---- trusted field and local attitude-face recovery ---- */
    float    safe_B[3];         /* last trusted world field                  */
    mag_local_bin_t local_bin[MAG_LOCAL_BINS];
    uint8_t  safe_B_valid;
    uint8_t  local_commit_done;
    float    local_rearm_s;

    /* ---- affine map from the secondary to the primary ---- */
    float    map[9], map_c[3];
    float    map_A[10];         /* 4x4 symmetric normal matrix, packed        */
    float    map_b[12];         /* rhs, 3 outputs x 4                         */
    float    map_n;
    float    map_last[3];
    float    map_res;
    uint32_t map_since;
    uint8_t  map_valid;
    uint8_t  have2;

    /* ---- latest normalised measurements, pre-alignment ---- */
    float    u1[3], u2[3];
    uint32_t seq1, seq2, seq_used;
    float    stale2_s;
    uint8_t  u1_valid, u2_valid;

    /* ---- world-frame chain ---- */
    float    wring[MAG_WORLD_AVG_MAX][3];
    float    wsum[3];
    float    w_now[3];          /* unfiltered world vector                    */
    float    w_lp[3];           /* boxcar output                              */
    float    w_prev[3], w_prev2[3];
    float    w_anchor[3];
    float    anchor_s;
    uint32_t wring_n;
    uint8_t  wring_i;
    uint8_t  w_hist;
    uint8_t  w_lp_valid, anchor_valid;

    /* ---- measured noise of the world unit vector ---- */
    float    sigma;
    uint32_t sigma_n;

    /* ---- adaptive gradient threshold ---- */
    float    grad_mu, grad_var, grad_thr, grad_now;
    uint32_t grad_n;

    /* ---- gates ---- */
    float    e_norm, e_dip, e_step, e_drift, e_grad, e_fleet;
    float    e_inst, e_lp;
    uint32_t drift_run;
    uint16_t diag_hold;

    /* ---- yaw Kalman ---- */
    float    yaw_corr, yaw_P;
    float    yaw_blend;          /* temporary output-only datum handoff       */
    float    mag_datum_offset;
    float    weight;
    float    good_s;
    uint8_t  yaw_valid, yaw_blend_valid, rejecting, cal_commit_pending, status;

    /* ---- coasting drift compensation ---- */
    float    drift_rate, drift_anchor, drift_acc_s, drift_bad_s, coast_acc;
    uint32_t drift_n;
    uint8_t  drift_valid, drift_anchored;

    /* ---- pre-disturbance snapshot and rebaselining ---- */
    float    safe_norm, safe_dip, safe_yaw_corr, safe_datum;
    float    cand_norm, cand_dip, cand_s, head_rebase_s, yaw_hold_s, stuck_s;
    float    yaw_trust;          /* continuous fast-path trust, 0..1       */
    float    drift_fired_s;      /* how long gate 4 has been fired           */
    uint8_t  safe_valid, safe_yaw_valid, hard_event, cand_valid;
    uint8_t  yaw_large_active;   /* hysteresis for large-innovation path   */

    /* ---- static attitude stabiliser ---- */
    float    q_stab[4];
    float    gyro_activity, gyro_bias[3];
    uint32_t motion_hold;
    uint8_t  q_stab_valid, gyro_valid;

    /* ---- reference field ---- */
    float    ref_norm, ref_norm0, ref_dip, ref_dip0;
    float    ref_dip_acc;
    uint32_t ref_dip_n;
    uint8_t  ref_dip_valid;

    /* ---- learning window ---- */
    float    learn_hold_s;       /* time in ARMED, or time in OPEN            */
    float    learn_cool_s;       /* LOCKED time still owed before re-arming   */
    uint16_t learn_opens;        /* windows opened since boot (diagnostic)    */
    uint8_t  learn_state;        /* MAG_LEARN_*                               */
    uint8_t  learn_adopted;      /* a model update landed in this window      */

    /* ---- previous-sample state ---- */
    float    q_prev[4];
    uint64_t ts_prev;
    float    stale_s, omega, dip_now, norm_now;
    uint8_t  have_prev;

    /* ---- direction-only disturbance detector ---- */
    float    m_dir_prev[3];
    float    q_dir_prev[4];
    float    dir_res, dir_lock_s;
    uint8_t  m_dir_valid;

    /* ---- cross-link consensus ---- */
    uint8_t  fleet_idx, fleet_joined;

    /* ---- 大修正路径诊断（供录制/上位机观测）------------------------------
     * 让"修正什么时候触发、落到哪一档、冻结多久、回正多快、这一拍实际修了
     * 多少"全部可观测。没有这些，三档逻辑在外面根本没法验证。
     * 全部是只读的诊断镜像，不参与任何控制。 */
    float    yawd_innov_deg;   /* 欠账 |target - yaw_corr|，度               */
    float    yawd_relax;       /* 航向域门放宽系数 1.0 / 1.25 / 1.5          */
    float    yawd_hold_s;      /* 本次冻结时长，0 = 不冻结                    */
    float    yawd_slew_dps;    /* 本次 slew 限速，deg/s                      */
    float    yawd_step_deg;    /* 这一拍实际修正量，度                        */
    float    yawd_trust;       /* 快路径信任度 0..1                          */
    uint8_t  yawd_large;       /* 1 = 在大修正路径                            */
    uint8_t  yawd_snap;        /* 1 = 走了全速回零档                          */
    uint8_t  yawd_clean;       /* clean 谓词是否通过                          */
    uint8_t  yawd_nis_big;     /* 1 = 因 NIS 而进入（而非欠账）               */

    /* ---- 权威磁航向 & Kalman 内部量（补齐 yaw 修正链路）---- */
    float    yawd_traw_deg;    /* ★固件自己算的磁航向（权威，未减 datum）    */
    float    yawd_target_deg;  /* 减掉 datum 后的修正目标                    */
    float    yawd_out_yaw_deg; /* ★输出 yaw（矫正后四元数的 yaw）           */
    float    yawd_err_deg;     /* ★traw - out_yaw，欠账的权威版本            */
    float    yawd_nis;         /* 归一化新息平方 NIS                         */
    float    yawd_yawP;        /* yaw 卡尔曼协方差 P                         */
    float    yawd_R;           /* 测量噪声 R                                 */
    float    yawd_K;           /* 卡尔曼增益 K                               */
    float    yawd_sig;         /* 噪声估计 sig_lp                            */
    float    yawd_horiz;       /* 磁场水平分量                               */
    float    yawd_omega_dps;   /* 角速度 deg/s                               */
    float    yawd_blend_deg;   /* datum 交接混合量                           */
    float    yawd_dirlock_ms;  /* 方向残差锁定剩余时间 ms                    */
    float    yawd_geo;         /* 场强误差（=P表 geo）                       */
    uint8_t  yawd_kalman;      /* 1 = 这一拍走了普通卡尔曼路径               */
    uint8_t  yawd_status;      /* mag_fusion status                          */
    float    yawd_gamerv_yaw_deg; /* ★未修正的 GAMERV yaw（输出 yaw 的基底）  */
    float    yawd_mag_raw[3];     /* ★真正的原始磁矢量（未标定、未 FIR）      */
    /* ---- 方向门运动自适应 ---- */
    float    dir_strength_eff;    /* 当前生效的方向门强度（0..1）             */
} mag_fusion_t;

/* ======================================================================== */
/*  API                                                                     */
/* ======================================================================== */

void    mag_fusion_init(mag_fusion_t *f);

/* Drop the heading lock and learned reference; keep calibration + alignment. */
void    mag_fusion_reset_runtime(mag_fusion_t *f);

/* Rebuild the magnetic reference while keeping the output yaw continuous. */
void    mag_fusion_rebase_runtime(mag_fusion_t *f);

void    mag_fusion_cal_start(mag_fusion_t *f);
uint8_t mag_fusion_cal_finish(mag_fusion_t *f);
void    mag_fusion_cal_clear(mag_fusion_t *f);

/* One raw sample from the primary / secondary magnetometer.  m_out receives
 * the calibrated vector in raw LSB units, in the IMU frame (pass NULL if not
 * needed).  Returns 1 if the output is calibrated, 0 if still raw. */
uint8_t mag_fusion_on_mag (mag_fusion_t *f, const float m_raw[3], float m_out[3]);
uint8_t mag_fusion_on_mag2(mag_fusion_t *f, const float m_raw[3], float m_out[3]);

/* Raw gyro, in whatever LSB the BHI360 hands over. */
void    mag_fusion_on_gyro(mag_fusion_t *f, const float g_raw[3]);

/* One GAMERV quaternion as (w,x,y,z) plus its BHY2 FIFO timestamp (15625 ns
 * ticks).  Corrects yaw in place and returns MAG_ST_*. */
uint8_t mag_fusion_on_quat(mag_fusion_t *f, uint64_t ts_ticks, float q[4]);

/* ---- optional cross-link consensus --------------------------------------
 * Call once per link after init, with a unique index.  Links that have joined
 * publish their yaw-invariant view of the field (|B| and dip) into a shared
 * table and use the median of the others as both a fast reference and a
 * disturbance detector with a whole-hand baseline.  Never calling it leaves
 * every link fully independent, exactly as before. */
#ifndef MAG_FLEET_MAX
#define MAG_FLEET_MAX           24u
#endif
void    mag_fusion_join_fleet(mag_fusion_t *f, uint8_t index);

/* Packed into bowie_Quat.accuracy - UNCHANGED from rev 1:
 *   status + 10*cal_result + 100*diag + 0.99*weight */
float   mag_fusion_accuracy(const mag_fusion_t *f);

uint8_t mag_fusion_dual_ready(const mag_fusion_t *f);
float   mag_fusion_stuck_seconds(const mag_fusion_t *f);

/* ---- learning window ----
 * mag_fusion_relearn() forces a window open from the host.  Use it when the
 * board is known to have changed - a magnet was brought near it, hardware was
 * swapped, the glove moved to a different vehicle - not as a periodic tidy-up.
 * mag_fusion_lock() slams it shut again and is what a diagnostic build calls
 * to pin the measurement path completely. */
void     mag_fusion_relearn(mag_fusion_t *f);
void     mag_fusion_lock(mag_fusion_t *f);
uint8_t  mag_fusion_learn_state(const mag_fusion_t *f);   /* MAG_LEARN_*      */
uint16_t mag_fusion_learn_opens(const mag_fusion_t *f);

/* Heading diagnostics.  mag_fusion_datum_deg() is the one to watch: it
 * must not move during ordinary use, because every degree it gains is a
 * degree of gyro drift that has been made permanent. */
float    mag_fusion_datum_deg(const mag_fusion_t *f);
float    mag_fusion_yaw_corr_deg(const mag_fusion_t *f);

/* Reporting helpers for glove_print_mag_status(). */
float   mag_fusion_sigma(const mag_fusion_t *f);
float   mag_fusion_align_deg(const mag_fusion_t *f);
uint16_t mag_fusion_fixes(const mag_fusion_t *f);

uint8_t mag_fusion_save(const mag_fusion_t *f, mag_cal_blob_t *blob);
uint8_t mag_fusion_load(mag_fusion_t *f, const mag_cal_blob_t *blob);

#ifdef __cplusplus
}
#endif

#endif /* INC_GLOVE_MAG_FUSION_H_ */

