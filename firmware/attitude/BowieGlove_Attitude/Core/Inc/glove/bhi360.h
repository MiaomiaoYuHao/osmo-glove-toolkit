#ifndef __BHI360_H__
#define __BHI360_H__

#ifdef __cplusplus
extern "C" {
#endif

//#include "main.h"
#include "bhy2.h"
#include "bhy2_parse.h"
#include "mag_fusion.h"


#define WORK_BUFFER_SIZE  2048

/* 6-axis (accel + gyro) rotation vector.  Yaw drifts but is immune to magnetic
 * disturbance; mag_fusion adds the absolute heading on top of it.  Do NOT
 * switch this to BHY2_SENSOR_ID_RV - that would hand yaw back to the BSX
 * fusion inside the BHI360 and the disturbance gating here would have nothing
 * left to protect. */
#define QUAT_SENSOR_ID    BHY2_SENSOR_ID_GAMERV

/* Raw, uncompensated magnetometer.  Calibration is done in mag_fusion so that
 * every link gets its own hard/soft-iron correction. */
#define MAG_ID            BHY2_SENSOR_ID_MAG_PASS

/*
 * Second BMM350 (driver 18, i2c addr 0x15 - the "Meta" instance in the
 * Bosch_Shuttle3_BHI360_BMM350C_Poll_Meta_12 firmware).
 *
 * Which VIRTUAL sensor id that physical driver is published on is decided by
 * the firmware image, not by this code, so it cannot be derived here.  Leave
 * MAG2_ID at 0 and the driver probes the candidates below and reports what it
 * finds; init_glove() also prints every virtual sensor the firmware exposes,
 * so you can read the right number off the console and pin it here.
 *
 * Everything degrades cleanly: with no second magnetometer the fusion runs
 * single-part, which is fully functional - it just loses the spatial-gradient
 * detector and the fast dual-confirmed rebaseline.
 */
#ifndef MAG2_ID
#define MAG2_ID           0            /* 0 = probe automatically */
#endif

/* Probed in this order; the first one present that is not MAG_ID wins. */
#define MAG2_CANDIDATES   { BHY2_SENSOR_ID_MAG, BHY2_SENSOR_ID_MAG_WU, \
                            BHY2_SENSOR_ID_MAG_RAW, 101, 102, 160, 161 }

extern uint8_t glove_mag2_id;          /* 0 until probed / not present */

/*
 * Print RAW alongside the published CAL vector, plus the model revision
 * counters.  Off by default - it is a bring-up and regression tool, not
 * something to ship.  1 line per MAG_DIAG_EVERY samples per link, so raise
 * the divider before enabling it on more than one or two links.
 */
#ifndef MAG_DIAG_PRINT
#define MAG_DIAG_PRINT    0
#endif
#ifndef MAG_DIAG_EVERY
#define MAG_DIAG_EVERY    100u         /* ~1 Hz at a 100 Hz mag rate */
#endif

/* Raw gyro is used only for motion detection in the static stabilizer. */
#define GYRO_ID           BHY2_SENSOR_ID_GYRO
#define ACC_ID            BHY2_SENSOR_ID_ACC

/* ---- 重放通道 ----------------------------------------------------------
 * 用合成 sensor_id 把算法的【原始输入】也送给主机，让一次录制可以离线重放
 * 整个算法。它们不参与任何控制，只是并行的一份输入副本。
 *
 *   240  原始 GAMERV 四元数（修正/稳定之前）—— 不可从输出反推（稳定器是低通）
 *   241  原始磁矢量（FIR 之前）
 *   242  原始陀螺
 *   243  原始加速度计
 * ---------------------------------------------------------------------- */
#define REPLAY_QUAT_ID    240u
#define REPLAY_MAG_ID     241u
#define REPLAY_GYRO_ID    242u
#define REPLAY_ACC_ID     243u
#define REPLAY_MAG2_ID    244u
#define REPLAY_CAL_ID     245u
#define REPLAY_STATE_ID   246u
#define REPLAY_YF_ID      247u
#define REPLAY_YAW_ID     248u
#define REPLAY_YAW2_ID    249u
#define REPLAY_CONFIG_ID  250u
#define REPLAY_EVENT_ID   251u
#define REPLAY_HEALTH_ID  252u
#define REPLAY_CLOCK_ID   253u

#define REPLAY_SCHEMA_VERSION 2u
#define REPLAY_FW_TAG         0x20260921u

/* 1 = 并行镜像算法输入和所有数据-only诊断。关闭后回到纯生产路径。 */
#ifndef REPLAY_DATA_ONLY
#define REPLAY_DATA_ONLY  1
#endif

/* Data-only D path keeps no text, but command/status text (P/H/J/G) stays
 * available because the existing host UI parses it for calibration state. */
#define REPLAY_PRINTF(...) printf(__VA_ARGS__)
#ifndef REPLAY_STREAM_ENABLE
#define REPLAY_STREAM_ENABLE 1
#endif

#ifndef BHI_DIAG_PRINT
#define BHI_DIAG_PRINT    0
#endif
#ifndef BHI_DIAG_EVERY
#define BHI_DIAG_EVERY    100u
#endif

/*
 * NOTE: the first eight members are positionally initialised in glove.c
 *       ({finger, link, sensor_id, mux_chan, dev_addr, count, init, dev}).
 *       Keep their order.
 */
typedef struct mag_data_dev {

	uint8_t finger;
	uint8_t link;
	uint8_t sensor_id;
	uint8_t mux_chan;
	uint8_t dev_addr;
	uint32_t count;
	bool init;
	struct bhy2_dev *dev;

	mag_fusion_t fuse;      /* calibration + disturbance-gated yaw correction */

	uint16_t diag_div;      /* MAG_DIAG_PRINT rate divider */

	/* Data-only replay bookkeeping.  These fields are not part of mag_fusion_t. */
	uint32_t replay_mag_count;
	uint32_t replay_gyro_count;
	uint32_t replay_acc_count;
	uint32_t replay_mag2_count;
	uint64_t replay_last_ts;
	uint64_t mag_last_ts;
	uint8_t  mag_last_ts_valid;
	uint8_t  replay_pad0;
	uint16_t replay_pad1;
	uint32_t replay_mag_dup_drop;
	uint32_t replay_cal_rev_prev;
	uint8_t  replay_hard_prev;
	uint8_t  replay_reject_prev;
	uint8_t  replay_learn_prev;
	uint8_t  replay_health_div;

#if BHI_DIAG_PRINT
	uint16_t diag_gyro_div;
	float    diag_gyro_corr[3];
	uint64_t diag_gyro_corr_ts;
	uint8_t  diag_gyro_corr_valid;
	uint8_t  diag_gyro_corr_available;
#endif

} t_mag_info;

int8_t bhi360_init(struct bhy2_dev *dev, struct mag_data_dev *device);

int8_t bhi360_upload_firmware(uint8_t boot_stat, struct bhy2_dev *dev);
int8_t upload_firmware_partly(struct bhy2_dev *dev);


void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_magnetometer(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_magnetometer2(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_gyroscope(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_accelerometer(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_gyroscope_corrected(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void bhi360_print_sensor_info(struct bhy2_dev *dev, uint8_t link);

/**
* @brief Function to get the Post Mortem data
* @param[in] pminfo     : Post Mortem Structure
* @param[in] bhy2       : Device reference
* @return API error codes
*/
int8_t get_post_mortem_data(struct bhy2_post_mortem *pminfo, struct bhy2_dev *bhy2);

#ifdef __cplusplus
}
#endif

#endif /* __BHI360_H__ */
