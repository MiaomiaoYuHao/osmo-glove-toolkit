
/*
 * glove.c
 *
 *      Author: Tess Hellebrekers
 *      Based of off metahand.h by Mike Lambeta
 */

#include "main.h"
//#include "fmpi2c.h"
#include "i2c.h"
#include <math.h>
#include <stdio.h>
#include <stddef.h>

#include "glove/glove.h"
#include "glove/bhi360.h"
#include "glove/common.h"
#include "comms/bowie.pb.h"
#include "comms/cobs.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "tusb.h"


//easy reference for init below
//typedef struct mag_data_dev {
//
//	uint8_t finger;
//	uint8_t link;
//	uint8_t sensor_id; //remove?
//	uint8_t mux_chan;
//	uint8_t dev_addr;
//	uint8_t count;
//	bool init;
//	struct bhy2_dev *dev;
//} t_mag_info;


struct bhy2_dev bhy2_devices[NUM_DEVICES] = {0};
// Note on sensor_id, until bosch updates firmware at end of month, we do not have access to sensor_id=2
//struct mag_data_dev glove_devices[NUM_DEVICES] = {
//		{bowie_Finger_pinky, 	1, 1, 0, 0x3B, 0, 0, NULL},
//};

// working on blue board 2/24/24
//struct mag_data_dev glove_devices[NUM_DEVICES] = {
//		{bowie_Finger_thumb, 	3, 1, 0, 0x0D, 0, 0, NULL},
//		{bowie_Finger_index, 	3, 1, 2, 0x3A, 0, 0, NULL},
//		{bowie_Finger_middle, 	1, 1, 3, 0x3A, 0, 0, NULL},
//		{bowie_Finger_index, 	1, 1, 5, 0x3A, 0, 0, NULL}
//};

//#define BOWIE_LINK_PER_FINGER	4
//#define BOWIE_FINGER_PINKY	0
//#define BOWIE_FINGER_RING	4
//#define BOWIE_FINGER_MIDDLE	8
//#define BOWIE_FINGER_INDEX	12
//#define BOWIE_FINGER_THUMB	16

////this is the final layout for full glove
uint8_t glove_mag2_id = 0;

/* Persistent per-link magnetic calibration.  STM32F446RE sector 7 is the
 * final 128 KB of flash and is outside the linked image, so firmware updates
 * that erase only sectors 0..6 keep the calibration intact. */
#define MAG_STORE_ADDR       0x08060000u
#define MAG_STORE_MAGIC      0x434D4742u   /* "BGMC" */
#define MAG_STORE_VERSION    1u

typedef struct mag_store {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    uint32_t blob_size;
    uint32_t crc;
    uint32_t reserved;
    mag_cal_blob_t blob[NUM_DEVICES];
} mag_store_t;

static mag_store_t mag_store __attribute__((aligned(4)));

static uint32_t mag_store_crc(const mag_cal_blob_t *blob, uint32_t bytes)
{
    const uint8_t *p = (const uint8_t *)blob;
    uint32_t h = 2166136261u;
    while (bytes--) { h ^= *p++; h *= 16777619u; }
    return h;
}

static uint8_t mag_store_valid(const mag_store_t *s)
{
    return (uint8_t)((s->magic == MAG_STORE_MAGIC) &&
                     (s->version == MAG_STORE_VERSION) &&
                     (s->count == NUM_DEVICES) &&
                     (s->blob_size == sizeof(mag_cal_blob_t)) &&
                     (s->crc == mag_store_crc(s->blob,
                         (uint32_t)sizeof(mag_cal_blob_t) * NUM_DEVICES)));
}


static float mag_store_det3(const float M[9])
{
    return M[0] * (M[4] * M[8] - M[5] * M[7])
         - M[1] * (M[3] * M[8] - M[5] * M[6])
         + M[2] * (M[3] * M[7] - M[4] * M[6]);
}

static uint8_t mag_store_blob_healthy(const mag_cal_blob_t *b)
{
    uint8_t i;
    float dw, da;

    if (!(b->flags & MAG_BLOB_F_CAL1)) return 0;
    if (!(b->radius[0] > 1.0f) || !(b->radius[0] < 1000.0f) ||
        !isfinite(b->radius[0])) return 0;

    for (i = 0; i < 3u; i++)
        if (!isfinite(b->offset[0][i])) return 0;
    for (i = 0; i < 9u; i++)
        if (!isfinite(b->W[0][i])) return 0;
    dw = mag_store_det3(b->W[0]);
    if (!(dw > 0.25f) || !(dw < 4.0f)) return 0;

    if (b->flags & MAG_BLOB_F_ALIGN)
    {
        for (i = 0; i < 9u; i++)
            if (!isfinite(b->align[i])) return 0;
        da = mag_store_det3(b->align);
        if (!(da > 0.25f) || !(da < 4.0f)) return 0;
    }
    if ((b->flags & MAG_BLOB_F_DIP) && !isfinite(b->dip)) return 0;
    return 1;
}


static uint8_t mag_store_runtime_healthy(const mag_fusion_t *f)
{
    if (!f->cal[0].valid || !f->fe.field_valid || !f->ref_dip_valid) return 0;
    if (!isfinite(f->cal[0].geo_q) || (f->cal[0].geo_q > 0.25f)) return 0;
    if (!isfinite(f->fe.res) || (f->fe.res > MAG_FE_RES_MAX)) return 0;
    if (!isfinite(f->ref_dip)) return 0;
    return 1;
}

static uint8_t mag_store_candidate_healthy(const mag_fusion_t *f, const mag_cal_blob_t *b)
{
    return (uint8_t)(mag_store_blob_healthy(b) && mag_store_runtime_healthy(f));
}

static uint8_t mag_store_reload_one(uint8_t idx)
{
    const mag_store_t *s = (const mag_store_t *)MAG_STORE_ADDR;

    if (!mag_store_valid(s)) return 0;
    if (idx >= NUM_DEVICES) return 0;
    if (!mag_store_blob_healthy(&s->blob[idx])) return 0;
    if (!(s->blob[idx].flags & MAG_BLOB_F_VERIFIED)) return 0;
    return mag_fusion_load(&glove_devices[idx].fuse, &s->blob[idx]);
}

static void mag_store_erase(void)
{
    FLASH_EraseInitTypeDef er = {0};
    uint32_t err = 0;

    HAL_FLASH_Unlock();
    er.TypeErase = FLASH_TYPEERASE_SECTORS;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    er.Sector = FLASH_SECTOR_7;
    er.NbSectors = 1;
    if (HAL_FLASHEx_Erase(&er, &err) != HAL_OK)
        REPLAY_PRINTF("[MAG] calibration erase failed (%lu)\r\n", (unsigned long)err);
    else
        glove_send_replay_event(7u, 0, 0);
    HAL_FLASH_Lock();
}

static void mag_store_load(uint8_t idx)
{
    if (mag_store_reload_one(idx)) {
        REPLAY_PRINTF("[MAG] calibration loaded for link %u\r\n", (unsigned)(glove_devices[idx].sensor_id));
        glove_send_replay_event_link(glove_devices[idx].link, 5u, 0, 0);
    }
}

static void mag_store_save(void)
{
    uint8_t i, any = 0;
    uint32_t off;
    const mag_store_t *old = (const mag_store_t *)MAG_STORE_ADDR;

    if (mag_store_valid(old)) memcpy(&mag_store, old, sizeof(mag_store));
    else memset(&mag_store, 0xFF, sizeof(mag_store));

    mag_store.magic = MAG_STORE_MAGIC;
    mag_store.version = MAG_STORE_VERSION;
    mag_store.count = NUM_DEVICES;
    mag_store.blob_size = sizeof(mag_cal_blob_t);
    mag_store.reserved = 0;

    for (i = 0; i < NUM_DEVICES; i++)
    {
        mag_fusion_t *f = &glove_devices[i].fuse;

        if (!glove_devices[i].init) continue;
        if (!f->cal[0].valid || !f->fe.field_valid) continue;
        if (!isfinite(f->cal[0].geo_q) || (f->cal[0].geo_q > 0.25f)) continue;
        if (!isfinite(f->fe.res) || (f->fe.res > MAG_FE_RES_MAX)) continue;
        if (mag_fusion_save(f, &mag_store.blob[i]))
        {
            if (mag_store_candidate_healthy(f, &mag_store.blob[i]))
            {
                mag_store.blob[i].flags |= MAG_BLOB_F_VERIFIED;
                if (f->cal[0].model == 2u) mag_store.blob[i].flags |= MAG_BLOB_F_MODEL_FULL;
                else if (f->cal[0].model == 1u) mag_store.blob[i].flags |= MAG_BLOB_F_MODEL_DIAG;
                any = 1;
            }
        }
    }
    if (!any)
    {
        REPLAY_PRINTF("[MAG] calibration not stored: quality gate failed\r\n");
        return;
    }

    mag_store.crc = mag_store_crc(mag_store.blob,
                                  (uint32_t)sizeof(mag_cal_blob_t) * NUM_DEVICES);

    HAL_FLASH_Unlock();
    {
        FLASH_EraseInitTypeDef er = {0};
        uint32_t err = 0;
        er.TypeErase = FLASH_TYPEERASE_SECTORS;
        er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        er.Sector = FLASH_SECTOR_7;
        er.NbSectors = 1;
        if (HAL_FLASHEx_Erase(&er, &err) != HAL_OK)
        {
            REPLAY_PRINTF("[MAG] calibration store erase failed (%lu)\r\n", (unsigned long)err);
            HAL_FLASH_Lock();
            return;
        }
    }
    for (off = 0; off < sizeof(mag_store); off += 4u)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MAG_STORE_ADDR + off,
                              *(uint32_t *)((uint8_t *)&mag_store + off)) != HAL_OK)
        {
            REPLAY_PRINTF("[MAG] calibration store write failed at %lu\r\n", (unsigned long)off);
            break;
        }
    HAL_FLASH_Lock();
    if (off >= sizeof(mag_store)) {
        REPLAY_PRINTF("[MAG] calibration stored (%u link(s))\r\n", (unsigned)any);
        glove_send_replay_event(6u, (int32_t)any, 0);
    }
}

static uint8_t mag_store_pending = 0u;
static uint32_t mag_store_next_try_ms = 0u;
static uint8_t mag_auto_cal_state[NUM_DEVICES];
static uint32_t mag_auto_cal_start_ms[NUM_DEVICES];

static void mag_store_request_save(void)
{
    mag_store_pending = 1u;
    mag_store_next_try_ms = 0u;
}

static void mag_store_periodic(void)
{
    uint8_t i;

    if (!mag_store_pending) return;
    if ((uint32_t)(HAL_GetTick() - mag_store_next_try_ms) < 1000u) return;
    mag_store_next_try_ms = HAL_GetTick();

    for (i = 0; i < NUM_DEVICES; i++)
    {
        if (!glove_devices[i].init) continue;
        if (!mag_store_runtime_healthy(&glove_devices[i].fuse)) return;
    }
    mag_store_save();
    mag_store_pending = 0u;
}

/* Print every virtual sensor this firmware publishes.  This is how you find
 * the id of the second magnetometer if the probe picks the wrong one, or
 * picks nothing: run it once and read the number off the console. */
void glove_dump_sensor_list(void)
{
	uint16_t id;
	uint8_t n = 0;
	if (!glove_devices[0].init) { REPLAY_PRINTF("[MAG] link 0 not up\r\n"); return; }
	REPLAY_PRINTF("[MAG] virtual sensors present:");
	for (id = 1u; id < 200u; id++)
	{
		if (bhy2_is_sensor_available((uint8_t)id, glove_devices[0].dev))
		{ REPLAY_PRINTF(" %u", (unsigned)id); n++; }
	}
	REPLAY_PRINTF("   (%u total; quat=%u mag=%u gyro=%u mag2=%u)\r\n",
	       n, QUAT_SENSOR_ID, MAG_ID, GYRO_ID, (unsigned)glove_mag2_id);
}

void glove_dump_sensor_info(void)
{
#if BHI_DIAG_PRINT
	uint8_t i;
	for (i = 0u; i < NUM_DEVICES; i++)
	{
		if (glove_devices[i].init && glove_devices[i].dev)
			bhi360_print_sensor_info(glove_devices[i].dev, glove_devices[i].sensor_id);
	}
#else
	REPLAY_PRINTF("[DIAG] sensor-info diagnostics disabled\r\n");
#endif
}

struct mag_data_dev glove_devices[NUM_DEVICES] = {
		{bowie_Finger_pinky, 	1, 1, 0, 0x1B, 0, 0, NULL}, // 0
		{bowie_Finger_pinky, 	2, 2, 0, 0x1C, 0, 0, NULL},
		{bowie_Finger_pinky, 	3, 3, 0, 0x1D, 0, 0, NULL},
		{bowie_Finger_pinky, 	4, 4, 0, 0x1E, 0, 0, NULL},
		{bowie_Finger_ring, 	1, 5, 2, 0x1B, 0, 0, NULL}, // 4
		{bowie_Finger_ring, 	2, 6, 2, 0x1C, 0, 0, NULL},
		{bowie_Finger_ring, 	3, 7, 2, 0x1D, 0, 0, NULL},
		{bowie_Finger_ring, 	4, 8, 2, 0x1E, 0, 0, NULL},
		{bowie_Finger_middle, 	1, 9, 3, 0x1B, 0, 0, NULL}, // 8
		{bowie_Finger_middle, 	2, 10, 3, 0x1C, 0, 0, NULL},
		{bowie_Finger_middle, 	3, 11, 3, 0x1D, 0, 0, NULL},
		{bowie_Finger_middle, 	4, 12, 3, 0x1E, 0, 0, NULL},
		{bowie_Finger_index, 	1, 13, 5, 0x1B, 0, 0, NULL}, // 12
		{bowie_Finger_index, 	2, 14, 5, 0x1C, 0, 0, NULL},
		{bowie_Finger_index, 	3, 15, 5, 0x1D, 0, 0, NULL},
		{bowie_Finger_index, 	4, 16, 5, 0x1E, 0, 0, NULL},
		{bowie_Finger_thumb, 	1, 17, 7, 0x1B, 0, 0, NULL}, // 16
		{bowie_Finger_thumb, 	2, 18, 7, 0x1C, 0, 0, NULL},
		{bowie_Finger_thumb, 	3, 19, 7, 0x1D, 0, 0, NULL},
		{bowie_Finger_thumb, 	4, 20, 7, 0x1E, 0, 0, NULL}
};

//It takes about 4.5 seconds to initialize ONE sensor. For 20 total, that is almost 2 min :(

//uint8_t work_buffer[WORK_BUFFER_SIZE];

//struct mag_data_dev dev;
// this will turn off all mux i2c channels
void glove_mux_reset() {
	uint16_t mux_addr = (MUX_ADDR)<<1;
	uint8_t mux_chan[1];
	mux_chan[0] = 0x00;

//	uint8_t mux_reset_addr = (MUX_ADDR)<<1;
	HAL_I2C_Master_Transmit(&hi2c1, mux_addr, mux_chan, 1, 100);
}

void scan_mux_channel(uint8_t channel)
{
	uint16_t i, ret;
	ret = glove_mux_set_channel(channel);
	HAL_Delay(10);


	for (i = 0; i < 128; i++) {
		ret = HAL_I2C_IsDeviceReady(&hi2c1, (i << 1), 3, 25);
		if (ret == HAL_OK) /* No ACK Received At That Address */
		{
			REPLAY_PRINTF("Channel %d found device at: %X \r\n", channel, i);
		}
	}
}

/*
 * channel input can be decimal 0-7 inclusive only
 *
 */
HAL_StatusTypeDef glove_mux_set_channel(uint8_t channel) {

//	glove_mux_reset();
	if(channel < 8)
	{
		uint8_t result;
		uint16_t mux_addr = (MUX_ADDR)<<1;
		uint8_t control_register[1];
		control_register[0] = ((uint8_t)1<<channel);
		result = HAL_I2C_Master_Transmit(&hi2c1, mux_addr, control_register, 1, 100);
		return result;
	}
	else
	{
		return HAL_ERROR;
	}

}

/* 文本诊断的帧边界。
 *
 * COBS 帧以 0x00 分界，而 printf 出来的文本里没有 0x00。主机会把“文本 + 下一个
 * 二进制帧”当成一帧，那个二进制帧就废了 —— 10 Hz 两行 = 20 坏帧/秒 ÷ 200 帧/秒
 * = 10% 丢帧（实测解码错误 5%、磁力计帧率低 8.5% 完全吻合）。
 *
 * 每行文本后面补一个 0x00，让它自己成为一帧，二进制帧就再也不会被打断。
 * 同一端点上的文本行本身就是一帧但解不出 protobuf，主机会把它当“文本/坏帧”
 * 单独归类，而不是丢掉紧随其后的数据帧。 */
static void __attribute__((unused)) diag_frame_end(void)
{
	static const uint8_t z = 0u;
	tud_cdc_write(&z, 1u);
}

/*
 * Data-only replay snapshot transport.
 *
 * No field names are transmitted.  Each frame carries four raw 32-bit words:
 *   index            = chunk index (header uses 0xFFFFFFFF)
 *   mag.seconds      = word 0
 *   mag.nanoseconds  = word 1
 *   quat.seconds     = word 2
 *   quat.nanoseconds = word 3
 * The header stores length/crc/version/kind in those four words.  The host
 * schema (replay_protocol.py) knows how to turn the raw words back into the
 * exact calibration blob or mag_fusion_t snapshot.
 */
static uint32_t replay_blob_crc(const uint8_t *p, uint32_t n)
{
	uint32_t h = 2166136261u;
	while (n--) { h ^= *p++; h *= 16777619u; }
	return h;
}

static void replay_send_blob_frame(uint8_t sensor_id, uint8_t finger, uint8_t link,
                                   uint32_t index, const uint32_t w[4])
{
	bowie_Data msg = bowie_Data_init_zero;
	msg.index = (int32_t)index;
	msg.finger = (bowie_Finger)finger;
	msg.link = link;
	msg.sensor_id = sensor_id;
	msg.has_mag = true;
	msg.has_quat = true;
	msg.mag.seconds = (int32_t)w[0];
	msg.mag.nanoseconds = (int32_t)w[1];
	msg.quat.seconds = (int32_t)w[2];
	msg.quat.nanoseconds = (int32_t)w[3];
	(void)glove_coms_write(&msg);
}

static void replay_send_blob(uint8_t sensor_id, uint8_t finger, uint8_t link,
                             const uint8_t *data, uint32_t len,
                             uint32_t version, uint32_t kind)
{
	uint32_t off = 0u;
	uint32_t index = 0u;
	uint32_t header[4];

	header[0] = len;
	header[1] = replay_blob_crc(data, len);
	header[2] = version;
	header[3] = kind;
	replay_send_blob_frame(sensor_id, finger, link, 0xFFFFFFFFu, header);

	while (off < len) {
		uint32_t words[4] = {0u, 0u, 0u, 0u};
		uint32_t n = len - off;
		if (n > 16u) n = 16u;
		memcpy(words, data + off, n);
		replay_send_blob_frame(sensor_id, finger, link, index++, words);
		off += n;
	}
}

static void replay_send_diag6(uint8_t sensor_id, uint8_t finger, uint8_t link,
                             uint32_t sample_id, uint32_t chunk,
                             const float *v, uint8_t n)
{
	bowie_Data msg = bowie_Data_init_zero;
	float f[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	uint8_t i;

	if (n > 6u) n = 6u;
	for (i = 0u; i < n; i++) f[i] = v[i];

	msg.index = (int32_t)(((sample_id & 0x0FFFFFFFu) << 4) | (chunk & 0x0Fu));
	msg.finger = (bowie_Finger)finger;
	msg.link = link;
	msg.sensor_id = sensor_id;
	msg.has_mag = true;
	msg.has_quat = true;
	msg.quat.x = f[0];
	msg.quat.y = f[1];
	msg.quat.z = f[2];
	msg.quat.w = f[3];
	msg.quat.accuracy = f[4];
	msg.mag.x = f[5];
	(void)glove_coms_write(&msg);
}

#define REPLAY_CONFIG_MAGIC   0x52504C59u   /* "RPLY" */
#define REPLAY_CONFIG_PARAMS  48u

typedef struct __attribute__((packed)) replay_config {
    uint32_t magic;
    uint32_t schema_version;
    uint32_t fw_tag;
    uint32_t build_crc;
    uint32_t size_mag_fusion;
    uint32_t size_mag_cal_blob;
    uint32_t layout_hash;
    uint32_t off_cal;
    uint32_t off_A;
    uint32_t off_fe;
    uint32_t off_safe_B;
    uint32_t off_wring;
    uint32_t off_yaw_corr;
    uint32_t off_mag_datum;
    uint32_t off_status;
    uint32_t off_yawd_innov;
    int32_t  params[REPLAY_CONFIG_PARAMS];
} replay_config_t;

static uint32_t replay_event_seq = 0u;
static uint32_t replay_cdc_errors = 0u;
static uint32_t replay_fifo_overflows = 0u;
static uint32_t replay_meta_errors = 0u;
static uint32_t replay_sensor_errors = 0u;
static uint32_t replay_hard_events = 0u;
static uint32_t replay_reject_events = 0u;
static uint32_t replay_recal_events = 0u;

static void replay_fill_params(int32_t p[REPLAY_CONFIG_PARAMS])
{
    p[0]  = (int32_t)(MAG_YAW_INNOV_MAX * 57.2957795f * 100.0f);
    p[1]  = (int32_t)(MAG_YAW_NIS_MAX * 10.0f);
    p[2]  = (int32_t)(MAG_YAW_INNOV_EXIT * 57.2957795f * 100.0f);
    p[3]  = (int32_t)(MAG_YAW_NIS_EXIT * 10.0f);
    p[4]  = (int32_t)MAG_YAW_SLEW_MAX;
    p[5]  = (int32_t)MAG_YAW_SLEW_FAST_MAX;
    p[6]  = (int32_t)(MAG_YAW_HOLD_S * 1000.0f);
    p[7]  = (int32_t)(MAG_YAW_HOLD_FAST_S * 1000.0f);
    p[8]  = (int32_t)MAG_YAW_RELAX_LO_DEG;
    p[9]  = (int32_t)MAG_YAW_RELAX_HI_DEG;
    p[10] = (int32_t)(MAG_YAW_RELAX_MAX * 100.0f);
    p[11] = (int32_t)MAG_YAW_SNAP_DEG;
    p[12] = (int32_t)(MAG_YAW_FAST_INST_MAX * 100.0f);
    p[13] = (int32_t)(MAG_YAW_FAST_LP_MAX * 100.0f);
    p[14] = (int32_t)(MAG_YAW_TRUST_RISE_S * 1000.0f);
    p[15] = (int32_t)(MAG_YAW_TRUST_FALL_S * 1000.0f);
    p[16] = (int32_t)MAG_YAW_SLEW_OMEGA_MAX;
    /* p[17] 改成【当前有效】门限，随运行时可调强度变化（原为编译期常量）。 */
    p[17] = (int32_t)(mag_fusion_dir_thr_deg() * 100.0f);
    p[18] = (int32_t)(MAG_DIR_LOCK_S * 1000.0f);
    p[19] = (int32_t)MAG_DIR_OMEGA_MAX;
    p[20] = (int32_t)(MAG_NORM_TOL * 100.0f);
    p[21] = (int32_t)(MAG_NORM_SIGMA * 10.0f);
    p[22] = (int32_t)(MAG_DIP_TOL_RAD * 57.2957795f * 100.0f);
    p[23] = (int32_t)(MAG_DIP_SIGMA * 10.0f);
    p[24] = (int32_t)(MAG_STEP_SIGMA * 10.0f);
    p[25] = (int32_t)(MAG_STEP_FLOOR * 1000.0f);
    p[26] = (int32_t)(MAG_STEP_OMEGA_K * 1000.0f);
    p[27] = (int32_t)(MAG_DRIFT_SIGMA * 10.0f);
    p[28] = (int32_t)(MAG_DRIFT_FLOOR_RAD * 57295.7795f);
    p[29] = (int32_t)(MAG_DRIFT_OMEGA_K * 1000.0f);
    p[30] = (int32_t)MAG_DRIFT_RUN;
    p[31] = (int32_t)(MAG_EVENT_ENTER * 100.0f);
    p[32] = (int32_t)(MAG_REJECT_AT * 100.0f);
    p[33] = (int32_t)(MAG_RECOVER_S * 1000.0f);
    p[34] = (int32_t)(MAG_E_TAU_S * 1000.0f);
    p[35] = (int32_t)MAG_STUCK_RECAL_S;
    p[36] = (int32_t)MAG_CAL_WINDOW_S;
    p[37] = (int32_t)MAG_CAL_FULL_MIN;
    p[38] = (int32_t)MAG_CAL_BINS_FULL;
    p[39] = (int32_t)(MAG_STAB_STATIC_TAU_S * 10.0f);
    p[40] = (int32_t)(MAG_STAB_MOTION_TAU_S * 1000.0f);
    p[41] = (int32_t)MAG_STAB_HOLD;
    p[42] = (int32_t)(MAG_STAB_BREAKOUT_RAD * 1000.0f);
    p[43] = (int32_t)MAG_FE_MIN_N;
    p[44] = (int32_t)(MAG_FE_OBS_MAX * 100.0f);
    p[45] = (int32_t)(MAG_FE_RES_MAX * 100.0f);
    p[46] = (int32_t)MAG_FE_EVERY;
    p[47] = (int32_t)REPLAY_DATA_ONLY;
}

/*
 * 运行时调参：目前只有一项 —— 方向残差门强度。
 * 协议（纯 ASCII，以换行结束）：  "T DIR <0..100>"
 *   0   = 关闭该门
 *   100 = 原行为（门限 3.0 度，命中后 e=4.0 硬事件）
 * 回一行 TUNE ... 方便上位机确认。
 */
void glove_set_dir_strength_pct(int pct)
{
    float s;
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    s = (float)pct / 100.0f;
    mag_fusion_set_dir_strength(s);
    REPLAY_PRINTF("TUNE dir_strength=%d dir_thr_d100=%d dir_emax_x100=%d\r\n",
                  (int)(s * 100.0f),
                  (int)(mag_fusion_dir_thr_deg() * 100.0f),
                  (int)((1.0f + 3.0f * s) * 100.0f));
}

void glove_tune_command(const char *line)
{
    const char *p = line;
    int pct = -1;
    int digits = 0;
    if (p == NULL) return;
    if (*p == 'T' || *p == 't') p++;
    while (*p == ' ' || *p == '\t') p++;
    if ((p[0] == 'D' || p[0] == 'd') && (p[1] == 'I' || p[1] == 'i') &&
        (p[2] == 'R' || p[2] == 'r'))
        p += 3;
    else
    {
        REPLAY_PRINTF("TUNE err: expect \"T DIR <0..100>\"\r\n");
        return;
    }
    while (*p == ' ' || *p == '\t') p++;
    pct = 0;
    while (*p >= '0' && *p <= '9' && digits < 4)
    {
        pct = pct * 10 + (int)(*p - '0');
        digits++;
        p++;
    }
    if (digits == 0)
    {
        REPLAY_PRINTF("TUNE err: expect \"T DIR <0..100>\"\r\n");
        return;
    }
    glove_set_dir_strength_pct(pct);
}

void glove_send_replay_config(void)
{
#if REPLAY_STREAM_ENABLE
    uint8_t i;
    replay_config_t cfg;
    uint32_t layout[16];

    memset(&cfg, 0, sizeof(cfg));
    cfg.magic = REPLAY_CONFIG_MAGIC;
    cfg.schema_version = REPLAY_SCHEMA_VERSION;
    cfg.fw_tag = REPLAY_FW_TAG;
    {
        static const char build[] = __DATE__ " " __TIME__;
        cfg.build_crc = replay_blob_crc((const uint8_t *)build, (uint32_t)(sizeof(build) - 1u));
    }
    cfg.size_mag_fusion = (uint32_t)sizeof(mag_fusion_t);
    cfg.size_mag_cal_blob = (uint32_t)sizeof(mag_cal_blob_t);
    cfg.off_cal = (uint32_t)offsetof(mag_fusion_t, cal);
    cfg.off_A = (uint32_t)offsetof(mag_fusion_t, A);
    cfg.off_fe = (uint32_t)offsetof(mag_fusion_t, fe);
    cfg.off_safe_B = (uint32_t)offsetof(mag_fusion_t, safe_B);
    cfg.off_wring = (uint32_t)offsetof(mag_fusion_t, wring);
    cfg.off_yaw_corr = (uint32_t)offsetof(mag_fusion_t, yaw_corr);
    cfg.off_mag_datum = (uint32_t)offsetof(mag_fusion_t, mag_datum_offset);
    cfg.off_status = (uint32_t)offsetof(mag_fusion_t, status);
    cfg.off_yawd_innov = (uint32_t)offsetof(mag_fusion_t, yawd_innov_deg);
    {
        int32_t params[REPLAY_CONFIG_PARAMS];
        replay_fill_params(params);
        memcpy(cfg.params, params, sizeof(params));
    }

    layout[0] = cfg.size_mag_fusion;
    layout[1] = cfg.size_mag_cal_blob;
    layout[2] = cfg.off_cal;
    layout[3] = cfg.off_A;
    layout[4] = cfg.off_fe;
    layout[5] = cfg.off_safe_B;
    layout[6] = cfg.off_wring;
    layout[7] = cfg.off_yaw_corr;
    layout[8] = cfg.off_mag_datum;
    layout[9] = cfg.off_status;
    layout[10] = cfg.off_yawd_innov;
    cfg.layout_hash = replay_blob_crc((const uint8_t *)layout, sizeof(layout));

    for (i = 0u; i < NUM_DEVICES; i++) {
        if (!glove_devices[i].init) continue;
        replay_send_blob(REPLAY_CONFIG_ID, glove_devices[i].finger, glove_devices[i].link,
                         (const uint8_t *)&cfg, (uint32_t)sizeof(cfg),
                         REPLAY_SCHEMA_VERSION, 3u);
    }
#endif
}

void glove_send_replay_event_link(uint8_t link, uint16_t code, int32_t a, int32_t b)
{
#if REPLAY_STREAM_ENABLE
    bowie_Data msg = bowie_Data_init_zero;
    msg.index = (int32_t)replay_event_seq++;
    msg.finger = bowie_Finger_none;
    msg.link = link;
    msg.sensor_id = REPLAY_EVENT_ID;
    msg.has_mag = true;
    msg.has_quat = true;
    msg.mag.seconds = (int32_t)code;
    msg.mag.nanoseconds = (int32_t)HAL_GetTick();
    msg.quat.seconds = a;
    msg.quat.nanoseconds = b;
    (void)glove_coms_write(&msg);
#endif
}

void glove_send_replay_event(uint16_t code, int32_t a, int32_t b)
{
    glove_send_replay_event_link(0u, code, a, b);
}

void glove_replay_note_meta(uint8_t kind, uint8_t link)
{
#if REPLAY_STREAM_ENABLE
    if (kind == 0u) {
        replay_fifo_overflows++;
        glove_send_replay_event_link(link, 24u, 0, 0);
    } else if (kind == 1u) {
        replay_sensor_errors++;
        glove_send_replay_event_link(link, 25u, 0, 0);
    } else {
        replay_meta_errors++;
        glove_send_replay_event_link(link, 26u, 0, 0);
    }
#else
    (void)kind;
    (void)link;
#endif
}

void glove_send_replay_health(void)
{
#if REPLAY_STREAM_ENABLE
    static uint32_t last_ms = 0u;
    uint32_t now = HAL_GetTick();
    uint8_t i;
    if ((uint32_t)(now - last_ms) < 1000u) return;
    last_ms = now;
    for (i = 0u; i < NUM_DEVICES; i++) {
        float v[30];
        const mag_fusion_t *f = &glove_devices[i].fuse;
        if (!glove_devices[i].init) continue;
        v[0] = (float)now;
        v[1] = (float)glove_devices[i].count;
        v[2] = (float)glove_devices[i].replay_mag_count;
        v[3] = (float)glove_devices[i].replay_gyro_count;
        v[4] = (float)glove_devices[i].replay_acc_count;
        v[5] = (float)glove_devices[i].replay_mag2_count;
        v[6] = (float)replay_cdc_errors;
        v[7] = (float)replay_event_seq;
        v[8] = (float)replay_fifo_overflows;
        v[9] = (float)replay_meta_errors;
        v[10] = (float)replay_sensor_errors;
        v[11] = (float)replay_hard_events;
        v[12] = (float)replay_reject_events;
        v[13] = (float)replay_recal_events;
        v[14] = (float)f->cal[0].revision;
        v[15] = (float)f->cal[0].valid;
        v[16] = (float)f->cal[0].active;
        v[17] = (float)mag_fusion_learn_state(f);
        v[18] = (float)mag_fusion_learn_opens(f);
        v[19] = (float)f->hard_event;
        v[20] = (float)f->rejecting;
        v[21] = (float)f->ref_dip_valid;
        v[22] = (float)f->fe.field_valid;
        v[23] = (float)(glove_devices[i].replay_last_ts & 0xFFFFFFFFu);
        v[24] = (float)(glove_devices[i].replay_last_ts >> 32);
        v[25] = (float)REPLAY_FW_TAG;
        v[26] = (float)REPLAY_SCHEMA_VERSION;
        v[27] = (float)REPLAY_DATA_ONLY;
        v[28] = (float)glove_devices[i].replay_mag_dup_drop;
        v[29] = 0.0f;
        replay_send_diag6(REPLAY_HEALTH_ID, glove_devices[i].finger, glove_devices[i].link,
                          now, 0u, v + 0u, 6u);
        replay_send_diag6(REPLAY_HEALTH_ID, glove_devices[i].finger, glove_devices[i].link,
                          now, 1u, v + 6u, 6u);
        replay_send_diag6(REPLAY_HEALTH_ID, glove_devices[i].finger, glove_devices[i].link,
                          now, 2u, v + 12u, 6u);
        replay_send_diag6(REPLAY_HEALTH_ID, glove_devices[i].finger, glove_devices[i].link,
                          now, 3u, v + 18u, 6u);
        replay_send_diag6(REPLAY_HEALTH_ID, glove_devices[i].finger, glove_devices[i].link,
                          now, 4u, v + 24u, 6u);
    }
#endif
}

static void replay_note_transitions(uint8_t idx)
{
#if REPLAY_STREAM_ENABLE
    mag_fusion_t *f = &glove_devices[idx].fuse;
    uint32_t rev = f->cal[0].revision;
    uint8_t hard = f->hard_event;
    uint8_t reject = f->rejecting;
    uint8_t learn = mag_fusion_learn_state(f);

    if (rev != glove_devices[idx].replay_cal_rev_prev) {
        glove_send_replay_event_link(glove_devices[idx].link, 17u,
                                     (int32_t)glove_devices[idx].replay_cal_rev_prev,
                                     (int32_t)rev);
        glove_devices[idx].replay_cal_rev_prev = rev;
    }
    if (hard != glove_devices[idx].replay_hard_prev) {
        if (hard) replay_hard_events++;
        glove_send_replay_event_link(glove_devices[idx].link, hard ? 18u : 19u, 0, 0);
        glove_devices[idx].replay_hard_prev = hard;
    }
    if (reject != glove_devices[idx].replay_reject_prev) {
        if (reject) replay_reject_events++;
        glove_send_replay_event_link(glove_devices[idx].link, reject ? 20u : 21u, 0, 0);
        glove_devices[idx].replay_reject_prev = reject;
    }
    if (learn != glove_devices[idx].replay_learn_prev) {
        glove_send_replay_event_link(glove_devices[idx].link, 22u,
                                     (int32_t)glove_devices[idx].replay_learn_prev,
                                     (int32_t)learn);
        glove_devices[idx].replay_learn_prev = learn;
    }
#else
    (void)idx;
#endif
}

void glove_send_replay_snapshot(void)
{
#if REPLAY_STREAM_ENABLE
	uint8_t i;
	glove_send_replay_config();
	glove_send_replay_event(1u, 0, 0);   /* snapshot/boot start */
	for (i = 0u; i < NUM_DEVICES; i++) {
		mag_cal_blob_t cal;
		if (!glove_devices[i].init) continue;
		if (mag_fusion_save(&glove_devices[i].fuse, &cal))
			replay_send_blob(REPLAY_CAL_ID, glove_devices[i].finger, glove_devices[i].link,
			                (const uint8_t *)&cal, (uint32_t)sizeof(cal),
			                MAG_CAL_BLOB_VERSION, 1u);
		replay_send_blob(REPLAY_STATE_ID, glove_devices[i].finger, glove_devices[i].link,
		                (const uint8_t *)&glove_devices[i].fuse,
		                (uint32_t)sizeof(mag_fusion_t), 2u, 2u);
		{
			uint64_t t = glove_devices[i].replay_last_ts;
			glove_send_replay_event_link(glove_devices[i].link, 27u,
			                             (int32_t)(t & 0xFFFFFFFFu),
			                             (int32_t)(t >> 32));
		}
	}
	glove_send_replay_event(2u, 0, 0);   /* snapshot/boot done */
#endif
}


void init_glove(){

	uint8_t i, channel;
	HAL_StatusTypeDef mux_result;
	HAL_StatusTypeDef probe_result;
	int8_t init_result;

	for (i = 0; i < NUM_DEVICES; i++) {
		channel = glove_devices[i].mux_chan;
		glove_devices[i].dev = &bhy2_devices[i];
		glove_devices[i].init = false;
		probe_result = HAL_ERROR;

		mux_result = glove_mux_set_channel(channel);
		if (mux_result == HAL_OK)
		{
			HAL_Delay(2);
			/* Do not spend time loading firmware into an empty slot. */
			probe_result = HAL_I2C_IsDeviceReady(
				&hi2c1,
				(uint16_t)(glove_devices[i].dev_addr << 1),
				2,
				20);
		}

		if (probe_result == HAL_OK)
		{
			init_result = init_island(i);
			if (init_result == 0)
			{
				glove_devices[i].init = true;
			}
		}

		REPLAY_PRINTF("[%s] Finger: %d, Link: %d, Mux_Chan: %d, Dev_addr: %X, Init: %s \r\n",
				(glove_devices[i].init) ? "PASS" : ((probe_result == HAL_OK) ? "FAIL" : "SKIP"),
				glove_devices[i].finger,
				glove_devices[i].link,
				glove_devices[i].mux_chan,
				glove_devices[i].dev_addr,
				(glove_devices[i].init) ? "true" : "false");
	}

	/* ---- 一次性打印所有编译期参数（零运行开销，开机就能抓到）----
	 * 把这些写进录制，任何一次实验都能事后对照"当时用的什么阈值"。 */
	#if !REPLAY_DATA_ONLY
	REPLAY_PRINTF("PARM yaw_innov_max_d100=%d yaw_nis_max_x10=%d yaw_innov_exit_d100=%d "
	       "yaw_nis_exit_x10=%d slew_dps=%d slew_fast_dps=%d hold_ms=%d hold_fast_ms=%d "
	       "relax_lo_deg=%d relax_hi_deg=%d relax_max_x100=%d snap_deg=%d "
	       "fast_inst_max_x100=%d fast_lp_max_x100=%d trust_rise_ms=%d trust_fall_ms=%d "
	       "slew_omega_max_dps=%d dir_res_d100=%d dir_lock_ms=%d dir_omega_max_dps=%d "
	       "norm_tol_x100=%d norm_sigma_x10=%d dip_tol_d100=%d dip_sigma_x10=%d "
	       "step_sigma_x10=%d step_floor_x1000=%d step_omk_x1000=%d "
	       "drift_sigma_x10=%d drift_floor_mdps=%d drift_omk_x1000=%d drift_run=%d "
	       "event_enter_x100=%d reject_at_x100=%d recover_ms=%d e_tau_ms=%d "
	       "stuck_recal_s=%d cal_win_s=%d cal_full_min=%d cal_bins_full=%d "
	       "stab_static_ds=%d stab_motion_ms=%d stab_hold=%d stab_break_d1000=%d "
	       "fe_min_n=%d fe_obs_max_x100=%d fe_res_max_x100=%d fe_every=%d "
	       "diag_hz=%d\r\n",
	       (int)(MAG_YAW_INNOV_MAX * 57.2957795f * 100.0f),
	       (int)(MAG_YAW_NIS_MAX * 10.0f),
	       (int)(MAG_YAW_INNOV_EXIT * 57.2957795f * 100.0f),
	       (int)(MAG_YAW_NIS_EXIT * 10.0f),
	       (int)MAG_YAW_SLEW_MAX, (int)MAG_YAW_SLEW_FAST_MAX,
	       (int)(MAG_YAW_HOLD_S * 1000.0f), (int)(MAG_YAW_HOLD_FAST_S * 1000.0f),
	       (int)MAG_YAW_RELAX_LO_DEG, (int)MAG_YAW_RELAX_HI_DEG,
	       (int)(MAG_YAW_RELAX_MAX * 100.0f), (int)MAG_YAW_SNAP_DEG,
	       (int)(MAG_YAW_FAST_INST_MAX * 100.0f), (int)(MAG_YAW_FAST_LP_MAX * 100.0f),
	       (int)(MAG_YAW_TRUST_RISE_S * 1000.0f), (int)(MAG_YAW_TRUST_FALL_S * 1000.0f),
	       (int)MAG_YAW_SLEW_OMEGA_MAX,
	       (int)(MAG_DIR_RES_RAD * 57.2957795f * 100.0f),
	       (int)(MAG_DIR_LOCK_S * 1000.0f),
	       (int)MAG_DIR_OMEGA_MAX,
	       (int)(MAG_NORM_TOL * 100.0f), (int)(MAG_NORM_SIGMA * 10.0f),
	       (int)(MAG_DIP_TOL_RAD * 57.2957795f * 100.0f), (int)(MAG_DIP_SIGMA * 10.0f),
	       (int)(MAG_STEP_SIGMA * 10.0f), (int)(MAG_STEP_FLOOR * 1000.0f),
	       (int)(MAG_STEP_OMEGA_K * 1000.0f),
	       (int)(MAG_DRIFT_SIGMA * 10.0f), (int)(MAG_DRIFT_FLOOR_RAD * 57295.7795f),
	       (int)(MAG_DRIFT_OMEGA_K * 1000.0f), (int)MAG_DRIFT_RUN,
	       (int)(MAG_EVENT_ENTER * 100.0f), (int)(MAG_REJECT_AT * 100.0f),
	       (int)(MAG_RECOVER_S * 1000.0f), (int)(MAG_E_TAU_S * 1000.0f),
	       (int)MAG_STUCK_RECAL_S, (int)MAG_CAL_WINDOW_S,
	       (int)MAG_CAL_FULL_MIN, (int)MAG_CAL_BINS_FULL,
	       (int)(MAG_STAB_STATIC_TAU_S * 10.0f), (int)(MAG_STAB_MOTION_TAU_S * 1000.0f),
	       (int)MAG_STAB_HOLD, (int)(MAG_STAB_BREAKOUT_RAD * 1000.0f),
	       (int)MAG_FE_MIN_N, (int)(MAG_FE_OBS_MAX * 100.0f),
	       (int)(MAG_FE_RES_MAX * 100.0f), (int)MAG_FE_EVERY,
	       (int)MAG_YAW_DIAG_HZ);
		diag_frame_end();
#endif

	glove_send_replay_snapshot();
}

int8_t init_island(uint8_t idx){

	uint8_t result = 0;
	uint8_t NUM_RESULTS = 23;
	int8_t results[NUM_RESULTS];
	uint16_t version = 0;
	uint8_t product_id = 0;
	uint8_t chip_id = 0;
	uint8_t boot_status = 0;
	uint8_t hintr_ctrl, hif_ctrl;
	uint8_t chip_ctrl;

	// init struct pointers, soft reset and confirm chip and product IDs
	results[0] = bhi360_init(glove_devices[idx].dev, &glove_devices[idx]);
	mag_store_load(idx);
	results[1] = bhy2_soft_reset(glove_devices[idx].dev);
 	results[2] = bhy2_get_product_id(&product_id, glove_devices[idx].dev);
	results[3] = bhy2_get_chip_id(&chip_id, glove_devices[idx].dev);

//	REPLAY_PRINTF("\tlink %d, init: %d, soft_reset: %d, product_id: %d, chip_id: %d\r\n", idx, results[0], results[1], results[2], results[3]);

	if (product_id != BHY2_PRODUCT_ID)
	{
	   results[3] = ERROR;
	}

	// load firmware in low-speed mode
	chip_ctrl = BHY2_CHIP_CTRL_TURBO_ENABLE; //BHY2_CHIP_CTRL_TURBO_ENABLE; // //
	results[4] = bhy2_set_chip_ctrl(chip_ctrl, glove_devices[idx].dev);

//	REPLAY_PRINTF("\tlink %d, set_chip_ctrl: %d\r\n", idx, results[4]);

	// host interrupt control, TODO confirm settings
//	hintr_ctrl = BHY2_ICTL_DISABLE_FAULT|BHY2_ICTL_DISABLE_FIFO_W| BHY2_ICTL_DISABLE_FIFO_NW |BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
	hintr_ctrl = BHY2_ICTL_ACTIVE_LOW;
//	hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
	results[5] = bhy2_set_host_interrupt_ctrl(hintr_ctrl, glove_devices[idx].dev);
//	REPLAY_PRINTF("\tlink %d, set_host_interrupt_ctrl: %d\r\n", idx, results[5]);
//	print_api_error(result1, &glove_devices[idx].dev);
//	bhy2_get_host_interrupt_ctrl(&hintr_ctrl, glove_devices[idx].dev);
//	print_api_error(result1, &glove_devices[idx].dev);

//	REPLAY_PRINTF("Host interrupt control\r\n");
//	REPLAY_PRINTF("    Wake up FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
//	REPLAY_PRINTF("    Non wake up FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
//	REPLAY_PRINTF("    Status FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
//	REPLAY_PRINTF("    Debugging %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
//	REPLAY_PRINTF("    Fault %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
//	REPLAY_PRINTF("    Interrupt is %s.\r\n", (hintr_ctrl & BHY2_ICTL_ACTIVE_LOW) ? "active low" : "active high");
//	REPLAY_PRINTF("    Interrupt is %s triggered.\r\n", (hintr_ctrl & BHY2_ICTL_EDGE) ? "pulse" : "level");
//	REPLAY_PRINTF("    Interrupt pin drive is %s.\r\n", (hintr_ctrl & BHY2_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");
//
	/* Configure the host interface */
	hif_ctrl = 0;
	results[6] = bhy2_set_host_intf_ctrl(hif_ctrl, glove_devices[idx].dev);
//	REPLAY_PRINTF("\tlink %d, set_host_intf_ctrl: %d\r\n", idx, results[6]);

	/* Check the host control settings */
	uint8_t host_ctrl = 0;
	results[7] = bhy2_set_host_ctrl(host_ctrl, glove_devices[idx].dev);
//	REPLAY_PRINTF("\tlink %d, sset_host_ctrl: %d\r\n", idx, results[7]);


//	results[7] = bhy2_get_virt_sensor_list(glove_devices[idx].dev);
	/* Check if the sensor is ready to load firmware */
	results[8] = bhy2_get_boot_status(&boot_status, glove_devices[idx].dev);
//	REPLAY_PRINTF("\tlink %d, get_boot_status: %d\r\n", idx, results[8]);

	if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
	{
//		results[8] = bhi360_upload_firmware(boot_status, glove_devices[idx].dev);
		results[8] = upload_firmware_partly(glove_devices[idx].dev);

	    if(results[8] != BHY2_OK)
	    {
	    	REPLAY_PRINTF("BAD UPLOAD! ");
	    	return results[8];
	    }

		results[8] = bhy2_boot_from_ram(glove_devices[idx].dev);

	    if(results[8] != BHY2_OK)
	    {
	    	REPLAY_PRINTF("BAD BOOT! ");
	    	return results[8];
	    }

		results[9] = bhy2_get_kernel_version(&version, glove_devices[idx].dev);
		results[10] = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, &glove_devices[idx], glove_devices[idx].dev);
		results[11] = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, &glove_devices[idx], glove_devices[idx].dev);
		results[12] = bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, parse_quaternion, &glove_devices[idx], glove_devices[idx].dev);
		results[13] = bhy2_register_fifo_parse_callback(MAG_ID, parse_magnetometer, &glove_devices[idx], glove_devices[idx].dev);
		results[14] = bhy2_register_fifo_parse_callback(GYRO_ID, parse_gyroscope, &glove_devices[idx], glove_devices[idx].dev);
		results[15] = BHY2_OK;
		/* Replay-only accelerometer mirror.  It is not consumed by mag_fusion yet. */
		results[21] = BHY2_OK;
		if (bhy2_is_sensor_available(ACC_ID, glove_devices[idx].dev))
			results[21] = bhy2_register_fifo_parse_callback(ACC_ID, parse_accelerometer, &glove_devices[idx], glove_devices[idx].dev);
		results[22] = BHY2_OK;

		/* Second BMM350, if this firmware publishes one.  Probe once: every
		 * link runs the same image, so the id is the same everywhere. */
		if (glove_mag2_id == 0u)
		{
			static const uint8_t cand[] = MAG2_CANDIDATES;
			uint8_t c;
			if (MAG2_ID != 0)
			{
				glove_mag2_id = (uint8_t)MAG2_ID;
			}
			else
			{
				for (c = 0; c < (uint8_t)(sizeof(cand)/sizeof(cand[0])); c++)
				{
					if (cand[c] == MAG_ID) continue;
					if (bhy2_is_sensor_available(cand[c], glove_devices[idx].dev))
					{ glove_mag2_id = cand[c]; break; }
				}
			}
			REPLAY_PRINTF("[MAG] second magnetometer: %s (id %u)\r\n",
			       glove_mag2_id ? "found" : "NOT FOUND - single-part mode",
			       (unsigned)glove_mag2_id);
		}
		if (glove_mag2_id != 0u)
			results[15] = bhy2_register_fifo_parse_callback(glove_mag2_id, parse_magnetometer2,
			                                               &glove_devices[idx], glove_devices[idx].dev);

//		REPLAY_PRINTF("\tlink %d, bhi360_upload_firmware: %d\r\n", idx, results[8]);
//		REPLAY_PRINTF("\tlink %d, bhy2_get_kernel_version: %d\r\n", idx, results[9]);
//		REPLAY_PRINTF("\tlink %d, 1register_fifo_parse_callback: %d\r\n", idx, results[10]);
//		REPLAY_PRINTF("\tlink %d, 2register_fifo_parse_callback: %d\r\n", idx, results[11]);
//		REPLAY_PRINTF("\tlink %d, 3register_fifo_parse_callback: %d\r\n", idx, results[12]);
//		REPLAY_PRINTF("\tlink %d, 4register_fifo_parse_callback: %d\r\n", idx, results[13]);


	}
	else
	{
//		REPLAY_PRINTF("Host interface not ready. Exiting\r\n");
		//TODO: turn off all the LEDS?
		return ERROR;
	}

	/* Update the callback table to enable parsing of sensor data */
	results[16] = bhy2_update_virtual_sensor_list(glove_devices[idx].dev);
#if BHI_DIAG_PRINT
	glove_devices[idx].diag_gyro_corr_available = 0u;
	if ((GYRO_ID != BHY2_SENSOR_ID_GYRO) && bhy2_is_sensor_available(BHY2_SENSOR_ID_GYRO, glove_devices[idx].dev))
	{
		results[16] = (int8_t)(results[16] | bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GYRO,
			parse_gyroscope_corrected, &glove_devices[idx], glove_devices[idx].dev));
		glove_devices[idx].diag_gyro_corr_available = 1u;
	}
#endif
#if BHI_DIAG_PRINT
	bhi360_print_sensor_info(glove_devices[idx].dev, idx);
#endif
//	REPLAY_PRINTF("\tlink %d, bhy2_update_virtual_sensor_list: %d\r\n", idx, results[14]);

//	results[14] = bhy2_get_virt_sensor_list(glove_devices[idx].dev);

//		uint8_t phys_sensor_id = 0x5;
//	for(uint8_t i=0; i< 64; i++)
//	{
//		struct bhy2_phys_sensor_info info;
//		result = bhy2_get_phys_sensor_info(i, &info, glove_devices[idx].dev);
//		if(result==HAL_OK)
//		{
//			REPLAY_PRINTF("found phys sensor!");
//		}
//	}


//	bhy2_get_virt_sensor_list(glove_devices[0].dev);
//	struct bhy2_sensor_info info;
//	bhy2_get_sensor_info(MAG_ID, &info, glove_devices[0].dev);
//	bhy2_get_sensor_info(101, &info, glove_devices[0].dev);
//	bhy2_get_sensor_info(5, &info, glove_devices[0].dev);

	// we can allow sample rate as an input to function, or keep it fixed like this
	float sample_rate = 100;
	uint32_t report_latency_ms = 0;
	results[17] = bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, sample_rate, report_latency_ms, glove_devices[idx].dev);
	results[18] = bhy2_set_virt_sensor_cfg(MAG_ID, sample_rate, report_latency_ms, glove_devices[idx].dev);
	results[19] = bhy2_set_virt_sensor_cfg(GYRO_ID, sample_rate, report_latency_ms, glove_devices[idx].dev);
	if (bhy2_is_sensor_available(ACC_ID, glove_devices[idx].dev))
		results[22] = bhy2_set_virt_sensor_cfg(ACC_ID, sample_rate, report_latency_ms, glove_devices[idx].dev);
#if BHI_DIAG_PRINT
	if (glove_devices[idx].diag_gyro_corr_available)
		results[19] = (int8_t)(results[19] | bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GYRO,
			sample_rate, report_latency_ms, glove_devices[idx].dev));
#endif
	results[20] = BHY2_OK;
	if (glove_mag2_id != 0u)
		results[20] = bhy2_set_virt_sensor_cfg(glove_mag2_id, sample_rate, report_latency_ms, glove_devices[idx].dev);
//	REPLAY_PRINTF("\tlink %d, bhy2_set_virt_sensor_cfg: %d\r\n", idx, results[16]);

//	REPLAY_PRINTF("Enable %s at %.2fHz.\r\n", get_sensor_name(MAG_ID), sample_rate);

	uint8_t i;
	for(i=0; i< NUM_RESULTS; i++)
	{
		// combine all the error reports into one. Will need to check this function for more detail on the failure mode
		result = result | results[i];
	}
//	REPLAY_PRINTF("FINAL RESULT: %d\r\n", result);
	return result;

}


static uint32_t mag_restore_last_ms[NUM_DEVICES];

uint8_t glove_reload_saved_calibration(void)
{
    uint8_t i, n = 0;

    for (i = 0; i < NUM_DEVICES; i++)
    {
        if (!glove_devices[i].init) continue;
        if (mag_store_reload_one(i))
        {
            glove_devices[i].fuse.stuck_s = 0.0f;
            n++;
        }
    }
    REPLAY_PRINTF("[MAG] last-good calibration restored on %u link(s)\r\n", (unsigned)n);
    glove_send_replay_event(23u, (int32_t)n, 0);
    return n;
}

static void mag_store_check_reload(uint8_t idx)
{
    mag_fusion_t *f = &glove_devices[idx].fuse;
    uint32_t now = HAL_GetTick();
    uint8_t corrupt = 0;

    if (!f->cal[0].valid) return;
    if (mag_auto_cal_state[idx] != 0u) return;
    if (!isfinite(f->cal[0].geo_q)) corrupt = 1u;
    else if ((f->cal[0].geo_q < 1.0e8f) && (f->cal[0].geo_q > 0.25f)) corrupt = 1u;
    if (f->fe.field_valid && (!isfinite(f->fe.res) || (f->fe.res > MAG_FE_RES_MAX * 2.0f)))
        corrupt = 1u;

    if (!f->hard_event && corrupt && (f->stuck_s > 3.0f) &&
        ((uint32_t)(now - mag_restore_last_ms[idx]) > 15000u))
    {
        mag_restore_last_ms[idx] = now;
        if (mag_store_reload_one(idx))
        {
            f->stuck_s = 0.0f;
            REPLAY_PRINTF("[MAG] auto-restored last-good calibration on link %u\r\n",
                   (unsigned)glove_devices[idx].sensor_id);
        }
    }
}


static void mag_auto_recal_service(uint8_t idx)
{
    mag_fusion_t *f = &glove_devices[idx].fuse;
    uint32_t now = HAL_GetTick();
    uint8_t severe;

    if (!glove_devices[idx].init) return;

    if (mag_auto_cal_state[idx] == 1u)
    {
        if (f->cal[0].valid && mag_store_runtime_healthy(f))
        {
            mag_auto_cal_state[idx] = 0u;
            mag_store_request_save();
            REPLAY_PRINTF("[MAG] auto-recalibration succeeded on link %u\r\n",
                   (unsigned)glove_devices[idx].sensor_id);
            glove_send_replay_event_link(glove_devices[idx].link, 15u, 0, 0);
        }
        else if ((uint32_t)(now - mag_auto_cal_start_ms[idx]) > 30000u)
        {
            if (mag_store_reload_one(idx))
                REPLAY_PRINTF("[MAG] auto-recalibration timeout; last-good restored on link %u\r\n",
                       (unsigned)glove_devices[idx].sensor_id);
            else
                REPLAY_PRINTF("[MAG] auto-recalibration failed on link %u\r\n",
                       (unsigned)glove_devices[idx].sensor_id);
            glove_send_replay_event_link(glove_devices[idx].link, 16u, 0, 0);
            mag_auto_cal_state[idx] = 2u;
            mag_auto_cal_start_ms[idx] = now;
        }
        return;
    }

    if (mag_auto_cal_state[idx] == 2u)
    {
        if ((uint32_t)(now - mag_auto_cal_start_ms[idx]) > 60000u)
            mag_auto_cal_state[idx] = 0u;
        return;
    }

    severe = (uint8_t)(f->hard_event && f->cal[0].valid &&
                       ((!isfinite(f->cal[0].geo_q)) ||
                        ((f->cal[0].geo_q < 1.0e8f) && (f->cal[0].geo_q > 0.50f)) ||
                        !f->fe.field_valid ||
                        (f->fe.field_valid && (f->fe.res > MAG_FE_RES_MAX))));

    if (severe && (f->fe.n >= MAG_FE_MIN_N) && (f->fe.obs <= MAG_FE_OBS_MAX))
    {
        mag_fusion_cal_start(f);
        mag_auto_cal_state[idx] = 1u;
        mag_auto_cal_start_ms[idx] = now;
        replay_recal_events++;
        glove_send_replay_event_link(glove_devices[idx].link, 14u, 0, 0);
        REPLAY_PRINTF("[MAG] auto-recalibration started on link %u\r\n",
               (unsigned)glove_devices[idx].sensor_id);
    }
}

void glove_update_data()
{

	// run through all the properly initialized devices
	// check fifos for available data
	// if data is available, the registered callbacks will be activated
	// NOTE: if we cannot scan through this list FASTER than the next data ready, it will get stuck in a loop on one sensor only.

	uint8_t idx;
	uint8_t channel, result;
//	uint32_t count = 0;
//	uint32_t NUM_READS = 100;
	uint8_t work_buffer[WORK_BUFFER_SIZE];

//	HAL_GPIO_WritePin(GPIOA, RED_LED_STATUS_Pin, GPIO_PIN_RESET);
//	int32_t start = HAL_GetTick();
	for(idx=0; idx < NUM_DEVICES; idx++)
	{

		// TODO: add error checking to skip sensor if data line is corrupted during movement
		if(glove_devices[idx].init == true)
		{
			// TODO: can optimize by checking current mux channel before sending another command
			channel = glove_devices[idx].mux_chan;
			result = glove_mux_set_channel(channel);
			if(result==HAL_OK)
			{

 				result = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, glove_devices[idx].dev);
				mag_store_check_reload(idx);
				mag_auto_recal_service(idx);
				mag_store_periodic();
				replay_note_transitions(idx);
//
// 				if(result==HAL_OK){
//					result = bhy2_get_error_reg(BHY2_REG_INT_STATUS, &error_code, glove_devices[idx].dev);
//					REPLAY_PRINTF("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_INT_STATUS, result, error_code);
//
//					result = bhy2_get_error_reg(BHY2_REG_ERROR_VALUE, &error_code, glove_devices[idx].dev);
//					REPLAY_PRINTF("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_ERROR_VALUE, result, error_code);
//
//					result = bhy2_get_error_reg(BHY2_REG_ERROR_AUX, &error_code, glove_devices[idx].dev);
//					REPLAY_PRINTF("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_ERROR_AUX, result, error_code);
//
//					result = bhy2_get_error_reg(BHY2_REG_DEBUG_VALUE, &error_code, glove_devices[idx].dev);
//					REPLAY_PRINTF("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_DEBUG_VALUE, result, error_code);
//
//					result = bhy2_get_error_reg(BHY2_REG_DEBUG_STATE, &error_code, glove_devices[idx].dev);
//					REPLAY_PRINTF("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_DEBUG_STATE, result, error_code);
// 				}
			}

		}

	}

#if MAG_YAW_TEXT_DIAG && MAG_YAW_DIAG_HZ > 0
	/* 周期打印 yaw 大修正路径诊断（只打这一行，不打整张 P 表）。
	 * 让"修正什么时候触发、落到哪一档、冻结多久、回正多快"可观测。 */
	{
		static uint32_t yawd_last_ms = 0u;
		uint32_t yawd_now = HAL_GetTick();
		if ((uint32_t)(yawd_now - yawd_last_ms) >= (1000u / (uint32_t)MAG_YAW_DIAG_HZ))
		{
			yawd_last_ms = yawd_now;
			for (idx = 0u; idx < NUM_DEVICES; idx++)
			{
				const mag_fusion_t *fy = &glove_devices[idx].fuse;
				if (!glove_devices[idx].init) continue;
				/* 一行打全 yaw 修正链路的 20 个量（缩放整数，newlib-nano 的 %f 不可靠）
				 * ★ = 权威量：固件自己算的磁航向 / 输出 yaw / 两者的欠账
				 * 采样 10 Hz，约 350 字节/行 => 3.5 KB/s，相对 15 KB/s 的数据流可接受 */
				REPLAY_PRINTF("YAW t=%lu link=%u innov_d10=%d traw_d10=%d target_d10=%d "
				       "outyaw_d10=%d err_d10=%d relax_x100=%d hold_ms=%d "
				       "slew_dps_x10=%d trust_x100=%d large=%u snap=%u clean=%u "
				       "nis=%u kalman=%u step_d100=%d yawc_d10=%d datum_d10=%d "
				       "blend_d10=%d weight_x100=%d status=%u NIS_x100=%d "
				       "yawP_x10000=%d R_x10000=%d K_x1000=%d sig_x1000=%d "
				       "horiz_x1000=%d geo_x1000=%d omega_dps_x10=%d dirlock_ms=%d\r\n",
				       (unsigned long)yawd_now,
				       (unsigned)glove_devices[idx].sensor_id,
				       (int)(fy->yawd_innov_deg * 10.0f),
				       (int)(fy->yawd_traw_deg * 10.0f),
				       (int)(fy->yawd_target_deg * 10.0f),
				       (int)(fy->yawd_out_yaw_deg * 10.0f),
				       (int)(fy->yawd_err_deg * 10.0f),
				       (int)(fy->yawd_relax * 100.0f),
				       (int)(fy->yawd_hold_s * 1000.0f),
				       (int)(fy->yawd_slew_dps * 10.0f),
				       (int)(fy->yawd_trust * 100.0f),
				       (unsigned)fy->yawd_large,
				       (unsigned)fy->yawd_snap,
				       (unsigned)fy->yawd_clean,
				       (unsigned)fy->yawd_nis_big,
				       (unsigned)fy->yawd_kalman,
				       (int)(fy->yawd_step_deg * 100.0f),
				       (int)(mag_fusion_yaw_corr_deg(fy) * 10.0f),
				       (int)(mag_fusion_datum_deg(fy) * 10.0f),
				       (int)(fy->yawd_blend_deg * 10.0f),
				       (int)(fy->weight * 100.0f),
				       (unsigned)fy->yawd_status,
				       (int)(fy->yawd_nis * 100.0f),
				       (int)(fy->yawd_yawP * 10000.0f),
				       (int)(fy->yawd_R * 10000.0f),
				       (int)(fy->yawd_K * 1000.0f),
				       (int)(fy->yawd_sig * 1000.0f),
				       (int)(fy->yawd_horiz * 1000.0f),
				       (int)(fy->yawd_geo * 1000.0f),
				       (int)(fy->yawd_omega_dps * 10.0f),
				       (int)fy->yawd_dirlock_ms);
		diag_frame_end();

				/* YAW2：修正环【外围】的全部状态。全部从结构体直读，不动算法。
				 * 用途：判断 dir_lock / 事件机 / 场估计器 是在正常工作还是误触发。 */
				REPLAY_PRINTF("YAW2 t=%lu link=%u dirlock_res_d100=%d e_inst_x1000=%d "
				       "e_lp_x1000=%d norm_now_x1000=%d dip_now_d100=%d ref_dip_d100=%d "
				       "ref_norm_x1000=%d sigma_x1000=%d coast_x1000=%d stuck_s_x10=%d "
				       "drift_mdps=%d grad_x1000=%d gradthr_x1000=%d fen=%d "
				       "feobs_x1000=%d feres_x1000=%d fed_x1000=%d feB_x1000=%d "
				       "cand_x100=%d diaghold=%u rej=%u hev=%u candv=%u yawv=%u "
				       "blendv=%u lrn=%u opn=%u calv=%u cala=%u calm=%u calrev=%u "
				       "qual_x1000=%d accu_x1000=%d map=%u align=%u refdip=%u "
				       "gamerv_d10=%d magraw_x1000=%d magraw_y1000=%d magraw_z1000=%d\r\n",
				       (unsigned long)yawd_now,
				       (unsigned)glove_devices[idx].sensor_id,
				       (int)(fy->dir_res * 57.2957795f * 100.0f),
				       (int)(fy->e_inst * 1000.0f),
				       (int)(fy->e_lp * 1000.0f),
				       (int)(fy->norm_now * 1000.0f),
				       (int)(fy->dip_now * 57.2957795f * 100.0f),
				       (int)(fy->ref_dip * 57.2957795f * 100.0f),
				       (int)(fy->ref_norm * 1000.0f),
				       (int)(fy->sigma * 1000.0f),
				       (int)(fy->coast_acc * 1000.0f),
				       (int)(fy->stuck_s * 10.0f),
				       (int)(fy->drift_rate * 57295.7795f),
				       (int)(fy->grad_now * 1000.0f),
				       (int)(fy->grad_thr * 1000.0f),
				       (int)fy->fe.n,
				       (int)(fy->fe.obs * 1000.0f),
				       (int)(fy->fe.res * 1000.0f),
				       (int)(sqrtf(fy->fe.d[0]*fy->fe.d[0]+fy->fe.d[1]*fy->fe.d[1]+fy->fe.d[2]*fy->fe.d[2]) * 1000.0f),
				       (int)(sqrtf(fy->fe.B[0]*fy->fe.B[0]+fy->fe.B[1]*fy->fe.B[1]+fy->fe.B[2]*fy->fe.B[2]) * 1000.0f),
				       (int)(fy->cand_s * 100.0f),
				       (unsigned)fy->diag_hold,
				       (unsigned)fy->rejecting,
				       (unsigned)fy->hard_event,
				       (unsigned)fy->cand_valid,
				       (unsigned)fy->yaw_valid,
				       (unsigned)fy->yaw_blend_valid,
				       (unsigned)mag_fusion_learn_state(fy),
				       (unsigned)mag_fusion_learn_opens(fy),
				       (unsigned)fy->cal[0].valid,
				       (unsigned)fy->cal[0].active,
				       (unsigned)fy->cal[0].model,
				       (unsigned)fy->cal[0].revision,
				       (int)(fy->cal[0].quality * 1000.0f),
				       (int)(mag_fusion_accuracy(fy) * 1000.0f),
				       (unsigned)fy->map_valid,
				       (unsigned)fy->fe.align_valid,
				       (unsigned)fy->ref_dip_valid,
				       (int)(fy->yawd_gamerv_yaw_deg * 10.0f),
				       (int)(fy->yawd_mag_raw[0] * 1000.0f),
				       (int)(fy->yawd_mag_raw[1] * 1000.0f),
				       (int)(fy->yawd_mag_raw[2] * 1000.0f));
		diag_frame_end();

#if MAG_YAW_FAST_HZ > 0
	/* YF：紧凑快速行。只打【快速变化】的 14 个量，100 Hz。
	 * 完整 YAW/YAW2 降到 MAG_YAW_DIAG_HZ(10 Hz) —— 全打 100 Hz 会淹链路。 */
	{
		static uint32_t yf_last_ms = 0u;
		uint32_t yf_now = HAL_GetTick();
		if ((uint32_t)(yf_now - yf_last_ms) >= (1000u / (uint32_t)MAG_YAW_FAST_HZ))
		{
			yf_last_ms = yf_now;
			for (idx = 0u; idx < NUM_DEVICES; idx++)
			{
				const mag_fusion_t *fy = &glove_devices[idx].fuse;
				if (!glove_devices[idx].init) continue;
				REPLAY_PRINTF("YF %lu %u %d %d %d %d %d %d %d %d %d %d %d %u%u%u%u\r\n",
				       (unsigned long)yf_now,
				       (unsigned)glove_devices[idx].sensor_id,
				       (int)(fy->yawd_innov_deg * 10.0f),
				       (int)(fy->yawd_err_deg * 10.0f),
				       (int)(fy->yawd_out_yaw_deg * 10.0f),
				       (int)(fy->yawd_gamerv_yaw_deg * 10.0f),
				       (int)(fy->yawd_step_deg * 100.0f),
				       (int)(fy->yawd_relax * 100.0f),
				       (int)(fy->yawd_hold_s * 1000.0f),
				       (int)(fy->yawd_slew_dps * 10.0f),
				       (int)(fy->yawd_trust * 100.0f),
				       (int)(fy->weight * 100.0f),
				       (int)(fy->dir_res * 57.2957795f * 100.0f),
				       (unsigned)fy->yawd_large,
				       (unsigned)fy->yawd_snap,
				       (unsigned)fy->yawd_clean,
				       (unsigned)fy->yawd_kalman);
				diag_frame_end();
			}
		}
	}
#endif
			}
		}
	}
#endif

#if REPLAY_STREAM_ENABLE
	/*
	 * 数据-only 诊断流：不发送字段名，只发送按 schema 排列的数值。
	 * YF 100 Hz，每帧 6 个 float；YAW/YAW2 10 Hz。
	 */
	{
		static uint32_t rb_fast_last_ms = 0u;
		static uint32_t rb_slow_last_ms = 0u;
		uint32_t rb_now = HAL_GetTick();
		uint8_t k;

		for (k = 0u; k < NUM_DEVICES; k++) {
			const mag_fusion_t *fy = &glove_devices[k].fuse;
			uint8_t finger, link;
			if (!glove_devices[k].init) continue;
			finger = glove_devices[k].finger;
			link = glove_devices[k].link;

			if ((uint32_t)(rb_now - rb_fast_last_ms) >= (1000u / (uint32_t)MAG_YAW_BIN_FAST_HZ)) {
				float v[30];
				v[0] = fy->yawd_innov_deg;
				v[1] = fy->yawd_err_deg;
				v[2] = fy->yawd_out_yaw_deg;
				v[3] = fy->yawd_gamerv_yaw_deg;
				v[4] = fy->yawd_step_deg;
				v[5] = fy->yawd_relax;
				v[6] = fy->yawd_hold_s;
				v[7] = fy->yawd_slew_dps;
				v[8] = fy->yawd_trust;
				v[9] = fy->weight;
				v[10] = fy->dir_res * 57.2957795f;
				v[11] = (float)fy->yawd_large;
				v[12] = (float)fy->yawd_snap;
				v[13] = (float)fy->yawd_clean;
				v[14] = (float)fy->yawd_kalman;
				v[15] = fy->yawd_traw_deg;
				v[16] = fy->yawd_target_deg;
				v[17] = mag_fusion_yaw_corr_deg(fy);
				v[18] = mag_fusion_datum_deg(fy);
				v[19] = fy->yawd_blend_deg;
				v[20] = (float)fy->hard_event;
				v[21] = (float)fy->rejecting;
				v[22] = fy->e_inst;
				v[23] = fy->e_lp;
				v[24] = fy->e_step;
				v[25] = fy->e_norm;
				v[26] = fy->e_dip;
				v[27] = fy->gyro_activity;
				v[28] = fy->safe_datum * 57.2957795f;
				v[29] = 0.0f;
				replay_send_diag6(REPLAY_YF_ID, finger, link, rb_now, 0u, v + 0u, 6u);
				replay_send_diag6(REPLAY_YF_ID, finger, link, rb_now, 1u, v + 6u, 6u);
				replay_send_diag6(REPLAY_YF_ID, finger, link, rb_now, 2u, v + 12u, 3u);
				replay_send_diag6(REPLAY_YF_ID, finger, link, rb_now, 3u, v + 15u, 6u);
				replay_send_diag6(REPLAY_YF_ID, finger, link, rb_now, 4u, v + 21u, 6u);
				replay_send_diag6(REPLAY_YF_ID, finger, link, rb_now, 5u, v + 27u, 3u);
			}

			if ((uint32_t)(rb_now - rb_slow_last_ms) >= (1000u / (uint32_t)MAG_YAW_BIN_DIAG_HZ)) {
				float v[41];
				v[0] = (float)(glove_devices[k].replay_last_ts * 1.5625e-5f * 1000.0f);
				v[1] = fy->yawd_innov_deg;
				v[2] = fy->yawd_traw_deg;
				v[3] = fy->yawd_target_deg;
				v[4] = fy->yawd_out_yaw_deg;
				v[5] = fy->yawd_err_deg;
				v[6] = fy->yawd_relax;
				v[7] = fy->yawd_hold_s;
				v[8] = fy->yawd_slew_dps;
				v[9] = fy->yawd_trust;
				v[10] = (float)fy->yawd_large;
				v[11] = (float)fy->yawd_snap;
				v[12] = (float)fy->yawd_clean;
				v[13] = (float)fy->yawd_nis_big;
				v[14] = (float)fy->yawd_kalman;
				v[15] = fy->yawd_step_deg;
				v[16] = mag_fusion_yaw_corr_deg(fy);
				v[17] = mag_fusion_datum_deg(fy);
				v[18] = fy->yawd_blend_deg;
				v[19] = fy->weight;
				v[20] = (float)fy->yawd_status;
				v[21] = fy->yawd_nis;
				v[22] = fy->yawd_yawP;
				v[23] = fy->yawd_R;
				v[24] = fy->yawd_K;
				v[25] = fy->yawd_sig;
				v[26] = fy->yawd_horiz;
				v[27] = fy->yawd_geo;
				v[28] = fy->yawd_omega_dps;
				v[29] = (float)fy->yawd_dirlock_ms;
				replay_send_diag6(REPLAY_YAW_ID, finger, link, rb_now, 0u, v + 0u, 6u);
				replay_send_diag6(REPLAY_YAW_ID, finger, link, rb_now, 1u, v + 6u, 6u);
				replay_send_diag6(REPLAY_YAW_ID, finger, link, rb_now, 2u, v + 12u, 6u);
				replay_send_diag6(REPLAY_YAW_ID, finger, link, rb_now, 3u, v + 18u, 6u);
				replay_send_diag6(REPLAY_YAW_ID, finger, link, rb_now, 4u, v + 24u, 6u);

				v[0] = (float)(glove_devices[k].replay_last_ts * 1.5625e-5f * 1000.0f);
				v[1] = fy->dir_res * 57.2957795f;
				v[2] = fy->e_inst;
				v[3] = fy->e_lp;
				v[4] = fy->norm_now;
				v[5] = fy->dip_now * 57.2957795f;
				v[6] = fy->ref_dip * 57.2957795f;
				v[7] = fy->ref_norm;
				v[8] = fy->sigma;
				v[9] = fy->coast_acc;
				v[10] = fy->stuck_s;
				v[11] = fy->drift_rate * 57295.7795f;
				v[12] = fy->grad_now;
				v[13] = fy->grad_thr;
				v[14] = (float)fy->fe.n;
				v[15] = fy->fe.obs;
				v[16] = fy->fe.res;
				v[17] = sqrtf(fy->fe.d[0]*fy->fe.d[0] + fy->fe.d[1]*fy->fe.d[1] + fy->fe.d[2]*fy->fe.d[2]);
				v[18] = sqrtf(fy->fe.B[0]*fy->fe.B[0] + fy->fe.B[1]*fy->fe.B[1] + fy->fe.B[2]*fy->fe.B[2]);
				v[19] = fy->cand_s;
				v[20] = (float)fy->diag_hold;
				v[21] = (float)fy->rejecting;
				v[22] = (float)fy->hard_event;
				v[23] = (float)fy->cand_valid;
				v[24] = (float)fy->yaw_valid;
				v[25] = (float)fy->yaw_blend_valid;
				v[26] = (float)mag_fusion_learn_state(fy);
				v[27] = (float)mag_fusion_learn_opens(fy);
				v[28] = (float)fy->cal[0].valid;
				v[29] = (float)fy->cal[0].active;
				v[30] = (float)fy->cal[0].model;
				v[31] = (float)fy->cal[0].revision;
				v[32] = fy->cal[0].quality;
				v[33] = mag_fusion_accuracy(fy);
				v[34] = (float)fy->map_valid;
				v[35] = (float)fy->fe.align_valid;
				v[36] = (float)fy->ref_dip_valid;
				v[37] = fy->yawd_gamerv_yaw_deg;
				v[38] = fy->yawd_mag_raw[0];
				v[39] = fy->yawd_mag_raw[1];
				v[40] = fy->yawd_mag_raw[2];
				v[41] = fy->dir_strength_eff;   /* 方向门当前生效强度（运动自适应） */
				replay_send_diag6(REPLAY_YAW2_ID, finger, link, rb_now, 0u, v + 0u, 6u);
				replay_send_diag6(REPLAY_YAW2_ID, finger, link, rb_now, 1u, v + 6u, 6u);
				replay_send_diag6(REPLAY_YAW2_ID, finger, link, rb_now, 2u, v + 12u, 6u);
				replay_send_diag6(REPLAY_YAW2_ID, finger, link, rb_now, 3u, v + 18u, 6u);
				replay_send_diag6(REPLAY_YAW2_ID, finger, link, rb_now, 4u, v + 24u, 6u);
				replay_send_diag6(REPLAY_YAW2_ID, finger, link, rb_now, 5u, v + 30u, 6u);
				replay_send_diag6(REPLAY_YAW2_ID, finger, link, rb_now, 6u, v + 36u, 6u);
			}
		}
		if ((uint32_t)(rb_now - rb_fast_last_ms) >= (1000u / (uint32_t)MAG_YAW_BIN_FAST_HZ)) rb_fast_last_ms = rb_now;
		if ((uint32_t)(rb_now - rb_slow_last_ms) >= (1000u / (uint32_t)MAG_YAW_BIN_DIAG_HZ)) rb_slow_last_ms = rb_now;
	}
#endif

	glove_send_replay_health();

//	int32_t stop =  HAL_GetTick();
//	int32_t elapsed = stop - start;
//	REPLAY_PRINTF("elapsed time: %d\r\n", elapsed);
//	HAL_GPIO_WritePin(GPIOA, RED_LED_STATUS_Pin, GPIO_PIN_SET);


}



void glove_send_quat_data(uint32_t index, uint32_t s, uint32_t ns, uint8_t finger, uint8_t link, uint8_t sensor_id, float x, float y, float z, float w, float accuracy)
{
//	typedef struct _bowie_Quat {
//	    int32_t seconds;
//	    int32_t nanoseconds;
//	    float x;
//	    float y;
//	    float z;
//	    float w;
//	    float accuracy;
//	} bowie_Quat;
//	typedef struct _bowie_Data {
//	    int32_t index;
//	    bowie_Finger finger;
//	    int32_t link;
//	    int32_t sensor_id;
//	    bool has_mag;
//	    bowie_Mag mag;
//	    bool has_quat;
//	    bowie_Quat quat;
//	} bowie_Data;

	bowie_Data msg = bowie_Data_init_zero;


//	msg.timestamp = HAL_GetTick();
	msg.index = index; //TODO update to system ticks
	msg.finger = finger;
	msg.link = link;
	msg.sensor_id = sensor_id; //TODO: update to 1

	msg.has_quat = true;
	msg.quat.seconds = s;
	msg.quat.nanoseconds = ns;
	msg.quat.x = x;
	msg.quat.y = y;
	msg.quat.z = z;
	msg.quat.w = w;
	/* status + gate weight, see mag_fusion_accuracy():
	 *   status = floorf(accuracy);  weight = (accuracy - status) / 0.99f;  */
	msg.quat.accuracy = accuracy;

//	msg.which_payload = bowie_Data_quat_tag;
//	msg.payload.quat.seconds = s;
//	msg.payload.quat.nanoseconds = ns; //TODO: validate output rate
//	msg.payload.quat.x = x / 16384.0f;
//	msg.payload.quat.y = y / 16384.0f;
//	msg.payload.quat.z = z / 16384.0f;
//	msg.payload.quat.w = w / 16384.0f;
//	msg.payload.quat.accuracy = (((accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);

	// send empty mag data to maintain packet size
//	msg.has_mag = false;
//	msg.mag.seconds = 0;  //TODO: validate
//	msg.mag.nanoseconds = 0; //TODO: validate output rate
//	msg.mag.x = 0;
//	msg.mag.y = 0;
//	msg.mag.z = 0;

//	REPLAY_PRINTF("quat msg %d\r\n", sizeof(msg));
	glove_coms_write(&msg);
}
// save sensor into protobuf structure message
void glove_send_mag_data(uint32_t index, uint32_t s, uint32_t ns, uint8_t finger, uint8_t link, uint8_t sensor_id, float mag_x, float mag_y, float mag_z)
{

//	typedef struct _bowie_Data {
//	    int32_t index;
//	    bowie_Finger finger;
//	    int32_t link;
//	    int32_t sensor_id;
//	    bool has_mag;
//	    bowie_Mag mag;
//	    bool has_quat;
//	    bowie_Quat quat;
//	} bowie_Data;

	bowie_Data msg = bowie_Data_init_zero;

//	msg.timestamp = HAL_GetTick();
	msg.index = index; //TODO update to system ticks
	msg.finger = finger;
	msg.link = link;
	msg.sensor_id = sensor_id;

	msg.has_mag = true;
	msg.mag.seconds = s;  //TODO: validate
	msg.mag.nanoseconds = ns; //TODO: validate output rate

	msg.mag.x = mag_x;
	msg.mag.y = mag_y;
	msg.mag.z = mag_z;

//	msg.mag.x = mag_x * (2500.0f / 32768.0f);
//	msg.mag.y = mag_y * (2500.0f / 32768.0f);
//	msg.mag.z = mag_z * (2500.0f / 32768.0f);

//	msg.which_payload = bowie_Data_mag_tag;
//	msg.payload.mag.seconds = s;  //TODO: validate
//	msg.payload.mag.nanoseconds = ns; //TODO: validate output rate
//	msg.payload.mag.x = mag_x / 2000.0f;
//	msg.payload.mag.y = mag_y / 2000.0f;
//	msg.payload.mag.z = mag_z / 2000.0f;


	// send empty quat data to maintain packet size
//	msg.has_quat = false;
//	msg.quat.seconds = 0;  //TODO: validate
//	msg.quat.nanoseconds = 0; //TODO: validate output rate
//	msg.quat.x = 0;
//	msg.quat.y = 0;
//	msg.quat.z = 0;
//	msg.quat.w = 0;
//	msg.quat.accuracy = 0;

//	REPLAY_PRINTF("mag msg %d\r\n", sizeof(msg));
	glove_coms_write(&msg);

}

// encode into protobuf, encode into cobs, then send to USB
Comms_StatusTypedef glove_coms_write(bowie_Data *msg)
{
	//
	cobs_encode_result res;
	static uint8_t out_buffer[128];
	static uint8_t pb_buffer[128];

	pb_ostream_t pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));

	if(!pb_encode(&pb_ostream, bowie_Data_fields, msg)) {
		replay_cdc_errors++;
		return COMM_PB_ERROR;
	}

	res = cobs_encode(out_buffer, sizeof(out_buffer),
				pb_buffer, pb_ostream.bytes_written);

	if(res.status != COBS_ENCODE_OK) {
		replay_cdc_errors++;
		return COMM_COBS_ERROR;
	}
//	REPLAY_PRINTF("pb_ostream %d\r\n", pb_ostream.bytes_written);
	if (tud_cdc_write(&out_buffer, res.out_len) != res.out_len) replay_cdc_errors++;
//	tud_cdc_write_flush();
	HAL_GPIO_TogglePin(GPIOA, BLUE_LED_STOP_Pin);
	return COMM_OK;
}

uint8_t flush_glove()
{
	uint8_t i, channel, result, result_mag, result_quat;
	for (i = 0; i < NUM_DEVICES; i++) {
		channel = glove_devices[i].mux_chan;
		result = glove_mux_set_channel(channel);

		if(result == HAL_OK)
		{
			result_quat = flush_quat_island(i);
			if(result_quat != HAL_OK)
			{
				return ERROR;
			}
			result_mag = flush_mag_island(i);
			if(result_mag != HAL_OK)
			{
				return ERROR;
			}
		}

	}
	return HAL_OK;

}

// Clear any data in the mag FIFO
uint8_t flush_mag_island(uint8_t index)
{
	uint8_t result = 0;

	result = bhy2_flush_fifo(0xFE, glove_devices[index].dev);

	return result;
}

uint8_t flush_quat_island(uint8_t index)
{
	uint8_t result = 0;

	result = bhy2_flush_fifo(0xFE, glove_devices[index].dev);

	return result;
}






void glove_rebaseline_mag(void)
{
	/* Drop the heading lock and the learned reference field, keep the
	 * hard/soft-iron calibration.  Use after moving to a new location. */
	for (uint8_t i = 0; i < NUM_DEVICES; i++)
	{
		if (!glove_devices[i].init) continue;
		mag_fusion_rebase_runtime(&glove_devices[i].fuse);
		glove_send_replay_event_link(glove_devices[i].link, 13u, 0, 0);
	}
	REPLAY_PRINTF("[MAG] heading reference dropped on all links\r\n");
}

void glove_hard_iron_start(void)
{
	for (uint8_t i = 0; i < NUM_DEVICES; i++)
	{
		if (!glove_devices[i].init) continue;
		mag_auto_cal_state[i] = 0u;
		mag_fusion_cal_start(&glove_devices[i].fuse);
		glove_send_replay_event_link(glove_devices[i].link, 8u, 0, 0);
	}
	REPLAY_PRINTF("[MAG] calibration started - rotate every finger through as many\r\n");
	REPLAY_PRINTF("      attitudes continuously; the fit auto-commits when valid.\r\n");
}

void glove_hard_iron_finish(void)
{
	uint8_t ok = 0, total = 0;

	for (uint8_t i = 0; i < NUM_DEVICES; i++)
	{
		if (!glove_devices[i].init) continue;
		total++;
		if (mag_fusion_cal_finish(&glove_devices[i].fuse) == MAG_CAL_OK) ok++;
	}
	REPLAY_PRINTF("[MAG] calibration finished: %u of %u links OK\r\n", ok, total);
	glove_send_replay_event(9u, (int32_t)ok, (int32_t)total);
	if (ok)
	{
		mag_store_request_save();
		mag_store_periodic();
	}
	glove_print_mag_status();
}

/*
 * Open a learning window on every link.  Use this when the board is KNOWN to
 * have changed - a magnet was brought near it, hardware was swapped, it moved
 * into a different vehicle - not as a periodic tidy-up.  Normal operation
 * keeps the model locked; see the long note in mag_fusion.h.
 */
void glove_mag_relearn(void)
{
	for (uint8_t i = 0; i < NUM_DEVICES; i++)
	{
		if (!glove_devices[i].init) continue;
		mag_fusion_relearn(&glove_devices[i].fuse);
		glove_send_replay_event_link(glove_devices[i].link, 11u, 0, 0);
	}
	REPLAY_PRINTF("[MAG] learning window opened on all links\r\n");
}

void glove_mag_lock(void)
{
	for (uint8_t i = 0; i < NUM_DEVICES; i++)
	{
		if (!glove_devices[i].init) continue;
		mag_fusion_lock(&glove_devices[i].fuse);
		glove_send_replay_event_link(glove_devices[i].link, 12u, 0, 0);
	}
	REPLAY_PRINTF("[MAG] learning locked on all links\r\n");
}

void glove_hard_iron_clear(void)
{
	for (uint8_t i = 0; i < NUM_DEVICES; i++)
	{
		if (!glove_devices[i].init) continue;
		mag_fusion_cal_clear(&glove_devices[i].fuse);
		glove_send_replay_event_link(glove_devices[i].link, 10u, 0, 0);
	}
	mag_store_pending = 0u;
	for (uint8_t i = 0; i < NUM_DEVICES; i++) mag_auto_cal_state[i] = 0u;
	mag_store_erase();
	REPLAY_PRINTF("[MAG] calibration cleared on all links\r\n");
}

/*
 * Human-readable status dump.
 *
 * NOTE: this shares the CDC endpoint with the COBS/protobuf stream, so the
 * host will see one corrupt frame around the text.  A COBS decoder resyncs at
 * the next zero byte, so it is harmless, but do not call it in a tight loop.
 * Everything is printed as scaled integers on purpose - newlib-nano drops %f
 * unless you link with -u _printf_float.
 */
void glove_print_mag_status(void)
{
	static const char *st[] = { "NO_CAL", "CAL", "ACQ", "LOCKED", "DEGRADED", "COAST" };

	REPLAY_PRINTF("\r\n link  status    trust  radius1 radius2 model  resid  align  grad/thr  drift   dip  stuck fixes  fen feobs feres fed feB feA  geo eStep eDr eLp eN eDi eGr eFl w rej hev cand cands good eIn  datum  yawc rev lrn opn\r\n");
	for (uint8_t i = 0; i < NUM_DEVICES; i++)
	{
		mag_fusion_t *f = &glove_devices[i].fuse;
		if (!glove_devices[i].init) continue;

		/* Everything is printed as scaled integers on purpose - newlib-nano
		 * drops %f unless you link with -u _printf_float. */
		float fed = sqrtf(f->fe.d[0]*f->fe.d[0] + f->fe.d[1]*f->fe.d[1] + f->fe.d[2]*f->fe.d[2]);
		float feb = sqrtf(f->fe.B[0]*f->fe.B[0] + f->fe.B[1]*f->fe.B[1] + f->fe.B[2]*f->fe.B[2]);
		REPLAY_PRINTF(" %2u    %-8s  %3d%%   %5d   %5d  %-5s %4d%%  %-5s  %4d/%-4d %5d %4d%s %4ds %3u  %3d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %3d %1d %1d %1d %4d %4d %5d %6d %5d %3lu %3u %3u\r\n",
		       (unsigned)glove_devices[i].sensor_id,
		       st[(f->status < 6u) ? f->status : 0u],
		       (int)(f->weight * 100.0f),
		       (int)f->cal[0].radius,
		       (int)f->cal[1].radius,
		       (f->cal[0].model == 2u) ? "full" : ((f->cal[0].model == 1u) ? "diag" : "sph"),
		       (int)(f->cal[0].quality * 100.0f),
		       mag_fusion_dual_ready(f) ? "yes" : (f->have2 ? "wait" : "none"),
		       (int)(f->grad_now * 1000.0f),
		       (int)(f->grad_thr * 1000.0f),
		       (int)(f->drift_rate * 57295.8f),      /* milli-deg/s */
		       (int)(f->ref_dip * 57.2957795f),
		       f->ref_dip_valid ? "" : " (?)",
		       (int)f->stuck_s,
		       (unsigned)mag_fusion_fixes(f),
		       (int)f->fe.n,
		       (int)(f->fe.obs * 1000.0f),
		       (int)(f->fe.res * 1000.0f),
		       (int)(fed * 1000.0f),
		       (int)(feb * 1000.0f),
		       (int)mag_fusion_align_deg(f),
		       (int)(f->cal[0].geo_q * 1000.0f),
		       (int)(f->e_step * 1000.0f),
		       (int)(f->e_drift * 1000.0f),
		       (int)(f->e_lp * 1000.0f),
		       (int)(f->e_norm * 1000.0f),
		       (int)(f->e_dip * 1000.0f),
		       (int)(f->e_grad * 1000.0f),
		       (int)(f->e_fleet * 1000.0f),
		       (int)(f->weight * 100.0f),
		       (int)f->rejecting,
		       (int)f->hard_event,
		       (int)f->cand_valid,
		       (int)(f->cand_s * 100.0f),
		       (int)(f->good_s * 100.0f),
		       (int)(f->e_inst * 1000.0f),
		       (int)mag_fusion_datum_deg(f),
		       (int)mag_fusion_yaw_corr_deg(f),
		       (unsigned long)f->cal[0].revision,
		       (unsigned)mag_fusion_learn_state(f),
		       (unsigned)mag_fusion_learn_opens(f));

		/* 第二行：yaw 大修正路径的诊断镜像。
		 * 让"修正什么时候触发、落到哪一档、冻结多久、回正多快、这一拍实际
		 * 修了多少"全部能从录制里看到 —— 没有这些，三档逻辑无法验证。
		 * 全部用缩放整数打印（newlib-nano 的 %f 不可靠）。 */
		REPLAY_PRINTF("      YAW innov_d10=%d relax_x100=%d hold_ms=%d slew_dps_x10=%d "
		       "trust_x100=%d large=%u snap=%u clean=%u nis=%u step_d100=%d "
		       "direff_x100=%d\r\n",
		       (int)(f->yawd_innov_deg * 10.0f),
		       (int)(f->yawd_relax * 100.0f),
		       (int)(f->yawd_hold_s * 1000.0f),
		       (int)(f->yawd_slew_dps * 10.0f),
		       (int)(f->yawd_trust * 100.0f),
		       (unsigned)f->yawd_large, (unsigned)f->yawd_snap,
		       (unsigned)f->yawd_clean, (unsigned)f->yawd_nis_big,
		       (int)(f->yawd_step_deg * 100.0f),
		       (int)(f->dir_strength_eff * 100.0f));
	}
	REPLAY_PRINTF("  grad/thr are in 1/1000 of field radius; drift in milli-deg/s.\r\n");
	REPLAY_PRINTF("  align 'none' = no second part seen, 'wait' = not aligned yet.\r\n");
	REPLAY_PRINTF("  stuck = seconds rejected; fixes = times a magnetised board was corrected.\r\n");
	REPLAY_PRINTF("  model: full = tilted soft iron resolved, diag/sph = fell back (expect bias).\r\n");
	REPLAY_PRINTF("  resid = fit residual in %% of field; it should keep dropping as you move.\r\n");
	REPLAY_PRINTF("  datum/yawc in degrees.  DATUM MUST NOT MOVE in ordinary use - every\r\n");
	REPLAY_PRINTF("  degree it gains is a degree of gyro drift made permanent.  yawc moving\r\n");
	REPLAY_PRINTF("  is the module working.  rev = ellipsoid refits committed.\r\n");
	REPLAY_PRINTF("  lrn: 0 bootstrap 1 locked 2 armed 3 open; opn = windows opened.\r\n\r\n");
}
