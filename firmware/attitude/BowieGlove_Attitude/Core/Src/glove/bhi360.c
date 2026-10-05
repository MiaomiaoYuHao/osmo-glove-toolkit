/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include "bhy2_defs.h"
#include "bhy2.h"
#include "bhy2_parse.h"

#include "firmware/bhi360/latest_11_13/Bosch_Shuttle3_BHI360_BMM350C_Poll_Meta_12.fw.h"

#include "tusb.h"
#include "glove/common.h"
#include "bhi360.h"
#include "comms/bowie.pb.h"
#include "glove.h"

int8_t bhi360_init(struct bhy2_dev *dev, struct mag_data_dev *device){
	int8_t rslt = BHY2_OK;

	mag_fusion_init(&device->fuse);

	rslt = bhy2_init(BHY2_I2C_INTERFACE, bhy2_i2c_read, bhy2_i2c_write, bhy2_delay_us, BHY2_RD_WR_LEN, device, dev);

	return rslt;
}


int8_t upload_firmware_partly(struct bhy2_dev *dev)
{
    uint32_t incr = 256; /* Max command packet size */
    uint32_t len = sizeof(bhy2_firmware_image);
    int8_t rslt = BHY2_OK;

    if ((incr % 4) != 0) /* Round off to higher 4 bytes */
    {
        incr = ((incr >> 2) + 1) << 2;
    }

    for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr)
    {
        if (incr > (len - i)) /* If last payload */
        {
            incr = len - i;
            if ((incr % 4) != 0) /* Round off to higher 4 bytes */
            {
                incr = ((incr >> 2) + 1) << 2;
            }
        }

        rslt = bhy2_upload_firmware_to_ram_partly(&bhy2_firmware_image[i], len, i, incr, dev);

//        REPLAY_PRINTF("%.2f%% complete\r", (float)(i + incr) / (float)len * 100.0f);
    }

//    REPLAY_PRINTF("\n");

    return rslt;
}

int8_t bhi360_upload_firmware(uint8_t boot_stat, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;


    //		results[8] = upload_firmware_partly(glove_devices[idx].dev);
    //		results[8] = bhy2_boot_from_ram(glove_devices[idx].dev);


//    REPLAY_PRINTF("Loading firmware into RAM.\r\n");

    rslt = upload_firmware_partly(dev);
//    rslt = bhy2_upload_firmware_to_ram(bhy2_firmware_image, sizeof(bhy2_firmware_image), dev);

//    temp_rslt = bhy2_get_error_value(&sensor_error, dev);
//    if (sensor_error)
//    {
//        REPLAY_PRINTF("%s\r\n", get_sensor_error_text(sensor_error));
//    }
    if(rslt != BHY2_OK)
    {
    	return rslt;
    }
//    print_api_error(rslt, dev);
//    print_api_error(temp_rslt, dev);


//    REPLAY_PRINTF("Booting from RAM.\r\n");
    rslt = bhy2_boot_from_ram(dev);
    REPLAY_PRINTF("boot_from_ram: %d\r\n", rslt);

//    temp_rslt = bhy2_get_error_value(&sensor_error, dev);
//    if (sensor_error)
//    {
//        REPLAY_PRINTF("%s\r\n", get_sensor_error_text(sensor_error));
//    }

//    print_api_error(rslt, dev);
//    print_api_error(temp_rslt, dev);
    return rslt;
}

#define Q_SCALE 16384.0f

/*
 * Quaternion callback.
 *
 * The BHI360 hands us a 6-axis Game Rotation Vector: roll and pitch are
 * gravity-locked, yaw is free-running.  mag_fusion_on_quat() rotates it about
 * the world vertical by a slowly-filtered, disturbance-gated correction so
 * that the result is an absolute attitude referenced to magnetic north.
 *
 * Because the correction is a pure yaw rotation, roll and pitch can never be
 * damaged by a magnetic disturbance - see mag_fusion.h.
 */
void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct mag_data_dev* dev = (struct mag_data_dev*)callback_ref;
    struct bhy2_data_quaternion data;
    uint64_t ts, ns64;
    uint32_t s, ns;
    float q[4];

    if (callback_info->data_size != 11) return;
    bhy2_parse_quaternion(callback_info->data_ptr, &data);

    ts = *callback_info->time_stamp;
    dev->replay_last_ts = ts;

    q[0] = (float)data.w / Q_SCALE;
    q[1] = (float)data.x / Q_SCALE;
    q[2] = (float)data.y / Q_SCALE;
    q[3] = (float)data.z / Q_SCALE;

#if REPLAY_STREAM_ENABLE
    /* 算法的原始输入镜像：这里发送的是 mag_fusion_on_quat() 之前的 GAMERV。
     * 只发数值，字段顺序和缩放由主机 replay_protocol.py 对齐。 */
    ns64 = ts * UINT64_C(15625);
    s    = (uint32_t)(ns64 / UINT64_C(1000000000));
    ns   = (uint32_t)(ns64 - ((uint64_t)s * UINT64_C(1000000000)));
    glove_send_quat_data(dev->count, s, ns, dev->finger, dev->link, REPLAY_QUAT_ID,
                         q[1], q[2], q[3], q[0], (float)data.accuracy);
#endif

#if BHI_DIAG_PRINT
    if ((dev->count % BHI_DIAG_EVERY) == 0u)
    {
        REPLAY_PRINTF("GQ f%u l%u id%u acc%u raw %d %d %d %d yawc %d\r\n",
               dev->finger, dev->link, dev->sensor_id,
               (unsigned)data.accuracy,
               (int)data.x, (int)data.y, (int)data.z, (int)data.w,
               (int)(mag_fusion_yaw_corr_deg(&dev->fuse) * 100.0f));
    }
#endif

    /* Corrects q in place; leaves it alone while uncalibrated. */
    (void)mag_fusion_on_quat(&dev->fuse, ts, q);

#if BHI_DIAG_PRINT
    if ((dev->count % BHI_DIAG_EVERY) == 0u)
    {
        const mag_fusion_t *mf = &dev->fuse;

        REPLAY_PRINTF("GM f%u l%u w%03d HE%u RJ%u ST%u OM%05d DLS%04d YV%u LA%u YT%03d YH%04d "
               "EI%04d EL%04d EN%04d ED%04d ES%04d EDR%04d EG%04d EF%04d\r\n",
               dev->finger, dev->link,
               (int)(mf->weight * 1000.0f),
               (unsigned)mf->hard_event, (unsigned)mf->rejecting, (unsigned)mf->status,
               (int)(mf->omega * 572.9578f),
               (int)(mf->dir_lock_s * 1000.0f),
               (unsigned)mf->yaw_valid, (unsigned)mf->yaw_large_active,
               (int)(mf->yaw_trust * 1000.0f),
               (int)(mf->yaw_hold_s * 1000.0f),
               (int)(mf->e_inst * 1000.0f), (int)(mf->e_lp * 1000.0f),
               (int)(mf->e_norm * 1000.0f), (int)(mf->e_dip * 1000.0f),
               (int)(mf->e_step * 1000.0f), (int)(mf->e_drift * 1000.0f),
               (int)(mf->e_grad * 1000.0f), (int)(mf->e_fleet * 1000.0f));
    }
#endif

    ns64 = ts * UINT64_C(15625);
    s    = (uint32_t)(ns64 / UINT64_C(1000000000));
    ns   = (uint32_t)(ns64 - ((uint64_t)s * UINT64_C(1000000000)));

    glove_send_quat_data(dev->count, s, ns, dev->finger, dev->link, dev->sensor_id,
                         q[1], q[2], q[3], q[0], mag_fusion_accuracy(&dev->fuse));
    dev->count++;
}

/*
 * Magnetometer callback.  Feeds the calibrator, applies the hard/soft-iron
 * correction and publishes the calibrated vector.  While the link is still
 * uncalibrated the raw vector is published and mag_fusion refuses to steer
 * yaw with it.
 */
void parse_magnetometer(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct mag_data_dev* dev = (struct mag_data_dev*)callback_ref;
    struct bhy2_data_xyz data;
    uint64_t ts, ns64;
    uint32_t s, ns;
    float raw[3], cal[3];

    if (callback_info->data_size != 7) return;
    bhy2_parse_xyz(callback_info->data_ptr, &data);

    raw[0] = (float)data.x;
    raw[1] = (float)data.y;
    raw[2] = (float)data.z;

    /*
     * The BHI image can publish two MAG_ID packets with the same timestamp:
     * the first is the real BMM350 vector, the second is a different/large
     * payload.  Keep only the first packet per timestamp.  This is a data
     * path guard and runs before both replay and mag_fusion.
     */
    ts = *callback_info->time_stamp;
    if (dev->mag_last_ts_valid && (ts == dev->mag_last_ts)) {
        dev->replay_mag_dup_drop++;
        return;
    }
    dev->mag_last_ts = ts;
    dev->mag_last_ts_valid = 1u;

#if REPLAY_STREAM_ENABLE
    /* 原始磁矢量（磁标定和 FIR 之前）。 */
    dev->replay_last_ts = ts;
    dev->replay_mag_count++;
    ns64 = ts * UINT64_C(15625);
    s    = (uint32_t)(ns64 / UINT64_C(1000000000));
    ns   = (uint32_t)(ns64 - ((uint64_t)s * UINT64_C(1000000000)));
    glove_send_mag_data(dev->count, s, ns, dev->finger, dev->link, REPLAY_MAG_ID,
                        raw[0], raw[1], raw[2]);
#endif

    (void)mag_fusion_on_mag(&dev->fuse, raw, cal);

#if MAG_DIAG_PRINT
    /*
     * RAW is the sensor.  CAL is what the host receives, and they are not the
     * same thing: CAL = A * W * (RAW - OFF).  Park the board at a repeatable
     * mechanical pose, note both, move it about, come back to the same pose.
     *
     *   RAW same, CAL moved  -> the model moved; REV/FIX/OPENS say which part
     *   RAW moved too        -> the field or the sensor really did change
     *
     * REV   cal[0].revision - ellipsoid refits committed (offset/W/radius)
     * FIX   fe.fixes        - hard-iron offsets folded in
     * OPENS learning windows opened since boot
     * LRN   0 bootstrap  1 locked  2 armed  3 open
     */
    if ((dev->fuse.cal[0].active != 1u) &&
        (++dev->diag_div >= MAG_DIAG_EVERY))
    {
        dev->diag_div = 0u;
        /* Scaled integers on purpose: newlib-nano drops %f unless you link
         * with -u _printf_float.  CAL and OFF are in 1/100 LSB, DATUM and
         * YAWC in 1/100 degree. */
        REPLAY_PRINTF("MAG f%u l%u RAW %6d %6d %6d | CAL %7d %7d %7d"
               " | OFF %7d %7d %7d | REV %lu FIX %u OPENS %u LRN %u"
               " | DATUM %6d YAWC %6d\r\n",
               dev->finger, dev->link,
               data.x, data.y, data.z,
               (int)(cal[0] * 100.0f),
               (int)(cal[1] * 100.0f),
               (int)(cal[2] * 100.0f),
               (int)(dev->fuse.cal[0].offset[0] * 100.0f),
               (int)(dev->fuse.cal[0].offset[1] * 100.0f),
               (int)(dev->fuse.cal[0].offset[2] * 100.0f),
               (unsigned long)dev->fuse.cal[0].revision,
               mag_fusion_fixes(&dev->fuse),
               mag_fusion_learn_opens(&dev->fuse),
               mag_fusion_learn_state(&dev->fuse),
               (int)(mag_fusion_datum_deg(&dev->fuse) * 100.0f),
               (int)(mag_fusion_yaw_corr_deg(&dev->fuse) * 100.0f));
    }
#endif

    ns64 = ts * UINT64_C(15625);
    s    = (uint32_t)(ns64 / UINT64_C(1000000000));
    ns   = (uint32_t)(ns64 - ((uint64_t)s * UINT64_C(1000000000)));

    glove_send_mag_data(dev->count, s, ns, dev->finger, dev->link, dev->sensor_id,
                        cal[0], cal[1], cal[2]);
}

void parse_gyroscope(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct mag_data_dev *dev = (struct mag_data_dev*)callback_ref;
    struct bhy2_data_xyz data;
    float raw[3];
    uint64_t ts, ns64;
    uint32_t s, ns;

    if (callback_info->data_size != 7) return;
    bhy2_parse_xyz(callback_info->data_ptr, &data);
    raw[0] = (float)data.x;
    raw[1] = (float)data.y;
    raw[2] = (float)data.z;

#if REPLAY_STREAM_ENABLE
    /* 实际送入 yaw 环的 corrected gyro，发送发生在 mag_fusion_on_gyro() 之前。 */
    ts = *callback_info->time_stamp;
    dev->replay_last_ts = ts;
    dev->replay_gyro_count++;
    ns64 = ts * UINT64_C(15625);
    s    = (uint32_t)(ns64 / UINT64_C(1000000000));
    ns   = (uint32_t)(ns64 - ((uint64_t)s * UINT64_C(1000000000)));
    glove_send_mag_data(dev->count, s, ns, dev->finger, dev->link, REPLAY_GYRO_ID,
                        raw[0], raw[1], raw[2]);
#endif

    mag_fusion_on_gyro(&dev->fuse, raw);

#if BHI_DIAG_PRINT
    if (++dev->diag_gyro_div >= (uint16_t)BHI_DIAG_EVERY)
    {
        int cx = 0, cy = 0, cz = 0;
        uint8_t cv = dev->diag_gyro_corr_valid;
        dev->diag_gyro_div = 0u;

        if (cv)
        {
            cx = (int)dev->diag_gyro_corr[0];
            cy = (int)dev->diag_gyro_corr[1];
            cz = (int)dev->diag_gyro_corr[2];
        }

        REPLAY_PRINTF("GY f%u l%u P %d %d %d C %d %d %d CV%u B %d %d %d ACT %d MH %u\r\n",
               dev->finger, dev->link,
               (int)data.x, (int)data.y, (int)data.z,
               cx, cy, cz, (unsigned)cv,
               (int)(dev->fuse.gyro_bias[0] * 10.0f),
               (int)(dev->fuse.gyro_bias[1] * 10.0f),
               (int)(dev->fuse.gyro_bias[2] * 10.0f),
               (int)(dev->fuse.gyro_activity * 10.0f),
               (unsigned)dev->fuse.motion_hold);
    }
#endif
}

/* Corrected gyroscope stream (id 13).  Diagnostic-only: it is not fed into
 * the 6-axis quaternion, it is logged alongside passthrough gyro so the host
 * can see exactly what the BHI360 internal calibration is doing. */
void parse_gyroscope_corrected(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
#if BHI_DIAG_PRINT
    struct mag_data_dev *dev = (struct mag_data_dev*)callback_ref;
    struct bhy2_data_xyz data;

    if (callback_info->data_size != 7) return;
    bhy2_parse_xyz(callback_info->data_ptr, &data);
    dev->diag_gyro_corr[0] = (float)data.x;
    dev->diag_gyro_corr[1] = (float)data.y;
    dev->diag_gyro_corr[2] = (float)data.z;
    dev->diag_gyro_corr_ts = *callback_info->time_stamp;
    dev->diag_gyro_corr_valid = 1u;
#else
    (void)callback_info;
    (void)callback_ref;
#endif
}

void bhi360_print_sensor_info(struct bhy2_dev *dev, uint8_t link)
{
#if BHI_DIAG_PRINT
    static const uint8_t ids[] = {
        QUAT_SENSOR_ID,
        GYRO_ID,
        BHY2_SENSOR_ID_GYRO_RAW,
        BHY2_SENSOR_ID_GYRO_BIAS,
        BHY2_SENSOR_ID_SI_GYROS
    };
    uint8_t i;

    for (i = 0u; i < (uint8_t)(sizeof(ids) / sizeof(ids[0])); i++)
    {
        struct bhy2_sensor_info info;
        int8_t r = bhy2_get_sensor_info(ids[i], &info, dev);

        if (r == BHY2_OK)
        {
            REPLAY_PRINTF("SINFO l%u id%u %s drv%u range%u res%u maxr%u minr%u\r\n",
                   (unsigned)link, (unsigned)ids[i], get_sensor_name(ids[i]),
                   (unsigned)info.driver_id,
                   (unsigned)info.max_range.u16_val,
                   (unsigned)info.resolution.u16_val,
                   (unsigned)(info.max_rate.f_val * 100.0f),
                   (unsigned)(info.min_rate.f_val * 100.0f));
        }
        else
        {
            REPLAY_PRINTF("SINFO l%u id%u %s err%d\r\n",
                   (unsigned)link, (unsigned)ids[i], get_sensor_name(ids[i]), (int)r);
        }
    }
#else
    (void)dev;
    (void)link;
#endif
}

/*
 * Second magnetometer.  Feeds the same calibrator machinery as the primary;
 * once both are calibrated the fusion estimates the fixed rotation between
 * them and starts using their disagreement as a disturbance detector.
 */
void parse_magnetometer2(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct mag_data_dev* dev = (struct mag_data_dev*)callback_ref;
    struct bhy2_data_xyz data;
    float raw[3];
    uint64_t ts, ns64;
    uint32_t s, ns;

    if (callback_info->data_size != 7) return;
    bhy2_parse_xyz(callback_info->data_ptr, &data);

    raw[0] = (float)data.x;
    raw[1] = (float)data.y;
    raw[2] = (float)data.z;

#if REPLAY_STREAM_ENABLE
    ts   = *callback_info->time_stamp;
    dev->replay_last_ts = ts;
    dev->replay_mag2_count++;
    ns64 = ts * UINT64_C(15625);
    s    = (uint32_t)(ns64 / UINT64_C(1000000000));
    ns   = (uint32_t)(ns64 - ((uint64_t)s * UINT64_C(1000000000)));
    glove_send_mag_data(dev->count, s, ns, dev->finger, dev->link, REPLAY_MAG2_ID,
                        raw[0], raw[1], raw[2]);
#endif

    (void)mag_fusion_on_mag2(&dev->fuse, raw, NULL);
}

/*
 * Accelerometer callback.  It is not yet an input to the yaw loop, but it is
 * mirrored so that later ZUPT/still-gate work can be replayed offline without
 * changing the recorded format.
 */
void parse_accelerometer(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct mag_data_dev *dev = (struct mag_data_dev*)callback_ref;
    struct bhy2_data_xyz data;
    float raw[3];
    uint64_t ts, ns64;
    uint32_t s, ns;

    if (callback_info->data_size != 7) return;
    bhy2_parse_xyz(callback_info->data_ptr, &data);
    raw[0] = (float)data.x;
    raw[1] = (float)data.y;
    raw[2] = (float)data.z;

#if REPLAY_STREAM_ENABLE
    ts   = *callback_info->time_stamp;
    dev->replay_last_ts = ts;
    dev->replay_acc_count++;
    ns64 = ts * UINT64_C(15625);
    s    = (uint32_t)(ns64 / UINT64_C(1000000000));
    ns   = (uint32_t)(ns64 - ((uint64_t)s * UINT64_C(1000000000)));
    glove_send_mag_data(dev->count, s, ns, dev->finger, dev->link, REPLAY_ACC_ID,
                        raw[0], raw[1], raw[2]);
#else
    (void)dev;
    (void)raw;
    (void)ts;
    (void)ns64;
    (void)s;
    (void)ns;
#endif
}

void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct mag_data_dev *dev = (struct mag_data_dev *)callback_ref;
    uint8_t replay_link = dev ? dev->link : 0u;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    char *event_text;

    if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    switch (meta_event_type)
    {
        case BHY2_META_EVENT_FLUSH_COMPLETE:
            REPLAY_PRINTF("%s Flush complete for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            REPLAY_PRINTF("%s Sample rate changed for sensor id %u\r\n", event_text, byte1);
            glove_send_replay_event_link(replay_link, 33u, byte1, 0);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
            REPLAY_PRINTF("%s Power mode changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
            REPLAY_PRINTF("%s Algorithm event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            REPLAY_PRINTF("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
            glove_send_replay_event_link(replay_link, 32u, byte1, byte2);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
            REPLAY_PRINTF("%s BSX event (do steps main)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
            REPLAY_PRINTF("%s BSX event (do steps calib)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            REPLAY_PRINTF("%s BSX event (get output signal)\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            REPLAY_PRINTF("%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1, byte2);
            glove_replay_note_meta(1u, replay_link);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            REPLAY_PRINTF("%s FIFO overflow\r\n", event_text);
            glove_replay_note_meta(0u, replay_link);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
            REPLAY_PRINTF("%s Dynamic range changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
            REPLAY_PRINTF("%s FIFO watermark reached\r\n", event_text);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            REPLAY_PRINTF("%s Firmware initialized. Firmware version %u\r\n", event_text, ((uint16_t)byte2 << 8) | byte1);
            glove_send_replay_event_link(replay_link, 30u,
                                         (int32_t)(((uint16_t)byte2 << 8) | byte1), 0);
            break;
        case BHY2_META_TRANSFER_CAUSE:
            REPLAY_PRINTF("%s Transfer cause for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
            REPLAY_PRINTF("%s Sensor framework event for sensor id %u\r\n", event_text, byte1);
            glove_send_replay_event_link(replay_link, 34u, byte1, byte2);
            break;
        case BHY2_META_EVENT_RESET:
            REPLAY_PRINTF("%s Reset event %d, 0x%02X,  0x%01X\r\n", event_text, meta_event_type, byte1, byte2);
            glove_send_replay_event_link(replay_link, 31u, byte1, byte2);
            ///error checking
            uint8_t error_code = 0;
            uint8_t result;
            result = bhy2_get_error_reg(BHY2_REG_INT_STATUS, &error_code, glove_devices[0].dev);
			REPLAY_PRINTF("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_INT_STATUS, result, error_code);

			result = bhy2_get_error_reg(BHY2_REG_ERROR_VALUE, &error_code, glove_devices[0].dev);
			REPLAY_PRINTF("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_ERROR_VALUE, result, error_code);

			result = bhy2_get_error_reg(BHY2_REG_ERROR_AUX, &error_code, glove_devices[0].dev);
			REPLAY_PRINTF("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_ERROR_AUX, result, error_code);

			result = bhy2_get_error_reg(BHY2_REG_DEBUG_VALUE, &error_code, glove_devices[0].dev);
			REPLAY_PRINTF("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_DEBUG_VALUE, result, error_code);

			result = bhy2_get_error_reg(BHY2_REG_DEBUG_STATE, &error_code, glove_devices[0].dev);
			REPLAY_PRINTF("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_DEBUG_STATE, result, error_code);

			/**
			 * @brief Function to get the post mortem data
			 * @param[out] post_mortem  : Reference to the data buffer to store the post mortem data
			 * @param[in] buffer_len    : Length of the data buffer
			 * @param[out] actual_len   : Actual length of the post mortem data
			 * @param[in] dev           : Device reference
			 * @return API error codes
			 */
//			int8_t bhy2_get_post_mortem_data(uint8_t *post_mortem, uint32_t buffer_len, uint32_t *actual_len, struct bhy2_dev *dev);

			struct bhy2_post_mortem post_mortem_data = { 0 };
			REPLAY_PRINTF("size of postmortem struct: %d", sizeof(struct bhy2_post_mortem));
			result = get_post_mortem_data(&post_mortem_data, glove_devices[0].dev);
//			result = bhy2_get_post_mortem_data(&post_mortem_data, sizeof(struct bhy2_post_mortem), &pmlen, glove_devices[0].dev);
			REPLAY_PRINTF("Post mortem data: %d", result);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            REPLAY_PRINTF("%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
            glove_replay_note_meta(2u, replay_link);
            break;
    }
}

/**
* @brief Function to get the Post Mortem data
*/
int8_t get_post_mortem_data(struct bhy2_post_mortem *pminfo, struct bhy2_dev *bhy2)
{
    uint32_t pmlen = 0;
    int8_t rslt;

    rslt = bhy2_get_post_mortem_data((uint8_t*)pminfo, sizeof(struct bhy2_post_mortem), &pmlen, bhy2);

    return rslt;
}




