/*
 * glove.h
 *
 *      Author: Tess Hellebrekers
 */

#ifndef INC_GLOVE_H_
#define INC_GLOVE_H_

#include "bowie.pb.h"
#include "bhi360.h"

#define MUX_ADDR 		0x70
//#define NUM_DEVICES 	1
#define NUM_DEVICES 	20

typedef enum {
	COMM_OK = 0x00,
	COMM_ERROR = 0x01,
	COMM_PB_ERROR = 0x02,
	COMM_COBS_ERROR = 0x03,
} Comms_StatusTypedef;

/*
 * Bowie glove sensor map
 * {finger, link, sensor_id, mux_channel, device address, bhy2_dev}
 */
extern struct mag_data_dev glove_devices[NUM_DEVICES];

HAL_StatusTypeDef glove_mux_set_channel(uint8_t channel);
void glove_mux_reset();

void glove_load_firmware();
void scan_mux_channel(uint8_t channel);


void glove_send_quat_data(uint32_t index, uint32_t s, uint32_t ns, uint8_t finger, uint8_t link, uint8_t sensor_id, float x, float y, float z, float w, float accuracy);
void glove_send_mag_data(uint32_t index, uint32_t s, uint32_t ns, uint8_t finger, uint8_t link, uint8_t sensor_id, float mag_x, float mag_y, float mag_z);
Comms_StatusTypedef glove_coms_write(bowie_Data *msg);

uint8_t flush_glove();
uint8_t flush_quat_island(uint8_t index);
uint8_t flush_mag_island(uint8_t index);

int8_t init_island(uint8_t idx);
void init_glove();
void glove_rebaseline_mag(void);
uint8_t glove_reload_saved_calibration(void);
void glove_hard_iron_start(void);
void glove_hard_iron_finish(void);
void glove_hard_iron_clear(void);
void glove_print_mag_status(void);
void glove_mag_relearn(void);
void glove_mag_lock(void);
void glove_dump_sensor_list(void);
void glove_dump_sensor_info(void);
void glove_send_replay_snapshot(void);
/* 运行时可调参数：解析 "T DIR <0..100>" 并把方向残差门强度下发到磁融合。 */
void glove_tune_command(const char *line);
void glove_set_dir_strength_pct(int pct);
void glove_send_replay_config(void);
void glove_send_replay_event(uint16_t code, int32_t a, int32_t b);
void glove_send_replay_event_link(uint8_t link, uint16_t code, int32_t a, int32_t b);
void glove_replay_note_meta(uint8_t kind, uint8_t link);
void glove_send_replay_health(void);
void glove_update_data();

#endif /* INC_GLOVE_H_ */
