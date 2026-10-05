#ifndef INC_GLOVE_MAG_EKF_H_
#define INC_GLOVE_MAG_EKF_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Six-state error-state Kalman filter for a 9-axis AHRS.
 *
 * Nominal state:
 *   q  : body -> world quaternion, (w, x, y, z)
 *   bg : gyro bias in rad/s, body frame
 *
 * Error state:
 *   dtheta(3), dbg(3)
 *
 * The gyro drives the nominal quaternion.  Gravity/accelerometer updates
 * observe roll and pitch.  The calibrated magnetic vector observes heading
 * relative to the learned world field, Bw.  Magnetic quality is supplied by
 * the existing mag_fusion disturbance detector; a zero quality disables the
 * update instead of allowing a magnet to pull attitude.
 */
typedef struct mag_ekf {
    float q[4];                    /* body -> world, w,x,y,z              */
    float bg[3];                   /* gyro bias, rad/s                    */
    float P[6][6];                 /* error covariance                    */

    float Bw[3];                   /* learned world magnetic unit vector  */
    float g_b[3];                  /* latest body gravity/up direction    */
    float m_b[3];                  /* latest calibrated body field        */

    float acc_g;                   /* last accel magnitude in g           */
    float mag_norm;                /* last calibrated field magnitude     */
    float gyro_rad_s[3];           /* last corrected gyro, rad/s          */
    float omega_rad_s;             /* last predicted body rate            */

    float static_s;                /* stable stationary time              */
    float motion_s;                /* moving time                           */
    float sat_s;                   /* gyro saturation time                   */
    float dt_sat;                  /* total saturated time                   */

    float gravity_nis;
    float mag_nis;
    float gravity_innov_rad;
    float mag_innov_rad;
    float q_gamerv_error_rad;
    float yaw_align_rad;
    float yaw_align_valid;

    uint32_t samples;
    uint32_t mag_updates;
    uint64_t last_gyro_ts;
    uint64_t last_acc_ts;
    uint64_t last_mag_ts;
    uint64_t last_gravity_ts;

    uint8_t have_gyro;
    uint8_t have_acc;
    uint8_t have_mag;
    uint8_t have_gravity;
    uint8_t q_valid;
    uint8_t bw_valid;
    uint8_t acc_sign;              /* +1/-1, auto-detected                */
    uint8_t saturated;
    uint8_t stationary;
    uint8_t mag_rebased;
    uint8_t reserved;
} mag_ekf_t;

void mag_ekf_init(mag_ekf_t *e);
void mag_ekf_seed_quat(mag_ekf_t *e, const float q_br[4]);
void mag_ekf_predict(mag_ekf_t *e, const float gyro_lsb[3], uint64_t ts_ticks);
void mag_ekf_update_accel(mag_ekf_t *e, const float acc_lsb[3], uint64_t ts_ticks);
void mag_ekf_update_gravity_quat(mag_ekf_t *e, const float q_meas[4], uint64_t ts_ticks);
void mag_ekf_update_mag(mag_ekf_t *e, const float mag[3], uint64_t ts_ticks, float quality);
void mag_ekf_rebase_mag(mag_ekf_t *e, const float mag[3]);
void mag_ekf_force_static_reinit(mag_ekf_t *e);
void mag_ekf_get_quat(const mag_ekf_t *e, float q[4]);
float mag_ekf_yaw_deg(const mag_ekf_t *e);
float mag_ekf_q_error_deg(const float qa[4], const float qb[4]);

#ifdef __cplusplus
}
#endif

#endif /* INC_GLOVE_MAG_EKF_H_ */

