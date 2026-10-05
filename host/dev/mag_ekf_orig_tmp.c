#include <math.h>
#ifdef EK_DEBUG
#include <stdio.h>
#endif
#include <string.h>
#include "mag_ekf.h"

#define EK_PI             3.14159265358979323846f
#define EK_TWO_PI         6.28318530717958647692f
#define EK_DEG2RAD        0.01745329251994329577f
#define EK_RAD2DEG        57.2957795130823208768f
#define EK_TICK_S         1.5625e-5f
#define EK_GYRO_LSB_RAD   (0.06103515625f * EK_DEG2RAD)
#define EK_ACC_LSB_G      (16.0f / 32768.0f)
#define EK_GYRO_SAT_LSB   32000.0f
#define EK_GYRO_SIGMA     0.006f
#define EK_BIAS_RW        1.5e-5f
#define EK_ACC_SIGMA      0.035f
#define EK_MAG_SIGMA      0.055f
#define EK_NIS_MAX2       16.0f

static float ek_clampf(float v, float lo, float hi)
{
    if (!(v > lo)) return lo;
    if (v > hi) return hi;
    return v;
}

static float ek_dot3(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static float ek_norm3(const float v[3])
{
    return sqrtf(ek_dot3(v, v));
}

static void ek_norm3v(const float v[3], float out[3])
{
    float n = ek_norm3(v);
    if (n > 1.0e-12f) {
        out[0] = v[0] / n;
        out[1] = v[1] / n;
        out[2] = v[2] / n;
    } else {
        out[0] = out[1] = out[2] = 0.0f;
    }
}

static void ek_q_normalize(float q[4])
{
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 1.0e-12f) {
        q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
    } else {
        q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
    }
}

static void ek_q_mul(const float a[4], const float b[4], float out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

static void ek_q_rotate(const float q[4], const float v[3], float out[3])
{
    float a = 2.0f * (q[2]*v[2] - q[3]*v[1]);
    float b = 2.0f * (q[3]*v[0] - q[1]*v[2]);
    float c = 2.0f * (q[1]*v[1] - q[2]*v[0]);
    out[0] = v[0] + q[0]*a + (q[2]*c - q[3]*b);
    out[1] = v[1] + q[0]*b + (q[3]*a - q[1]*c);
    out[2] = v[2] + q[0]*c + (q[1]*b - q[2]*a);
}

static void ek_q_rotate_inv(const float q[4], const float v[3], float out[3])
{
    float qi[4] = { q[0], -q[1], -q[2], -q[3] };
    ek_q_rotate(qi, v, out);
}

static void ek_q_from_delta(const float d[3], float dq[4])
{
    float a = ek_norm3(d);
    if (a > 1.0e-8f) {
        float s = sinf(0.5f * a) / a;
        dq[0] = cosf(0.5f * a);
        dq[1] = s * d[0];
        dq[2] = s * d[1];
        dq[3] = s * d[2];
    } else {
        dq[0] = 1.0f;
        dq[1] = 0.5f * d[0];
        dq[2] = 0.5f * d[1];
        dq[3] = 0.5f * d[2];
    }
    ek_q_normalize(dq);
}

static float ek_q_angle(const float a[4], const float b[4])
{
    float d = fabsf(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]);
    return 2.0f * acosf(ek_clampf(d, 0.0f, 1.0f));
}

static void ek_mat6_zero(float A[6][6])
{
    memset(A, 0, sizeof(float) * 36u);
}

static void ek_mat6_eye(float A[6][6])
{
    ek_mat6_zero(A);
    for (int i = 0; i < 6; i++) A[i][i] = 1.0f;
}

static void ek_mat6_mul(const float A[6][6], const float B[6][6], float C[6][6])
{
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
    }
}

static void ek_mat6_sym(float A[6][6])
{
    for (int i = 0; i < 6; i++) {
        for (int j = i + 1; j < 6; j++) {
            float s = 0.5f * (A[i][j] + A[j][i]);
            A[i][j] = A[j][i] = s;
        }
    }
}

static void ek_mat6_clamp(float P[6][6])
{
    for (int i = 0; i < 6; i++) {
        float lo = (i < 3) ? 1.0e-6f : 1.0e-9f;
        float hi = (i < 3) ? 1.0f : 0.02f;
        P[i][i] = ek_clampf(P[i][i], lo, hi);
    }
}

static int ek_mat3_inv(const float A[3][3], float O[3][3])
{
    float a = A[0][0], b = A[0][1], c = A[0][2];
    float d = A[1][0], e = A[1][1], f = A[1][2];
    float g = A[2][0], h = A[2][1], i = A[2][2];
    float A00 = e*i - f*h;
    float A01 = c*h - b*i;
    float A02 = b*f - c*e;
    float A10 = f*g - d*i;
    float A11 = a*i - c*g;
    float A12 = c*d - a*f;
    float A20 = d*h - e*g;
    float A21 = b*g - a*h;
    float A22 = a*e - b*d;
    float det = a*A00 + b*A10 + c*A20;
    if (fabsf(det) < 1.0e-12f) return 0;
    float id = 1.0f / det;
    O[0][0] = A00*id; O[0][1] = A10*id; O[0][2] = A20*id;
    O[1][0] = A01*id; O[1][1] = A11*id; O[1][2] = A21*id;
    O[2][0] = A02*id; O[2][1] = A12*id; O[2][2] = A22*id;
    return 1;
}

static void ek_skew(const float v[3], float S[3][3])
{
    S[0][0] = 0.0f; S[0][1] = -v[2]; S[0][2] = v[1];
    S[1][0] = v[2]; S[1][1] = 0.0f; S[1][2] = -v[0];
    S[2][0] = -v[1]; S[2][1] = v[0]; S[2][2] = 0.0f;
}

static int ek_measurement3(mag_ekf_t *e, const float z[3], const float h[3],
                           float sigma, float quality, float *nis_out, float *innov_out)
{
    float R[3][3], S[3][3], Si[3][3], H[3][6], PH[6][3], K[6][3];
    float r[3], sk[3][3], q;
    float nis = 0.0f;
    float angle = 0.0f;

    if (!e->q_valid || (quality <= 0.01f)) return 0;

    r[0] = z[0] - h[0];
    r[1] = z[1] - h[1];
    r[2] = z[2] - h[2];
    angle = acosf(ek_clampf(ek_dot3(z, h), -1.0f, 1.0f));

    ek_skew(h, sk);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 6; j++) H[i][j] = 0.0f;
        for (int j = 0; j < 3; j++) H[i][j] = -sk[i][j];
    }

    q = ek_clampf(quality, 0.02f, 1.0f);
    float rv = (sigma * sigma) / q;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) R[i][j] = (i == j) ? rv : 0.0f;
    }

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) s += e->P[i][k] * H[j][k];
            PH[i][j] = s;
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) s += H[i][k] * PH[k][j];
            S[i][j] = s + R[i][j];
        }
    }
#ifdef EK_DEBUG
    {
        static int ekdbg = 0;
        if (ekdbg++ < 24) {
            fprintf(stderr, "EKDBG r=(%.6f %.6f %.6f) h=(%.6f %.6f %.6f) S=(%.9g %.9g %.9g / %.9g %.9g %.9g / %.9g %.9g %.9g) det=%.9g\n",
                    r[0],r[1],r[2],h[0],h[1],h[2],
                    S[0][0],S[0][1],S[0][2],S[1][0],S[1][1],S[1][2],S[2][0],S[2][1],S[2][2],
                    S[0][0]*(S[1][1]*S[2][2]-S[1][2]*S[2][1]) - S[0][1]*(S[1][0]*S[2][2]-S[1][2]*S[2][0]) + S[0][2]*(S[1][0]*S[2][1]-S[1][1]*S[2][0]));
        }
    }
#endif
    if (!ek_mat3_inv(S, Si)) return 0;

    {
        float rs[3];
        for (int i = 0; i < 3; i++) {
            float s = 0.0f;
            for (int j = 0; j < 3; j++) s += r[j] * Si[j][i];
            rs[i] = s;
        }
        nis = r[0]*rs[0] + r[1]*rs[1] + r[2]*rs[2];
    }
    if (nis_out) *nis_out = nis;
    if (innov_out) *innov_out = angle;
    if (nis > EK_NIS_MAX2) return 0;

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += PH[i][k] * Si[k][j];
            K[i][j] = s;
        }
    }

    {
        float dx[6], dq[4], qn[4];
        for (int i = 0; i < 6; i++) {
            float s = 0.0f;
            for (int j = 0; j < 3; j++) s += K[i][j] * r[j];
            dx[i] = s;
        }
        for (int bi = 0; bi < 3; bi++)
            dx[3 + bi] = 0.0f;
        ek_q_from_delta(dx, dq);
        ek_q_mul(e->q, dq, qn);
        memcpy(e->q, qn, sizeof(qn));
        ek_q_normalize(e->q);
        for (int i = 0; i < 3; i++) e->bg[i] += dx[3 + i];
    }

    {
        float IKH[6][6], T1[6][6], Pn[6][6];
        float KR[6][3];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                float s = 0.0f;
                for (int k = 0; k < 3; k++) s += K[i][k] * H[k][j];
                IKH[i][j] = ((i == j) ? 1.0f : 0.0f) - s;
            }
        }
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++) {
                float s = 0.0f;
                for (int k = 0; k < 6; k++) s += IKH[i][k] * e->P[k][j];
                T1[i][j] = s;
            }
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++) {
                float s = 0.0f;
                for (int k = 0; k < 6; k++) s += T1[i][k] * IKH[j][k];
                Pn[i][j] = s;
            }
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 3; j++) {
                float s = 0.0f;
                for (int k = 0; k < 3; k++) s += K[i][k] * R[k][j];
                KR[i][j] = s;
            }
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++) {
                float s = 0.0f;
                for (int k = 0; k < 3; k++) s += KR[i][k] * K[j][k];
                Pn[i][j] += s;
            }
        memcpy(e->P, Pn, sizeof(Pn));
        ek_mat6_sym(e->P);
    }
    ek_mat6_clamp(e->P);
    return 1;
}

void mag_ekf_init(mag_ekf_t *e)
{
    memset(e, 0, sizeof(*e));
    e->q[0] = 1.0f;
    e->acc_sign = 1u;
    for (int i = 0; i < 6; i++) {
        e->P[i][i] = (i < 3) ? 1.0f : 0.01f;
    }
}

void mag_ekf_seed_quat(mag_ekf_t *e, const float q_br[4])
{
    if (!e || !q_br) return;
    memcpy(e->q, q_br, sizeof(e->q));
    ek_q_normalize(e->q);
    e->q_valid = 1u;
    e->samples = 0u;
    e->last_gravity_ts = 0u;
}

void mag_ekf_predict(mag_ekf_t *e, const float gyro_lsb[3], uint64_t ts_ticks)
{
    float dt = 0.01f;
    float om[3];
    float sat = 0.0f;
    float omega_norm;
    float qd[4], qn[4];

    if (!e || !gyro_lsb || !e->q_valid) return;

    if (e->have_gyro && (ts_ticks > e->last_gyro_ts)) {
        dt = (float)(uint32_t)(ts_ticks - e->last_gyro_ts) * EK_TICK_S;
        dt = ek_clampf(dt, 0.001f, 0.10f);
    }
    e->last_gyro_ts = ts_ticks;
    e->have_gyro = 1u;

    for (int i = 0; i < 3; i++) {
        e->gyro_rad_s[i] = gyro_lsb[i] * EK_GYRO_LSB_RAD;
        om[i] = e->gyro_rad_s[i] - e->bg[i];
        sat = fmaxf(sat, fabsf(gyro_lsb[i]));
    }
    e->omega_rad_s = ek_norm3(om);
    e->saturated = (sat > EK_GYRO_SAT_LSB) ? 1u : 0u;

    if (e->saturated) {
        e->sat_s += dt;
        e->dt_sat += dt;
    } else {
        e->sat_s = fmaxf(0.0f, e->sat_s - dt);
    }

    {
        float d[3] = { om[0]*dt, om[1]*dt, om[2]*dt };
        ek_q_from_delta(d, qd);
        ek_q_mul(e->q, qd, qn);
        memcpy(e->q, qn, sizeof(qn));
        ek_q_normalize(e->q);
    }

    {
        float S[3][3], F[6][6], T[6][6], FT[6][6], PN[6][6];
        float qa = EK_GYRO_SIGMA * EK_GYRO_SIGMA * dt;
        float qb = EK_BIAS_RW * EK_BIAS_RW * dt;

        ek_skew(om, S);
        ek_mat6_eye(F);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) F[i][j] = ((i == j) ? 1.0f : 0.0f) - S[i][j] * dt;
            F[i][3 + i] = -dt;
        }
        ek_mat6_mul(F, e->P, T);
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++) FT[i][j] = F[j][i];
        ek_mat6_mul(T, FT, PN);

        if (e->saturated) {
            for (int i = 0; i < 3; i++) PN[i][i] += 0.08f;
            qa *= 20.0f;
        }
        for (int i = 0; i < 6; i++) PN[i][i] += (i < 3) ? qa : qb;
        memcpy(e->P, PN, sizeof(PN));
        ek_mat6_sym(e->P);
        ek_mat6_clamp(e->P);
    }

    /* A still board gives a direct bias observation.  This is deliberately
     * slow: the EKF remains the primary estimator, and this only prevents a
     * small residual from becoming a permanent yaw ramp on a resting hand. */
    omega_norm = ek_norm3(e->gyro_rad_s);
    if (!e->saturated && (omega_norm < 0.030f) &&
        ((e->acc_g > 0.0f) ? (fabsf(e->acc_g - 1.0f) < 0.08f) : e->have_gravity)) {
        e->static_s += dt;
    } else {
        e->static_s = fmaxf(0.0f, e->static_s - dt);
    }
    e->motion_s += dt;
    e->stationary = (e->static_s > 0.5f) ? 1u : 0u;

    if (e->stationary) {
        float k = 0.0025f * (dt * 100.0f);
        k = ek_clampf(k, 0.0f, 0.08f);
        for (int i = 0; i < 3; i++) {
            e->bg[i] += k * (e->gyro_rad_s[i] - e->bg[i]);
            e->bg[i] = ek_clampf(e->bg[i], -0.030f, 0.030f);
        }
    }
    e->samples++;
}

void mag_ekf_update_accel(mag_ekf_t *e, const float acc_lsb[3], uint64_t ts_ticks)
{
    float n, ag, z[3], h[3], d, quality;
    if (!e || !acc_lsb || !e->q_valid) return;

    n = ek_norm3(acc_lsb);
    if (n < 1.0f) return;
    ag = n * EK_ACC_LSB_G;
    z[0] = acc_lsb[0] / n;
    z[1] = acc_lsb[1] / n;
    z[2] = acc_lsb[2] / n;

    {
        float up[3] = { 0.0f, 0.0f, 1.0f };
        ek_q_rotate_inv(e->q, up, h);
    }
    d = ek_dot3(z, h);
    if (d < -0.25f) {
        e->acc_sign = (uint8_t)(e->acc_sign ? 0u : 1u);
        z[0] = -z[0]; z[1] = -z[1]; z[2] = -z[2];
        d = -d;
    }
    if (d < 0.25f) return;

    e->acc_g = ag;
    e->g_b[0] = z[0]; e->g_b[1] = z[1]; e->g_b[2] = z[2];
    e->have_acc = 1u;
    e->have_gravity = 1u;
    e->last_acc_ts = ts_ticks;
    e->last_gravity_ts = ts_ticks;

    quality = 1.0f;
    if (fabsf(ag - 1.0f) > 0.12f)
        quality = ek_clampf(1.0f - (fabsf(ag - 1.0f) - 0.12f) * 4.0f, 0.02f, 1.0f);
    (void)ek_measurement3(e, z, h, EK_ACC_SIGMA, quality,
                          &e->gravity_nis, &e->gravity_innov_rad);
}

void mag_ekf_update_gravity_quat(mag_ekf_t *e, const float q_meas[4], uint64_t ts_ticks)
{
    float qm[4], z[3], h[3], up[3] = { 0.0f, 0.0f, 1.0f };
    if (!e || !q_meas || !e->q_valid) return;
    memcpy(qm, q_meas, sizeof(qm));
    ek_q_normalize(qm);
    ek_q_rotate_inv(qm, up, z);
    ek_q_rotate_inv(e->q, up, h);

    e->g_b[0] = z[0]; e->g_b[1] = z[1]; e->g_b[2] = z[2];
    e->have_gravity = 1u;
    e->last_gravity_ts = ts_ticks;
    e->q_gamerv_error_rad = ek_q_angle(qm, e->q);
    (void)ek_measurement3(e, z, h, 0.025f, 0.85f,
                          &e->gravity_nis, &e->gravity_innov_rad);
}

void mag_ekf_update_mag(mag_ekf_t *e, const float mag[3], uint64_t ts_ticks, float quality)
{
    float n, z[3], h[3];
    if (!e || !mag || !e->q_valid) return;
    n = ek_norm3(mag);
    if (n < 1.0e-6f) return;
    z[0] = mag[0] / n; z[1] = mag[1] / n; z[2] = mag[2] / n;

    e->mag_norm = n;
    e->m_b[0] = z[0]; e->m_b[1] = z[1]; e->m_b[2] = z[2];
    e->have_mag = 1u;
    e->last_mag_ts = ts_ticks;

    if (!e->bw_valid) {
        if (quality < 0.50f) return;
        ek_q_rotate(e->q, z, e->Bw);
        ek_norm3v(e->Bw, e->Bw);
        e->bw_valid = 1u;
        e->mag_rebased = 1u;
        return;
    }

    if (quality <= 0.01f) return;
    ek_q_rotate_inv(e->q, e->Bw, h);
    if (ek_measurement3(e, z, h, EK_MAG_SIGMA, quality,
                        &e->mag_nis, &e->mag_innov_rad))
        e->mag_updates++;
}

void mag_ekf_rebase_mag(mag_ekf_t *e, const float mag[3])
{
    float n, z[3];
    if (!e || !mag || !e->q_valid) return;
    n = ek_norm3(mag);
    if (n < 1.0e-6f) return;
    z[0] = mag[0] / n; z[1] = mag[1] / n; z[2] = mag[2] / n;
    ek_q_rotate(e->q, z, e->Bw);
    ek_norm3v(e->Bw, e->Bw);
    e->bw_valid = 1u;
    e->mag_rebased = 1u;
}

/* Explicit reset helper retained for a user-commanded attitude reset.  The
 * regular runtime path never calls this during ordinary motion. */
void mag_ekf_force_static_reinit(mag_ekf_t *e)
{
    float up[3], north[3], east[3];
    float dot, R[9];
    float q[4];
    if (!e || !e->q_valid || !e->have_gravity || !e->have_mag) return;

    up[0] = e->g_b[0]; up[1] = e->g_b[1]; up[2] = e->g_b[2];
    ek_norm3v(up, up);
    dot = ek_dot3(e->m_b, up);
    north[0] = e->m_b[0] - dot * up[0];
    north[1] = e->m_b[1] - dot * up[1];
    north[2] = e->m_b[2] - dot * up[2];
    ek_norm3v(north, north);
    if (ek_norm3(north) < 0.20f) return;
    east[0] = up[1]*north[2] - up[2]*north[1];
    east[1] = up[2]*north[0] - up[0]*north[2];
    east[2] = up[0]*north[1] - up[1]*north[0];
    ek_norm3v(east, east);

    /* Columns are the world-axis directions expressed in body coordinates. */
    R[0] = north[0]; R[3] = east[0]; R[6] = up[0];
    R[1] = north[1]; R[4] = east[1]; R[7] = up[1];
    R[2] = north[2]; R[5] = east[2]; R[8] = up[2];

    {
        float tr = R[0] + R[4] + R[8];
        if (tr > 0.0f) {
            float s = sqrtf(tr + 1.0f) * 2.0f;
            q[0] = 0.25f * s;
            q[1] = (R[7] - R[5]) / s;
            q[2] = (R[2] - R[6]) / s;
            q[3] = (R[3] - R[1]) / s;
        } else if ((R[0] > R[4]) && (R[0] > R[8])) {
            float s = sqrtf(1.0f + R[0] - R[4] - R[8]) * 2.0f;
            q[0] = (R[7] - R[5]) / s;
            q[1] = 0.25f * s;
            q[2] = (R[1] + R[3]) / s;
            q[3] = (R[2] + R[6]) / s;
        } else if (R[4] > R[8]) {
            float s = sqrtf(1.0f + R[4] - R[0] - R[8]) * 2.0f;
            q[0] = (R[2] - R[6]) / s;
            q[1] = (R[1] + R[3]) / s;
            q[2] = 0.25f * s;
            q[3] = (R[5] + R[7]) / s;
        } else {
            float s = sqrtf(1.0f + R[8] - R[0] - R[4]) * 2.0f;
            q[0] = (R[3] - R[1]) / s;
            q[1] = (R[2] + R[6]) / s;
            q[2] = (R[5] + R[7]) / s;
            q[3] = 0.25f * s;
        }
        ek_q_normalize(q);
        memcpy(e->q, q, sizeof(q));
    }
    mag_ekf_rebase_mag(e, e->m_b);
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            e->P[i][j] = (i == j) ? ((i < 3) ? 0.05f : 0.001f) : 0.0f;
}

void mag_ekf_get_quat(const mag_ekf_t *e, float q[4])
{
    if (!e || !q) return;
    memcpy(q, e->q, sizeof(e->q));
}

float mag_ekf_yaw_deg(const mag_ekf_t *e)
{
    if (!e || !e->q_valid) return 0.0f;
    return atan2f(2.0f * (e->q[0]*e->q[3] + e->q[1]*e->q[2]),
                  1.0f - 2.0f * (e->q[2]*e->q[2] + e->q[3]*e->q[3])) * EK_RAD2DEG;
}

float mag_ekf_q_error_deg(const float qa[4], const float qb[4])
{
    if (!qa || !qb) return 0.0f;
    return ek_q_angle(qa, qb) * EK_RAD2DEG;
}









