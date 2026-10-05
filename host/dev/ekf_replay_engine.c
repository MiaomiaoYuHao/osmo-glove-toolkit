/*
 * Offline replay for the six-state ESKF.  It runs the same mag_fusion
 * calibration/magnetic-quality path and the same mag_ekf.c as the firmware.
 * The recorded raw quaternion supplies the gravity observation when a legacy
 * trace has no RAW_ACC frames; real RAW_ACC frames take over when present.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mag_fusion.h"
#include "mag_ekf.h"

typedef struct __attribute__((packed)) replay_header {
    char magic[8];
    uint32_t version;
    uint32_t state_size;
    uint32_t link;
    uint32_t reserved;
} replay_header_t;

typedef struct __attribute__((packed)) replay_record {
    uint8_t kind;
    uint8_t pad[3];
    uint32_t index;
    uint64_t ts_ticks;
    float v[5];
} replay_record_t;

enum { REC_QUAT = 0, REC_MAG = 1, REC_GYRO = 2, REC_ACC = 3, REC_MAG2 = 4 };

static float yaw_deg(const float q[4])
{
    return atan2f(2.0f * (q[0]*q[3] + q[1]*q[2]),
                  1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3])) * 57.29577951308232f;
}

int main(int argc, char **argv)
{
    FILE *in, *out;
    replay_header_t header;
    replay_record_t rec;
    mag_fusion_t fusion;
    mag_ekf_t ekf;
    uint64_t seq = 0;
    uint8_t ready = 0;
    uint8_t blend = 0;

    if (argc != 3) {
        fprintf(stderr, "usage: %s replay_input.bin replay_ekf_output.csv\n", argv[0]);
        return 2;
    }
    in = fopen(argv[1], "rb");
    if (!in) { perror("open input"); return 3; }
    out = fopen(argv[2], "w");
    if (!out) { perror("open output"); fclose(in); return 3; }

    if (fread(&header, sizeof(header), 1, in) != 1 ||
        memcmp(header.magic, "MFREPLAY", 8) != 0 ||
        header.version != 1u || header.state_size != sizeof(mag_fusion_t)) {
        fprintf(stderr, "bad replay header\n");
        fclose(in); fclose(out); return 4;
    }
    if (fread(&fusion, 1, sizeof(fusion), in) != sizeof(fusion)) {
        fprintf(stderr, "truncated fusion state\n");
        fclose(in); fclose(out); return 4;
    }

    mag_ekf_init(&ekf);

    fprintf(out,
            "seq,index,kind,ts_ticks,ekf_qw,ekf_qx,ekf_qy,ekf_qz,"
            "ekf_yaw_deg,old_yaw_deg,bgx,bgy,bgz,acc_g,mag_norm,"
            "mag_nis,gravity_nis,mag_innov_rad,gravity_innov_rad,"
            "sat,static,stationary,mag_updates,bw_valid,qerr_deg,out_qw\n");

    while (fread(&rec, sizeof(rec), 1, in) == 1) {
        float mout[3] = {0.0f, 0.0f, 0.0f};
        float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        float qg[4];
        float qe[4];
        float old_yaw = 0.0f;
        float qerr = 0.0f;
        float outqw = 0.0f;

        if (rec.kind == REC_GYRO) {
            mag_fusion_on_gyro(&fusion, rec.v);
            mag_ekf_predict(&ekf, rec.v, rec.ts_ticks);
        } else if (rec.kind == REC_MAG) {
            uint8_t ok = mag_fusion_on_mag(&fusion, rec.v, mout);
            if (ok && fusion.cal[0].valid) {
                float quality = fusion.weight;
                if (fusion.hard_event || fusion.rejecting) quality = 0.0f;
#ifndef NO_MAG_UPDATE
                mag_ekf_update_mag(&ekf, mout, rec.ts_ticks, quality);
#else
                (void)quality;
#endif
            }
        } else if (rec.kind == REC_ACC) {
            mag_ekf_update_accel(&ekf, rec.v, rec.ts_ticks);
        } else if (rec.kind == REC_QUAT) {
            q[0] = rec.v[3]; q[1] = rec.v[0]; q[2] = rec.v[1]; q[3] = rec.v[2];
            memcpy(qg, q, sizeof(qg));
            (void)mag_fusion_on_quat(&fusion, rec.ts_ticks, q);
            old_yaw = yaw_deg(q);
            if (!ekf.q_valid) {
                mag_ekf_seed_quat(&ekf, q);
            } else {
                mag_ekf_update_gravity_quat(&ekf, qg, rec.ts_ticks);
            }
            mag_ekf_get_quat(&ekf, qe);
            qerr = mag_ekf_q_error_deg(q, qe);
            outqw = qe[0];

            /* Mirror the firmware's guarded, short output fade-in. */
            if (ekf.bw_valid && ekf.q_valid) {
                if (!ready && qerr < 8.0f) { ready = 1; blend = 0; }
                if (ready) {
                    if (blend < 100u) {
                        float a = (float)blend / 100.0f;
                        for (int i = 0; i < 4; i++)
                            qe[i] = (1.0f - a) * q[i] + a * qe[i];
                        {
                            float qn = sqrtf(qe[0]*qe[0] + qe[1]*qe[1] +
                                             qe[2]*qe[2] + qe[3]*qe[3]);
                            if (qn > 1.0e-12f) {
                                qe[0] /= qn; qe[1] /= qn;
                                qe[2] /= qn; qe[3] /= qn;
                            }
                        }
                        blend = (blend <= 95u) ? (uint8_t)(blend + 5u) : 100u;
                    }
                    outqw = qe[0];
                }
            }
        } else if (rec.kind == REC_MAG2) {
            (void)mag_fusion_on_mag2(&fusion, rec.v, NULL);
        } else {
            return 5;
        }

        mag_ekf_get_quat(&ekf, qe);
        fprintf(out,
                "%I64u,%u,%u,%I64u,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                "%u,%u,%u,%u,%u,%.9f,%.9f\n",
                (unsigned long long)seq++, (unsigned)rec.index, (unsigned)rec.kind,
                (unsigned long long)rec.ts_ticks,
                (double)qe[0], (double)qe[1], (double)qe[2], (double)qe[3],
                (double)yaw_deg(qe), (double)old_yaw,
                (double)ekf.bg[0], (double)ekf.bg[1], (double)ekf.bg[2],
                (double)ekf.acc_g, (double)ekf.mag_norm,
                (double)ekf.mag_nis, (double)ekf.gravity_nis,
                (double)ekf.mag_innov_rad, (double)ekf.gravity_innov_rad,
                (unsigned)ekf.saturated, (unsigned)(ekf.static_s > 0.5f),
                (unsigned)ekf.stationary, (unsigned)ekf.mag_updates,
                (unsigned)ekf.bw_valid, (double)qerr, (double)outqw);
    }

    fclose(in);
    fclose(out);
    return 0;
}


