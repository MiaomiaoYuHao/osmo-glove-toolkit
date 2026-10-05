/*
 * Drives the fusion with a PERFECT magnetometer - constant earth field, board
 * never moves - while the reported 6-axis quaternion drifts in yaw, exactly as
 * a GAMERV does.  The corrected output heading must stay put.
 *
 *   gcc -std=c99 -I Core/Inc/glove -o td test_drift.c -lm && ./td
 *
 * Point MF_SRC at the old file to see the failure:
 *   gcc -std=c99 -DMF_SRC='"old/mag_fusion.c"' -I old -o td0 test_drift.c -lm
 */
#ifndef MF_SRC
#define MF_SRC "Core/Src/glove/mag_fusion.c"
#endif
#include MF_SRC
#include <stdio.h>

#define DIP     (-0.96f)          /* about -55 deg */
#define DT      0.01f
#define D2R     (MF_PI / 180.0f)
#define R2D     (180.0f / MF_PI)

typedef struct { mag_fusion_t f; uint64_t tick; float yaw, dip, out; } sim_t;

static void prime(sim_t *s)
{
    memset(s, 0, sizeof(*s));
    mag_fusion_init(&s->f);
    mf_m3_identity(s->f.cal[0].W);
    s->f.cal[0].radius   = 1.0f;
    s->f.cal[0].valid    = 1u;
    s->f.cal[0].active   = 0u;         /* a loaded model: never refits */
    s->f.cal[0].revision = 1u;
    s->f.fe.align_valid  = 1u;
    s->tick = 1000;
    s->dip  = DIP;
}

/* yaw_rate: drift of the REPORTED attitude.  dip_rate: the field really tilting. */
static void run(sim_t *s, float secs, float yaw_rate, float dip_rate)
{
    float t;
    for (t = 0.0f; t < secs; t += DT)
    {
        float m[3], o[3], q[4];
        s->dip += dip_rate * DT;
        s->yaw += yaw_rate * DT;

        m[0] = cosf(s->dip); m[1] = 0.0f; m[2] = sinf(s->dip);
        mag_fusion_on_mag(&s->f, m, o);

        q[0] = cosf(0.5f * s->yaw); q[1] = 0.0f; q[2] = 0.0f;
        q[3] = sinf(0.5f * s->yaw);
        s->tick += (uint64_t)(DT / MF_TICK_S);
        mag_fusion_on_quat(&s->f, s->tick, q);

        /* heading of the CORRECTED output */
        s->out = atan2f(2.0f * (q[0]*q[3] + q[1]*q[2]),
                        1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]));
    }
}

int main(void)
{
    sim_t s;
    int fail = 0;
    float d0;

#define CHECK(c, m) do { printf("  %-50s %s\n", (m), (c) ? "ok" : "FAIL"); \
                         if (!(c)) fail = 1; } while (0)

    printf("\nA. 6-axis drifts 10 deg/s; magnetometer is perfect\n");
    prime(&s);
    run(&s, 6.0f, 0.0f, 0.0f);
    d0 = s.f.mag_datum_offset;
    run(&s, 40.0f, 10.0f * D2R, 0.0f);
    printf("     6-axis drifted %.0f deg | output heading %.1f deg\n",
           s.yaw * R2D, s.out * R2D);
    printf("     datum moved %.1f deg | yaw_corr %.1f deg\n",
           mf_wrap_pi(s.f.mag_datum_offset - d0) * R2D, s.f.yaw_corr * R2D);
    CHECK(fabsf(mf_wrap_pi(s.f.mag_datum_offset - d0)) < 5.0f * D2R,
          "datum did NOT chase the gyro");
    CHECK(fabsf(mf_wrap_pi(s.out)) < 15.0f * D2R,
          "output heading held despite the drift");

    printf("\nB. violent slew, then settle (the reported failure)\n");
    prime(&s);
    run(&s, 6.0f, 0.0f, 0.0f);
    run(&s, 3.0f, 6.0f, 0.0f);              /* about 344 deg/s */
    run(&s, 25.0f, 0.0f, 0.0f);             /* stop dead */
    printf("     6-axis left %.0f deg off | output heading %.1f deg\n",
           s.yaw * R2D, s.out * R2D);
    CHECK(fabsf(mf_wrap_pi(s.out)) < 15.0f * D2R, "output recovered after the slew");
    CHECK(s.f.stuck_s < 10.0f, "never stuck, so no 90 s datum reset ahead");

    printf("\nC. a REAL disturbance is still rejected\n");
    prime(&s);
    run(&s, 6.0f, 0.0f, 0.0f);
    d0 = s.f.mag_datum_offset;
    run(&s, 10.0f, 0.0f, 6.0f * D2R);       /* the field itself tilts */
    printf("     dip moved to %.0f deg | e_dip %.2f e_drift %.2f weight %.2f\n",
           s.dip * R2D, s.f.e_dip, s.f.e_drift, s.f.weight);
    CHECK((s.f.e_dip > 1.0f) || (s.f.e_drift > 1.0f), "the disturbance was caught");
    CHECK(fabsf(mf_wrap_pi(s.f.mag_datum_offset - d0)) < 5.0f * D2R,
          "a disturbed field is not a new north");

    printf("\nD. gate 4: azimuth is the signal, elevation is the alarm\n");
    prime(&s);
    run(&s, 6.0f, 0.0f, 0.0f);
    run(&s, 20.0f, 20.0f * D2R, 0.0f);      /* fast pure yaw drift */
    printf("     pure yaw drift: e_drift %.2f\n", s.f.e_drift);
    CHECK(s.f.e_drift <= 1.0f, "quiet on pure azimuth drift");
    prime(&s);
    run(&s, 6.0f, 0.0f, 0.0f);
    run(&s, 6.0f, 0.0f, 12.0f * D2R);       /* elevation really moves */
    printf("     dip moving:     e_drift %.2f\n", s.f.e_drift);
    CHECK(s.f.e_drift > 1.0f, "fires when the elevation moves");

    printf("\nE. a transient hard event must not move the datum\n");
    prime(&s);
    run(&s, 6.0f, 0.0f, 0.0f);
    d0 = s.f.mag_datum_offset;
    /* Slam the field sideways to throw a hard event, hold it, then put it
     * back exactly where it was - a magnet waved past, or e_step/e_grad
     * spiking during a violent slew.  Meanwhile the 6-axis drifts. */
    run(&s, 2.0f, 8.0f * D2R, 25.0f * D2R);
    run(&s, 2.0f, 8.0f * D2R, -25.0f * D2R);      /* field returns */
    run(&s, 20.0f, 8.0f * D2R, 0.0f);
    printf("     dip back to %.0f deg | datum moved %.1f deg | out %.1f deg\n",
           s.dip * R2D, mf_wrap_pi(s.f.mag_datum_offset - d0) * R2D, s.out * R2D);
    CHECK(fabsf(mf_wrap_pi(s.f.mag_datum_offset - d0)) < 5.0f * D2R,
          "field returned, so the datum stayed put");
    CHECK(fabsf(mf_wrap_pi(s.out)) < 20.0f * D2R,
          "output heading still correct afterwards");

    printf("\n%s\n\n", fail ? "FAILURES" : "all checks passed");
    return fail;
}
