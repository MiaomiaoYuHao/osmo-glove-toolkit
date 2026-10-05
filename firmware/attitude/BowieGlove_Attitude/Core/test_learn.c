/* Drives mf_learn_service() directly.  Includes the .c so the statics are
 * visible; this is a host-side test, not part of the firmware build. */
#ifndef MF_SRC
#define MF_SRC "Core/Src/glove/mag_fusion.c"
#endif
#include MF_SRC
#include <stdio.h>

static const char *name(uint8_t s)
{
    switch (s) {
    case MAG_LEARN_BOOTSTRAP: return "BOOTSTRAP";
    case MAG_LEARN_LOCKED:    return "LOCKED";
    case MAG_LEARN_ARMED:     return "ARMED";
    case MAG_LEARN_OPEN:      return "OPEN";
    }
    return "?";
}

/* Put f in the state a board is in after a good calibration. */
static void settle(mag_fusion_t *f)
{
    mag_fusion_init(f);
    f->cal[0].valid  = 1u;
    f->cal[0].active = 2u;
    f->e_norm = f->e_dip = f->e_grad = f->e_fleet = 0.0f;
    mf_learn_service(f, 0.01f);   /* BOOTSTRAP -> LOCKED */
}

/* Hold a physical inconsistency of p for n seconds. */
static void hold(mag_fusion_t *f, float p, float secs)
{
    float t;
    f->e_norm = p;
    for (t = 0.0f; t < secs; t += 0.01f) mf_learn_service(f, 0.01f);
}

int main(void)
{
    mag_fusion_t f;
    int fail = 0;

#define CHECK(cond, msg) do {                                   \
        printf("  %-46s %s\n", (msg), (cond) ? "ok" : "FAIL");  \
        if (!(cond)) fail = 1;                                  \
    } while (0)

    printf("\n1. first commit locks the model\n");
    settle(&f);
    CHECK(f.learn_state == MAG_LEARN_LOCKED, "state == LOCKED");
    CHECK(mf_learn_open(&f) == 0u, "learning closed");

    printf("\n2. transient disturbance must NOT open a window\n");
    settle(&f);
    hold(&f, 4.0f, 3.0f);                       /* big, but only 3 s */
    CHECK(f.learn_state == MAG_LEARN_ARMED, "armed while the magnet is near");
    CHECK(mf_learn_open(&f) == 0u, "still closed while armed");
    hold(&f, 0.2f, 1.0f);                       /* walk away */
    CHECK(f.learn_state == MAG_LEARN_LOCKED, "back to LOCKED, nothing learned");
    CHECK(f.learn_opens == 0u, "no window was ever opened");

    printf("\n3. heading-domain error must NOT open a window\n");
    settle(&f);
    f.e_step = 9.0f; f.e_drift = 9.0f;          /* huge, but not physical */
    hold(&f, 0.0f, 20.0f);
    CHECK(f.learn_state == MAG_LEARN_LOCKED, "e_step/e_drift are ignored");
    CHECK(f.learn_opens == 0u, "no window opened");

    printf("\n4. persistent inconsistency DOES open, after the confirm time\n");
    settle(&f);
    hold(&f, 4.0f, MAG_RELEARN_CONFIRM_S - 1.0f);
    CHECK(f.learn_state == MAG_LEARN_ARMED, "still only armed at T-1s");
    hold(&f, 4.0f, 1.5f);
    CHECK(f.learn_state == MAG_LEARN_OPEN, "opened after the confirm time");
    CHECK(mf_learn_open(&f) == 1u, "learning permitted");
    CHECK(f.learn_opens == 1u, "one window counted");

    printf("\n5. one adoption closes the window\n");
    f.learn_adopted = 1u;                       /* a model update landed */
    mf_learn_service(&f, 0.01f);
    CHECK(f.learn_state == MAG_LEARN_LOCKED, "closed after a single adoption");

    printf("\n6. an open window is time-bounded even with nothing adopted\n");
    settle(&f);
    hold(&f, 4.0f, MAG_RELEARN_CONFIRM_S + 0.5f);
    CHECK(f.learn_state == MAG_LEARN_OPEN, "open");
    hold(&f, 4.0f, MAG_RELEARN_WINDOW_S + 1.0f);
    CHECK(f.learn_state == MAG_LEARN_LOCKED, "closed on the window timeout");
    hold(&f, 4.0f, MAG_RELEARN_COOLDOWN_S - 5.0f);
    CHECK(f.learn_state == MAG_LEARN_LOCKED, "stays locked through the cooldown");
    CHECK(f.learn_opens == 1u, "no relearn loop: still one window");

    printf("\n6b. cooldown expires and a persistent fault may retry\n");
    hold(&f, 4.0f, 6.0f + MAG_RELEARN_CONFIRM_S);
    CHECK(f.learn_state == MAG_LEARN_OPEN, "retries once the cooldown lapses");
    CHECK(f.learn_opens == 2u, "second window counted");

    printf("\n7. host can force a window, and lock it shut\n");
    settle(&f);
    mag_fusion_relearn(&f);
    CHECK(mf_learn_open(&f) == 1u, "relearn() opens it");
    mag_fusion_lock(&f);
    CHECK(mf_learn_open(&f) == 0u, "lock() shuts it");

    printf("\n8. an uncalibrated board still bootstraps\n");
    mag_fusion_init(&f);
    mf_learn_service(&f, 0.01f);
    CHECK(f.learn_state == MAG_LEARN_BOOTSTRAP, "BOOTSTRAP while uncalibrated");
    CHECK(mf_learn_open(&f) == 1u, "learning wide open");

    printf("\n9. an explicit host calibration reopens it\n");
    settle(&f);
    mag_fusion_cal_start(&f);
    mf_learn_service(&f, 0.01f);
    CHECK(mf_learn_open(&f) == 1u, "cal_start() reopens");

    printf("\n%s\n\n", fail ? "FAILURES" : "all checks passed");
    return fail;
}
