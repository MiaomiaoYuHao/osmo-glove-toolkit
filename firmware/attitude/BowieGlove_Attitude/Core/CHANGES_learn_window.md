# Magnetometer fixes: learning window, heading convergence, gates

Four files changed against the uploaded tree; everything else is untouched.

    Inc/glove/mag_fusion.h
    Inc/glove/bhi360.h
    Src/glove/mag_fusion.c
    Src/glove/bhi360.c

## What was wrong

`parse_magnetometer()` publishes `cal`, not `raw`:

    m_out = A * W * (raw - offset)

`offset`, `W` and `A` were all being rewritten at runtime, so the reported
field moved at a fixed mechanical pose while the sensor read the same number.
Both estimators that move them are driven by attitude change - the ellipsoid
bins and the field estimator's 3 deg decimation gate - so the model advanced
during vigorous motion and froze at rest. At the output that is
indistinguishable from gyro drift.

`MAG_CAL_REFINE` (mf_cal_feed -> mf_cal_solve) was the path left running by
every earlier `nolearn` experiment. It replaces `offset` and `W` outright,
with no bound on how far the centre may move, guarded only by a relative
residual scored on the current window's own data.

## What changed

A four-state window, LOCKED by default:

    BOOTSTRAP  no committed model - wide open, or it could never bootstrap
    LOCKED     normal operation - nothing in the measurement path moves
    ARMED      a physical inconsistency is present, waiting on persistence
    OPEN       bounded window, at most one adopted update

`mf_learn_open()` gates six sites: the `mf_cal_feed` call, the `f->A`
adoption, the `fe->d_run` fold, `mf_local_d_update`, `mf_map_update` and
`mf_ref_update`.

Arming looks only at PHYSICAL error - `e_norm`, `e_dip`, `e_grad`, `e_fleet`.
`e_step` and `e_drift` are heading-domain and must never open a window.

A window opens only after the inconsistency has PERSISTED for
`MAG_RELEARN_CONFIRM_S`. A transient disturbance is a property of where the
board is and clears when you carry it away; a wrong model follows the board.

`MAG_RELEARN_COOLDOWN_S` prevents a relearn loop when a window closes with
the anomaly still present - that case means relearning is not the answer.

Relearn refits need `MAG_RELEARN_BINS` (14 of 24) coverage rather than the
bootstrap `MAG_CAL_BINS_SPH` (6 of 24).

## Tuning (Inc/glove/mag_fusion.h)

    MAG_RELEARN_ENTER       2.00f   arm above this
    MAG_RELEARN_EXIT        1.00f   below this the anomaly is over
    MAG_RELEARN_CONFIRM_S   8.0f    persistence needed to open
    MAG_RELEARN_WINDOW_S    30.0f   hard bound on an open window
    MAG_RELEARN_COOLDOWN_S  120.0f  minimum LOCKED time between windows
    MAG_RELEARN_BINS        14u     coverage required to replace a model

Net effect: the measurement path can change at most once per ~2.5 minutes,
and only while a physical inconsistency has persisted for 8 s.

## New API

    void     mag_fusion_relearn(mag_fusion_t *f);   /* host forces a window */
    void     mag_fusion_lock(mag_fusion_t *f);      /* shut it              */
    uint8_t  mag_fusion_learn_state(const mag_fusion_t *f);
    uint16_t mag_fusion_learn_opens(const mag_fusion_t *f);

Call `mag_fusion_relearn()` when the board is KNOWN to have changed - a magnet
was brought near it, hardware was swapped - not as a periodic tidy-up.

## Diagnostics

In Inc/glove/bhi360.h:

    #define MAG_DIAG_PRINT 1        /* default 0 */
    #define MAG_DIAG_EVERY 100u     /* ~1 Hz at a 100 Hz mag rate */

One line per link:

    MAG f0 l0 RAW -1250 -288 714 | CAL -31.23 -7.20 17.86
              | OFF ... | REV 3 FIX 1 OPENS 0 LRN 1

    REV    cal[0].revision - ellipsoid refits committed
    FIX    fe.fixes        - hard-iron offsets folded in
    OPENS  learning windows opened since boot
    LRN    0 bootstrap  1 locked  2 armed  3 open

Park at a repeatable pose, note RAW and CAL, move the board, return. RAW same
and CAL moved means the model moved, and REV/FIX say which part. After this
change both counters should stay still through ordinary motion.

## Tests

`test_learn.c` (shipped separately) drives the state machine on the host:

    gcc -std=c99 -I Core/Inc/glove -o tl test_learn.c -lm && ./tl


---

# Part 2 - the yaw drift

Locking the model was not enough: the large yaw offsets came from three
further bugs in the heading path, all reproducible on the bench with a
PERFECT magnetometer (constant field, board never moves) and a 6-axis
quaternion that drifts in yaw, which is what a GAMERV does. See
`test_drift.c`.

## 1. Gyro drift was being rebased into the magnetic datum

`mag_fusion_on_quat()`, the yaw innovation gate. When the mag and the 6-axis
disagreed by more than `MAG_YAW_INNOV_MAX` (30 deg) and every magnetic health
gate was clean, rev 1 concluded "a persistent, well-shaped heading offset is a
new magnetic datum" and after `MAG_HEAD_REBASE_S` (3 s) rebased
`mag_datum_offset`.

That inference is backwards. Clean gates plus a large steady heading
disagreement is GYRO YAW DRIFT - the error the module exists to remove.
Rebasing redefines north so the disagreement vanishes, which makes the gyro
error permanent.

Measured on the baseline, 10 deg/s of pure 6-axis drift:

    i= 809  6-axis yaw=  30.9 deg  >>> datum jumped   0.0 -> -30.4 deg
    i=1117  6-axis yaw=  61.7 deg  >>> datum jumped -30.4 -> -60.8 deg
    ... 17 rebases ...
    final: 550 deg of gyro drift absorbed into the datum, yaw_corr -6.7 deg

`yaw_corr` never moved. Every degree of drift went into the datum. One rebase
per 3 s, ~30 deg each, is how a session of hard shaking reaches 165 deg.

Now: when the gates are clean, converge on the measurement at
`MAG_YAW_SLEW_MAX` (20 deg/s). The magnetometer is the absolute reference;
when it is healthy and disagrees, believe it. When the gates are NOT clean the
sample is rejected as before, and the datum still does not move - a disturbed
field is not a new north.

## 2. Gate 4 reported gyro drift as a magnetic disturbance

The world vector is built by rotating the body vector with the 6-axis
quaternion, so its AZIMUTH rotates at the gyro's yaw drift rate. Rev 1
compared the full 3-D direction against an anchor, so it fired on exactly the
thing being corrected: stop after a fast slew, the `MAG_DRIFT_OMEGA_K * omega`
padding vanishes with omega, the residual drift clears the 1.15 deg floor, the
weight collapses, and the correction that would have removed the drift is
rejected. The anchor was never refreshed while fired, so the angle grew every
window until the 90 s stuck-recal fired a datum reset.

Measured: 20 deg/s of pure yaw drift gave `e_drift = 3.42` on the baseline,
`0.00` now.

Gate 4 now watches the ELEVATION of the field, which is what stays invariant
under a yaw datum error. It keeps the gate's real value - a magnet creeping in
moves the dip well below the noise floor of gates 1 and 2 - and
`MAG_DRIFT_REANCHOR_S` bounds how long it can stay fired against one anchor.

## 3. The dip gate never armed

`mf_ref_update()`'s fallback accumulated 400 samples, computed the average,
stored it and reset the accumulator - without ever setting `ref_dip_valid`.
It looped for ever, so gate 2 stayed switched off unless the field estimator
happened to converge. On the bench `e_dip` read 0.00 through a 60 deg dip
excursion.

It now latches. The reference is provisional: `mf_fe_service()` overwrites it
outright the first time it solves, and that estimate is yaw-free and averaged
over every attitude actually visited.

## Results

`test_drift.c`, nine checks. Baseline fails five, including both cases that
match the reported symptom:

    A  6-axis drifts 400 deg   output heading   34.8 deg -> 0.3 deg
    B  violent slew, settle    output heading  -45.3 deg -> 0.0 deg, not stuck
    C  real disturbance        e_dip 0.00 (gate off) -> 2.54, weight 0
    D  pure azimuth drift      e_drift 3.42 -> 0.00

## New tuning

    MAG_YAW_SLEW_MAX        0.35f   rad/s, convergence on a large clean innovation
    MAG_DRIFT_REANCHOR_S    6.0f    bound on how long gate 4 may stay fired

`MAG_HEAD_REBASE_S` is now unused by this path and left only for the other
rebase sites.

## Tests

    gcc -std=c99 -I Core/Inc/glove -o td test_drift.c -lm && ./td
    gcc -std=c99 -I Core/Inc/glove -o tl test_learn.c  -lm && ./tl

---

# Part 3 - the remaining datum paths, diagnostics, commands

## 4. The candidate rebase had the same inverted inference

`mag_fusion_on_quat()`, hard-event recovery. When a hard event resolved with a
field that had been stable for `MAG_REBASE_S`, rev 1 rewrote
`mag_datum_offset` so the new heading measurement equalled the current
`yaw_corr` - absorbing whatever the 6-axis had drifted to meanwhile.

The candidate test only asks whether the field is stable WITH ITSELF. It never
asks whether it differs from the reference it is replacing. `e_step` and
`e_grad` spike during a violent slew, which throws a hard event; the field then
resolves back to exactly where it was; and rev 1 still moved the datum.

It now compares the candidate against `ref_norm0` / `ref_dip0` and only moves
the datum if the field really changed. A field that came back means the event
was transient.

The third datum site - the `cal_commit_pending` re-anchor - is legitimate and
unchanged. When A or the offset move, `alpha` steps, and re-anchoring keeps the
output continuous. With the learning window locked those commits are rare.

## 5. The diagnostic would have printed garbage

`MAG_DIAG_PRINT` used `%8.2f`. newlib-nano drops `%f` unless you link with
`-u _printf_float`, which this project does not - `glove_print_mag_status()`
says so in a comment and prints scaled integers throughout. The diagnostic now
does the same: CAL and OFF in 1/100 LSB, DATUM and YAWC in 1/100 degree.

## 6. Status dump

`glove_print_mag_status()` ('P') gains five columns:

    datum  yawc rev lrn opn

`datum` is the one to watch. **It must not move during ordinary use.** Every
degree it gains is a degree of gyro drift that has been made permanent. `yawc`
moving is the module working.

## 7. Commands

    'W'  glove_mag_relearn()  open a learning window on every link
    'X'  glove_mag_lock()     lock learning shut on every link

Use 'W' when the board is KNOWN to have changed - a magnet was brought near it,
hardware was swapped. Not as a periodic tidy-up.

## Test results

`test_drift.c`, eleven checks. Baseline fails seven, fixed passes all.

    A  6-axis drifts 400 deg    output heading   34.8 deg -> 0.3 deg
    B  violent slew, settle     output heading  -45.3 deg -> 0.0 deg, not stuck
    C  real disturbance         e_dip 0.00 (gate off) -> 2.54, weight 0
    D  pure azimuth drift       e_drift 3.42 -> 0.00
    E  transient hard event     datum moved -> datum held

## Bench procedure

1. Build with `MAG_DIAG_PRINT 1`. Park at a repeatable mechanical pose, note
   RAW and CAL. Shake hard. Return to the same pose.
2. RAW unchanged and CAL unchanged, `REV`/`FIX`/`OPENS` all still - the model
   held. (You have already confirmed RAW does not change.)
3. Press 'P'. `datum` must read the same as before the shake. If it moved,
   there is a fourth path: capture the whole status line at the moment it
   moves - `eN eDi eGr eStep eFl rej hev cand` are all in it.
4. `yawc` should have moved, and the output heading should have returned.
