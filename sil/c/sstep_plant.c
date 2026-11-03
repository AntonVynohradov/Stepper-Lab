/*******************************************************************************
 * sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
 *
 * File:        sil/c/sstep_plant.c
 * Description: Plant model in portable C (see sstep_plant.h). State
 *              x = [iA iB w pe], pe = p theta, p = Ns / 4:
 *
 *                L diA/dt = vA - R iA + Kt w sin(pe)
 *                L diB/dt = vB - R iB - Kt w cos(pe)
 *                J dw/dt  = Kt (-iA sin(pe) + iB cos(pe)) - Td sin(4 pe) - TL
 *                           - B w - Tc tanh(w / w_c)
 *                dpe/dt   = p w
 *
 *              pe is kept in [-pi, pi) and the electrical turns are counted
 *              apart, so a single-precision build keeps its accuracy however far
 *              the rotor turns. Same equations as model/stepper_motor_f.sci.
 *
 * Author:      Anton Vynohradov
 * Email:       avynohradov@systemfromscratch.com
 * License:     Apache License 2.0
 * Copyright:   (c) 2026 Anton Vynohradov
 *
 * SPDX-License-Identifier: Apache-2.0
 ******************************************************************************/

#include "sstep_plant.h"

#include <math.h>
#include <stdlib.h>

typedef sstep_plant_real_t real;

#if SSTEP_PLANT_FLOAT
#define R(x) (x##f)
#define SIN sinf
#define COS cosf
#define TANH tanhf
#define FABS fabsf
#define FLOOR floorf
#define CEIL ceilf
#define FMAX fmaxf
#else
#define R(x) (x)
#define SIN sin
#define COS cos
#define TANH tanh
#define FABS fabs
#define FLOOR floor
#define CEIL ceil
#define FMAX fmax
#endif

#define PI_R R(3.14159265358979323846)
#define TWO_PI_R R(6.28318530717958647692)

/* ==============================================================================
 * MOTOR
 * =============================================================================*/

/* open: windings open (outputs off, SSTEP_PLANT_OFF_OPEN), the currents stay 0 */
static void deriv(const struct sstep_plant* p, const real* s, real va, real vb, real tl,
                  int open, real* d)
{
    const struct sstep_plant_motor* m = &p->motor;
    const real sn = SIN(s[3]);
    const real cs = COS(s[3]);
    real te = m->kt * (-s[0] * sn + s[1] * cs);
    real tf = m->b * s[2];

    if (m->td != R(0.0))
    {
        te -= m->td * SIN(R(4.0) * s[3]);
    }
    if (m->tc != R(0.0))
    {
        tf += m->tc * TANH(s[2] / m->w_c); /* dry friction smoothed around w = 0 */
    }
    if (open)
    {
        d[0] = R(0.0);
        d[1] = R(0.0);
    }
    else
    {
        d[0] = (va - m->r * s[0] + m->kt * s[2] * sn) / m->l;
        d[1] = (vb - m->r * s[1] - m->kt * s[2] * cs) / m->l;
    }
    if (p->stage.locked)
    {
        d[2] = R(0.0);
        d[3] = R(0.0);
    }
    else
    {
        d[2] = (te - tl - tf) / m->j;
        d[3] = (m->ns / R(4.0)) * s[2];
    }
}

static void wrap_angle(struct sstep_plant* p)
{
    if ((p->x[3] >= PI_R) || (p->x[3] < -PI_R))
    {
        const real turns = FLOOR((p->x[3] + PI_R) / TWO_PI_R);
        p->x[3] -= turns * TWO_PI_R;
        p->e_turns += (int32_t)turns;
    }
}

static void rk4(struct sstep_plant* p, real va, real vb, real tl, int open, real h)
{
    real k1[4], k2[4], k3[4], k4[4], y[4];
    real* x = p->x;
    int i;

    deriv(p, x, va, vb, tl, open, k1);
    for (i = 0; i < 4; i++) { y[i] = x[i] + h / R(2.0) * k1[i]; }
    deriv(p, y, va, vb, tl, open, k2);
    for (i = 0; i < 4; i++) { y[i] = x[i] + h / R(2.0) * k2[i]; }
    deriv(p, y, va, vb, tl, open, k3);
    for (i = 0; i < 4; i++) { y[i] = x[i] + h * k3[i]; }
    deriv(p, y, va, vb, tl, open, k4);
    for (i = 0; i < 4; i++)
    {
        x[i] = x[i] + h / R(6.0) * (k1[i] + R(2.0) * k2[i] + R(2.0) * k3[i] + k4[i]);
    }
    wrap_angle(p);
}

/* ==============================================================================
 * AVERAGED STAGE
 * =============================================================================*/

sstep_plant_real_t sstep_plant_leg_error(const struct sstep_plant_stage* stage, real i)
{
    const unsigned n = stage->leg_error_n;
    const real* ti = stage->leg_error_i;
    const real* tv = stage->leg_error_v;
    unsigned k;

    if (n == 0U)
    {
        return R(0.0);
    }
    if (i <= ti[0])
    {
        return tv[0];
    }
    for (k = 1U; k < n; k++)
    {
        if (i <= ti[k])
        {
            return tv[k - 1U] + (tv[k] - tv[k - 1U]) * (i - ti[k - 1U]) / (ti[k] - ti[k - 1U]);
        }
    }
    return tv[n - 1U];
}

static void period_avg(struct sstep_plant* p, real tl)
{
    const struct sstep_plant_stage* st = &p->stage;
    const int open = (!p->enabled) && (st->off == SSTEP_PLANT_OFF_OPEN);
    const int with_error = p->enabled && (st->leg_error_n > 0U);
    const real h = R(1.0) / st->fs / (real)st->nsub;
    unsigned k;

    if (open)
    {
        p->x[0] = R(0.0);
        p->x[1] = R(0.0);
    }
    for (k = 0U; k < st->nsub; k++)
    {
        real va = p->v[0];
        real vb = p->v[1];
        if (with_error)
        {
            const real ia = p->x[0];
            const real ib = p->x[1];
            if (st->topology == SSTEP_PLANT_SHARED_LEG)
            {
                const real ew = sstep_plant_leg_error(st, -(ia + ib));
                va += sstep_plant_leg_error(st, ia) - ew;
                vb += sstep_plant_leg_error(st, ib) - ew;
            }
            else
            {
                va += sstep_plant_leg_error(st, ia) - sstep_plant_leg_error(st, -ia);
                vb += sstep_plant_leg_error(st, ib) - sstep_plant_leg_error(st, -ib);
            }
        }
        rk4(p, va, vb, tl, open, h);
    }
    p->peak[0] = FABS(p->x[0]);
    p->peak[1] = FABS(p->x[1]);
}

/* ==============================================================================
 * SWITCHING STAGE
 * Same algorithm as model/bridge_switching.sci (stepper_period_sw) and
 * tools/ref_switching.py. The period starts at the counter peak (all low sides
 * on, the sampling instant); high side on in [Ts/2 (1 - d), Ts/2 (1 + d)], every
 * turn-on delayed by the dead time.
 * =============================================================================*/

#if SSTEP_PLANT_SWITCHING

#define LEG_LOW 0
#define LEG_HIGH 1
#define LEG_DEAD 2

struct leg_ev
{
    int n;
    real t[5];
    int s[5];
};

static void leg_events(real d, real ts, real td, struct leg_ev* ev)
{
    ev->n = 0;
    if (d >= R(1.0))
    {
        ev->t[0] = R(0.0);
        ev->s[0] = LEG_HIGH;
        ev->n = 1;
    }
    else if (d <= R(0.0))
    {
        ev->t[0] = R(0.0);
        ev->s[0] = LEG_LOW;
        ev->n = 1;
    }
    else
    {
        const real t1 = ts / R(2.0) * (R(1.0) - d);
        const real t2 = ts / R(2.0) * (R(1.0) + d);
        ev->t[0] = R(0.0);
        ev->s[0] = LEG_LOW;
        ev->t[1] = t1;
        ev->s[1] = LEG_DEAD;
        ev->n = 2;
        if (t1 + td < t2)
        {
            ev->t[ev->n] = t1 + td;
            ev->s[ev->n] = LEG_HIGH;
            ev->n++;
        }
        ev->t[ev->n] = t2;
        ev->s[ev->n] = LEG_DEAD;
        ev->n++;
        if (t2 + td < ts)
        {
            ev->t[ev->n] = t2 + td;
            ev->s[ev->n] = LEG_LOW;
            ev->n++;
        }
    }
}

static int leg_state(const struct leg_ev* ev, real t)
{
    int s = ev->s[0];
    int k;
    for (k = 0; k < ev->n; k++)
    {
        if (ev->t[k] <= t)
        {
            s = ev->s[k];
        }
    }
    return s;
}

static real leg_voltage(int state, real i_leg, real vbus, real rds, real vd)
{
    real v;
    if (state == LEG_HIGH)
    {
        v = vbus - i_leg * rds;
    }
    else if (state == LEG_LOW)
    {
        v = -i_leg * rds;
    }
    else
    {
        v = (i_leg >= R(0.0)) ? -vd : (vbus + vd); /* dead time: a body diode conducts */
    }
    return v;
}

static int cmp_real(const void* a, const void* b)
{
    const real x = *(const real*)a;
    const real y = *(const real*)b;
    return (x < y) ? -1 : ((x > y) ? 1 : 0);
}

static void period_sw(struct sstep_plant* p, real tl)
{
    const struct sstep_plant_stage* st = &p->stage;
    const real ts = R(1.0) / st->fs;
    const real td = st->dead_ns * R(1e-9);
    const int shared = st->topology == SSTEP_PLANT_SHARED_LEG;
    const int nleg = shared ? 3 : 4;
    const real hmax = ts / (real)st->sw_nstep;
    struct leg_ev ev[SSTEP_PLANT_LEGS];
    real bps[2 + 5 * SSTEP_PLANT_LEGS];
    real* x = p->x;
    int nb = 0;
    int j;
    int k;

    bps[nb++] = R(0.0);
    bps[nb++] = ts;
    for (k = 0; k < nleg; k++)
    {
        if (p->enabled)
        {
            leg_events(p->duty[k], ts, td, &ev[k]);
        }
        else
        {
            ev[k].n = 1;
            ev[k].t[0] = R(0.0);
            ev[k].s[0] = LEG_DEAD;
        }
        for (j = 0; j < ev[k].n; j++)
        {
            if ((ev[k].t[j] > R(0.0)) && (ev[k].t[j] < ts))
            {
                bps[nb++] = ev[k].t[j];
            }
        }
    }
    qsort(bps, (size_t)nb, sizeof(real), cmp_real);

    p->peak[0] = FABS(x[0]);
    p->peak[1] = FABS(x[1]);
    for (j = 0; j + 1 < nb; j++)
    {
        const real ta = bps[j];
        const real tb = bps[j + 1];
        int s[SSTEP_PLANT_LEGS];
        int n;
        int i;
        real h;
        if (tb <= ta)
        {
            continue; /* equal breakpoints */
        }
        for (k = 0; k < nleg; k++)
        {
            s[k] = leg_state(&ev[k], ta);
        }
        n = (int)CEIL((tb - ta) / hmax - R(1e-9));
        h = (tb - ta) / (real)n;
        for (i = 0; i < n; i++)
        {
            real va;
            real vb;
            if (shared)
            {
                const real vw = leg_voltage(s[2], -(x[0] + x[1]), st->vbus, st->rds_on, st->vd);
                va = leg_voltage(s[0], x[0], st->vbus, st->rds_on, st->vd) - vw;
                vb = leg_voltage(s[1], x[1], st->vbus, st->rds_on, st->vd) - vw;
            }
            else
            {
                va = leg_voltage(s[0], x[0], st->vbus, st->rds_on, st->vd) -
                     leg_voltage(s[1], -x[0], st->vbus, st->rds_on, st->vd);
                vb = leg_voltage(s[2], x[1], st->vbus, st->rds_on, st->vd) -
                     leg_voltage(s[3], -x[1], st->vbus, st->rds_on, st->vd);
            }
            rk4(p, va, vb, tl, 0, h);
            p->peak[0] = FMAX(p->peak[0], FABS(x[0]));
            p->peak[1] = FMAX(p->peak[1], FABS(x[1]));
        }
    }
}

#endif /* SSTEP_PLANT_SWITCHING */

/* ==============================================================================
 * PUBLIC FUNCTIONS
 * =============================================================================*/

int sstep_plant_init(struct sstep_plant* p, const struct sstep_plant_motor* motor,
                     const struct sstep_plant_stage* stage)
{
    unsigned k;

    if ((motor->l <= R(0.0)) || (motor->j <= R(0.0)) || (motor->ns <= R(0.0)) ||
        (stage->fs <= R(0.0)) || (stage->period <= R(0.0)) || (stage->nsub == 0U) ||
        (stage->leg_error_n > SSTEP_PLANT_LEG_ERROR_MAX) ||
        ((motor->tc != R(0.0)) && (motor->w_c <= R(0.0))))
    {
        return -1;
    }
    for (k = 1U; k < stage->leg_error_n; k++)
    {
        if (stage->leg_error_i[k] <= stage->leg_error_i[k - 1U])
        {
            return -1; /* the table must rise */
        }
    }
#if SSTEP_PLANT_SWITCHING
    if ((stage->model == SSTEP_PLANT_STAGE_SWITCHING) && (stage->sw_nstep == 0U))
    {
        return -1;
    }
#else
    if (stage->model == SSTEP_PLANT_STAGE_SWITCHING)
    {
        return -1; /* built without the switching stage */
    }
#endif

    p->motor = *motor;
    p->stage = *stage;
    for (k = 0U; k < 4U; k++)
    {
        p->x[k] = R(0.0);
    }
    p->e_turns = 0;
    p->v[0] = R(0.0);
    p->v[1] = R(0.0);
    for (k = 0U; k < SSTEP_PLANT_LEGS; k++)
    {
        p->duty[k] = R(0.5);
    }
    p->enabled = 0;
    p->peak[0] = R(0.0);
    p->peak[1] = R(0.0);
    return 0;
}

void sstep_plant_set_compare(struct sstep_plant* p, const sstep_plant_real_t* compare,
                             int enabled)
{
    const struct sstep_plant_stage* st = &p->stage;
    real leg[SSTEP_PLANT_LEGS];
    unsigned k;

    for (k = 0U; k < SSTEP_PLANT_LEGS; k++)
    {
        p->duty[k] = compare[k] / st->period;
        leg[k] = p->duty[k] * st->vbus;
    }
    p->enabled = enabled ? 1 : 0;
    if (!p->enabled)
    {
        p->v[0] = R(0.0); /* outputs disabled */
        p->v[1] = R(0.0);
    }
    else if (st->topology == SSTEP_PLANT_SHARED_LEG)
    {
        p->v[0] = leg[0] - leg[2]; /* shared leg: A = U - W, B = V - W */
        p->v[1] = leg[1] - leg[2];
    }
    else
    {
        p->v[0] = leg[0] - leg[1]; /* two H-bridges */
        p->v[1] = leg[2] - leg[3];
    }
}

void sstep_plant_period(struct sstep_plant* p, sstep_plant_real_t load)
{
#if SSTEP_PLANT_SWITCHING
    if (p->stage.model == SSTEP_PLANT_STAGE_SWITCHING)
    {
        period_sw(p, load);
        return;
    }
#endif
    period_avg(p, load);
}

static real adc_clamp(real v, real hi)
{
    return (v < R(0.0)) ? R(0.0) : ((v > hi) ? hi : v);
}

/* round half away from zero, as lround() */
static real round_counts(real v)
{
    return (v >= R(0.0)) ? FLOOR(v + R(0.5)) : -FLOOR(-v + R(0.5));
}

void sstep_plant_adc_sample(const struct sstep_plant* p, const struct sstep_plant_adc* adc,
                            sstep_plant_real_t out[3])
{
    out[0] = adc_clamp(round_counts(adc->zero + p->x[0] / adc->i_full * adc->span), adc->full);
    out[1] = adc_clamp(round_counts(adc->zero + p->x[1] / adc->i_full * adc->span), adc->full);
    out[2] = adc_clamp(round_counts(p->stage.vbus / adc->v_full * adc->full), R(65535.0));
}

sstep_plant_real_t sstep_plant_theta(const struct sstep_plant* p)
{
    return ((real)p->e_turns * TWO_PI_R + p->x[3]) / (p->motor.ns / R(4.0));
}
