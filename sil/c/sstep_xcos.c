/*******************************************************************************
 * sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
 *
 * File:        sil/c/sstep_xcos.c
 * Description: Xcos computational functions (type 4) for software-in-the-loop:
 *              - sstep_xcos_ctrl:    the Stepper-C library (fast and slow loop),
 *              - sstep_xcos_plant:   power stage (averaged or switching with dead
 *                                    time and diodes) + motor + ADC (RK4),
 *              - sstep_xcos_monitor: signals for scopes and the workspace,
 *              - sstep_xcos_check:   configuration check, called from Scilab.
 *              The motor and the power stage use the same equations as the
 *              Scilab prototype (model directory) and the Stepper-C unit tests.
 *
 * Author:      Anton Vynohradov
 * Email:       avynohradov@systemfromscratch.com
 * License:     Apache License 2.0
 * Copyright:   (c) 2026 Anton Vynohradov
 *
 * SPDX-License-Identifier: Apache-2.0
 ******************************************************************************/

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "scicos_block4.h"
#include "sstep.h"

/* Functions of the Scilab scicos library (scicos.h needs more Scilab headers) */
extern void Coserror(const char* fmt, ...);
extern double get_scicos_time(void);

/* ==============================================================================
 * PARAMETER LAYOUT (rpar), built by model/sil_ctrl_rpar.sci and sil_plant_rpar.sci
 * =============================================================================*/

/* controller */
enum
{
    C_R_MOHM = 0, C_L_UH, C_I_RATED, C_KE, C_NS,
    C_TOPO, C_VNOM, C_VMIN, C_VMAX, C_IFULL, C_ITRIP, C_PERIOD, C_PWMHZ, C_DEAD,
    C_ADC_ZERO, C_ADC_SPAN, C_ADC_VFULL, C_ADC_FULL,
    C_DRIVE, C_FAST, C_SLOW, C_MICRO, C_BW, C_IRUN, C_IHOLD, C_HOLDMS, C_VMAXS, C_ACC, C_DEC,
    C_NCMD,
    C_NPAR /* commands follow: t, kind, value */
};

/* plant */
enum
{
    P_R = 0, P_L, P_KT, P_NS, P_J, P_B, P_TD, P_TC, P_WC,
    P_TOPO, P_VBUS, P_PERIOD, P_IFULL, P_SPAN, P_ZERO, P_VFULL, P_ADC_FULL,
    P_FS, P_NSUB, P_LOCKED,
    P_STAGE, P_DEAD_NS, P_RDS, P_VD, P_SW_NSTEP, /* power stage model, bridge_switching.sci */
    P_NLOAD,
    P_NPAR /* load steps follow: t, TL */
};

#define CMD_MOVE 1
#define CMD_RUN 2
#define CMD_STOP 3

#define PI_D 3.14159265358979323846

/* ==============================================================================
 * CONTROLLER BLOCK
 * =============================================================================*/

struct sil_ctrl
{
    struct sstep drv;
    uint16_t compare[SSTEP_LEGS_MAX];
    int enabled;
    uint32_t tick;
    uint32_t ratio;
    double Ts;
    int ncmd;
    int ci;
    int64_t phi_turns; /* unwrapped electrical angle applied [2^-32 turn] */
    uint32_t phi_last;
    double i_full;
};

static int hal_pwm(void* ctx, const uint16_t cmp[SSTEP_LEGS_MAX])
{
    struct sil_ctrl* s = (struct sil_ctrl*)ctx;
    for (unsigned k = 0; k < SSTEP_LEGS_MAX; k++)
    {
        s->compare[k] = cmp[k];
    }
    return 0;
}

static int hal_enable(void* ctx, bool en)
{
    ((struct sil_ctrl*)ctx)->enabled = en ? 1 : 0;
    return 0;
}

static uint32_t u32(double v)
{
    return (v <= 0.0) ? 0U : (uint32_t)(v + 0.5);
}

static void make_config(const double* p, struct sstep_config* c, void* ctx)
{
    c->motor.r_mohm = u32(p[C_R_MOHM]);
    c->motor.l_uh = u32(p[C_L_UH]);
    c->motor.i_rated_ma = u32(p[C_I_RATED]);
    c->motor.ke_uv_s_per_rad = u32(p[C_KE]);
    c->motor.full_steps_per_rev = (uint16_t)u32(p[C_NS]);
    c->stage.topology = (p[C_TOPO] > 0.5) ? SSTEP_TOPOLOGY_SHARED_LEG : SSTEP_TOPOLOGY_FULL_BRIDGE;
    c->stage.vbus_nominal_mv = u32(p[C_VNOM]);
    c->stage.vbus_min_mv = u32(p[C_VMIN]);
    c->stage.vbus_max_mv = u32(p[C_VMAX]);
    c->stage.i_full_scale_ma = u32(p[C_IFULL]);
    c->stage.i_trip_ma = u32(p[C_ITRIP]);
    c->stage.pwm_period_ticks = (uint16_t)u32(p[C_PERIOD]);
    c->stage.pwm_freq_hz = u32(p[C_PWMHZ]);
    c->stage.dead_time_ns = u32(p[C_DEAD]);
    c->stage.adc_i_zero = (uint16_t)u32(p[C_ADC_ZERO]);
    c->stage.adc_i_span = (int32_t)lround(p[C_ADC_SPAN]);
    c->stage.adc_vbus_mv_full = u32(p[C_ADC_VFULL]);
    c->stage.adc_full = (uint16_t)u32(p[C_ADC_FULL]);
    c->ctrl.drive_mode = (p[C_DRIVE] > 0.5) ? SSTEP_DRIVE_VOLTAGE : SSTEP_DRIVE_CURRENT;
    c->ctrl.fast_loop_hz = u32(p[C_FAST]);
    c->ctrl.slow_loop_hz = u32(p[C_SLOW]);
    c->ctrl.microsteps = (uint16_t)u32(p[C_MICRO]);
    c->ctrl.current_bw_hz = u32(p[C_BW]);
    c->ctrl.i_run_ma = u32(p[C_IRUN]);
    c->ctrl.i_hold_ma = u32(p[C_IHOLD]);
    c->ctrl.hold_delay_ms = u32(p[C_HOLDMS]);
    c->ctrl.v_max = u32(p[C_VMAXS]);
    c->ctrl.accel = u32(p[C_ACC]);
    c->ctrl.decel = u32(p[C_DEC]);
    c->hal.context = ctx;
    c->hal.pwm_write = hal_pwm;
    c->hal.output_enable = hal_enable;
    c->hal.on_fault = NULL;
    c->hal.on_event = NULL;
}

static double real_to_d(sstep_real_t x)
{
#if (SSTEP_CFG_MATH_BACKEND == SSTEP_MATH_BACKEND_FIXED)
    return (double)x / 32768.0;
#else
    return (double)x;
#endif
}

static const char* status_text(int r)
{
    const char* t = "unknown error";
    switch (r)
    {
    case SSTEP_ERR_PARAM:
        t = "a value is out of range (SSTEP_ERR_PARAM)";
        break;
    case SSTEP_ERR_CONFIG:
        t = "inconsistent configuration (SSTEP_ERR_CONFIG)";
        break;
    case SSTEP_ERR_STATE:
        t = "command not allowed in this state (SSTEP_ERR_STATE)";
        break;
    case SSTEP_ERR_UNSUPPORTED:
        t = "topology not compiled in (SSTEP_ERR_UNSUPPORTED)";
        break;
    default:
        break;
    }
    return t;
}

static int apply_command(struct sil_ctrl* s, int kind, double value)
{
    int r = SSTEP_OK;
    if (kind == CMD_MOVE)
    {
        r = sstep_move_to(&s->drv, (int32_t)lround(value));
    }
    else if (kind == CMD_RUN)
    {
        r = sstep_run(&s->drv, (int32_t)lround(value));
    }
    else if (kind == CMD_STOP)
    {
        r = sstep_stop(&s->drv);
    }
    return r; /* other kinds (load torque) belong to the plant */
}

/* Inputs:  1 ADC sample [i_a i_b vbus] (counts)
 * Outputs: 1 [compare(4) enabled]
 *          2 [phi(rad, unwrapped) position(usteps) velocity(usteps/s) state faults
 *             iA_ref(A) iB_ref(A)]
 * Activation: one event input at the fast loop rate (fs). */
void sstep_xcos_ctrl(scicos_block* blk, scicos_flag flag)
{
    struct sil_ctrl* s = (struct sil_ctrl*)(*blk->work);
    const double* p = blk->rpar;

    if (flag == Initialization)
    {
        struct sstep_config cfg;
        int r;
        if ((blk->nrpar < C_NPAR) || (blk->nrpar != C_NPAR + 3 * (int)lround(p[C_NCMD])))
        {
            Coserror("Stepper-C block: wrong parameter vector (use sil_ctrl_rpar)");
            return;
        }
        s = (struct sil_ctrl*)calloc(1, sizeof(*s));
        if (s == NULL)
        {
            Coserror("Stepper-C block: out of memory");
            return;
        }
        *blk->work = s;
        make_config(p, &cfg, s);
        r = sstep_init(&s->drv, &cfg);
        if (r != SSTEP_OK)
        {
            Coserror("Stepper-C: sstep_init failed: %s", status_text(r));
            return;
        }
        s->ratio = cfg.ctrl.fast_loop_hz / cfg.ctrl.slow_loop_hz;
        s->Ts = 1.0 / (double)cfg.ctrl.fast_loop_hz;
        s->ncmd = (int)lround(p[C_NCMD]);
        s->i_full = p[C_IFULL] / 1000.0;
        (void)sstep_enable(&s->drv);
    }
    else if ((flag == OutputUpdate) && (blk->nevprt > 0) && (s != NULL))
    {
        const double* in = (const double*)blk->inptr[0];
        double* out = (double*)blk->outptr[0];
        double* mon = (double*)blk->outptr[1];
        struct sstep_sample smp;
        struct sstep_status st;
        uint32_t used;
        double ang;
        double amp;

        if ((s->tick % s->ratio) == 0U)
        {
            const double t = (double)s->tick * s->Ts;
            while ((s->ci < s->ncmd) && (p[C_NPAR + 3 * s->ci] <= t + 1e-12))
            {
                const int kind = (int)lround(p[C_NPAR + 3 * s->ci + 1]);
                const double val = p[C_NPAR + 3 * s->ci + 2];
                const int r = apply_command(s, kind, val);
                if (r != SSTEP_OK)
                {
                    Coserror("Stepper-C: command %d (kind %d, value %g) at t = %g s: %s",
                             s->ci + 1, kind, val, t, status_text(r));
                    return;
                }
                s->ci++;
            }
            (void)sstep_slow_loop(&s->drv);
        }

        smp.i_a = (uint16_t)lround(in[0]);
        smp.i_b = (uint16_t)lround(in[1]);
        smp.vbus = (uint16_t)lround(in[2]);
        (void)sstep_fast_loop(&s->drv, &smp);
        s->tick++;

        /* angle used in this period: the fast loop advances it after use */
        used = s->drv.cur.angle - (uint32_t)s->drv.cur.dangle;
        s->phi_turns += (int32_t)(used - s->phi_last);
        s->phi_last = used;

        for (unsigned k = 0; k < SSTEP_LEGS_MAX; k++)
        {
            out[k] = (double)s->compare[k];
        }
        out[4] = (double)s->enabled;

        (void)sstep_get_status(&s->drv, &st);
        ang = (double)used * (2.0 * PI_D / 4294967296.0);
        amp = real_to_d(s->drv.cur.i_amp) * s->i_full;
        mon[0] = (double)s->phi_turns * (2.0 * PI_D / 4294967296.0);
        mon[1] = (double)st.position;
        mon[2] = (double)st.velocity;
        mon[3] = (double)st.state;
        mon[4] = (double)st.faults;
        mon[5] = amp * cos(ang);
        mon[6] = amp * sin(ang);
    }
    else if (flag == Ending)
    {
        free(s);
        *blk->work = NULL;
    }
}

/* Called from Scilab with call(): err = sstep_init() result of the configuration. */
void sstep_xcos_check(double* rpar, int* n, int* err)
{
    struct sil_ctrl tmp;
    struct sstep_config cfg;
    if ((*n < C_NPAR) || (*n != C_NPAR + 3 * (int)lround(rpar[C_NCMD])))
    {
        *err = -100;
    }
    else
    {
        make_config(rpar, &cfg, &tmp);
        *err = sstep_init(&tmp.drv, &cfg);
    }
}

/* ==============================================================================
 * PLANT BLOCK: power stage, motor (RK4), ADC
 * =============================================================================*/

#define Z_DUTY 6   /* z[6..9]  leg duties of the next period */
#define Z_EN 10    /* z[10]    outputs enabled in the next period */
#define Z_PEAK 11  /* z[11..12] largest |iA|, |iB| in the last period */
#define Z_N 13

#define LEG_LOW 0
#define LEG_HIGH 1
#define LEG_DEAD 2

struct motor
{
    double R, L, Kt, p, J, B, Td, Tc, wc;
};

static void deriv(const struct motor* m, const double* x, double vA, double vB, double TL,
                  int locked, double* d)
{
    const double s = sin(m->p * x[3]);
    const double c = cos(m->p * x[3]);
    const double Te = m->Kt * (-x[0] * s + x[1] * c) - m->Td * sin(4.0 * m->p * x[3]);
    d[0] = (vA - m->R * x[0] + m->Kt * x[2] * s) / m->L;
    d[1] = (vB - m->R * x[1] - m->Kt * x[2] * c) / m->L;
    /* viscous and dry friction (dry part smoothed around w = 0, model/stepper_motor_f.sci) */
    const double Tf = m->B * x[2] + m->Tc * tanh(x[2] / m->wc);
    d[2] = locked ? 0.0 : (Te - TL - Tf) / m->J;
    d[3] = locked ? 0.0 : x[2];
}

static void rk4(const struct motor* m, double* x, double vA, double vB, double TL, int locked,
                double h)
{
    double k1[4], k2[4], k3[4], k4[4], y[4];
    deriv(m, x, vA, vB, TL, locked, k1);
    for (int i = 0; i < 4; i++) { y[i] = x[i] + h / 2 * k1[i]; }
    deriv(m, y, vA, vB, TL, locked, k2);
    for (int i = 0; i < 4; i++) { y[i] = x[i] + h / 2 * k2[i]; }
    deriv(m, y, vA, vB, TL, locked, k3);
    for (int i = 0; i < 4; i++) { y[i] = x[i] + h * k3[i]; }
    deriv(m, y, vA, vB, TL, locked, k4);
    for (int i = 0; i < 4; i++)
    {
        x[i] = x[i] + h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
}

/* Switching stage, same algorithm as model/bridge_switching.sci (stepper_period_sw) and
 * tools/ref_switching.py. The period starts at the counter peak (all low sides on, the
 * sampling instant); high side on in [Ts/2 (1 - d), Ts/2 (1 + d)], every turn-on delayed
 * by the dead time. */
struct leg_ev
{
    int n;
    double t[5];
    int s[5];
};

static void leg_events(double d, double Ts, double td, struct leg_ev* ev)
{
    ev->n = 0;
    if (d >= 1.0)
    {
        ev->t[0] = 0.0;
        ev->s[0] = LEG_HIGH;
        ev->n = 1;
    }
    else if (d <= 0.0)
    {
        ev->t[0] = 0.0;
        ev->s[0] = LEG_LOW;
        ev->n = 1;
    }
    else
    {
        const double t1 = Ts / 2 * (1.0 - d);
        const double t2 = Ts / 2 * (1.0 + d);
        ev->t[0] = 0.0;
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
        if (t2 + td < Ts)
        {
            ev->t[ev->n] = t2 + td;
            ev->s[ev->n] = LEG_LOW;
            ev->n++;
        }
    }
}

static int leg_state(const struct leg_ev* ev, double t)
{
    int s = ev->s[0];
    for (int k = 0; k < ev->n; k++)
    {
        if (ev->t[k] <= t)
        {
            s = ev->s[k];
        }
    }
    return s;
}

static double leg_voltage(int state, double i_leg, double vbus, double rds, double vd)
{
    double v;
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
        v = (i_leg >= 0.0) ? -vd : (vbus + vd); /* dead time: a body diode conducts */
    }
    return v;
}

static int cmp_double(const void* a, const void* b)
{
    const double x = *(const double*)a;
    const double y = *(const double*)b;
    return (x < y) ? -1 : ((x > y) ? 1 : 0);
}

/* one PWM period; peak[2] = largest |iA|, |iB| */
static void period_sw(const struct motor* m, const double* p, double* x, const double* duty,
                      int enabled, double TL, int locked, double* peak)
{
    const double Ts = 1.0 / p[P_FS];
    const double td = p[P_DEAD_NS] * 1e-9;
    const int shared = p[P_TOPO] > 0.5;
    const int nleg = shared ? 3 : 4;
    const double hmax = Ts / p[P_SW_NSTEP];
    struct leg_ev ev[SSTEP_LEGS_MAX];
    double bps[2 + 5 * SSTEP_LEGS_MAX];
    int nb = 0;

    bps[nb++] = 0.0;
    bps[nb++] = Ts;
    for (int k = 0; k < nleg; k++)
    {
        if (enabled)
        {
            leg_events(duty[k], Ts, td, &ev[k]);
        }
        else
        {
            ev[k].n = 1;
            ev[k].t[0] = 0.0;
            ev[k].s[0] = LEG_DEAD;
        }
        for (int j = 0; j < ev[k].n; j++)
        {
            if ((ev[k].t[j] > 0.0) && (ev[k].t[j] < Ts))
            {
                bps[nb++] = ev[k].t[j];
            }
        }
    }
    qsort(bps, (size_t)nb, sizeof(double), cmp_double);

    peak[0] = fabs(x[0]);
    peak[1] = fabs(x[1]);
    for (int j = 0; j + 1 < nb; j++)
    {
        const double ta = bps[j];
        const double tb = bps[j + 1];
        int st[SSTEP_LEGS_MAX];
        if (tb <= ta)
        {
            continue; /* equal breakpoints */
        }
        for (int k = 0; k < nleg; k++)
        {
            st[k] = leg_state(&ev[k], ta);
        }
        const int n = (int)ceil((tb - ta) / hmax - 1e-9);
        const double h = (tb - ta) / n;
        for (int i = 0; i < n; i++)
        {
            double vA;
            double vB;
            if (shared)
            {
                const double vw = leg_voltage(st[2], -(x[0] + x[1]), p[P_VBUS], p[P_RDS], p[P_VD]);
                vA = leg_voltage(st[0], x[0], p[P_VBUS], p[P_RDS], p[P_VD]) - vw;
                vB = leg_voltage(st[1], x[1], p[P_VBUS], p[P_RDS], p[P_VD]) - vw;
            }
            else
            {
                vA = leg_voltage(st[0], x[0], p[P_VBUS], p[P_RDS], p[P_VD]) -
                     leg_voltage(st[1], -x[0], p[P_VBUS], p[P_RDS], p[P_VD]);
                vB = leg_voltage(st[2], x[1], p[P_VBUS], p[P_RDS], p[P_VD]) -
                     leg_voltage(st[3], -x[1], p[P_VBUS], p[P_RDS], p[P_VD]);
            }
            rk4(m, x, vA, vB, TL, locked, h);
            peak[0] = fmax(peak[0], fabs(x[0]));
            peak[1] = fmax(peak[1], fabs(x[1]));
        }
    }
}

static double adc_clamp(double v, double hi)
{
    return (v < 0.0) ? 0.0 : ((v > hi) ? hi : v);
}

/* Discrete state z: [iA iB w theta vA vB duty(4) enabled peakA peakB], vA vB = averaged
 * voltages of the next period, duty / enabled = leg duties of the next period.
 * Input:   1 [compare(4) enabled] from the controller
 * Outputs: 1 ADC sample [i_a i_b vbus] (counts)
 *          2 [iA iB w theta vA vB |iA|max |iB|max] (A, A, rad/s, rad, V, V, A, A),
 *            maxima over the last period (switching stage: including the ripple)
 * Power stage (rpar P_STAGE): 0 averaged, 1 switching (dead time, Rds(on), diodes).
 * Activation: one event input at fs. The voltage computed in period k is applied in
 * period k + 1 (one-period computation delay of the MCU). */
void sstep_xcos_plant(scicos_block* blk, scicos_flag flag)
{
    const double* p = blk->rpar;
    double* z = blk->z;

    if (flag == Initialization)
    {
        if ((blk->nrpar < P_NPAR) || (blk->nrpar != P_NPAR + 2 * (int)lround(p[P_NLOAD])) ||
            (blk->nz != Z_N))
        {
            Coserror("Stepper plant block: wrong parameter vector (use sil_plant_rpar)");
        }
    }
    else if (flag == OutputUpdate)
    {
        double* adc = (double*)blk->outptr[0];
        double* y = (double*)blk->outptr[1];
        const double full = p[P_ADC_FULL];
        adc[0] = adc_clamp((double)lround(p[P_ZERO] + z[0] / p[P_IFULL] * p[P_SPAN]), full);
        adc[1] = adc_clamp((double)lround(p[P_ZERO] + z[1] / p[P_IFULL] * p[P_SPAN]), full);
        adc[2] = adc_clamp((double)lround(p[P_VBUS] / p[P_VFULL] * full), 65535.0);
        for (int i = 0; i < 6; i++)
        {
            y[i] = z[i];
        }
        y[6] = z[Z_PEAK];
        y[7] = z[Z_PEAK + 1];
    }
    else if ((flag == StateUpdate) && (blk->nevprt > 0))
    {
        const double* u = (const double*)blk->inptr[0];
        const struct motor m = {p[P_R],  p[P_L], p[P_KT], p[P_NS] / 4.0, p[P_J],
                                p[P_B],  p[P_TD], p[P_TC], p[P_WC]};
        const int nsub = (int)lround(p[P_NSUB]);
        const double h = 1.0 / p[P_FS] / nsub;
        const double t = get_scicos_time();
        const int nload = (int)lround(p[P_NLOAD]);
        double TL = 0.0;
        double leg[SSTEP_LEGS_MAX];

        for (int k = 0; k < nload; k++)
        {
            if (p[P_NPAR + 2 * k] <= t + 1e-12)
            {
                TL = p[P_NPAR + 2 * k + 1];
            }
        }
        if (p[P_STAGE] > 0.5)
        {
            period_sw(&m, p, z, &z[Z_DUTY], z[Z_EN] > 0.5, TL, p[P_LOCKED] > 0.5, &z[Z_PEAK]);
        }
        else
        {
            for (int k = 0; k < nsub; k++)
            {
                rk4(&m, z, z[4], z[5], TL, p[P_LOCKED] > 0.5, h);
            }
            z[Z_PEAK] = fabs(z[0]);
            z[Z_PEAK + 1] = fabs(z[1]);
        }
        /* voltage of the next period from the compare values just written */
        for (unsigned k = 0; k < SSTEP_LEGS_MAX; k++)
        {
            z[Z_DUTY + k] = u[k] / p[P_PERIOD];
            leg[k] = z[Z_DUTY + k] * p[P_VBUS];
        }
        z[Z_EN] = (u[4] < 0.5) ? 0.0 : 1.0;
        if (u[4] < 0.5)
        {
            z[4] = 0.0; /* outputs disabled */
            z[5] = 0.0;
        }
        else if (p[P_TOPO] > 0.5)
        {
            z[4] = leg[0] - leg[2]; /* shared leg: A = U - W, B = V - W */
            z[5] = leg[1] - leg[2];
        }
        else
        {
            z[4] = leg[0] - leg[1]; /* two H-bridges */
            z[5] = leg[2] - leg[3];
        }
    }
}

/* ==============================================================================
 * MONITOR BLOCK
 * =============================================================================*/

/* Inputs:  1 plant [iA iB w theta vA vB |iA|max |iB|max], 2 controller monitor (7)
 * Outputs: 1 currents [iA iB iA_ref iB_ref] (A)
 *          2 speed (rpm)
 *          3 load angle (electrical deg; positive = rotor behind the field)
 *          4 position [commanded rotor] (microsteps)
 *          5 all of the above + [vA vB |iA|max |iB|max] (12 values) for the workspace
 * rpar: [Ns M]. Activation inherited from the inputs. */
void sstep_xcos_monitor(scicos_block* blk, scicos_flag flag)
{
    if (flag == OutputUpdate)
    {
        const double* pl = (const double*)blk->inptr[0];
        const double* mo = (const double*)blk->inptr[1];
        const double pole = blk->rpar[0] / 4.0;
        const double M = blk->rpar[1];
        double* cur = (double*)blk->outptr[0];
        double* rpm = (double*)blk->outptr[1];
        double* ang = (double*)blk->outptr[2];
        double* pos = (double*)blk->outptr[3];
        double* all = (double*)blk->outptr[4];

        cur[0] = pl[0];
        cur[1] = pl[1];
        cur[2] = mo[5];
        cur[3] = mo[6];
        rpm[0] = pl[2] * 60.0 / (2.0 * PI_D);
        ang[0] = (mo[0] - pole * pl[3]) * 180.0 / PI_D;
        pos[0] = mo[0] / (PI_D / 2.0) * M;
        pos[1] = pole * pl[3] / (PI_D / 2.0) * M;
        for (int i = 0; i < 4; i++)
        {
            all[i] = cur[i];
        }
        all[4] = rpm[0];
        all[5] = ang[0];
        all[6] = pos[0];
        all[7] = pos[1];
        all[8] = pl[4];
        all[9] = pl[5];
        all[10] = pl[6];
        all[11] = pl[7];
    }
}
