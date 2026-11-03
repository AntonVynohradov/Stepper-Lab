/*******************************************************************************
 * sfs-stepper-sim: Stepper motor model and control simulation (Scilab/Xcos)
 *
 * File:        sil/c/sstep_xcos.c
 * Description: Xcos computational functions (type 4) for software-in-the-loop:
 *              - sstep_xcos_ctrl:    the Stepper-C library (fast and slow loop),
 *              - sstep_xcos_plant:   power stage (averaged or switching with dead
 *                                    time and diodes) + motor + ADC (RK4): a thin
 *                                    wrapper of sstep_plant.c, the model shared
 *                                    with the firmware simulation,
 *              - sstep_xcos_monitor: signals for scopes and the workspace,
 *              - sstep_xcos_check:   configuration check, called from Scilab.
 *              The motor and the power stage use the same equations as the
 *              Scilab prototype (model directory) and the Stepper-C unit tests.
 *              The monitor reads the setpoint of the library through its internal
 *              header (sstep_internal.h, copied into the build by sil_build).
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
#include "sstep_internal.h"
#include "sstep_plant.h"

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
    C_STALL_MS, C_STALL_FILT, C_STALL_SPEED, C_STALL_SYNC, /* stall detection (1.1.0) */
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
    P_NERR,  /* points of the leg voltage error table (averaged stage), 0 = none */
    P_NLOAD,
    P_NPAR /* load steps follow: t, TL; then the leg error table: i(P_NERR), v(P_NERR) */
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
    *c = (struct sstep_config){0}; /* fields of later versions keep their default */
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
    c->ctrl.stall_time_ms = u32(p[C_STALL_MS]);
    c->ctrl.stall_filter_us = u32(p[C_STALL_FILT]);
    c->ctrl.stall_speed = u32(p[C_STALL_SPEED]);
    c->ctrl.stall_sync_pct = (uint16_t)u32(p[C_STALL_SYNC]);
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
 *             iA_ref(A) iB_ref(A) emf_d(mV) emf_q(mV) rotor_velocity(usteps/s)
 *             load_angle(cdeg) stall_level(%)], the last five from the stall
 *             detector (sstep_get_status, 0 when it is off)
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
        const struct sstep_ctx* ctx = sstep_ctx_of_const(&s->drv);
        used = ctx->cur.angle - (uint32_t)ctx->cur.dangle;
        s->phi_turns += (int32_t)(used - s->phi_last);
        s->phi_last = used;

        for (unsigned k = 0; k < SSTEP_LEGS_MAX; k++)
        {
            out[k] = (double)s->compare[k];
        }
        out[4] = (double)s->enabled;

        (void)sstep_get_status(&s->drv, &st);
        ang = (double)used * (2.0 * PI_D / 4294967296.0);
        amp = real_to_d(ctx->cur.i_amp) * s->i_full;
        mon[0] = (double)s->phi_turns * (2.0 * PI_D / 4294967296.0);
        mon[1] = (double)st.position;
        mon[2] = (double)st.velocity;
        mon[3] = (double)st.state;
        mon[4] = (double)st.faults;
        mon[5] = amp * cos(ang);
        mon[6] = amp * sin(ang);
        mon[7] = (double)st.emf_d_mv;
        mon[8] = (double)st.emf_q_mv;
        mon[9] = (double)st.rotor_velocity;
        mon[10] = (double)st.load_angle_cdeg;
        mon[11] = (double)st.stall_level_pct;
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
 * PLANT BLOCK: power stage, motor (RK4), ADC - the model is sstep_plant.c
 * =============================================================================*/

/* Discrete state of the diagram block (xcos/blocks): kept for the diagram, the state
 * lives in struct sstep_plant. */
#define Z_N 13

struct sil_plant
{
    struct sstep_plant model;
    struct sstep_plant_adc adc;
};

/* Input:   1 [compare(4) enabled] from the controller
 * Outputs: 1 ADC sample [i_a i_b vbus] (counts)
 *          2 [iA iB w theta vA vB |iA|max |iB|max] (A, A, rad/s, rad, V, V, A, A),
 *            maxima over the last period (switching stage: including the ripple)
 * Power stage (rpar P_STAGE): 0 averaged, 1 switching (dead time, Rds(on), diodes);
 * averaged with a leg voltage error table when P_NERR > 0 (ngspice).
 * Activation: one event input at fs. The voltage computed in period k is applied in
 * period k + 1 (one-period computation delay of the MCU). */
void sstep_xcos_plant(scicos_block* blk, scicos_flag flag)
{
    struct sil_plant* s = (struct sil_plant*)(*blk->work);
    const double* p = blk->rpar;

    if (flag == Initialization)
    {
        struct sstep_plant_motor mot;
        struct sstep_plant_stage stg;
        int nload;
        int nerr;
        if ((blk->nrpar < P_NPAR) || (blk->nz != Z_N))
        {
            Coserror("Stepper plant block: wrong parameter vector (use sil_plant_rpar)");
            return;
        }
        nload = (int)lround(p[P_NLOAD]);
        nerr = (int)lround(p[P_NERR]);
        if ((nerr < 0) || (nerr > SSTEP_PLANT_LEG_ERROR_MAX) ||
            (blk->nrpar != P_NPAR + 2 * nload + 2 * nerr))
        {
            Coserror("Stepper plant block: wrong parameter vector (use sil_plant_rpar)");
            return;
        }
        s = (struct sil_plant*)calloc(1, sizeof(*s));
        if (s == NULL)
        {
            Coserror("Stepper plant block: out of memory");
            return;
        }
        *blk->work = s;

        mot.r = p[P_R];
        mot.l = p[P_L];
        mot.kt = p[P_KT];
        mot.ns = p[P_NS];
        mot.j = p[P_J];
        mot.b = p[P_B];
        mot.td = p[P_TD];
        mot.tc = p[P_TC];
        mot.w_c = p[P_WC];
        stg.topology = (p[P_TOPO] > 0.5) ? SSTEP_PLANT_SHARED_LEG : SSTEP_PLANT_FULL_BRIDGE;
        stg.model = (p[P_STAGE] > 0.5) ? SSTEP_PLANT_STAGE_SWITCHING : SSTEP_PLANT_STAGE_AVG;
        stg.off = SSTEP_PLANT_OFF_ZERO_VOLTAGE; /* as the Scilab prototype */
        stg.locked = p[P_LOCKED] > 0.5;
        stg.vbus = p[P_VBUS];
        stg.fs = p[P_FS];
        stg.period = p[P_PERIOD];
        stg.nsub = (unsigned)lround(p[P_NSUB]);
        stg.dead_ns = p[P_DEAD_NS];
        stg.rds_on = p[P_RDS];
        stg.vd = p[P_VD];
        stg.sw_nstep = (unsigned)lround(p[P_SW_NSTEP]);
        stg.leg_error_n = (unsigned)nerr;
        for (int k = 0; k < nerr; k++)
        {
            stg.leg_error_i[k] = p[P_NPAR + 2 * nload + k];
            stg.leg_error_v[k] = p[P_NPAR + 2 * nload + nerr + k];
        }
        if (sstep_plant_init(&s->model, &mot, &stg) != 0)
        {
            Coserror("Stepper plant block: invalid motor or power stage parameters");
            return;
        }
        s->adc.i_full = p[P_IFULL];
        s->adc.span = p[P_SPAN];
        s->adc.zero = p[P_ZERO];
        s->adc.v_full = p[P_VFULL];
        s->adc.full = p[P_ADC_FULL];
    }
    else if ((flag == OutputUpdate) && (s != NULL))
    {
        const struct sstep_plant* m = &s->model;
        double* adc = (double*)blk->outptr[0];
        double* y = (double*)blk->outptr[1];
        sstep_plant_adc_sample(m, &s->adc, adc);
        y[0] = m->x[0];
        y[1] = m->x[1];
        y[2] = m->x[2];
        y[3] = sstep_plant_theta(m);
        y[4] = m->v[0];
        y[5] = m->v[1];
        y[6] = m->peak[0];
        y[7] = m->peak[1];
    }
    else if ((flag == StateUpdate) && (blk->nevprt > 0) && (s != NULL))
    {
        const double* u = (const double*)blk->inptr[0];
        const double t = get_scicos_time();
        const int nload = (int)lround(p[P_NLOAD]);
        double TL = 0.0;

        for (int k = 0; k < nload; k++)
        {
            if (p[P_NPAR + 2 * k] <= t + 1e-12)
            {
                TL = p[P_NPAR + 2 * k + 1];
            }
        }
        sstep_plant_period(&s->model, TL);
        /* voltage of the next period from the compare values just written */
        sstep_plant_set_compare(&s->model, u, u[4] >= 0.5);
    }
    else if (flag == Ending)
    {
        free(s);
        *blk->work = NULL;
    }
}

/* ==============================================================================
 * MONITOR BLOCK
 * =============================================================================*/

/* Inputs:  1 plant [iA iB w theta vA vB |iA|max |iB|max], 2 controller monitor (12)
 * Outputs: 1 currents [iA iB iA_ref iB_ref] (A)
 *          2 speed (rpm)
 *          3 load angle (electrical deg; positive = rotor behind the field)
 *          4 position [commanded rotor] (microsteps)
 *          5 all of the above + [vA vB |iA|max |iB|max] + [state faults emf_d emf_q
 *            rotor_velocity load_angle_est stall_level] (19 values) for the
 *            workspace: library state and fault flags, stall detector estimates
 *            (mV, mV, usteps/s, electrical deg, %)
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
        all[12] = mo[3];
        all[13] = mo[4];
        all[14] = mo[7];
        all[15] = mo[8];
        all[16] = mo[9];
        all[17] = mo[10] / 100.0;
        all[18] = mo[11];
    }
}
