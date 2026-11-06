/*******************************************************************************
 * Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
 *
 * File:        sil/c/sstep_plant.h
 * Description: Plant model in portable C: power stage (averaged, optionally with
 *              the leg voltage error measured with ngspice, or switching with dead
 *              time and diodes), two-phase hybrid stepper motor (RK4) and current
 *              sensing (ADC counts). The single implementation of the model in C:
 *              used by the Xcos plant block (sstep_xcos.c, double) and by the
 *              builds for other targets (float, averaged only).
 *              Same equations as the Scilab prototype (model directory).
 *
 * Author:      Anton Vynohradov
 * Email:       avynohradov@systemfromscratch.com
 * License:     Apache License 2.0
 * Copyright:   (c) 2026 Anton Vynohradov
 *
 * Build options (preprocessor):
 *   SSTEP_PLANT_FLOAT      1: single precision (MCU with FPU); default 0 (double)
 *   SSTEP_PLANT_SWITCHING  0: leave the switching stage out (no qsort); default 1
 *
 * SPDX-License-Identifier: Apache-2.0
 ******************************************************************************/

#ifndef SSTEP_PLANT_H
#define SSTEP_PLANT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SSTEP_PLANT_FLOAT
#define SSTEP_PLANT_FLOAT 0
#endif

#ifndef SSTEP_PLANT_SWITCHING
#define SSTEP_PLANT_SWITCHING 1
#endif

#if SSTEP_PLANT_FLOAT
typedef float sstep_plant_real_t;
#else
typedef double sstep_plant_real_t;
#endif

#define SSTEP_PLANT_LEGS 4             /**< Legs of the largest topology. */
#define SSTEP_PLANT_LEG_ERROR_MAX 16   /**< Points of the leg voltage error table. */

enum sstep_plant_topology
{
    SSTEP_PLANT_FULL_BRIDGE = 0, /**< Two H-bridges: A = legs 0-1, B = legs 2-3. */
    SSTEP_PLANT_SHARED_LEG = 1   /**< Three legs: A = U - W, B = V - W (B-G431B-ESC1). */
};

enum sstep_plant_stage_model
{
    SSTEP_PLANT_STAGE_AVG = 0,      /**< Mean leg voltage per period (+ leg error table). */
    SSTEP_PLANT_STAGE_SWITCHING = 1 /**< Switching instants, dead time, diodes. */
};

enum sstep_plant_off
{
    SSTEP_PLANT_OFF_ZERO_VOLTAGE = 0, /**< Outputs off = 0 V on the phases (prototype). */
    SSTEP_PLANT_OFF_OPEN = 1          /**< Outputs off = open windings, zero current. */
};

/** Motor parameters, SI units (params/motor_*.sci / .json). */
struct sstep_plant_motor
{
    sstep_plant_real_t r;   /**< Phase resistance [Ohm]. */
    sstep_plant_real_t l;   /**< Phase inductance [H]. */
    sstep_plant_real_t kt;  /**< Torque constant [N*m/A] = back-EMF constant [V*s/rad]. */
    sstep_plant_real_t ns;  /**< Full steps per revolution. */
    sstep_plant_real_t j;   /**< Rotor (and load) inertia [kg*m^2]. */
    sstep_plant_real_t b;   /**< Viscous friction [N*m*s/rad]. */
    sstep_plant_real_t td;  /**< Detent torque amplitude [N*m]. */
    sstep_plant_real_t tc;  /**< Dry friction [N*m]. */
    sstep_plant_real_t w_c; /**< Dry friction smoothing speed [rad/s]. */
};

/** Power stage parameters. */
struct sstep_plant_stage
{
    enum sstep_plant_topology topology;
    enum sstep_plant_stage_model model;
    enum sstep_plant_off off;
    int locked;                 /**< 1: rotor locked (current loop tests). */
    sstep_plant_real_t vbus;    /**< Bus voltage [V]. */
    sstep_plant_real_t fs;      /**< PWM frequency = sampling rate [Hz]. */
    sstep_plant_real_t period;  /**< Compare value for 100 % duty. */
    unsigned nsub;              /**< RK4 steps per period (averaged stage). */
    /* switching stage (model/bridge_switching.sci) */
    sstep_plant_real_t dead_ns; /**< Effective dead time [ns]. */
    sstep_plant_real_t rds_on;  /**< Leg resistance [Ohm]. */
    sstep_plant_real_t vd;      /**< Body diode voltage [V]. */
    unsigned sw_nstep;          /**< Largest RK4 steps per period between edges. */
    /* averaged stage: leg voltage error (actual - duty * Vbus) against the leg
     * current (positive out of the leg), from ngspice (leg_error_table of
     * sfs-stepper-power-spice); linear between the points, constant beyond them.
     * leg_error_n = 0: ideal legs. */
    unsigned leg_error_n;
    sstep_plant_real_t leg_error_i[SSTEP_PLANT_LEG_ERROR_MAX]; /**< Leg current [A], rising. */
    sstep_plant_real_t leg_error_v[SSTEP_PLANT_LEG_ERROR_MAX]; /**< Voltage error [V]. */
};

/** Current sensing: ADC counts = zero + i / i_full * span (clamped to 0..full). */
struct sstep_plant_adc
{
    sstep_plant_real_t i_full; /**< Phase current for |span| counts [A]. */
    sstep_plant_real_t span;   /**< Counts from 0 A to +i_full (signed). */
    sstep_plant_real_t zero;   /**< Counts at 0 A. */
    sstep_plant_real_t v_full; /**< Bus voltage at `full` counts [V]. */
    sstep_plant_real_t full;   /**< Largest reading. */
};

/** Model instance. Fields are public for reading; change them with the functions. */
struct sstep_plant
{
    struct sstep_plant_motor motor;
    struct sstep_plant_stage stage;
    /** State: iA [A], iB [A], w [rad/s], electrical angle p*theta in [-pi, pi) [rad]. */
    sstep_plant_real_t x[4];
    int32_t e_turns;                         /**< Electrical turns of the rotor. */
    sstep_plant_real_t v[2];                 /**< Mean phase voltages of the period [V]. */
    sstep_plant_real_t duty[SSTEP_PLANT_LEGS]; /**< Leg duties of the period. */
    int enabled;                             /**< Outputs on in the period. */
    sstep_plant_real_t peak[2];              /**< Largest |iA|, |iB| in the last period [A]. */
};

/**
 * @brief  Set up the model: rotor at rest at angle 0, no current, outputs off.
 * @retval 0, or -1 when a parameter is invalid (inductance, inertia, rates, table).
 */
int sstep_plant_init(struct sstep_plant* p, const struct sstep_plant_motor* motor,
                     const struct sstep_plant_stage* stage);

/**
 * @brief  Compare values written by the controller in this period: they drive the
 *         legs in the NEXT period (one period of computation delay, as on the MCU).
 * @param  compare  SSTEP_PLANT_LEGS compare values (0..period); legs not used by the
 *                  topology are ignored.
 * @param  enabled  Outputs on.
 */
void sstep_plant_set_compare(struct sstep_plant* p, const sstep_plant_real_t* compare,
                             int enabled);

/** @brief Advance the model by one PWM period with load torque @p load [N*m]. */
void sstep_plant_period(struct sstep_plant* p, sstep_plant_real_t load);

/** @brief ADC counts of the present state: out[0] iA, out[1] iB, out[2] Vbus. */
void sstep_plant_adc_sample(const struct sstep_plant* p, const struct sstep_plant_adc* adc,
                            sstep_plant_real_t out[3]);

/** @brief Mechanical rotor angle [rad], unwrapped. */
sstep_plant_real_t sstep_plant_theta(const struct sstep_plant* p);

/** @brief Leg voltage error of the table at leg current @p i [V]. */
sstep_plant_real_t sstep_plant_leg_error(const struct sstep_plant_stage* stage,
                                         sstep_plant_real_t i);

#ifdef __cplusplus
}
#endif

#endif /* SSTEP_PLANT_H */
