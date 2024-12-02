/*
 * robocar_pp_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "robocar_pp".
 *
 * Model version              : 2.26
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Wed Nov 13 20:12:39 2024
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Apple->ARM64
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "robocar_pp.h"

/* Block parameters (default storage) */
P_robocar_pp_T robocar_pp_P = {
  /* Variable: wheel_base
   * Referenced by: '<S1>/MATLAB System'
   */
  0.5,

  /* Mask Parameter: BicycleKinematicModel_InitialSt
   * Referenced by: '<S1>/Integrator'
   */
  { 0.0, 0.0, 0.0 },

  /* Mask Parameter: BicycleKinematicModel_MaxSteeri
   * Referenced by: '<S1>/MATLAB System'
   */
  1.0471975511965976,

  /* Mask Parameter: BicycleKinematicModel_VehicleSp
   * Referenced by: '<S1>/MATLAB System'
   */
  { -INFINITY, INFINITY },

  /* Expression: 2
   * Referenced by: '<Root>/Pure Pursuit'
   */
  2.0,

  /* Expression: 1.0
   * Referenced by: '<Root>/Pure Pursuit'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Pure Pursuit'
   */
  1.0
};
