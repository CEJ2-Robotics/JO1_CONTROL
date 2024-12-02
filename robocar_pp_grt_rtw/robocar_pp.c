/*
 * robocar_pp.c
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
#include "robocar_pp_types.h"
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include <string.h>
#include "robocar_pp_private.h"
#include <math.h>
#include "rt_defines.h"

/* Block signals (default storage) */
B_robocar_pp_T robocar_pp_B;

/* Continuous states */
X_robocar_pp_T robocar_pp_X;

/* Disabled State Vector */
XDis_robocar_pp_T robocar_pp_XDis;

/* Block states (default storage) */
DW_robocar_pp_T robocar_pp_DW;

/* Real-time model */
static RT_MODEL_robocar_pp_T robocar_pp_M_;
RT_MODEL_robocar_pp_T *const robocar_pp_M = &robocar_pp_M_;

/* Forward declaration for local functions */
static void BicycleKinematics_assignModelPr(robotics_slmobile_internal_bl_T *obj);
static void rob_BicycleKinematics_setupImpl(robotics_slmobile_internal_bl_T *obj);
static real_T robocar_pp_norm(const real_T x[2]);
static real_T robocar_pp_closestPointOnLine(const real_T pt1[2], real_T pt2[2],
  const real_T refPt[2]);
static void bicycleKinematics_getGeneralize(const
  c_bicycleKinematics_robocar_p_T *obj, real_T v, real_T psi, real_T *b_v,
  real_T *omega);

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 3;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  robocar_pp_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  robocar_pp_step();
  robocar_pp_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  robocar_pp_step();
  robocar_pp_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static void BicycleKinematics_assignModelPr(robotics_slmobile_internal_bl_T *obj)
{
  int32_T b_kstr;
  static const char_T tmp[25] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
    'e', 'e', 'd', 'H', 'e', 'a', 'd', 'i', 'n', 'g', 'R', 'a', 't', 'e', '-',
    '-' };

  for (b_kstr = 0; b_kstr < 25; b_kstr++) {
    /* Start for MATLABSystem: '<S1>/MATLAB System' */
    obj->KinModel.VehicleInputsInternal[b_kstr] = tmp[b_kstr];
  }

  real_T b;
  b = obj->WheelBase;

  /* Start for MATLABSystem: '<S1>/MATLAB System' */
  obj->KinModel.WheelBase = b;
  obj->KinModel.VehicleSpeedRange[0] = obj->VehicleSpeedRange[0];
  obj->KinModel.VehicleSpeedRange[1] = obj->VehicleSpeedRange[1];
  b = obj->MaxSteeringAngle;

  /* Start for MATLABSystem: '<S1>/MATLAB System' */
  obj->KinModel.MaxSteeringAngle = b;
}

static void rob_BicycleKinematics_setupImpl(robotics_slmobile_internal_bl_T *obj)
{
  int32_T ret;
  char_T vehicleInputs[25];
  static const char_T b[25] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p', 'e',
    'e', 'd', 'S', 't', 'e', 'e', 'r', 'i', 'n', 'g', 'A', 'n', 'g', 'l', 'e' };

  boolean_T b_bool;
  static const char_T b_0[25] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
    'e', 'e', 'd', 'S', 't', 'e', 'e', 'r', 'i', 'n', 'g', 'A', 'n', 'g', 'l',
    'e' };

  static const char_T tmp[25] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
    'e', 'e', 'd', 'H', 'e', 'a', 'd', 'i', 'n', 'g', 'R', 'a', 't', 'e', '-',
    '-' };

  /* Start for MATLABSystem: '<S1>/MATLAB System' */
  obj->KinModel.WheelBase = 1.0;
  obj->KinModel.VehicleSpeedRange[0] = (rtMinusInf);
  obj->KinModel.VehicleSpeedRange[1] = (rtInf);
  obj->KinModel.MaxSteeringAngle = 0.78539816339744828;
  for (ret = 0; ret < 25; ret++) {
    /* Start for MATLABSystem: '<S1>/MATLAB System' */
    vehicleInputs[ret] = b_0[ret];
  }

  /* Start for MATLABSystem: '<S1>/MATLAB System' */
  ret = memcmp(&vehicleInputs[0], &b[0], 25);
  b_bool = (ret == 0);
  for (ret = 0; ret < 25; ret++) {
    if (b_bool) {
      /* Start for MATLABSystem: '<S1>/MATLAB System' */
      obj->KinModel.VehicleInputsInternal[ret] = b_0[ret];
    } else {
      /* Start for MATLABSystem: '<S1>/MATLAB System' */
      obj->KinModel.VehicleInputsInternal[ret] = tmp[ret];
    }
  }

  BicycleKinematics_assignModelPr(obj);
}

static real_T robocar_pp_norm(const real_T x[2])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;

  /* Start for MATLABSystem: '<Root>/Pure Pursuit' */
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  /* Start for MATLABSystem: '<Root>/Pure Pursuit' */
  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

static real_T robocar_pp_closestPointOnLine(const real_T pt1[2], real_T pt2[2],
  const real_T refPt[2])
{
  real_T refPt_0[2];
  real_T alpha;
  real_T distance;
  real_T v12;
  real_T v12_0;
  real_T v12_1;
  int32_T b_k;
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;

  /* Start for MATLABSystem: '<Root>/Pure Pursuit' */
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 2)) {
    if (!(pt1[b_k] == pt2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  if (p) {
    alpha = pt1[0];
    pt2[0] = alpha;
    refPt_0[0] = refPt[0] - alpha;
    alpha = pt1[1];
    pt2[1] = alpha;
    refPt_0[1] = refPt[1] - alpha;
    distance = robocar_pp_norm(refPt_0);
  } else {
    alpha = pt2[0];
    v12_1 = alpha - pt1[0];
    v12 = (alpha - refPt[0]) * v12_1;
    v12_0 = v12_1 * v12_1;
    alpha = pt2[1];
    v12_1 = alpha - pt1[1];
    v12 += (alpha - refPt[1]) * v12_1;
    v12_0 += v12_1 * v12_1;
    alpha = v12 / v12_0;
    if (alpha > 1.0) {
      pt2[0] = pt1[0];
      pt2[1] = pt1[1];
    } else if (!(alpha < 0.0)) {
      v12_1 = 1.0 - alpha;
      pt2[0] = alpha * pt1[0] + v12_1 * pt2[0];
      pt2[1] = alpha * pt1[1] + v12_1 * pt2[1];
    }

    refPt_0[0] = refPt[0] - pt2[0];
    refPt_0[1] = refPt[1] - pt2[1];
    distance = robocar_pp_norm(refPt_0);
  }

  /* End of Start for MATLABSystem: '<Root>/Pure Pursuit' */
  return distance;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u1 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u0 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp_0, tmp);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

static void bicycleKinematics_getGeneralize(const
  c_bicycleKinematics_robocar_p_T *obj, real_T v, real_T psi, real_T *b_v,
  real_T *omega)
{
  real_T a_data;
  int32_T i;
  int32_T trueCount;
  boolean_T aboveThreshold;
  boolean_T b;
  *b_v = v;

  /* Start for MATLABSystem: '<S1>/MATLAB System' */
  aboveThreshold = (fabs(psi) > obj->MaxSteeringAngle);
  b = !aboveThreshold;
  if (!b) {
    real_T u;
    trueCount = 0;
    if (aboveThreshold) {
      for (i = 0; i < 1; i++) {
        trueCount++;
      }
    }

    i = trueCount;
    if (i - 1 >= 0) {
      a_data = psi;
    }

    for (trueCount = 0; trueCount < i; trueCount++) {
      u = a_data;
      if (rtIsNaN(u)) {
        u = (rtNaN);
      } else if (u < 0.0) {
        u = -1.0;
      } else {
        u = (u > 0.0);
      }

      a_data = u;
    }

    u = obj->MaxSteeringAngle;
    i--;
    for (trueCount = 0; trueCount <= i; trueCount++) {
      a_data *= u;
    }

    if (aboveThreshold) {
      psi = a_data;
    }
  }

  *omega = v / obj->WheelBase * tan(psi);

  /* End of Start for MATLABSystem: '<S1>/MATLAB System' */
}

/* Model step function */
void robocar_pp_step(void)
{
  real_T waypoints_data[8];
  real_T J[6];
  real_T stateReshaped[3];
  real_T lookaheadStartPt[2];
  real_T dist;
  real_T lookaheadIdx;
  real_T minDistance;
  int32_T b_k;
  int32_T ret;
  int8_T tmp_data[4];
  boolean_T b[8];
  boolean_T searchFlag;
  static const char_T a[25] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p', 'e',
    'e', 'd', 'S', 't', 'e', 'e', 'r', 'i', 'n', 'g', 'A', 'n', 'g', 'l', 'e' };

  static const char_T a_0[25] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
    'e', 'e', 'd', 'H', 'e', 'a', 'd', 'i', 'n', 'g', 'R', 'a', 't', 'e', '-',
    '-' };

  real_T waypoints[2];
  real_T waypoints_0[2];
  real_T lookaheadEndPt_idx_0;
  real_T lookaheadEndPt_idx_1;
  real_T lookaheadStartPt_tmp;
  int32_T i_tmp;
  int32_T i_tmp_0;
  boolean_T nanidx_tmp[4];
  boolean_T isInSecondQuadrant;
  static const int8_T tmp[8] = { 0, 10, 0, 10, 0, 0, 10, 10 };

  int32_T tmp_size_idx_0;
  boolean_T exitg1;
  if (rtmIsMajorTimeStep(robocar_pp_M)) {
    /* set solver stop time */
    if (!(robocar_pp_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&robocar_pp_M->solverInfo,
                            ((robocar_pp_M->Timing.clockTickH0 + 1) *
        robocar_pp_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&robocar_pp_M->solverInfo,
                            ((robocar_pp_M->Timing.clockTick0 + 1) *
        robocar_pp_M->Timing.stepSize0 + robocar_pp_M->Timing.clockTickH0 *
        robocar_pp_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(robocar_pp_M)) {
    robocar_pp_M->Timing.t[0] = rtsiGetT(&robocar_pp_M->solverInfo);
  }

  /* Integrator: '<S1>/Integrator' */
  robocar_pp_B.Integrator[0] = robocar_pp_X.Integrator_CSTATE[0];
  robocar_pp_B.Integrator[1] = robocar_pp_X.Integrator_CSTATE[1];
  robocar_pp_B.Integrator[2] = robocar_pp_X.Integrator_CSTATE[2];
  if (rtmIsMajorTimeStep(robocar_pp_M)) {
    /* MATLAB Function: '<Root>/MATLAB Function2' */
    for (b_k = 0; b_k < 8; b_k++) {
      robocar_pp_B.path[b_k] = tmp[b_k];
    }

    /* End of MATLAB Function: '<Root>/MATLAB Function2' */
    /* :  x = [0 10 0  10]; */
    /* :  y = [0 0  10 10]; */
    /* :  path = [x; y]'; */
  }

  /* MATLABSystem: '<Root>/Pure Pursuit' */
  if (robocar_pp_DW.obj.DesiredLinearVelocity !=
      robocar_pp_P.PurePursuit_DesiredLinearVeloci) {
    robocar_pp_DW.obj.DesiredLinearVelocity =
      robocar_pp_P.PurePursuit_DesiredLinearVeloci;
  }

  if (robocar_pp_DW.obj.MaxAngularVelocity !=
      robocar_pp_P.PurePursuit_MaxAngularVelocity) {
    robocar_pp_DW.obj.MaxAngularVelocity =
      robocar_pp_P.PurePursuit_MaxAngularVelocity;
  }

  if (robocar_pp_DW.obj.LookaheadDistance !=
      robocar_pp_P.PurePursuit_LookaheadDistance) {
    robocar_pp_DW.obj.LookaheadDistance =
      robocar_pp_P.PurePursuit_LookaheadDistance;
  }

  searchFlag = false;
  isInSecondQuadrant = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 8)) {
    if ((robocar_pp_DW.obj.WaypointsInternal[b_k] == robocar_pp_B.path[b_k]) ||
        (rtIsNaN(robocar_pp_DW.obj.WaypointsInternal[b_k]) && rtIsNaN
         (robocar_pp_B.path[b_k]))) {
      b_k++;
    } else {
      isInSecondQuadrant = false;
      exitg1 = true;
    }
  }

  if (isInSecondQuadrant) {
    searchFlag = true;
  }

  if (!searchFlag) {
    memcpy(&robocar_pp_DW.obj.WaypointsInternal[0], &robocar_pp_B.path[0],
           sizeof(real_T) << 3U);
    robocar_pp_DW.obj.ProjectionLineIndex = 0.0;
  }

  for (b_k = 0; b_k < 8; b_k++) {
    b[b_k] = !rtIsNaN(robocar_pp_B.path[b_k]);
  }

  nanidx_tmp[0] = (b[0] && b[4]);
  nanidx_tmp[1] = (b[1] && b[5]);
  nanidx_tmp[2] = (b[2] && b[6]);
  nanidx_tmp[3] = (b[3] && b[7]);
  ret = 0;
  for (b_k = 0; b_k < 4; b_k++) {
    /* Start for MATLABSystem: '<Root>/Pure Pursuit' */
    if (nanidx_tmp[b_k]) {
      ret++;
    }
  }

  tmp_size_idx_0 = ret;
  ret = 0;
  for (b_k = 0; b_k < 4; b_k++) {
    /* Start for MATLABSystem: '<Root>/Pure Pursuit' */
    if (nanidx_tmp[b_k]) {
      tmp_data[ret] = (int8_T)b_k;
      ret++;
    }
  }

  /* MATLABSystem: '<Root>/Pure Pursuit' */
  ret = tmp_size_idx_0;
  for (b_k = 0; b_k < 2; b_k++) {
    for (i_tmp_0 = 0; i_tmp_0 < ret; i_tmp_0++) {
      waypoints_data[i_tmp_0 + tmp_size_idx_0 * b_k] = robocar_pp_B.path[(b_k <<
        2) + tmp_data[i_tmp_0]];
    }
  }

  if (tmp_size_idx_0 == 0) {
    dist = 0.0;
    minDistance = 0.0;
  } else {
    searchFlag = false;
    if (robocar_pp_DW.obj.ProjectionLineIndex == 0.0) {
      searchFlag = true;
      robocar_pp_DW.obj.ProjectionPoint[0] = waypoints_data[0];
      robocar_pp_DW.obj.ProjectionPoint[1] = waypoints_data[tmp_size_idx_0];
      robocar_pp_DW.obj.ProjectionLineIndex = 1.0;
    }

    if (tmp_size_idx_0 == 1) {
      robocar_pp_DW.obj.ProjectionPoint[0] = waypoints_data[0];
      robocar_pp_DW.obj.ProjectionPoint[1] = waypoints_data[tmp_size_idx_0];
      lookaheadEndPt_idx_0 = waypoints_data[0];
      lookaheadEndPt_idx_1 = waypoints_data[tmp_size_idx_0];
    } else {
      b_k = (int32_T)(robocar_pp_DW.obj.ProjectionLineIndex + 1.0);
      lookaheadStartPt_tmp = waypoints_data[b_k - 1];
      lookaheadStartPt[0] = lookaheadStartPt_tmp;
      dist = waypoints_data[(b_k + tmp_size_idx_0) - 1];
      lookaheadStartPt[1] = dist;
      minDistance = robocar_pp_closestPointOnLine
        (robocar_pp_DW.obj.ProjectionPoint, lookaheadStartPt,
         &robocar_pp_B.Integrator[0]);
      lookaheadIdx = lookaheadStartPt[0];
      robocar_pp_DW.obj.ProjectionPoint[0] = lookaheadIdx;
      waypoints[0] = lookaheadIdx - lookaheadStartPt_tmp;
      lookaheadIdx = lookaheadStartPt[1];
      robocar_pp_DW.obj.ProjectionPoint[1] = lookaheadIdx;
      waypoints[1] = lookaheadIdx - dist;
      dist = robocar_pp_norm(waypoints);
      lookaheadIdx = robocar_pp_DW.obj.ProjectionLineIndex + 1.0;
      b_k = (int32_T)((1.0 - (robocar_pp_DW.obj.ProjectionLineIndex + 1.0)) +
                      ((real_T)tmp_size_idx_0 - 1.0)) - 1;
      ret = 0;
      exitg1 = false;
      while ((!exitg1) && (ret <= b_k)) {
        lookaheadStartPt_tmp = lookaheadIdx + (real_T)ret;
        if ((!searchFlag) && (dist > robocar_pp_DW.obj.LookaheadDistance)) {
          exitg1 = true;
        } else {
          i_tmp_0 = (int32_T)lookaheadStartPt_tmp;
          i_tmp = (int32_T)(lookaheadStartPt_tmp + 1.0);
          lookaheadEndPt_idx_0 = waypoints_data[i_tmp - 1];
          lookaheadEndPt_idx_1 = waypoints_data[i_tmp_0 - 1];
          waypoints[0] = lookaheadEndPt_idx_1 - lookaheadEndPt_idx_0;
          lookaheadStartPt[0] = lookaheadEndPt_idx_0;
          waypoints_0[0] = lookaheadEndPt_idx_1;
          lookaheadEndPt_idx_0 = waypoints_data[(i_tmp + tmp_size_idx_0) - 1];
          lookaheadEndPt_idx_1 = waypoints_data[(i_tmp_0 + tmp_size_idx_0) - 1];
          waypoints[1] = lookaheadEndPt_idx_1 - lookaheadEndPt_idx_0;
          lookaheadStartPt[1] = lookaheadEndPt_idx_0;
          waypoints_0[1] = lookaheadEndPt_idx_1;
          dist += robocar_pp_norm(waypoints);
          lookaheadEndPt_idx_0 = robocar_pp_closestPointOnLine(waypoints_0,
            lookaheadStartPt, &robocar_pp_B.Integrator[0]);
          if (lookaheadEndPt_idx_0 < minDistance) {
            minDistance = lookaheadEndPt_idx_0;
            robocar_pp_DW.obj.ProjectionPoint[0] = lookaheadStartPt[0];
            robocar_pp_DW.obj.ProjectionPoint[1] = lookaheadStartPt[1];
            robocar_pp_DW.obj.ProjectionLineIndex = lookaheadStartPt_tmp;
          }

          ret++;
        }
      }

      b_k = (int32_T)(robocar_pp_DW.obj.ProjectionLineIndex + 1.0);
      lookaheadEndPt_idx_0 = waypoints_data[b_k - 1];
      waypoints[0] = robocar_pp_DW.obj.ProjectionPoint[0] - lookaheadEndPt_idx_0;
      lookaheadEndPt_idx_1 = waypoints_data[(b_k + tmp_size_idx_0) - 1];
      waypoints[1] = robocar_pp_DW.obj.ProjectionPoint[1] - lookaheadEndPt_idx_1;
      dist = robocar_pp_norm(waypoints);
      lookaheadStartPt[0] = robocar_pp_DW.obj.ProjectionPoint[0];
      lookaheadStartPt[1] = robocar_pp_DW.obj.ProjectionPoint[1];
      minDistance = dist - robocar_pp_DW.obj.LookaheadDistance;
      lookaheadIdx = robocar_pp_DW.obj.ProjectionLineIndex;
      while ((minDistance < 0.0) && (lookaheadIdx < (real_T)tmp_size_idx_0 - 1.0))
      {
        lookaheadIdx++;
        b_k = (int32_T)lookaheadIdx;
        ret = (int32_T)(lookaheadIdx + 1.0);
        lookaheadStartPt_tmp = waypoints_data[b_k - 1];
        lookaheadStartPt[0] = lookaheadStartPt_tmp;
        lookaheadEndPt_idx_0 = waypoints_data[ret - 1];
        waypoints[0] = lookaheadStartPt_tmp - lookaheadEndPt_idx_0;
        lookaheadStartPt_tmp = waypoints_data[(b_k + tmp_size_idx_0) - 1];
        lookaheadStartPt[1] = lookaheadStartPt_tmp;
        lookaheadEndPt_idx_1 = waypoints_data[(ret + tmp_size_idx_0) - 1];
        waypoints[1] = lookaheadStartPt_tmp - lookaheadEndPt_idx_1;
        dist += robocar_pp_norm(waypoints);
        minDistance = dist - robocar_pp_DW.obj.LookaheadDistance;
      }

      waypoints[0] = lookaheadStartPt[0] - lookaheadEndPt_idx_0;
      waypoints[1] = lookaheadStartPt[1] - lookaheadEndPt_idx_1;
      dist = minDistance / robocar_pp_norm(waypoints);
      if (dist > 0.0) {
        minDistance = 1.0 - dist;
        lookaheadEndPt_idx_0 = dist * lookaheadStartPt[0] + minDistance *
          lookaheadEndPt_idx_0;
        lookaheadEndPt_idx_1 = dist * lookaheadStartPt[1] + minDistance *
          lookaheadEndPt_idx_1;
      }
    }

    robocar_pp_DW.obj.LookaheadPoint[0] = lookaheadEndPt_idx_0;
    robocar_pp_DW.obj.LookaheadPoint[1] = lookaheadEndPt_idx_1;
    dist = rt_atan2d_snf(robocar_pp_DW.obj.LookaheadPoint[1] -
                         robocar_pp_B.Integrator[1],
                         robocar_pp_DW.obj.LookaheadPoint[0] -
                         robocar_pp_B.Integrator[0]) - robocar_pp_B.Integrator[2];
    searchFlag = !(fabs(dist) > 3.1415926535897931);
    if (!searchFlag) {
      if (rtIsNaN(dist + 3.1415926535897931) || rtIsInf(dist +
           3.1415926535897931)) {
        minDistance = (rtNaN);
      } else if (dist + 3.1415926535897931 == 0.0) {
        minDistance = 0.0;
      } else {
        minDistance = fmod(dist + 3.1415926535897931, 6.2831853071795862);
        searchFlag = (minDistance == 0.0);
        if (!searchFlag) {
          lookaheadIdx = fabs((dist + 3.1415926535897931) / 6.2831853071795862);
          searchFlag = !(fabs(lookaheadIdx - floor(lookaheadIdx + 0.5)) >
                         2.2204460492503131E-16 * lookaheadIdx);
        }

        if (searchFlag) {
          minDistance = 0.0;
        } else if (minDistance < 0.0) {
          minDistance += 6.2831853071795862;
        }
      }

      if ((minDistance == 0.0) && (dist + 3.1415926535897931 > 0.0)) {
        minDistance = 6.2831853071795862;
      }

      dist = minDistance - 3.1415926535897931;
    }

    minDistance = 2.0 * sin(dist) / robocar_pp_DW.obj.LookaheadDistance;
    searchFlag = rtIsNaN(minDistance);
    if (searchFlag) {
      minDistance = 0.0;
    }

    if (fabs(fabs(dist) - 3.1415926535897931) < 1.4901161193847656E-8) {
      dist = minDistance;
      if (rtIsNaN(dist)) {
        minDistance = (rtNaN);
      } else if (dist < 0.0) {
        minDistance = -1.0;
      } else {
        minDistance = (dist > 0.0);
      }
    }

    if (fabs(minDistance) > robocar_pp_DW.obj.MaxAngularVelocity) {
      if (rtIsNaN(minDistance)) {
        dist = (rtNaN);
      } else if (minDistance < 0.0) {
        dist = -1.0;
      } else {
        dist = (minDistance > 0.0);
      }

      minDistance = dist * robocar_pp_DW.obj.MaxAngularVelocity;
    }

    dist = robocar_pp_DW.obj.DesiredLinearVelocity;
    robocar_pp_DW.obj.LastPose[0] = robocar_pp_B.Integrator[0];
    robocar_pp_DW.obj.LastPose[1] = robocar_pp_B.Integrator[1];
    robocar_pp_DW.obj.LastPose[2] = robocar_pp_B.Integrator[2];
  }

  lookaheadStartPt[0] = dist;
  lookaheadStartPt[1] = minDistance;
  dist = lookaheadStartPt[0];
  minDistance = lookaheadStartPt[1];

  /* MATLABSystem: '<Root>/Pure Pursuit' */
  robocar_pp_B.PurePursuit_o1 = dist;

  /* MATLABSystem: '<Root>/Pure Pursuit' */
  robocar_pp_B.PurePursuit_o2 = minDistance;

  /* MATLABSystem: '<S1>/MATLAB System' */
  if (robocar_pp_DW.obj_f.WheelBase != robocar_pp_P.wheel_base) {
    if (robocar_pp_DW.obj_f.isInitialized == 1) {
      robocar_pp_DW.obj_f.TunablePropsChanged = true;
      robocar_pp_DW.obj_f.tunablePropertyChanged[0] = true;
    }

    robocar_pp_DW.obj_f.WheelBase = robocar_pp_P.wheel_base;
  }

  searchFlag = false;
  isInSecondQuadrant = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 2)) {
    if (!(robocar_pp_DW.obj_f.VehicleSpeedRange[b_k] ==
          robocar_pp_P.BicycleKinematicModel_VehicleSp[b_k])) {
      isInSecondQuadrant = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (isInSecondQuadrant) {
    searchFlag = true;
  }

  if (!searchFlag) {
    if (robocar_pp_DW.obj_f.isInitialized == 1) {
      robocar_pp_DW.obj_f.TunablePropsChanged = true;
      robocar_pp_DW.obj_f.tunablePropertyChanged[1] = true;
    }

    robocar_pp_DW.obj_f.VehicleSpeedRange[0] =
      robocar_pp_P.BicycleKinematicModel_VehicleSp[0];
    robocar_pp_DW.obj_f.VehicleSpeedRange[1] =
      robocar_pp_P.BicycleKinematicModel_VehicleSp[1];
  }

  if (robocar_pp_DW.obj_f.MaxSteeringAngle !=
      robocar_pp_P.BicycleKinematicModel_MaxSteeri) {
    if (robocar_pp_DW.obj_f.isInitialized == 1) {
      robocar_pp_DW.obj_f.TunablePropsChanged = true;
      robocar_pp_DW.obj_f.tunablePropertyChanged[2] = true;
    }

    robocar_pp_DW.obj_f.MaxSteeringAngle =
      robocar_pp_P.BicycleKinematicModel_MaxSteeri;
  }

  if (robocar_pp_DW.obj_f.TunablePropsChanged) {
    robocar_pp_DW.obj_f.TunablePropsChanged = false;
    if (robocar_pp_DW.obj_f.tunablePropertyChanged[1]) {
      robocar_pp_DW.obj_f.KinModel.VehicleSpeedRange[0] =
        robocar_pp_DW.obj_f.VehicleSpeedRange[0];
      robocar_pp_DW.obj_f.KinModel.VehicleSpeedRange[1] =
        robocar_pp_DW.obj_f.VehicleSpeedRange[1];
    }

    if (robocar_pp_DW.obj_f.tunablePropertyChanged[0]) {
      robocar_pp_DW.obj_f.KinModel.WheelBase = robocar_pp_DW.obj_f.WheelBase;
    }

    if (robocar_pp_DW.obj_f.tunablePropertyChanged[2]) {
      robocar_pp_DW.obj_f.KinModel.MaxSteeringAngle =
        robocar_pp_DW.obj_f.MaxSteeringAngle;
    }

    robocar_pp_DW.obj_f.tunablePropertyChanged[0] = false;
    robocar_pp_DW.obj_f.tunablePropertyChanged[1] = false;
    robocar_pp_DW.obj_f.tunablePropertyChanged[2] = false;
  }

  /* Start for MATLABSystem: '<S1>/MATLAB System' */
  lookaheadStartPt[0] = robocar_pp_B.PurePursuit_o1;
  lookaheadStartPt[1] = robocar_pp_B.PurePursuit_o2;

  /* MATLABSystem: '<S1>/MATLAB System' */
  stateReshaped[2] = robocar_pp_B.Integrator[2];
  lookaheadEndPt_idx_0 = lookaheadStartPt[0];
  lookaheadEndPt_idx_1 = lookaheadStartPt[1];
  dist = fmin(fmax(lookaheadEndPt_idx_0,
                   robocar_pp_DW.obj_f.KinModel.VehicleSpeedRange[0]),
              robocar_pp_DW.obj_f.KinModel.VehicleSpeedRange[1]);
  minDistance = 0.0;
  ret = memcmp(&a[0], &robocar_pp_DW.obj_f.KinModel.VehicleInputsInternal[0], 25);
  if (ret == 0) {
    lookaheadIdx = 0.0;
  } else {
    ret = memcmp(&a_0[0], &robocar_pp_DW.obj_f.KinModel.VehicleInputsInternal[0],
                 25);
    if (ret == 0) {
      lookaheadIdx = 1.0;
    } else {
      lookaheadIdx = -1.0;
    }
  }

  switch ((int32_T)lookaheadIdx) {
   case 0:
    bicycleKinematics_getGeneralize(&robocar_pp_DW.obj_f.KinModel, dist,
      lookaheadEndPt_idx_1, &lookaheadIdx, &minDistance);
    dist = lookaheadIdx;
    break;

   case 1:
    lookaheadIdx = rt_atan2d_snf(lookaheadEndPt_idx_1 *
      robocar_pp_DW.obj_f.KinModel.WheelBase, lookaheadEndPt_idx_0);
    isInSecondQuadrant = (lookaheadIdx < -1.5707963267948966);
    searchFlag = !(lookaheadIdx > 1.5707963267948966);
    if ((!searchFlag) && (lookaheadIdx > 1.5707963267948966)) {
      lookaheadIdx -= 3.1415926535897931;
    }

    searchFlag = !isInSecondQuadrant;
    if ((!searchFlag) && isInSecondQuadrant) {
      lookaheadIdx += 3.1415926535897931;
    }

    bicycleKinematics_getGeneralize(&robocar_pp_DW.obj_f.KinModel,
      lookaheadEndPt_idx_0, lookaheadIdx, &dist, &minDistance);
    break;
  }

  for (b_k = 0; b_k < 6; b_k++) {
    J[b_k] = 0.0;
  }

  J[0] = cos(stateReshaped[2]);
  J[1] = sin(stateReshaped[2]);
  J[5] = 1.0;

  /* Start for MATLABSystem: '<S1>/MATLAB System' */
  lookaheadStartPt[0] = dist;
  lookaheadStartPt[1] = minDistance;
  minDistance = lookaheadStartPt[0];
  lookaheadIdx = lookaheadStartPt[1];

  /* MATLABSystem: '<S1>/MATLAB System' */
  for (ret = 0; ret < 3; ret++) {
    dist = J[ret] * minDistance;
    dist += J[ret + 3] * lookaheadIdx;
    stateReshaped[ret] = dist;
  }

  /* MATLABSystem: '<S1>/MATLAB System' */
  robocar_pp_B.MATLABSystem[0] = stateReshaped[0];
  robocar_pp_B.MATLABSystem[1] = stateReshaped[1];
  robocar_pp_B.MATLABSystem[2] = stateReshaped[2];
  if (rtmIsMajorTimeStep(robocar_pp_M)) {
    /* Matfile logging */
    rt_UpdateTXYLogVars(robocar_pp_M->rtwLogInfo, (robocar_pp_M->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(robocar_pp_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(robocar_pp_M)!=-1) &&
          !((rtmGetTFinal(robocar_pp_M)-(((robocar_pp_M->Timing.clockTick1+
               robocar_pp_M->Timing.clockTickH1* 4294967296.0)) * 0.001)) >
            (((robocar_pp_M->Timing.clockTick1+robocar_pp_M->Timing.clockTickH1*
               4294967296.0)) * 0.001) * (DBL_EPSILON))) {
        rtmSetErrorStatus(robocar_pp_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&robocar_pp_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++robocar_pp_M->Timing.clockTick0)) {
      ++robocar_pp_M->Timing.clockTickH0;
    }

    robocar_pp_M->Timing.t[0] = rtsiGetSolverStopTime(&robocar_pp_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      robocar_pp_M->Timing.clockTick1++;
      if (!robocar_pp_M->Timing.clockTick1) {
        robocar_pp_M->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void robocar_pp_derivatives(void)
{
  XDot_robocar_pp_T *_rtXdot;
  _rtXdot = ((XDot_robocar_pp_T *) robocar_pp_M->derivs);

  /* Derivatives for Integrator: '<S1>/Integrator' */
  _rtXdot->Integrator_CSTATE[0] = robocar_pp_B.MATLABSystem[0];
  _rtXdot->Integrator_CSTATE[1] = robocar_pp_B.MATLABSystem[1];
  _rtXdot->Integrator_CSTATE[2] = robocar_pp_B.MATLABSystem[2];
}

/* Model initialize function */
void robocar_pp_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)robocar_pp_M, 0,
                sizeof(RT_MODEL_robocar_pp_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&robocar_pp_M->solverInfo,
                          &robocar_pp_M->Timing.simTimeStep);
    rtsiSetTPtr(&robocar_pp_M->solverInfo, &rtmGetTPtr(robocar_pp_M));
    rtsiSetStepSizePtr(&robocar_pp_M->solverInfo,
                       &robocar_pp_M->Timing.stepSize0);
    rtsiSetdXPtr(&robocar_pp_M->solverInfo, &robocar_pp_M->derivs);
    rtsiSetContStatesPtr(&robocar_pp_M->solverInfo, (real_T **)
                         &robocar_pp_M->contStates);
    rtsiSetNumContStatesPtr(&robocar_pp_M->solverInfo,
      &robocar_pp_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&robocar_pp_M->solverInfo,
      &robocar_pp_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&robocar_pp_M->solverInfo,
      &robocar_pp_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&robocar_pp_M->solverInfo,
      &robocar_pp_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&robocar_pp_M->solverInfo, (boolean_T**)
      &robocar_pp_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&robocar_pp_M->solverInfo, (&rtmGetErrorStatus
      (robocar_pp_M)));
    rtsiSetRTModelPtr(&robocar_pp_M->solverInfo, robocar_pp_M);
  }

  rtsiSetSimTimeStep(&robocar_pp_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&robocar_pp_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&robocar_pp_M->solverInfo, false);
  robocar_pp_M->intgData.y = robocar_pp_M->odeY;
  robocar_pp_M->intgData.f[0] = robocar_pp_M->odeF[0];
  robocar_pp_M->intgData.f[1] = robocar_pp_M->odeF[1];
  robocar_pp_M->intgData.f[2] = robocar_pp_M->odeF[2];
  robocar_pp_M->contStates = ((X_robocar_pp_T *) &robocar_pp_X);
  robocar_pp_M->contStateDisabled = ((XDis_robocar_pp_T *) &robocar_pp_XDis);
  robocar_pp_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&robocar_pp_M->solverInfo, (void *)&robocar_pp_M->intgData);
  rtsiSetSolverName(&robocar_pp_M->solverInfo,"ode3");
  rtmSetTPtr(robocar_pp_M, &robocar_pp_M->Timing.tArray[0]);
  rtmSetTFinal(robocar_pp_M, 60.0);
  robocar_pp_M->Timing.stepSize0 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    robocar_pp_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(robocar_pp_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(robocar_pp_M->rtwLogInfo, (NULL));
    rtliSetLogT(robocar_pp_M->rtwLogInfo, "tout");
    rtliSetLogX(robocar_pp_M->rtwLogInfo, "");
    rtliSetLogXFinal(robocar_pp_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(robocar_pp_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(robocar_pp_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(robocar_pp_M->rtwLogInfo, 0);
    rtliSetLogDecimation(robocar_pp_M->rtwLogInfo, 1);
    rtliSetLogY(robocar_pp_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(robocar_pp_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(robocar_pp_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &robocar_pp_B), 0,
                sizeof(B_robocar_pp_T));

  /* states (continuous) */
  {
    (void) memset((void *)&robocar_pp_X, 0,
                  sizeof(X_robocar_pp_T));
  }

  /* disabled states */
  {
    (void) memset((void *)&robocar_pp_XDis, 0,
                  sizeof(XDis_robocar_pp_T));
  }

  /* states (dwork) */
  (void) memset((void *)&robocar_pp_DW, 0,
                sizeof(DW_robocar_pp_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(robocar_pp_M->rtwLogInfo, 0.0, rtmGetTFinal
    (robocar_pp_M), robocar_pp_M->Timing.stepSize0, (&rtmGetErrorStatus
    (robocar_pp_M)));

  {
    real_T b;
    int32_T i;

    /* InitializeConditions for Integrator: '<S1>/Integrator' */
    b = robocar_pp_P.BicycleKinematicModel_InitialSt[0];
    robocar_pp_X.Integrator_CSTATE[0] = b;
    b = robocar_pp_P.BicycleKinematicModel_InitialSt[1];
    robocar_pp_X.Integrator_CSTATE[1] = b;
    b = robocar_pp_P.BicycleKinematicModel_InitialSt[2];
    robocar_pp_X.Integrator_CSTATE[2] = b;

    /* Start for MATLABSystem: '<Root>/Pure Pursuit' */
    robocar_pp_DW.objisempty = true;
    robocar_pp_DW.obj.DesiredLinearVelocity =
      robocar_pp_P.PurePursuit_DesiredLinearVeloci;
    robocar_pp_DW.obj.MaxAngularVelocity =
      robocar_pp_P.PurePursuit_MaxAngularVelocity;
    robocar_pp_DW.obj.LookaheadDistance =
      robocar_pp_P.PurePursuit_LookaheadDistance;
    robocar_pp_DW.obj.isInitialized = 1;
    for (i = 0; i < 8; i++) {
      robocar_pp_DW.obj.WaypointsInternal[i] = (rtNaN);
    }

    /* InitializeConditions for MATLABSystem: '<Root>/Pure Pursuit' */
    robocar_pp_DW.obj.LookaheadPoint[0] = 0.0;
    robocar_pp_DW.obj.LookaheadPoint[1] = 0.0;
    robocar_pp_DW.obj.LastPose[0] = 0.0;
    robocar_pp_DW.obj.LastPose[1] = 0.0;
    robocar_pp_DW.obj.LastPose[2] = 0.0;
    robocar_pp_DW.obj.ProjectionPoint[0] = (rtNaN);
    robocar_pp_DW.obj.ProjectionPoint[1] = (rtNaN);
    robocar_pp_DW.obj.ProjectionLineIndex = 0.0;

    /* Start for MATLABSystem: '<S1>/MATLAB System' */
    robocar_pp_DW.obj_f.tunablePropertyChanged[0] = false;
    robocar_pp_DW.obj_f.tunablePropertyChanged[1] = false;
    robocar_pp_DW.obj_f.tunablePropertyChanged[2] = false;
    robocar_pp_DW.objisempty_g = true;
    robocar_pp_DW.obj_f.WheelBase = robocar_pp_P.wheel_base;
    robocar_pp_DW.obj_f.VehicleSpeedRange[0] =
      robocar_pp_P.BicycleKinematicModel_VehicleSp[0];
    robocar_pp_DW.obj_f.VehicleSpeedRange[1] =
      robocar_pp_P.BicycleKinematicModel_VehicleSp[1];
    robocar_pp_DW.obj_f.MaxSteeringAngle =
      robocar_pp_P.BicycleKinematicModel_MaxSteeri;
    robocar_pp_DW.obj_f.isInitialized = 1;
    rob_BicycleKinematics_setupImpl(&robocar_pp_DW.obj_f);
    robocar_pp_DW.obj_f.TunablePropsChanged = false;

    /* InitializeConditions for MATLABSystem: '<S1>/MATLAB System' */
    BicycleKinematics_assignModelPr(&robocar_pp_DW.obj_f);
  }
}

/* Model terminate function */
void robocar_pp_terminate(void)
{
  /* (no terminate code required) */
}
