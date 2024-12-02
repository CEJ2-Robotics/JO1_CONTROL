/*
 * robocar_pp_types.h
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

#ifndef robocar_pp_types_h_
#define robocar_pp_types_h_
#include "rtwtypes.h"
#ifndef struct_tag_xmlVIyDZou8I9iyJ03LCyD
#define struct_tag_xmlVIyDZou8I9iyJ03LCyD

struct tag_xmlVIyDZou8I9iyJ03LCyD
{
  real_T WheelBase;
  real_T VehicleSpeedRange[2];
  real_T MaxSteeringAngle;
  char_T VehicleInputsInternal[25];
};

#endif                                 /* struct_tag_xmlVIyDZou8I9iyJ03LCyD */

#ifndef typedef_c_bicycleKinematics_robocar_p_T
#define typedef_c_bicycleKinematics_robocar_p_T

typedef struct tag_xmlVIyDZou8I9iyJ03LCyD c_bicycleKinematics_robocar_p_T;

#endif                             /* typedef_c_bicycleKinematics_robocar_p_T */

#ifndef struct_tag_he9aNs1MdLdWPu3rDacNEC
#define struct_tag_he9aNs1MdLdWPu3rDacNEC

struct tag_he9aNs1MdLdWPu3rDacNEC
{
  boolean_T tunablePropertyChanged[3];
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  c_bicycleKinematics_robocar_p_T KinModel;
  real_T WheelBase;
  real_T VehicleSpeedRange[2];
  real_T MaxSteeringAngle;
};

#endif                                 /* struct_tag_he9aNs1MdLdWPu3rDacNEC */

#ifndef typedef_robotics_slmobile_internal_bl_T
#define typedef_robotics_slmobile_internal_bl_T

typedef struct tag_he9aNs1MdLdWPu3rDacNEC robotics_slmobile_internal_bl_T;

#endif                             /* typedef_robotics_slmobile_internal_bl_T */

#ifndef struct_tag_BlgwLpgj2bjudmbmVKWwDE
#define struct_tag_BlgwLpgj2bjudmbmVKWwDE

struct tag_BlgwLpgj2bjudmbmVKWwDE
{
  uint32_T f1[8];
};

#endif                                 /* struct_tag_BlgwLpgj2bjudmbmVKWwDE */

#ifndef typedef_cell_wrap_robocar_pp_T
#define typedef_cell_wrap_robocar_pp_T

typedef struct tag_BlgwLpgj2bjudmbmVKWwDE cell_wrap_robocar_pp_T;

#endif                                 /* typedef_cell_wrap_robocar_pp_T */

#ifndef struct_tag_CIbQxi4MXXEY3EQxc7PabH
#define struct_tag_CIbQxi4MXXEY3EQxc7PabH

struct tag_CIbQxi4MXXEY3EQxc7PabH
{
  int32_T isInitialized;
  cell_wrap_robocar_pp_T inputVarSize[2];
  real_T MaxAngularVelocity;
  real_T LookaheadDistance;
  real_T DesiredLinearVelocity;
  real_T ProjectionPoint[2];
  real_T ProjectionLineIndex;
  real_T LookaheadPoint[2];
  real_T LastPose[3];
  real_T WaypointsInternal[8];
};

#endif                                 /* struct_tag_CIbQxi4MXXEY3EQxc7PabH */

#ifndef typedef_nav_slalgs_internal_PurePursu_T
#define typedef_nav_slalgs_internal_PurePursu_T

typedef struct tag_CIbQxi4MXXEY3EQxc7PabH nav_slalgs_internal_PurePursu_T;

#endif                             /* typedef_nav_slalgs_internal_PurePursu_T */

#ifndef SS_UINT64
#define SS_UINT64                      19
#endif

#ifndef SS_INT64
#define SS_INT64                       20
#endif

/* Parameters (default storage) */
typedef struct P_robocar_pp_T_ P_robocar_pp_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_robocar_pp_T RT_MODEL_robocar_pp_T;

#endif                                 /* robocar_pp_types_h_ */
