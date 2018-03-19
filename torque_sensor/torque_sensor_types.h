/*
 * torque_sensor_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "torque_sensor".
 *
 * Model version              : 1.16
 * Simulink Coder version : 8.12 (R2017a) 16-Feb-2017
 * C++ source code generated on : Tue Aug 29 13:37:19 2017
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_torque_sensor_types_h_
#define RTW_HEADER_torque_sensor_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_torque_sensor_std_msgs_Float64_
#define DEFINED_TYPEDEF_FOR_SL_Bus_torque_sensor_std_msgs_Float64_

typedef struct {
  real_T Data;
} SL_Bus_torque_sensor_std_msgs_Float64;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_torque_sensor_std_msgs_String_
#define DEFINED_TYPEDEF_FOR_SL_Bus_torque_sensor_std_msgs_String_

typedef struct {
  uint8_T Data[128];
  SL_Bus_ROSVariableLengthArrayInfo Data_SL_Info;
} SL_Bus_torque_sensor_std_msgs_String;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_torque_sensor_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_torque_sensor_ros_time_Time_

typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_torque_sensor_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_torque_sensor_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_torque_sensor_std_msgs_Header_

typedef struct {
  uint32_T Seq;
  uint8_T FrameId[128];
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;
  SL_Bus_torque_sensor_ros_time_Time Stamp;
} SL_Bus_torque_sensor_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_torque_sensor_sensor_msgs_JointState_
#define DEFINED_TYPEDEF_FOR_SL_Bus_torque_sensor_sensor_msgs_JointState_

typedef struct {
  SL_Bus_torque_sensor_std_msgs_String Name[16];
  SL_Bus_ROSVariableLengthArrayInfo Name_SL_Info;
  real_T Position[128];
  SL_Bus_ROSVariableLengthArrayInfo Position_SL_Info;
  real_T Velocity[128];
  SL_Bus_ROSVariableLengthArrayInfo Velocity_SL_Info;
  real_T Effort[128];
  SL_Bus_ROSVariableLengthArrayInfo Effort_SL_Info;
  SL_Bus_torque_sensor_std_msgs_Header Header;
} SL_Bus_torque_sensor_sensor_msgs_JointState;

#endif

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_block_T;

#endif                                 /*typedef_robotics_slros_internal_block_T*/

#ifndef typedef_robotics_slros_internal_blo_a_T
#define typedef_robotics_slros_internal_blo_a_T

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_blo_a_T;

#endif                                 /*typedef_robotics_slros_internal_blo_a_T*/

#ifndef typedef_struct_T_torque_sensor_T
#define typedef_struct_T_torque_sensor_T

typedef struct {
  real_T f1[2];
} struct_T_torque_sensor_T;

#endif                                 /*typedef_struct_T_torque_sensor_T*/

#ifndef typedef_struct_T_torque_sensor_a_T
#define typedef_struct_T_torque_sensor_a_T

typedef struct {
  char_T f1[4];
} struct_T_torque_sensor_a_T;

#endif                                 /*typedef_struct_T_torque_sensor_a_T*/

#ifndef typedef_struct_T_torque_sensor_ax_T
#define typedef_struct_T_torque_sensor_ax_T

typedef struct {
  char_T f1[8];
} struct_T_torque_sensor_ax_T;

#endif                                 /*typedef_struct_T_torque_sensor_ax_T*/

#ifndef typedef_struct_T_torque_sensor_axg_T
#define typedef_struct_T_torque_sensor_axg_T

typedef struct {
  char_T f1[7];
} struct_T_torque_sensor_axg_T;

#endif                                 /*typedef_struct_T_torque_sensor_axg_T*/

#ifndef typedef_struct_T_torque_sensor_axg1_T
#define typedef_struct_T_torque_sensor_axg1_T

typedef struct {
  char_T f1[8];
  char_T f2[7];
  char_T f3[6];
} struct_T_torque_sensor_axg1_T;

#endif                                 /*typedef_struct_T_torque_sensor_axg1_T*/

/* Parameters (auto storage) */
typedef struct P_torque_sensor_T_ P_torque_sensor_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_torque_sensor_T RT_MODEL_torque_sensor_T;

#endif                                 /* RTW_HEADER_torque_sensor_types_h_ */
