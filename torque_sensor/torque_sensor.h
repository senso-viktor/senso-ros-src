/*
 * torque_sensor.h
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

#ifndef RTW_HEADER_torque_sensor_h_
#define RTW_HEADER_torque_sensor_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef torque_sensor_COMMON_INCLUDES_
# define torque_sensor_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#endif                                 /* torque_sensor_COMMON_INCLUDES_ */

#include "torque_sensor_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

/* Block signals (auto storage) */
typedef struct {
  SL_Bus_torque_sensor_sensor_msgs_JointState In1;/* '<S6>/In1' */
  SL_Bus_torque_sensor_sensor_msgs_JointState varargout_2;
  real_T Position[128];
  real_T ManualSwitch;                 /* '<Root>/Manual Switch' */
} B_torque_sensor_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWs_PWORK;                      /* '<S3>/FromWs' */

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWs_PWORK_h;                    /* '<S5>/FromWs' */

  struct {
    void *LoggedData;
  } torquesensor_PWORK;                /* '<Root>/torque sensor' */

  struct {
    void *LoggedData;
  } jointstates_PWORK;                 /* '<Root>/joint states' */

  void *SourceBlock_PWORK;             /* '<S4>/SourceBlock' */
  void *SinkBlock_PWORK;               /* '<S2>/SinkBlock' */
  robotics_slros_internal_block_T obj; /* '<S2>/SinkBlock' */
  robotics_slros_internal_blo_a_T obj_n;/* '<S4>/SourceBlock' */
  struct {
    int_T PrevIndex;
  } FromWs_IWORK;                      /* '<S3>/FromWs' */

  struct {
    int_T PrevIndex;
  } FromWs_IWORK_d;                    /* '<S5>/FromWs' */

  boolean_T objisempty;                /* '<S4>/SourceBlock' */
  boolean_T objisempty_h;              /* '<S2>/SinkBlock' */
} DW_torque_sensor_T;

/* Parameters (auto storage) */
struct P_torque_sensor_T_ {
  SL_Bus_torque_sensor_sensor_msgs_JointState Out1_Y0;/* Computed Parameter: Out1_Y0
                                                       * Referenced by: '<S6>/Out1'
                                                       */
  SL_Bus_torque_sensor_sensor_msgs_JointState Constant_Value;/* Computed Parameter: Constant_Value
                                                              * Referenced by: '<S4>/Constant'
                                                              */
  SL_Bus_torque_sensor_std_msgs_Float64 Constant_Value_d;/* Computed Parameter: Constant_Value_d
                                                          * Referenced by: '<S1>/Constant'
                                                          */
  uint8_T ManualSwitch_CurrentSetting; /* Computed Parameter: ManualSwitch_CurrentSetting
                                        * Referenced by: '<Root>/Manual Switch'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_torque_sensor_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    SimTimeStep simTimeStep;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (auto storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern P_torque_sensor_T torque_sensor_P;

#ifdef __cplusplus

}
#endif

/* Block signals (auto storage) */
extern B_torque_sensor_T torque_sensor_B;

/* Block states (auto storage) */
extern DW_torque_sensor_T torque_sensor_DW;

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

#ifdef __cplusplus

extern "C" {

#endif

  /* Model entry point functions */
  extern void torque_sensor_initialize(void);
  extern void torque_sensor_step(void);
  extern void torque_sensor_terminate(void);

#ifdef __cplusplus

}
#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_torque_sensor_T *const torque_sensor_M;

#ifdef __cplusplus

}
#endif

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'torque_sensor'
 * '<S1>'   : 'torque_sensor/Blank Message'
 * '<S2>'   : 'torque_sensor/Publish'
 * '<S3>'   : 'torque_sensor/ROS node'
 * '<S4>'   : 'torque_sensor/Subscribe'
 * '<S5>'   : 'torque_sensor/real scara'
 * '<S6>'   : 'torque_sensor/Subscribe/Enabled Subsystem'
 */
#endif                                 /* RTW_HEADER_torque_sensor_h_ */
