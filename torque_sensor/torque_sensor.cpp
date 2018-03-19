/*
 * torque_sensor.cpp
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

#include "torque_sensor.h"
#include "torque_sensor_private.h"
#define torque_sensor_MessageQueueLen  (1)

/* Block signals (auto storage) */
B_torque_sensor_T torque_sensor_B;

/* Block states (auto storage) */
DW_torque_sensor_T torque_sensor_DW;

/* Real-time model */
RT_MODEL_torque_sensor_T torque_sensor_M_;
RT_MODEL_torque_sensor_T *const torque_sensor_M = &torque_sensor_M_;

/* Model step function */
void torque_sensor_step(void)
{
  /* local block i/o variables */
  real_T rtb_FromWs;
  real_T rtb_FromWs_o;
  boolean_T varargout_1;
  SL_Bus_torque_sensor_std_msgs_Float64 rtb_BusAssignment;

  /* FromWorkspace: '<S3>/FromWs' */
  {
    real_T *pDataValues = (real_T *) torque_sensor_DW.FromWs_PWORK.DataPtr;
    real_T *pTimeValues = (real_T *) torque_sensor_DW.FromWs_PWORK.TimePtr;
    int_T currTimeIndex = torque_sensor_DW.FromWs_IWORK.PrevIndex;
    real_T t = torque_sensor_M->Timing.t[0];

    /* Get index */
    if (t <= pTimeValues[0]) {
      currTimeIndex = 0;
    } else if (t >= pTimeValues[3]) {
      currTimeIndex = 2;
    } else {
      if (t < pTimeValues[currTimeIndex]) {
        while (t < pTimeValues[currTimeIndex]) {
          currTimeIndex--;
        }
      } else {
        while (t >= pTimeValues[currTimeIndex + 1]) {
          currTimeIndex++;
        }
      }
    }

    torque_sensor_DW.FromWs_IWORK.PrevIndex = currTimeIndex;

    /* Post output */
    {
      real_T t1 = pTimeValues[currTimeIndex];
      real_T t2 = pTimeValues[currTimeIndex + 1];
      if (t1 == t2) {
        if (t < t1) {
          rtb_FromWs = pDataValues[currTimeIndex];
        } else {
          rtb_FromWs = pDataValues[currTimeIndex + 1];
        }
      } else {
        real_T f1 = (t2 - t) / (t2 - t1);
        real_T f2 = 1.0 - f1;
        real_T d1;
        real_T d2;
        int_T TimeIndex= currTimeIndex;
        d1 = pDataValues[TimeIndex];
        d2 = pDataValues[TimeIndex + 1];
        rtb_FromWs = (real_T) rtInterpolate(d1, d2, f1, f2);
        pDataValues += 4;
      }
    }
  }

  /* FromWorkspace: '<S5>/FromWs' */
  {
    real_T *pDataValues = (real_T *) torque_sensor_DW.FromWs_PWORK_h.DataPtr;
    real_T *pTimeValues = (real_T *) torque_sensor_DW.FromWs_PWORK_h.TimePtr;
    int_T currTimeIndex = torque_sensor_DW.FromWs_IWORK_d.PrevIndex;
    real_T t = torque_sensor_M->Timing.t[0];

    /* Get index */
    if (t <= pTimeValues[0]) {
      currTimeIndex = 0;
    } else if (t >= pTimeValues[13]) {
      currTimeIndex = 12;
    } else {
      if (t < pTimeValues[currTimeIndex]) {
        while (t < pTimeValues[currTimeIndex]) {
          currTimeIndex--;
        }
      } else {
        while (t >= pTimeValues[currTimeIndex + 1]) {
          currTimeIndex++;
        }
      }
    }

    torque_sensor_DW.FromWs_IWORK_d.PrevIndex = currTimeIndex;

    /* Post output */
    {
      real_T t1 = pTimeValues[currTimeIndex];
      real_T t2 = pTimeValues[currTimeIndex + 1];
      if (t1 == t2) {
        if (t < t1) {
          rtb_FromWs_o = pDataValues[currTimeIndex];
        } else {
          rtb_FromWs_o = pDataValues[currTimeIndex + 1];
        }
      } else {
        real_T f1 = (t2 - t) / (t2 - t1);
        real_T f2 = 1.0 - f1;
        real_T d1;
        real_T d2;
        int_T TimeIndex= currTimeIndex;
        d1 = pDataValues[TimeIndex];
        d2 = pDataValues[TimeIndex + 1];
        rtb_FromWs_o = (real_T) rtInterpolate(d1, d2, f1, f2);
        pDataValues += 14;
      }
    }
  }

  /* ManualSwitch: '<Root>/Manual Switch' */
  if (torque_sensor_P.ManualSwitch_CurrentSetting == 1) {
    torque_sensor_B.ManualSwitch = rtb_FromWs;
  } else {
    torque_sensor_B.ManualSwitch = rtb_FromWs_o;
  }

  /* End of ManualSwitch: '<Root>/Manual Switch' */

  /* BusAssignment: '<Root>/Bus Assignment' */
  rtb_BusAssignment.Data = torque_sensor_B.ManualSwitch;

  /* Outputs for Atomic SubSystem: '<Root>/Publish' */
  /* Start for MATLABSystem: '<S2>/SinkBlock' incorporates:
   *  MATLABSystem: '<S2>/SinkBlock'
   */
  Pub_torque_sensor_2.publish(&rtb_BusAssignment);

  /* End of Outputs for SubSystem: '<Root>/Publish' */
  /* Outputs for Atomic SubSystem: '<Root>/Subscribe' */
  /* Start for MATLABSystem: '<S4>/SourceBlock' incorporates:
   *  Inport: '<S6>/In1'
   *  MATLABSystem: '<S4>/SourceBlock'
   */
  varargout_1 = Sub_torque_sensor_1.getLatestMessage
    (&torque_sensor_B.varargout_2);

  /* Outputs for Enabled SubSystem: '<S4>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S6>/Enable'
   */
  if (varargout_1) {
    torque_sensor_B.In1 = torque_sensor_B.varargout_2;
  }

  /* End of Start for MATLABSystem: '<S4>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S4>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<Root>/Subscribe' */

  /* SignalConversion: '<Root>/SigConversion_InsertedFor_Bus Selector_at_outport_0' */
  memcpy(&torque_sensor_B.Position[0], &torque_sensor_B.In1.Position[0], sizeof
         (real_T) << 7U);

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++torque_sensor_M->Timing.clockTick0)) {
    ++torque_sensor_M->Timing.clockTickH0;
  }

  torque_sensor_M->Timing.t[0] = torque_sensor_M->Timing.clockTick0 *
    torque_sensor_M->Timing.stepSize0 + torque_sensor_M->Timing.clockTickH0 *
    torque_sensor_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.005s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.005, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    torque_sensor_M->Timing.clockTick1++;
    if (!torque_sensor_M->Timing.clockTick1) {
      torque_sensor_M->Timing.clockTickH1++;
    }
  }
}

/* Model initialize function */
void torque_sensor_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)torque_sensor_M, 0,
                sizeof(RT_MODEL_torque_sensor_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&torque_sensor_M->solverInfo,
                          &torque_sensor_M->Timing.simTimeStep);
    rtsiSetTPtr(&torque_sensor_M->solverInfo, &rtmGetTPtr(torque_sensor_M));
    rtsiSetStepSizePtr(&torque_sensor_M->solverInfo,
                       &torque_sensor_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&torque_sensor_M->solverInfo, (&rtmGetErrorStatus
      (torque_sensor_M)));
    rtsiSetRTModelPtr(&torque_sensor_M->solverInfo, torque_sensor_M);
  }

  rtsiSetSimTimeStep(&torque_sensor_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&torque_sensor_M->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(torque_sensor_M, &torque_sensor_M->Timing.tArray[0]);
  torque_sensor_M->Timing.stepSize0 = 0.005;

  /* block I/O */
  (void) memset(((void *) &torque_sensor_B), 0,
                sizeof(B_torque_sensor_T));

  /* states (dwork) */
  (void) memset((void *)&torque_sensor_DW, 0,
                sizeof(DW_torque_sensor_T));

  {
    static const char_T tmp[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's', 't',
      'a', 't', 'e', 's' };

    static const char_T tmp_0[13] = { '/', 't', 'o', 'r', 'q', 'u', 'e', 'S',
      'e', 'n', 's', 'o', 'r' };

    char_T tmp_1[14];
    int32_T i;

    /* Start for FromWorkspace: '<S3>/FromWs' */
    {
      static real_T pTimeValues0[] = { 0.0, 4.0, 4.0, 10000.0 } ;

      static real_T pDataValues0[] = { 0.0, -0.0, -0.0, 10.0 } ;

      torque_sensor_DW.FromWs_PWORK.TimePtr = (void *) pTimeValues0;
      torque_sensor_DW.FromWs_PWORK.DataPtr = (void *) pDataValues0;
      torque_sensor_DW.FromWs_IWORK.PrevIndex = 0;
    }

    /* Start for FromWorkspace: '<S5>/FromWs' */
    {
      static real_T pTimeValues0[] = { 0.0, 4.0, 4.0, 2500.0, 2500.0, 3680.0,
        3680.0, 5000.0, 5000.0, 6800.0, 6800.0, 8680.0, 8680.0, 10000.0 } ;

      static real_T pDataValues0[] = { 0.0, -0.0, -0.0, 4.4, 4.4,
        15.100000000000001, 15.100000000000001, 3.9000000000000004,
        3.9000000000000004, 14.600000000000001, 14.600000000000001, 0.4, 0.4,
        16.0 } ;

      torque_sensor_DW.FromWs_PWORK_h.TimePtr = (void *) pTimeValues0;
      torque_sensor_DW.FromWs_PWORK_h.DataPtr = (void *) pDataValues0;
      torque_sensor_DW.FromWs_IWORK_d.PrevIndex = 0;
    }

    /* Start for Atomic SubSystem: '<Root>/Publish' */
    /* Start for MATLABSystem: '<S2>/SinkBlock' */
    torque_sensor_DW.obj.isInitialized = 0;
    torque_sensor_DW.objisempty_h = true;
    torque_sensor_DW.obj.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_1[i] = tmp_0[i];
    }

    tmp_1[13] = '\x00';
    Pub_torque_sensor_2.createPublisher(tmp_1, torque_sensor_MessageQueueLen);

    /* End of Start for MATLABSystem: '<S2>/SinkBlock' */
    /* End of Start for SubSystem: '<Root>/Publish' */
    /* Start for Atomic SubSystem: '<Root>/Subscribe' */
    /* Start for MATLABSystem: '<S4>/SourceBlock' */
    torque_sensor_DW.obj_n.isInitialized = 0;
    torque_sensor_DW.objisempty = true;
    torque_sensor_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_1[i] = tmp[i];
    }

    tmp_1[13] = '\x00';
    Sub_torque_sensor_1.createSubscriber(tmp_1, torque_sensor_MessageQueueLen);

    /* End of Start for MATLABSystem: '<S4>/SourceBlock' */
    /* End of Start for SubSystem: '<Root>/Subscribe' */
  }

  /* SystemInitialize for Atomic SubSystem: '<Root>/Subscribe' */
  /* SystemInitialize for Enabled SubSystem: '<S4>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S6>/Out1' */
  torque_sensor_B.In1 = torque_sensor_P.Out1_Y0;

  /* End of SystemInitialize for SubSystem: '<S4>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<Root>/Subscribe' */
}

/* Model terminate function */
void torque_sensor_terminate(void)
{
  /* Terminate for Atomic SubSystem: '<Root>/Publish' */
  /* Start for MATLABSystem: '<S2>/SinkBlock' incorporates:
   *  Terminate for MATLABSystem: '<S2>/SinkBlock'
   */
  if (torque_sensor_DW.obj.isInitialized == 1) {
    torque_sensor_DW.obj.isInitialized = 2;
  }

  /* End of Start for MATLABSystem: '<S2>/SinkBlock' */
  /* End of Terminate for SubSystem: '<Root>/Publish' */

  /* Terminate for Atomic SubSystem: '<Root>/Subscribe' */
  /* Start for MATLABSystem: '<S4>/SourceBlock' incorporates:
   *  Terminate for MATLABSystem: '<S4>/SourceBlock'
   */
  if (torque_sensor_DW.obj_n.isInitialized == 1) {
    torque_sensor_DW.obj_n.isInitialized = 2;
  }

  /* End of Start for MATLABSystem: '<S4>/SourceBlock' */
  /* End of Terminate for SubSystem: '<Root>/Subscribe' */
}
