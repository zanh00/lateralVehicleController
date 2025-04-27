/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LateralController.h
 *
 * Code generated for Simulink model 'LateralController'.
 *
 * Model version                  : 1.21
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Sun Apr 27 19:44:56 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef LateralController_h_
#define LateralController_h_
#ifndef LateralController_COMMON_INCLUDES_
#define LateralController_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* LateralController_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real32_T last_x_PreviousInput[5];    /* '<S4>/last_x' */
  real32_T LastPcov_PreviousInput[25]; /* '<S4>/LastPcov' */
  real32_T fv[1932];
  real32_T c_Hv[2520];
  real32_T Su[1200];
  real32_T last_mv_DSTATE;             /* '<S4>/last_mv' */
  real32_T UnitDelay_DSTATE;           /* '<S1>/Unit Delay' */
  boolean_T Memory_PreviousInput[46];  /* '<S4>/Memory' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: lastPcov
   * Referenced by: '<S4>/LastPcov'
   */
  real32_T LastPcov_InitialCondition[25];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T lateral_deviation;          /* '<Root>/lateral_deviation' */
  real32_T relative_yaw_angle;         /* '<Root>/relative_yaw_angle' */
  real32_T curvature;                  /* '<Root>/curvature' */
  real32_T velocity_sim;               /* '<Root>/velocity' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T steering_angle;             /* '<Root>/steering_angle' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void LateralController_initialize(void);
extern void LateralController_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S4>/Constant' : Unused code path elimination
 * Block '<S4>/Floor' : Unused code path elimination
 * Block '<S4>/Floor1' : Unused code path elimination
 * Block '<S5>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S6>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S7>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S8>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S9>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S10>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S11>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S12>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S13>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S14>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S15>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S16>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S17>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S18>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S19>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S20>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S21>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S22>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S23>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S24>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S25>/Vector Dimension Check' : Unused code path elimination
 * Block '<S26>/Vector Dimension Check' : Unused code path elimination
 * Block '<S27>/Vector Dimension Check' : Unused code path elimination
 * Block '<S28>/Vector Dimension Check' : Unused code path elimination
 * Block '<S29>/Vector Dimension Check' : Unused code path elimination
 * Block '<S30>/Vector Dimension Check' : Unused code path elimination
 * Block '<S4>/Min' : Unused code path elimination
 * Block '<S31>/Vector Dimension Check' : Unused code path elimination
 * Block '<S4>/u_scale' : Unused code path elimination
 * Block '<S4>/useq_scale' : Unused code path elimination
 * Block '<S4>/useq_scale1' : Unused code path elimination
 * Block '<S4>/ym_zero' : Unused code path elimination
 * Block '<S2>/m_zero' : Unused code path elimination
 * Block '<S2>/p_zero' : Unused code path elimination
 * Block '<S4>/Reshape' : Reshape block reduction
 * Block '<S4>/Reshape1' : Reshape block reduction
 * Block '<S4>/Reshape2' : Reshape block reduction
 * Block '<S4>/Reshape3' : Reshape block reduction
 * Block '<S4>/Reshape4' : Reshape block reduction
 * Block '<S4>/Reshape5' : Reshape block reduction
 * Block '<S4>/ext.mv_scale' : Eliminated nontunable gain of 1
 * Block '<S4>/umin_scale4' : Eliminated nontunable gain of 1
 * Block '<S4>/uref_scale' : Eliminated nontunable gain of 1
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('model_V2slx/LateralController')    - opens subsystem model_V2slx/LateralController
 * hilite_system('model_V2slx/LateralController/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'model_V2slx'
 * '<S1>'   : 'model_V2slx/LateralController'
 * '<S2>'   : 'model_V2slx/LateralController/Adaptive MPC Controller'
 * '<S3>'   : 'model_V2slx/LateralController/MATLAB Function'
 * '<S4>'   : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC'
 * '<S5>'   : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check'
 * '<S6>'   : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check A'
 * '<S7>'   : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check B'
 * '<S8>'   : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check C'
 * '<S9>'   : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check D'
 * '<S10>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check DX'
 * '<S11>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check U'
 * '<S12>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check X'
 * '<S13>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check Y'
 * '<S14>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check1'
 * '<S15>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check2'
 * '<S16>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check'
 * '<S17>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check1'
 * '<S18>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check2'
 * '<S19>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check3'
 * '<S20>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check4'
 * '<S21>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check5'
 * '<S22>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check6'
 * '<S23>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check7'
 * '<S24>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check8'
 * '<S25>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Scalar Signal Check'
 * '<S26>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Scalar Signal Check1'
 * '<S27>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Scalar Signal Check2'
 * '<S28>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Vector Signal Check'
 * '<S29>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Vector Signal Check1'
 * '<S30>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/MPC Vector Signal Check6'
 * '<S31>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/moorx'
 * '<S32>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/optimizer'
 * '<S33>'  : 'model_V2slx/LateralController/Adaptive MPC Controller/MPC/optimizer/FixedHorizonOptimizer'
 */
#endif                                 /* LateralController_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
