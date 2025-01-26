/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LateralController.h
 *
 * Code generated for Simulink model 'LateralController'.
 *
 * Model version                  : 1.6
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Sun Jan 26 14:48:33 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. RAM efficiency
 *    2. Execution efficiency
 * Validation result: Not run
 */

#ifndef LateralController_h_
#define LateralController_h_
#ifndef LateralController_COMMON_INCLUDES_
#define LateralController_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* LateralController_COMMON_INCLUDES_ */

#include "rtw_modelmap.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm)         ((rtm)->DataMapInfo)
#endif

#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val)    ((rtm)->DataMapInfo = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

#ifndef SS_UINT64
#define SS_UINT64                      21
#endif

#ifndef SS_INT64
#define SS_INT64                       22
#endif

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real32_T LastPcov_PreviousInput[36]; /* '<S4>/LastPcov' */
  real32_T last_x_PreviousInput[6];    /* '<S4>/last_x' */
  real32_T c_Hv[3720];
  real32_T Su[1800];
  real32_T last_mv_DSTATE;             /* '<S4>/last_mv' */
  real32_T DiscreteTimeIntegrator_DSTATE;/* '<S1>/Discrete-Time Integrator' */
  boolean_T Memory_PreviousInput[8];   /* '<S4>/Memory' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: lastPcov
   * Referenced by: '<S4>/LastPcov'
   */
  real32_T LastPcov_InitialCondition[36];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T lateral_deviation;          /* '<Root>/lateral_deviation' */
  real32_T relative_yaw_angle;         /* '<Root>/relative_yaw_angle' */
  real32_T curvature;                  /* '<Root>/curvature' */
  real32_T longitudinalvelocity;       /* '<Root>/velocity' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T steering_angle;             /* '<Root>/steering_angle' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;

  /*
   * DataMapInfo:
   * The following substructure contains information regarding
   * structures generated in the model's C API.
   */
  struct {
    rtwCAPI_ModelMappingInfo mmi;
  } DataMapInfo;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern real_T L;                       /* Variable: L
                                        * Referenced by: '<S1>/L'
                                        */

/* Model entry point functions */
extern void LateralController_initialize(void);
extern void LateralController_step(void);

/* Function to get C API Model Mapping Static Info */
extern const rtwCAPI_ModelMappingStaticInfo*
  LateralController_GetCAPIStaticMap(void);

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
 * Block '<S4>/u_scale' : Eliminated nontunable gain of 1
 * Block '<S4>/umin_scale4' : Eliminated nontunable gain of 1
 * Block '<S4>/uref_scale' : Eliminated nontunable gain of 1
 * Block '<S4>/ymin_scale2' : Eliminated nontunable gain of 1
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
 * hilite_system('Lateral_vehicle_MPC_controller/LateralController')    - opens subsystem Lateral_vehicle_MPC_controller/LateralController
 * hilite_system('Lateral_vehicle_MPC_controller/LateralController/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Lateral_vehicle_MPC_controller'
 * '<S1>'   : 'Lateral_vehicle_MPC_controller/LateralController'
 * '<S2>'   : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller'
 * '<S3>'   : 'Lateral_vehicle_MPC_controller/LateralController/MATLAB Function'
 * '<S4>'   : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC'
 * '<S5>'   : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check'
 * '<S6>'   : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check A'
 * '<S7>'   : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check B'
 * '<S8>'   : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check C'
 * '<S9>'   : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check D'
 * '<S10>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check DX'
 * '<S11>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check U'
 * '<S12>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check X'
 * '<S13>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check Y'
 * '<S14>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check1'
 * '<S15>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Matrix Signal Check2'
 * '<S16>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check'
 * '<S17>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check1'
 * '<S18>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check2'
 * '<S19>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check3'
 * '<S20>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check4'
 * '<S21>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check5'
 * '<S22>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check6'
 * '<S23>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check7'
 * '<S24>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Preview Signal Check8'
 * '<S25>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Scalar Signal Check'
 * '<S26>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Scalar Signal Check1'
 * '<S27>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Scalar Signal Check2'
 * '<S28>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Vector Signal Check'
 * '<S29>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Vector Signal Check1'
 * '<S30>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/MPC Vector Signal Check6'
 * '<S31>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/moorx'
 * '<S32>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/optimizer'
 * '<S33>'  : 'Lateral_vehicle_MPC_controller/LateralController/Adaptive MPC Controller/MPC/optimizer/FixedHorizonOptimizer'
 */
#endif                                 /* LateralController_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
