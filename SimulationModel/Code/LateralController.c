/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LateralController.c
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

#include "LateralController.h"
#include "rtwtypes.h"
#include <string.h>
#include <math.h>
#include "LateralController_capi.h"
#include "math.h"

/* Named constants for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
#define Wdu                            (0.04F)
#define degrees                        (5)
#define p                              (30)

/* Exported block parameters */
real_T L = 0.2;                        /* Variable: L
                                        * Referenced by: '<S1>/L'
                                        */

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real32_T rt_powf_snf(real32_T u0, real32_T u1);
extern real32_T rt_hypotf_snf(real32_T u0, real32_T u1);
static int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator);

/* Forward declaration for local functions */
static int32_T xpotrf(real32_T b_A[25]);
static real32_T minimum(const real32_T x[5]);
static void trisolve(const real32_T b_A[25], real32_T b_B[25]);
static real32_T norm(const real32_T x[5]);
static real32_T maximum(const real32_T x[5]);
static real32_T xnrm2(int32_T n, const real32_T x[25], int32_T ix0);
static void xgemv(int32_T b_m, int32_T n, const real32_T b_A[25], int32_T ia0,
                  const real32_T x[25], int32_T ix0, real32_T y[5]);
static void xgerc(int32_T b_m, int32_T n, real32_T alpha1, int32_T ix0, const
                  real32_T y[5], real32_T b_A[25], int32_T ia0);
static real32_T KWIKfactor(const real32_T b_Ac[40], const int32_T iC[8], int32_T
  nA, const real32_T b_Linv[25], real32_T RLinv[25], real32_T b_D[25], real32_T
  b_H[25], int32_T n);
static void DropConstraint(int32_T kDrop, boolean_T iA[8], int32_T *nA, int32_T
  iC[8]);
static void qpkwik(const real32_T b_Linv[25], const real32_T b_Hinv[25], const
                   real32_T f[5], const real32_T b_Ac[40], const real32_T b[8],
                   boolean_T iA[8], int32_T maxiter, real32_T FeasTol, real32_T
                   x[5], real32_T lambda[8], int32_T *status);
static void mpcblock_optimizer(const real32_T rseq[60], const real32_T vseq[62],
  const real32_T x[6], real32_T old_u, const boolean_T iA[8], const real32_T
  b_Mlim[8], real32_T b_Mx[48], real32_T b_Mu1[8], real32_T b_Mv[496], const
  real32_T b_utarget[30], real32_T b_uoff, real32_T b_H[25], real32_T b_Ac[40],
  const real32_T b_Wy[2], const real32_T b_Jm[120], const real32_T b_I1[30],
  const real32_T b_A[36], const real32_T Bu[186], const real32_T Bv[372], const
  real32_T b_C[12], const real32_T Dv[124], const int32_T b_Mrows[8], real32_T
  *u, real32_T useq[31], real32_T *status, boolean_T iAout[8]);
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);
extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
real_T rtNaN = -(real_T)NAN;
real_T rtInf = (real_T)INFINITY;
real_T rtMinusInf = -(real_T)INFINITY;
real32_T rtNaNF = -(real32_T)NAN;
real32_T rtInfF = (real32_T)INFINITY;
real32_T rtMinusInfF = -(real32_T)INFINITY;

/* Return rtInf needed by the generated code. */
static real_T rtGetInf(void)
{
  return rtInf;
}

/* Get rtInfF needed by the generated code. */
static real32_T rtGetInfF(void)
{
  return rtInfF;
}

/* Return rtMinusInf needed by the generated code. */
static real_T rtGetMinusInf(void)
{
  return rtMinusInf;
}

/* Return rtMinusInfF needed by the generated code. */
static real32_T rtGetMinusInfF(void)
{
  return rtMinusInfF;
}

/* Return rtNaN needed by the generated code. */
static real_T rtGetNaN(void)
{
  return rtNaN;
}

/* Return rtNaNF needed by the generated code. */
static real32_T rtGetNaNF(void)
{
  return rtNaNF;
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)isinf(value);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)isinf(value);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  return (boolean_T)(isnan(value) != 0);
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  return (boolean_T)(isnan(value) != 0);
}

static int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static int32_T xpotrf(real32_T b_A[25])
{
  int32_T b_k;
  int32_T info;
  int32_T j;
  int32_T jm1;
  boolean_T exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 5)) {
    int32_T idxAjj;
    real32_T c;
    real32_T ssq;
    idxAjj = j * 5 + j;
    ssq = 0.0F;
    if (j >= 1) {
      for (b_k = 0; b_k < j; b_k++) {
        c = b_A[b_k * 5 + j];
        ssq += c * c;
      }
    }

    ssq = b_A[idxAjj] - ssq;
    if (ssq > 0.0F) {
      ssq = sqrtf(ssq);
      b_A[idxAjj] = ssq;
      if (j + 1 < 5) {
        if (j != 0) {
          int32_T b_iy;
          b_iy = ((j - 1) * 5 + j) + 2;
          for (b_k = j + 2; b_k <= b_iy; b_k += 5) {
            int32_T d;
            jm1 = b_k - j;
            c = -b_A[div_nde_s32_floor(jm1 - 2, 5) * 5 + j];
            d = jm1 + 3;
            for (jm1 = b_k; jm1 <= d; jm1++) {
              int32_T tmp;
              tmp = ((idxAjj + jm1) - b_k) + 1;
              b_A[tmp] += b_A[jm1 - 1] * c;
            }
          }
        }

        ssq = 1.0F / ssq;
        jm1 = (idxAjj - j) + 5;
        for (b_k = idxAjj + 2; b_k <= jm1; b_k++) {
          b_A[b_k - 1] *= ssq;
        }
      }

      j++;
    } else {
      b_A[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }

  return info;
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static real32_T minimum(const real32_T x[5])
{
  int32_T idx;
  int32_T k;
  real32_T ex;
  if (!rtIsNaNF(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 6)) {
      if (!rtIsNaNF(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 6; k++) {
      real32_T x_0;
      x_0 = x[k - 1];
      if (ex > x_0) {
        ex = x_0;
      }
    }
  }

  return ex;
}

real32_T rt_powf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else {
    real32_T tmp;
    real32_T tmp_0;
    tmp = fabsf(u0);
    tmp_0 = fabsf(u1);
    if (rtIsInfF(u1)) {
      if (tmp == 1.0F) {
        y = 1.0F;
      } else if (tmp > 1.0F) {
        if (u1 > 0.0F) {
          y = (rtInfF);
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = (rtInfF);
      }
    } else if (tmp_0 == 0.0F) {
      y = 1.0F;
    } else if (tmp_0 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = sqrtf(u0);
    } else if ((u0 < 0.0F) && (u1 > floorf(u1))) {
      y = (rtNaNF);
    } else {
      y = powf(u0, u1);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void trisolve(const real32_T b_A[25], real32_T b_B[25])
{
  int32_T b_k;
  int32_T i;
  int32_T j;
  for (j = 0; j < 5; j++) {
    int32_T jBcol;
    jBcol = 5 * j;
    for (b_k = 0; b_k < 5; b_k++) {
      int32_T b_B_tmp;
      int32_T kAcol;
      real32_T b_B_0;
      kAcol = 5 * b_k;
      b_B_tmp = b_k + jBcol;
      b_B_0 = b_B[b_B_tmp];
      if (b_B_0 != 0.0F) {
        b_B[b_B_tmp] = b_B_0 / b_A[b_k + kAcol];
        for (i = b_k + 2; i < 6; i++) {
          int32_T tmp;
          tmp = (i + jBcol) - 1;
          b_B[tmp] -= b_A[(i + kAcol) - 1] * b_B[b_B_tmp];
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static real32_T norm(const real32_T x[5])
{
  int32_T k;
  real32_T scale;
  real32_T y;
  y = 0.0F;
  scale = 1.29246971E-26F;
  for (k = 0; k < 5; k++) {
    real32_T absxk;
    absxk = fabsf(x[k]);
    if (absxk > scale) {
      real32_T t;
      t = scale / absxk;
      y = y * t * t + 1.0F;
      scale = absxk;
    } else {
      real32_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrtf(y);
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static real32_T maximum(const real32_T x[5])
{
  int32_T idx;
  int32_T k;
  real32_T ex;
  if (!rtIsNaNF(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 6)) {
      if (!rtIsNaNF(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 6; k++) {
      real32_T x_0;
      x_0 = x[k - 1];
      if (ex < x_0) {
        ex = x_0;
      }
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static real32_T xnrm2(int32_T n, const real32_T x[25], int32_T ix0)
{
  int32_T k;
  real32_T y;
  y = 0.0F;
  if (n >= 1) {
    if (n == 1) {
      y = fabsf(x[ix0 - 1]);
    } else {
      int32_T kend;
      real32_T scale;
      scale = 1.29246971E-26F;
      kend = ix0 + n;
      for (k = ix0; k < kend; k++) {
        real32_T absxk;
        absxk = fabsf(x[k - 1]);
        if (absxk > scale) {
          real32_T t;
          t = scale / absxk;
          y = y * t * t + 1.0F;
          scale = absxk;
        } else {
          real32_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrtf(y);
    }
  }

  return y;
}

real32_T rt_hypotf_snf(real32_T u0, real32_T u1)
{
  real32_T a;
  real32_T b;
  real32_T y;
  a = fabsf(u0);
  b = fabsf(u1);
  if (a < b) {
    a /= b;
    y = sqrtf(a * a + 1.0F) * b;
  } else if (a > b) {
    b /= a;
    y = sqrtf(b * b + 1.0F) * a;
  } else if (rtIsNaNF(b)) {
    y = (rtNaNF);
  } else {
    y = a * 1.41421354F;
  }

  return y;
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void xgemv(int32_T b_m, int32_T n, const real32_T b_A[25], int32_T ia0,
                  const real32_T x[25], int32_T ix0, real32_T y[5])
{
  int32_T b_iy;
  int32_T ia;
  if ((b_m != 0) && (n != 0)) {
    int32_T b;
    if (n - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)n * sizeof(real32_T));
    }

    b = (n - 1) * 5 + ia0;
    for (b_iy = ia0; b_iy <= b; b_iy += 5) {
      int32_T d;
      real32_T c;
      c = 0.0F;
      d = (b_iy + b_m) - 1;
      for (ia = b_iy; ia <= d; ia++) {
        c += x[((ix0 + ia) - b_iy) - 1] * b_A[ia - 1];
      }

      ia = div_nde_s32_floor(b_iy - ia0, 5);
      y[ia] += c;
    }
  }
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void xgerc(int32_T b_m, int32_T n, real32_T alpha1, int32_T ix0, const
                  real32_T y[5], real32_T b_A[25], int32_T ia0)
{
  int32_T ijA;
  int32_T j;
  if (!(alpha1 == 0.0F)) {
    int32_T jA;
    jA = ia0;
    for (j = 0; j < n; j++) {
      real32_T temp;
      temp = y[j];
      if (temp != 0.0F) {
        int32_T b;
        temp *= alpha1;
        b = (b_m + jA) - 1;
        for (ijA = jA; ijA <= b; ijA++) {
          b_A[ijA - 1] += b_A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 5;
    }
  }
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static real32_T KWIKfactor(const real32_T b_Ac[40], const int32_T iC[8], int32_T
  nA, const real32_T b_Linv[25], real32_T RLinv[25], real32_T b_D[25], real32_T
  b_H[25], int32_T n)
{
  int32_T b_coltop;
  int32_T b_lastv;
  int32_T coltop;
  int32_T exitg1;
  int32_T ii;
  int32_T k_i;
  int32_T knt;
  real32_T R[25];
  real32_T TL[25];
  real32_T b_A[25];
  real32_T tau[5];
  real32_T work[5];
  real32_T RLinv_0;
  real32_T Status;
  real32_T b_A_0;
  real32_T beta1;
  boolean_T exitg2;
  Status = 1.0F;
  memset(&RLinv[0], 0, 25U * sizeof(real32_T));
  for (k_i = 0; k_i < nA; k_i++) {
    b_lastv = iC[k_i];
    for (b_coltop = 0; b_coltop < 5; b_coltop++) {
      RLinv_0 = 0.0F;
      for (knt = 0; knt < 5; knt++) {
        RLinv_0 += b_Ac[((knt << 3) + b_lastv) - 1] * b_Linv[5 * knt + b_coltop];
      }

      RLinv[b_coltop + 5 * k_i] = RLinv_0;
    }
  }

  memcpy(&b_A[0], &RLinv[0], 25U * sizeof(real32_T));
  for (k_i = 0; k_i < 5; k_i++) {
    tau[k_i] = 0.0F;
    work[k_i] = 0.0F;
  }

  for (k_i = 0; k_i < 5; k_i++) {
    ii = k_i * 5 + k_i;
    if (k_i + 1 < 5) {
      RLinv_0 = b_A[ii];
      b_lastv = ii + 2;
      tau[k_i] = 0.0F;
      beta1 = xnrm2(4 - k_i, b_A, ii + 2);
      if (beta1 != 0.0F) {
        b_A_0 = b_A[ii];
        beta1 = rt_hypotf_snf(b_A_0, beta1);
        if (b_A_0 >= 0.0F) {
          beta1 = -beta1;
        }

        if (fabsf(beta1) < 9.86076132E-32F) {
          knt = 0;
          coltop = (ii - k_i) + 5;
          do {
            knt++;
            for (b_coltop = b_lastv; b_coltop <= coltop; b_coltop++) {
              b_A[b_coltop - 1] *= 1.01412048E+31F;
            }

            beta1 *= 1.01412048E+31F;
            RLinv_0 *= 1.01412048E+31F;
          } while ((fabsf(beta1) < 9.86076132E-32F) && (knt < 20));

          beta1 = rt_hypotf_snf(RLinv_0, xnrm2(4 - k_i, b_A, ii + 2));
          if (RLinv_0 >= 0.0F) {
            beta1 = -beta1;
          }

          tau[k_i] = (beta1 - RLinv_0) / beta1;
          RLinv_0 = 1.0F / (RLinv_0 - beta1);
          for (b_coltop = b_lastv; b_coltop <= coltop; b_coltop++) {
            b_A[b_coltop - 1] *= RLinv_0;
          }

          for (b_lastv = 0; b_lastv < knt; b_lastv++) {
            beta1 *= 9.86076132E-32F;
          }

          RLinv_0 = beta1;
        } else {
          tau[k_i] = (beta1 - b_A_0) / beta1;
          RLinv_0 = 1.0F / (b_A_0 - beta1);
          b_coltop = (ii - k_i) + 5;
          for (knt = b_lastv; knt <= b_coltop; knt++) {
            b_A[knt - 1] *= RLinv_0;
          }

          RLinv_0 = beta1;
        }
      }

      b_A[ii] = 1.0F;
      if (tau[k_i] != 0.0F) {
        b_lastv = 5 - k_i;
        knt = (ii - k_i) + 4;
        while ((b_lastv > 0) && (b_A[knt] == 0.0F)) {
          b_lastv--;
          knt--;
        }

        knt = 4 - k_i;
        exitg2 = false;
        while ((!exitg2) && (knt > 0)) {
          b_coltop = ((knt - 1) * 5 + ii) + 5;
          coltop = b_coltop;
          do {
            exitg1 = 0;
            if (coltop + 1 <= b_coltop + b_lastv) {
              if (b_A[coltop] != 0.0F) {
                exitg1 = 1;
              } else {
                coltop++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        b_lastv = 0;
        knt = 0;
      }

      if (b_lastv > 0) {
        xgemv(b_lastv, knt, b_A, ii + 6, b_A, ii + 1, work);
        xgerc(b_lastv, knt, -tau[k_i], ii + 1, work, b_A, ii + 6);
      }

      b_A[ii] = RLinv_0;
    } else {
      tau[4] = 0.0F;
    }
  }

  for (k_i = 0; k_i < 5; k_i++) {
    for (ii = 0; ii <= k_i; ii++) {
      R[ii + 5 * k_i] = b_A[5 * k_i + ii];
    }

    for (ii = k_i + 2; ii < 6; ii++) {
      R[(ii + 5 * k_i) - 1] = 0.0F;
    }

    work[k_i] = 0.0F;
  }

  for (k_i = 4; k_i >= 0; k_i--) {
    b_lastv = (k_i * 5 + k_i) + 6;
    if (k_i + 1 < 5) {
      b_A[b_lastv - 6] = 1.0F;
      if (tau[k_i] != 0.0F) {
        knt = 5 - k_i;
        b_coltop = b_lastv - k_i;
        while ((knt > 0) && (b_A[b_coltop - 2] == 0.0F)) {
          knt--;
          b_coltop--;
        }

        b_coltop = 4 - k_i;
        exitg2 = false;
        while ((!exitg2) && (b_coltop > 0)) {
          coltop = (b_coltop - 1) * 5 + b_lastv;
          ii = coltop;
          do {
            exitg1 = 0;
            if (ii <= (coltop + knt) - 1) {
              if (b_A[ii - 1] != 0.0F) {
                exitg1 = 1;
              } else {
                ii++;
              }
            } else {
              b_coltop--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        knt = 0;
        b_coltop = 0;
      }

      if (knt > 0) {
        xgemv(knt, b_coltop, b_A, b_lastv, b_A, b_lastv - 5, work);
        xgerc(knt, b_coltop, -tau[k_i], b_lastv - 5, work, b_A, b_lastv);
      }

      b_coltop = b_lastv - k_i;
      for (knt = b_lastv - 4; knt < b_coltop; knt++) {
        b_A[knt - 1] *= -tau[k_i];
      }
    }

    b_A[b_lastv - 6] = 1.0F - tau[k_i];
    for (knt = 0; knt < k_i; knt++) {
      b_A[(b_lastv - knt) - 7] = 0.0F;
    }
  }

  k_i = 0;
  do {
    exitg1 = 0;
    if (k_i <= nA - 1) {
      if (fabsf(R[5 * k_i + k_i]) < 1.0E-12F) {
        Status = -2.0F;
        exitg1 = 1;
      } else {
        k_i++;
      }
    } else {
      for (k_i = 0; k_i < n; k_i++) {
        for (ii = 0; ii < n; ii++) {
          RLinv_0 = 0.0F;
          for (b_coltop = 0; b_coltop < 5; b_coltop++) {
            RLinv_0 += b_Linv[5 * k_i + b_coltop] * b_A[5 * ii + b_coltop];
          }

          TL[k_i + 5 * ii] = RLinv_0;
        }
      }

      memset(&RLinv[0], 0, 25U * sizeof(real32_T));
      for (k_i = nA; k_i >= 1; k_i--) {
        b_coltop = (k_i - 1) * 5;
        knt = (k_i + b_coltop) - 1;
        RLinv[knt] = 1.0F;
        for (ii = k_i; ii <= nA; ii++) {
          coltop = ((ii - 1) * 5 + k_i) - 1;
          RLinv[coltop] /= R[knt];
        }

        if (k_i > 1) {
          for (ii = 0; ii <= k_i - 2; ii++) {
            for (b_lastv = k_i; b_lastv <= nA; b_lastv++) {
              knt = (b_lastv - 1) * 5;
              coltop = knt + ii;
              RLinv[coltop] -= RLinv[(knt + k_i) - 1] * R[b_coltop + ii];
            }
          }
        }
      }

      for (k_i = 0; k_i < n; k_i++) {
        for (ii = k_i + 1; ii <= n; ii++) {
          b_coltop = (ii - 1) * 5 + k_i;
          b_H[b_coltop] = 0.0F;
          for (b_lastv = nA + 1; b_lastv <= n; b_lastv++) {
            knt = (b_lastv - 1) * 5;
            b_H[b_coltop] -= TL[(knt + ii) - 1] * TL[knt + k_i];
          }

          b_H[(ii + 5 * k_i) - 1] = b_H[b_coltop];
        }
      }

      for (k_i = 0; k_i < nA; k_i++) {
        for (ii = 0; ii < n; ii++) {
          b_coltop = 5 * k_i + ii;
          b_D[b_coltop] = 0.0F;
          for (b_lastv = k_i + 1; b_lastv <= nA; b_lastv++) {
            knt = (b_lastv - 1) * 5;
            b_D[b_coltop] += TL[knt + ii] * RLinv[knt + k_i];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return Status;
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void DropConstraint(int32_T kDrop, boolean_T iA[8], int32_T *nA, int32_T
  iC[8])
{
  int32_T i;
  if (kDrop > 0) {
    iA[iC[kDrop - 1] - 1] = false;
    if (kDrop < *nA) {
      for (i = kDrop; i < *nA; i++) {
        iC[i - 1] = iC[i];
      }
    }

    iC[*nA - 1] = 0;
    (*nA)--;
  }
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void qpkwik(const real32_T b_Linv[25], const real32_T b_Hinv[25], const
                   real32_T f[5], const real32_T b_Ac[40], const real32_T b[8],
                   boolean_T iA[8], int32_T maxiter, real32_T FeasTol, real32_T
                   x[5], real32_T lambda[8], int32_T *status)
{
  int32_T iC[8];
  int32_T b_exponent;
  int32_T exitg1;
  int32_T exitg3;
  int32_T exponent;
  int32_T i;
  int32_T iC_0;
  int32_T iSave;
  int32_T kDrop;
  int32_T nA;
  real32_T RLinv[25];
  real32_T U[25];
  real32_T b_D[25];
  real32_T b_H[25];
  real32_T Opt[10];
  real32_T Rhs[10];
  real32_T cTol[8];
  real32_T r[5];
  real32_T z[5];
  real32_T Xnorm0;
  real32_T cMin;
  real32_T cVal;
  real32_T rMin;
  real32_T t;
  boolean_T ColdReset;
  boolean_T DualFeasible;
  boolean_T cTolComputed;
  boolean_T exitg2;
  boolean_T exitg4;
  boolean_T guard1;
  boolean_T guard2;
  for (i = 0; i < 5; i++) {
    x[i] = 0.0F;
  }

  for (i = 0; i < 8; i++) {
    lambda[i] = 0.0F;
  }

  *status = 1;
  for (i = 0; i < 5; i++) {
    r[i] = 0.0F;
  }

  rMin = 0.0F;
  cTolComputed = false;
  for (i = 0; i < 8; i++) {
    cTol[i] = 1.0F;
    iC[i] = 0;
  }

  nA = 0;
  for (i = 0; i < 8; i++) {
    if (iA[i]) {
      nA++;
      iC[nA - 1] = i + 1;
    }
  }

  guard1 = false;
  if (nA > 0) {
    for (i = 0; i < 10; i++) {
      Opt[i] = 0.0F;
    }

    for (i = 0; i < 5; i++) {
      Rhs[i] = f[i];
      Rhs[i + 5] = 0.0F;
    }

    DualFeasible = false;
    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (nA > 0) && (*status <= maxiter)) {
        Xnorm0 = KWIKfactor(b_Ac, iC, nA, b_Linv, RLinv, b_D, b_H, degrees);
        if (Xnorm0 < 0.0F) {
          if (ColdReset) {
            *status = -2;
            exitg3 = 2;
          } else {
            nA = 0;
            for (i = 0; i < 8; i++) {
              iA[i] = false;
              iC[i] = 0;
            }

            ColdReset = true;
          }
        } else {
          for (i = 0; i < nA; i++) {
            Rhs[i + 5] = b[iC[i] - 1];
            for (kDrop = i + 1; kDrop <= nA; kDrop++) {
              iC_0 = (5 * i + kDrop) - 1;
              U[iC_0] = 0.0F;
              for (iSave = 0; iSave < nA; iSave++) {
                U[iC_0] += RLinv[(5 * iSave + kDrop) - 1] * RLinv[5 * iSave + i];
              }

              U[i + 5 * (kDrop - 1)] = U[iC_0];
            }
          }

          for (i = 0; i < 5; i++) {
            Xnorm0 = 0.0F;
            for (iC_0 = 0; iC_0 < 5; iC_0++) {
              Xnorm0 += b_H[5 * iC_0 + i] * Rhs[iC_0];
            }

            Opt[i] = Xnorm0;
            for (kDrop = 0; kDrop < nA; kDrop++) {
              Opt[i] += b_D[5 * kDrop + i] * Rhs[kDrop + 5];
            }
          }

          for (i = 0; i < nA; i++) {
            Xnorm0 = 0.0F;
            for (iC_0 = 0; iC_0 < 5; iC_0++) {
              Xnorm0 += b_D[5 * i + iC_0] * Rhs[iC_0];
            }

            Opt[i + 5] = Xnorm0;
            for (kDrop = 0; kDrop < nA; kDrop++) {
              Opt[i + 5] += U[5 * kDrop + i] * Rhs[kDrop + 5];
            }
          }

          Xnorm0 = -1.0E-12F;
          kDrop = -1;
          for (i = 0; i < nA; i++) {
            cMin = Opt[i + 5];
            lambda[iC[i] - 1] = cMin;
            if ((cMin < Xnorm0) && (i + 1 <= nA)) {
              kDrop = i;
              Xnorm0 = cMin;
            }
          }

          if (kDrop + 1 <= 0) {
            DualFeasible = true;
            for (i = 0; i < 5; i++) {
              x[i] = Opt[i];
            }
          } else {
            (*status)++;
            if (*status > 5) {
              nA = 0;
              for (i = 0; i < 8; i++) {
                iA[i] = false;
                iC[i] = 0;
              }

              ColdReset = true;
            } else {
              lambda[iC[kDrop] - 1] = 0.0F;
              DropConstraint(kDrop + 1, iA, &nA, iC);
            }
          }
        }
      } else {
        if (nA <= 0) {
          for (i = 0; i < 8; i++) {
            lambda[i] = 0.0F;
          }

          for (i = 0; i < 5; i++) {
            Xnorm0 = 0.0F;
            for (iC_0 = 0; iC_0 < 5; iC_0++) {
              Xnorm0 += -b_Hinv[5 * iC_0 + i] * f[iC_0];
            }

            x[i] = Xnorm0;
          }
        }

        exitg3 = 1;
      }
    } while (exitg3 == 0);

    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    for (i = 0; i < 5; i++) {
      Xnorm0 = 0.0F;
      for (iC_0 = 0; iC_0 < 5; iC_0++) {
        Xnorm0 += -b_Hinv[5 * iC_0 + i] * f[iC_0];
      }

      x[i] = Xnorm0;
    }

    guard1 = true;
  }

  if (guard1) {
    Xnorm0 = norm(x);
    exitg2 = false;
    while ((!exitg2) && (*status <= maxiter)) {
      cMin = -FeasTol;
      i = -1;
      for (kDrop = 0; kDrop < 8; kDrop++) {
        if (!cTolComputed) {
          for (iSave = 0; iSave < 5; iSave++) {
            z[iSave] = fabsf(b_Ac[(iSave << 3) + kDrop] * x[iSave]);
          }

          cTol[kDrop] = fmaxf(cTol[kDrop], maximum(z));
        }

        if (!iA[kDrop]) {
          t = 0.0F;
          for (iC_0 = 0; iC_0 < 5; iC_0++) {
            t += b_Ac[(iC_0 << 3) + kDrop] * x[iC_0];
          }

          cVal = (t - b[kDrop]) / cTol[kDrop];
          if (cVal < cMin) {
            cMin = cVal;
            i = kDrop;
          }
        }
      }

      cTolComputed = true;
      if (i + 1 <= 0) {
        exitg2 = true;
      } else if (*status == maxiter) {
        *status = 0;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0;
          if ((i + 1 > 0) && (*status <= maxiter)) {
            guard2 = false;
            if (nA == 0) {
              for (iC_0 = 0; iC_0 < 5; iC_0++) {
                cMin = 0.0F;
                for (kDrop = 0; kDrop < 5; kDrop++) {
                  cMin += b_Hinv[5 * kDrop + iC_0] * b_Ac[(kDrop << 3) + i];
                }

                z[iC_0] = cMin;
              }

              guard2 = true;
            } else {
              cMin = KWIKfactor(b_Ac, iC, nA, b_Linv, RLinv, b_D, b_H, degrees);
              if (cMin <= 0.0F) {
                *status = -2;
                exitg1 = 1;
              } else {
                for (iC_0 = 0; iC_0 < 25; iC_0++) {
                  U[iC_0] = -b_H[iC_0];
                }

                for (iC_0 = 0; iC_0 < 5; iC_0++) {
                  cMin = 0.0F;
                  for (kDrop = 0; kDrop < 5; kDrop++) {
                    cMin += U[5 * kDrop + iC_0] * b_Ac[(kDrop << 3) + i];
                  }

                  z[iC_0] = cMin;
                }

                for (kDrop = 0; kDrop < nA; kDrop++) {
                  t = 0.0F;
                  for (iC_0 = 0; iC_0 < 5; iC_0++) {
                    t += b_Ac[(iC_0 << 3) + i] * b_D[5 * kDrop + iC_0];
                  }

                  r[kDrop] = t;
                }

                guard2 = true;
              }
            }

            if (guard2) {
              kDrop = 0;
              cMin = 0.0F;
              DualFeasible = true;
              ColdReset = true;
              if (nA > 0) {
                iSave = 0;
                exitg4 = false;
                while ((!exitg4) && (iSave <= nA - 1)) {
                  if (r[iSave] >= 1.0E-12F) {
                    ColdReset = false;
                    exitg4 = true;
                  } else {
                    iSave++;
                  }
                }
              }

              if ((nA != 0) && (!ColdReset)) {
                for (iSave = 0; iSave < nA; iSave++) {
                  cVal = r[iSave];
                  if (cVal > 1.0E-12F) {
                    cVal = lambda[iC[iSave] - 1] / cVal;
                    if ((kDrop == 0) || (cVal < rMin)) {
                      rMin = cVal;
                      kDrop = iSave + 1;
                    }
                  }
                }

                if (kDrop > 0) {
                  cMin = rMin;
                  DualFeasible = false;
                }
              }

              cVal = 0.0F;
              for (iSave = 0; iSave < 5; iSave++) {
                cVal += b_Ac[(iSave << 3) + i] * z[iSave];
              }

              if (cVal <= 0.0F) {
                cVal = 0.0F;
                ColdReset = true;
              } else {
                t = 0.0F;
                for (iC_0 = 0; iC_0 < 5; iC_0++) {
                  t += b_Ac[(iC_0 << 3) + i] * x[iC_0];
                }

                cVal = (b[i] - t) / cVal;
                ColdReset = false;
              }

              if (DualFeasible && ColdReset) {
                *status = -1;
                exitg1 = 1;
              } else {
                if (ColdReset) {
                  t = cMin;
                } else if (DualFeasible) {
                  t = cVal;
                } else if (cMin < cVal) {
                  t = cMin;
                } else {
                  t = cVal;
                }

                for (iSave = 0; iSave < nA; iSave++) {
                  iC_0 = iC[iSave];
                  lambda[iC_0 - 1] -= t * r[iSave];
                  if ((iC_0 <= 8) && (lambda[iC_0 - 1] < 0.0F)) {
                    lambda[iC_0 - 1] = 0.0F;
                  }
                }

                lambda[i] += t;
                frexpf(1.0F, &exponent);
                if (fabsf(t - cMin) < 1.1920929E-7F) {
                  DropConstraint(kDrop, iA, &nA, iC);
                }

                if (!ColdReset) {
                  for (iC_0 = 0; iC_0 < 5; iC_0++) {
                    x[iC_0] += t * z[iC_0];
                  }

                  frexpf(1.0F, &b_exponent);
                  if (fabsf(t - cVal) < 1.1920929E-7F) {
                    if (nA == degrees) {
                      *status = -1;
                      exitg1 = 1;
                    } else {
                      nA++;
                      iC[nA - 1] = i + 1;
                      kDrop = nA - 1;
                      exitg4 = false;
                      while ((!exitg4) && (kDrop + 1 > 1)) {
                        iC_0 = iC[kDrop - 1];
                        if (iC[kDrop] > iC_0) {
                          exitg4 = true;
                        } else {
                          iSave = iC[kDrop];
                          iC[kDrop] = iC_0;
                          iC[kDrop - 1] = iSave;
                          kDrop--;
                        }
                      }

                      iA[i] = true;
                      i = -1;
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                } else {
                  (*status)++;
                }
              }
            }
          } else {
            cMin = norm(x);
            if (fabsf(cMin - Xnorm0) > 0.001F) {
              Xnorm0 = cMin;
              for (i = 0; i < 8; i++) {
                cTol[i] = fmaxf(fabsf(b[i]), 1.0F);
              }

              cTolComputed = false;
            }

            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void mpcblock_optimizer(const real32_T rseq[60], const real32_T vseq[62],
  const real32_T x[6], real32_T old_u, const boolean_T iA[8], const real32_T
  b_Mlim[8], real32_T b_Mx[48], real32_T b_Mu1[8], real32_T b_Mv[496], const
  real32_T b_utarget[30], real32_T b_uoff, real32_T b_H[25], real32_T b_Ac[40],
  const real32_T b_Wy[2], const real32_T b_Jm[120], const real32_T b_I1[30],
  const real32_T b_A[36], const real32_T Bu[186], const real32_T Bv[372], const
  real32_T b_C[12], const real32_T Dv[124], const int32_T b_Mrows[8], real32_T
  *u, real32_T useq[31], real32_T *status, boolean_T iAout[8])
{
  int32_T CA_tmp;
  int32_T b_Jm_tmp;
  int32_T i;
  int32_T i1;
  int32_T kidx;
  real32_T c_Sx[360];
  real32_T c_Kv[248];
  real32_T WySuJm[240];
  real32_T c_SuJm[240];
  real32_T CA_0[124];
  real32_T I2Jm[120];
  real32_T WduJm[120];
  real32_T WuI2Jm[120];
  real32_T Sum_0[60];
  real32_T c_Su1[60];
  real32_T L_0[25];
  real32_T c_Kx[24];
  real32_T CA[12];
  real32_T CA_1[12];
  real32_T b_Mlim_0[8];
  real32_T b_Mlim_1[8];
  real32_T b_Mv_0[8];
  real32_T varargin_1[5];
  real32_T zopt[5];
  real32_T b_C_0[4];
  real32_T Sum[2];
  real32_T WuI2Jm_0;
  real32_T b_Jm_0;
  real32_T normH;
  real32_T s;
  int16_T ixw;
  int8_T a[900];
  int8_T b[25];
  int8_T rows[2];
  int8_T kidx_0;
  int8_T rows_0;
  static const int8_T c_A[900] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
    1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1 };

  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T guard2;
  memset(&useq[0], 0, 31U * sizeof(real32_T));
  for (i = 0; i < 8; i++) {
    iAout[i] = false;
  }

  for (i1 = 0; i1 < 2; i1++) {
    Sum[i1] = 0.0F;
    for (i = 0; i < 6; i++) {
      normH = 0.0F;
      for (CA_tmp = 0; CA_tmp < 6; CA_tmp++) {
        normH += b_C[(CA_tmp << 1) + i1] * b_A[6 * i + CA_tmp];
      }

      CA_tmp = (i << 1) + i1;
      CA[CA_tmp] = normH;
      Sum[i1] += b_C[CA_tmp] * Bu[i];
    }

    for (i = 0; i < 2; i++) {
      normH = 0.0F;
      for (CA_tmp = 0; CA_tmp < 6; CA_tmp++) {
        normH += b_C[(CA_tmp << 1) + i1] * Bv[6 * i + CA_tmp];
      }

      b_C_0[i1 + (i << 1)] = normH;
    }
  }

  rtDW.c_Hv[0] = b_C_0[0];
  rtDW.c_Hv[120] = Dv[0];
  rtDW.c_Hv[1] = b_C_0[1];
  rtDW.c_Hv[121] = Dv[1];
  rtDW.c_Hv[60] = b_C_0[2];
  rtDW.c_Hv[180] = Dv[2];
  rtDW.c_Hv[61] = b_C_0[3];
  rtDW.c_Hv[181] = Dv[3];
  for (i1 = 0; i1 < 58; i1++) {
    i = (i1 + 4) * 60;
    rtDW.c_Hv[i] = 0.0F;
    rtDW.c_Hv[i + 1] = 0.0F;
  }

  for (i1 = 0; i1 < 62; i1++) {
    memset(&rtDW.c_Hv[i1 * 60 + 2], 0, 58U * sizeof(real32_T));
  }

  for (i1 = 0; i1 < 6; i1++) {
    i = i1 << 1;
    c_Sx[60 * i1] = CA[i];
    c_Sx[60 * i1 + 1] = CA[i + 1];
    memset(&c_Sx[i1 * 60 + 2], 0, 58U * sizeof(real32_T));
  }

  c_Su1[0] = Sum[0];
  c_Su1[1] = Sum[1];
  memset(&c_Su1[2], 0, 58U * sizeof(real32_T));
  rtDW.Su[0] = Sum[0];
  rtDW.Su[1] = Sum[1];
  for (i1 = 0; i1 < 29; i1++) {
    i = (i1 + 1) * 60;
    rtDW.Su[i] = 0.0F;
    rtDW.Su[i + 1] = 0.0F;
  }

  for (i1 = 0; i1 < 30; i1++) {
    memset(&rtDW.Su[i1 * 60 + 2], 0, 58U * sizeof(real32_T));
  }

  for (kidx = 0; kidx < 29; kidx++) {
    kidx_0 = (int8_T)(((kidx + 1) << 1) + 1);
    for (i1 = 0; i1 < 2; i1++) {
      rows_0 = (int8_T)(i1 + kidx_0);
      rows[i1] = rows_0;
      normH = 0.0F;
      for (i = 0; i < 6; i++) {
        normH += CA[(i << 1) + i1] * Bu[i];
      }

      normH += Sum[i1];
      Sum[i1] = normH;
      c_Su1[rows_0 - 1] = normH;
      Sum_0[i1] = normH;
    }

    rows_0 = rows[0];
    kidx_0 = rows[1];
    for (i1 = 0; i1 < 29; i1++) {
      i = (i1 + 1) << 1;
      Sum_0[i] = rtDW.Su[(60 * i1 + rows_0) - 3];
      Sum_0[i + 1] = rtDW.Su[(60 * i1 + kidx_0) - 3];
    }

    rows_0 = rows[0];
    kidx_0 = rows[1];
    for (i1 = 0; i1 < 30; i1++) {
      i = i1 << 1;
      rtDW.Su[(rows_0 + 60 * i1) - 1] = Sum_0[i];
      rtDW.Su[(kidx_0 + 60 * i1) - 1] = Sum_0[i + 1];
    }

    for (i1 = 0; i1 < 2; i1++) {
      for (i = 0; i < 2; i++) {
        normH = 0.0F;
        for (CA_tmp = 0; CA_tmp < 6; CA_tmp++) {
          normH += CA[(CA_tmp << 1) + i1] * Bv[6 * i + CA_tmp];
        }

        b_C_0[i1 + (i << 1)] = normH;
      }
    }

    CA_0[0] = b_C_0[0];
    CA_0[1] = b_C_0[1];
    CA_0[2] = b_C_0[2];
    CA_0[3] = b_C_0[3];
    rows_0 = rows[0];
    kidx_0 = rows[1];
    for (i1 = 0; i1 < 60; i1++) {
      CA_tmp = (i1 + 2) << 1;
      CA_0[CA_tmp] = rtDW.c_Hv[(60 * i1 + rows_0) - 3];
      CA_0[CA_tmp + 1] = rtDW.c_Hv[(60 * i1 + kidx_0) - 3];
    }

    rows_0 = rows[0];
    kidx_0 = rows[1];
    for (i1 = 0; i1 < 62; i1++) {
      i = i1 << 1;
      rtDW.c_Hv[(rows_0 + 60 * i1) - 1] = CA_0[i];
      rtDW.c_Hv[(kidx_0 + 60 * i1) - 1] = CA_0[i + 1];
    }

    for (i1 = 0; i1 < 2; i1++) {
      for (i = 0; i < 6; i++) {
        normH = 0.0F;
        for (CA_tmp = 0; CA_tmp < 6; CA_tmp++) {
          normH += CA[(CA_tmp << 1) + i1] * b_A[6 * i + CA_tmp];
        }

        CA_1[i1 + (i << 1)] = normH;
      }
    }

    for (i1 = 0; i1 < 12; i1++) {
      CA[i1] = CA_1[i1];
    }

    rows_0 = rows[0];
    kidx_0 = rows[1];
    for (i1 = 0; i1 < 6; i1++) {
      i = i1 << 1;
      c_Sx[(rows_0 + 60 * i1) - 1] = CA[i];
      c_Sx[(kidx_0 + 60 * i1) - 1] = CA[i + 1];
    }
  }

  for (i1 = 0; i1 < 4; i1++) {
    for (i = 0; i < 60; i++) {
      s = 0.0F;
      for (CA_tmp = 0; CA_tmp < 30; CA_tmp++) {
        s += rtDW.Su[60 * CA_tmp + i] * b_Jm[30 * i1 + CA_tmp];
      }

      c_SuJm[i + 60 * i1] = s;
    }
  }

  if (b_Mrows[0] > 0) {
    kidx = 0;
    exitg1 = false;
    while ((!exitg1) && (kidx < 8)) {
      if (b_Mrows[kidx] <= 60) {
        i = b_Mrows[kidx];
        b_Ac[kidx] = -c_SuJm[i - 1];
        b_Ac[kidx + 8] = -c_SuJm[i + 59];
        b_Ac[kidx + 16] = -c_SuJm[i + 119];
        b_Ac[kidx + 24] = -c_SuJm[i + 179];
        i = b_Mrows[kidx];
        for (i1 = 0; i1 < 6; i1++) {
          b_Mx[kidx + (i1 << 3)] = -c_Sx[(60 * i1 + i) - 1];
        }

        b_Mu1[kidx] = -c_Su1[b_Mrows[kidx] - 1];
        i = b_Mrows[kidx];
        for (i1 = 0; i1 < 62; i1++) {
          b_Mv[kidx + (i1 << 3)] = -rtDW.c_Hv[(60 * i1 + i) - 1];
        }

        kidx++;
      } else if (b_Mrows[kidx] <= 120) {
        i = b_Mrows[kidx];
        b_Ac[kidx] = c_SuJm[i - 61];
        b_Ac[kidx + 8] = c_SuJm[i - 1];
        b_Ac[kidx + 16] = c_SuJm[i + 59];
        b_Ac[kidx + 24] = c_SuJm[i + 119];
        i = b_Mrows[kidx];
        for (i1 = 0; i1 < 6; i1++) {
          b_Mx[kidx + (i1 << 3)] = c_Sx[(60 * i1 + i) - 61];
        }

        b_Mu1[kidx] = c_Su1[b_Mrows[kidx] - 61];
        i = b_Mrows[kidx];
        for (i1 = 0; i1 < 62; i1++) {
          b_Mv[kidx + (i1 << 3)] = rtDW.c_Hv[(60 * i1 + i) - 61];
        }

        kidx++;
      } else {
        exitg1 = true;
      }
    }
  }

  kidx = -1;
  for (i = 0; i < 30; i++) {
    for (i1 = 0; i1 < 30; i1++) {
      a[(kidx + i1) + 1] = c_A[30 * i + i1];
    }

    kidx += 30;
  }

  for (i1 = 0; i1 < 4; i1++) {
    for (i = 0; i < 30; i++) {
      normH = 0.0F;
      for (CA_tmp = 0; CA_tmp < 30; CA_tmp++) {
        normH += (real32_T)a[30 * CA_tmp + i] * b_Jm[30 * i1 + CA_tmp];
      }

      I2Jm[i + 30 * i1] = normH;
    }
  }

  ixw = 1;
  for (kidx = 0; kidx < 60; kidx++) {
    normH = b_Wy[ixw - 1];
    WySuJm[kidx] = normH * c_SuJm[kidx];
    WySuJm[kidx + 60] = c_SuJm[kidx + 60] * normH;
    WySuJm[kidx + 120] = c_SuJm[kidx + 120] * normH;
    WySuJm[kidx + 180] = c_SuJm[kidx + 180] * normH;
    ixw++;
    if (ixw > 2) {
      ixw = 1;
    }
  }

  for (i1 = 0; i1 < 120; i1++) {
    WuI2Jm[i1] = I2Jm[i1];
    WduJm[i1] = b_Jm[i1] * Wdu;
  }

  for (i1 = 0; i1 < 4; i1++) {
    for (i = 0; i < 4; i++) {
      s = 0.0F;
      for (CA_tmp = 0; CA_tmp < 60; CA_tmp++) {
        s += c_SuJm[60 * i1 + CA_tmp] * WySuJm[60 * i + CA_tmp];
      }

      normH = 0.0F;
      b_Jm_0 = 0.0F;
      for (CA_tmp = 0; CA_tmp < 30; CA_tmp++) {
        kidx = 30 * i1 + CA_tmp;
        b_Jm_tmp = 30 * i + CA_tmp;
        b_Jm_0 += b_Jm[kidx] * WduJm[b_Jm_tmp];
        normH += I2Jm[kidx] * WuI2Jm[b_Jm_tmp];
      }

      b_H[i1 + 5 * i] = (s + b_Jm_0) + normH;
    }

    normH = 0.0F;
    for (i = 0; i < 60; i++) {
      normH += WySuJm[60 * i1 + i] * c_Su1[i];
    }

    s = 0.0F;
    for (i = 0; i < 30; i++) {
      s += WuI2Jm[30 * i1 + i] * b_I1[i];
    }

    b_C_0[i1] = normH + s;
  }

  for (i1 = 0; i1 < 120; i1++) {
    WuI2Jm[i1] = -WuI2Jm[i1];
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i = 0; i < 4; i++) {
      normH = 0.0F;
      for (CA_tmp = 0; CA_tmp < 60; CA_tmp++) {
        normH += c_Sx[60 * i1 + CA_tmp] * WySuJm[60 * i + CA_tmp];
      }

      c_Kx[i1 + 6 * i] = normH;
    }
  }

  for (i1 = 0; i1 < 62; i1++) {
    for (i = 0; i < 4; i++) {
      s = 0.0F;
      for (CA_tmp = 0; CA_tmp < 60; CA_tmp++) {
        s += rtDW.c_Hv[60 * i1 + CA_tmp] * WySuJm[60 * i + CA_tmp];
      }

      c_Kv[i1 + 62 * i] = s;
    }
  }

  for (i1 = 0; i1 < 240; i1++) {
    WySuJm[i1] = -WySuJm[i1];
  }

  kidx = 0;
  memcpy(&L_0[0], &b_H[0], 25U * sizeof(real32_T));
  i = xpotrf(L_0);
  guard1 = false;
  if (i == 0) {
    for (i = 0; i < 5; i++) {
      varargin_1[i] = L_0[5 * i + i];
    }

    if (minimum(varargin_1) > 1.49011612E-7F) {
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    normH = 0.0F;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i < 5)) {
      s = 0.0F;
      for (i1 = 0; i1 < 5; i1++) {
        s += fabsf(b_H[5 * i1 + i]);
      }

      if (rtIsNaNF(s)) {
        normH = (rtNaNF);
        exitg2 = true;
      } else {
        if (s > normH) {
          normH = s;
        }

        i++;
      }
    }

    if (normH >= 1.0E+10F) {
      kidx = 2;
    } else {
      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i <= 4)) {
        normH = rt_powf_snf(10.0F, (real32_T)i) * 1.49011612E-7F;
        for (i1 = 0; i1 < 25; i1++) {
          b[i1] = 0;
        }

        for (kidx = 0; kidx < 5; kidx++) {
          b[kidx + 5 * kidx] = 1;
        }

        for (i1 = 0; i1 < 25; i1++) {
          s = normH * (real32_T)b[i1] + b_H[i1];
          b_H[i1] = s;
          L_0[i1] = s;
        }

        kidx = xpotrf(L_0);
        guard2 = false;
        if (kidx == 0) {
          for (kidx = 0; kidx < 5; kidx++) {
            varargin_1[kidx] = L_0[5 * kidx + kidx];
          }

          if (minimum(varargin_1) > 1.49011612E-7F) {
            kidx = 1;
            exitg1 = true;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }

        if (guard2) {
          kidx = 3;
          i++;
        }
      }
    }
  }

  if (kidx > 1) {
    *u = old_u + b_uoff;
    for (i = 0; i < 31; i++) {
      useq[i] = *u;
    }

    *status = -2.0F;
  } else {
    for (i1 = 0; i1 < 25; i1++) {
      b[i1] = 0;
    }

    for (kidx = 0; kidx < 5; kidx++) {
      b[kidx + 5 * kidx] = 1;
    }

    for (kidx = 0; kidx < 5; kidx++) {
      for (i = 0; i < 5; i++) {
        i1 = 5 * kidx + i;
        b_H[i1] = b[i1];
      }

      varargin_1[kidx] = 0.0F;
    }

    trisolve(L_0, b_H);
    for (kidx = 0; kidx < 4; kidx++) {
      normH = 0.0F;
      for (i1 = 0; i1 < 6; i1++) {
        normH += c_Kx[6 * kidx + i1] * x[i1];
      }

      b_Jm_0 = 0.0F;
      for (i1 = 0; i1 < 60; i1++) {
        b_Jm_0 += WySuJm[60 * kidx + i1] * rseq[i1];
      }

      s = 0.0F;
      for (i1 = 0; i1 < 62; i1++) {
        s += c_Kv[62 * kidx + i1] * vseq[i1];
      }

      WuI2Jm_0 = 0.0F;
      for (i1 = 0; i1 < 30; i1++) {
        WuI2Jm_0 += WuI2Jm[30 * kidx + i1] * b_utarget[i1];
      }

      varargin_1[kidx] = (((normH + b_Jm_0) + b_C_0[kidx] * old_u) + s) +
        WuI2Jm_0;
    }

    for (i = 0; i < 8; i++) {
      iAout[i] = iA[i];
      normH = 0.0F;
      for (i1 = 0; i1 < 6; i1++) {
        normH += b_Mx[(i1 << 3) + i] * x[i1];
      }

      b_Mlim_0[i] = (b_Mlim[i] + normH) + b_Mu1[i] * old_u;
      normH = 0.0F;
      for (i1 = 0; i1 < 62; i1++) {
        normH += b_Mv[(i1 << 3) + i] * vseq[i1];
      }

      b_Mv_0[i] = normH;
    }

    for (i1 = 0; i1 < 5; i1++) {
      for (i = 0; i < 5; i++) {
        s = 0.0F;
        for (CA_tmp = 0; CA_tmp < 5; CA_tmp++) {
          s += b_H[5 * i1 + CA_tmp] * b_H[5 * i + CA_tmp];
        }

        L_0[i1 + 5 * i] = s;
      }
    }

    for (i1 = 0; i1 < 8; i1++) {
      b_Mlim_1[i1] = -(b_Mlim_0[i1] + b_Mv_0[i1]);
    }

    qpkwik(b_H, L_0, varargin_1, b_Ac, b_Mlim_1, iAout, 120, 1.0E-6F, zopt,
           b_Mlim_0, &kidx);
    if (((int32_T)(real32_T)kidx < 0) || ((int32_T)(real32_T)kidx == 0)) {
      for (i = 0; i < 5; i++) {
        zopt[i] = 0.0F;
      }
    }

    *status = (real32_T)kidx;
    *u = (old_u + zopt[0]) + b_uoff;
  }
}

/* Model step function */
void LateralController_step(void)
{
  int32_T CovMat_tmp;
  int32_T i;
  int32_T r1;
  int32_T r2;
  real32_T tmp_1[496];
  real32_T Bv[372];
  real32_T Cm[372];
  real32_T Bu[186];
  real32_T Dv[124];
  real32_T CovMat[64];
  real32_T vseq[62];
  real32_T rseq[60];
  real32_T b_tmp[48];
  real32_T b_B[42];
  real32_T L_tmp[36];
  real32_T L_tmp_0[36];
  real32_T L_tmp_1[36];
  real32_T b_A[36];
  real32_T rtb_useq[31];
  real32_T tmp_2[30];
  real32_T tmp_3[30];
  real32_T Cm_0[12];
  real32_T L_1[12];
  real32_T b_C[12];
  real32_T c_A_tmp[12];
  real32_T tmp_0[12];
  real32_T b_b[8];
  real32_T xk[6];
  real32_T xk_0[6];
  real32_T Kinv[4];
  real32_T b_xoff_tmp[4];
  real32_T tmp[2];
  real32_T y_innov[2];
  real32_T CovMat_0;
  real32_T a22;
  real32_T a22_tmp;
  real32_T rtb_measure_disturbance;
  real32_T v_idx_1;
  int8_T Dvm[124];
  int8_T b_D[14];
  int8_T UnknownIn[6];
  int8_T c_B[4];
  static const int8_T c[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2, 2, 0, 0,
    0, 0, 0, 2, 10, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  static const real32_T d[42] = { 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.05F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.05F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };

  static const real32_T e[12] = { 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
    1.0F, 0.0F, 0.0F, 1.0F };

  static const int8_T f[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1 };

  static const int8_T tmp_4[8] = { 0, 1, 0, 0, 0, 0, 0, 1 };

  static const int8_T tmp_5[8] = { 0, 0, 1, 0, 0, 1, 0, 0 };

  static const int8_T g[8] = { 10, 1, 10, 1, 10, 1, 10, 1 };

  static const real32_T t[8] = { -1.0F, -1.0F, -1.0F, -1.0F, 1.0F, 1.0F, 1.0F,
    1.0F };

  static const real32_T w[25] = { 542.841614F, 524.203186F, 505.460785F,
    486.718414F, 0.0F, 524.203186F, 524.099182F, 505.460785F, 486.718414F, 0.0F,
    505.460785F, 505.460785F, 505.356812F, 486.718414F, 0.0F, 486.718414F,
    486.718414F, 486.718414F, 486.61441F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1000.0F
  };

  static const real32_T y[40] = { -1.0F, -1.0F, -1.0F, -1.0F, 1.0F, 1.0F, 1.0F,
    1.0F, -0.0F, -1.0F, -1.0F, -1.0F, 0.0F, 1.0F, 1.0F, 1.0F, -0.0F, -0.0F,
    -1.0F, -1.0F, 0.0F, 0.0F, 1.0F, 1.0F, -0.0F, -0.0F, -0.0F, -1.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };

  static const real32_T ab[120] = { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };

  static const int32_T b_Mrows[8] = { 121, 122, 123, 124, 151, 152, 153, 154 };

  real32_T y_0[40];
  real32_T w_0[25];
  real32_T t_0[8];
  boolean_T tmp_6[8];

  /* Product: '<S1>/Product' incorporates:
   *  Inport: '<Root>/curvature'
   *  Inport: '<Root>/velocity'
   */
  rtb_measure_disturbance = rtU.longitudinalvelocity * rtU.curvature;

  /* MATLAB Function: '<S32>/FixedHorizonOptimizer' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/L'
   *  Inport: '<Root>/velocity'
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  Memory: '<S4>/LastPcov'
   *  Memory: '<S4>/last_x'
   */
  memset(&Bu[0], 0, 186U * sizeof(real32_T));
  memset(&Bv[0], 0, 372U * sizeof(real32_T));
  memset(&Dv[0], 0, 124U * sizeof(real32_T));
  memset(&Dvm[0], 0, 124U * sizeof(int8_T));
  memset(&Cm[0], 0, 372U * sizeof(real32_T));
  for (i = 0; i < 36; i++) {
    b_A[i] = c[i];
  }

  memcpy(&b_B[0], &d[0], 42U * sizeof(real32_T));
  for (i = 0; i < 12; i++) {
    b_C[i] = e[i];
  }

  for (i = 0; i < 14; i++) {
    b_D[i] = f[i];
  }

  for (i = 0; i < 8; i++) {
    b_b[i] = tmp_4[i];
  }

  b_A[0] = 0.0F;
  b_A[6] = 0.0F;
  b_A[12] = -rtU.longitudinalvelocity;
  b_A[18] = 0.0F;
  b_A[1] = 0.0F;
  b_A[7] = 0.0F;
  b_A[13] = rtU.longitudinalvelocity;
  b_A[19] = rtU.longitudinalvelocity;
  b_A[2] = 0.0F;
  b_A[8] = 0.0F;
  b_A[14] = 0.0F;
  b_A[20] = rtU.longitudinalvelocity / (real32_T)L;
  for (i = 0; i < 4; i++) {
    r2 = i << 1;
    b_C[r2] = (real32_T)tmp_5[r2] / (real32_T)g[r2];
    b_C[r2 + 1] = (real32_T)tmp_5[r2 + 1] / (real32_T)g[r2 + 1];
    b_A[6 * i + 3] = 0.0F;
    b_B[i] = b_b[i + 4];
    b_B[i + 6] = b_b[i];
  }

  b_D[2] = 0;
  b_D[3] = 0;
  for (i = 0; i < 6; i++) {
    Bu[i] = b_B[i];
  }

  for (i = 0; i < 12; i++) {
    Bv[i] = b_B[i + 6];
  }

  Dvm[0] = 0;
  Dvm[1] = 0;
  Dvm[2] = b_D[4];
  Dvm[3] = b_D[5];
  UnknownIn[0] = 1;
  UnknownIn[1] = 2;
  UnknownIn[2] = 4;
  UnknownIn[3] = 5;
  UnknownIn[4] = 6;
  UnknownIn[5] = 7;
  for (i = 0; i < 6; i++) {
    r2 = i << 1;
    Cm[r2] = b_C[r2];
    Cm[r2 + 1] = b_C[r2 + 1];
    for (r2 = 0; r2 < 6; r2++) {
      b_tmp[r2 + (i << 3)] = b_B[(UnknownIn[i] - 1) * 6 + r2];
    }

    r2 = (UnknownIn[i] - 1) << 1;
    r1 = i << 3;
    b_tmp[r1 + 6] = b_D[r2];
    b_tmp[r1 + 7] = b_D[r2 + 1];
  }

  Dv[0] = 0.0F;
  Dv[1] = 0.0F;
  Dv[2] = b_D[4];
  Dv[3] = b_D[5];
  for (i = 0; i < 8; i++) {
    for (r2 = 0; r2 < 8; r2++) {
      CovMat_0 = 0.0F;
      for (r1 = 0; r1 < 6; r1++) {
        CovMat_tmp = r1 << 3;
        CovMat_0 += b_tmp[CovMat_tmp + i] * b_tmp[CovMat_tmp + r2];
      }

      CovMat[i + (r2 << 3)] = CovMat_0;
    }

    b_b[i] = 0.2F;
  }

  Bv[6] = 0.0F;
  Bv[7] = 0.0F;
  Bv[8] = 0.0F;
  Bv[9] = 0.0F;
  memset(&vseq[0], 0, 62U * sizeof(real32_T));
  for (r1 = 0; r1 < 31; r1++) {
    vseq[(r1 << 1) + 1] = 1.0F;
  }

  for (r1 = 0; r1 < 30; r1++) {
    i = r1 << 1;
    rseq[i] = 0.0F;
    rseq[i + 1] = 0.0F;
  }

  for (r1 = 0; r1 < 31; r1++) {
    vseq[r1 << 1] = rtb_measure_disturbance;
  }

  CovMat_0 = vseq[0];
  v_idx_1 = vseq[1];
  c_B[0] = 0;
  c_B[1] = 0;
  c_B[2] = 0;
  c_B[3] = 0;
  for (r1 = 0; r1 < 2; r1++) {
    c_B[r1 + (r1 << 1)] = 1;
    for (i = 0; i < 6; i++) {
      CovMat_tmp = (i << 1) + r1;
      c_A_tmp[i + 6 * r1] = Cm[CovMat_tmp];
      rtb_measure_disturbance = 0.0F;
      for (r2 = 0; r2 < 6; r2++) {
        rtb_measure_disturbance += Cm[(r2 << 1) + r1] *
          rtDW.LastPcov_PreviousInput[6 * i + r2];
      }

      Cm_0[CovMat_tmp] = rtb_measure_disturbance;
    }
  }

  for (i = 0; i < 2; i++) {
    for (r2 = 0; r2 < 2; r2++) {
      a22_tmp = 0.0F;
      for (r1 = 0; r1 < 6; r1++) {
        a22_tmp += Cm_0[(r1 << 1) + i] * c_A_tmp[6 * r2 + r1];
      }

      b_xoff_tmp[i + (r2 << 1)] = CovMat[(((r2 + 6) << 3) + i) + 6] + a22_tmp;
    }
  }

  if (fabsf(b_xoff_tmp[1]) > fabsf(b_xoff_tmp[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  rtb_measure_disturbance = b_xoff_tmp[r2] / b_xoff_tmp[r1];
  a22_tmp = b_xoff_tmp[r1 + 2];
  a22 = b_xoff_tmp[r2 + 2] - a22_tmp * rtb_measure_disturbance;
  i = r1 << 1;
  Kinv[i] = (real32_T)c_B[0] / b_xoff_tmp[r1];
  r2 <<= 1;
  Kinv[r2] = ((real32_T)c_B[2] - Kinv[i] * a22_tmp) / a22;
  Kinv[i] -= Kinv[r2] * rtb_measure_disturbance;
  Kinv[i + 1] = (real32_T)c_B[1] / b_xoff_tmp[r1];
  Kinv[r2 + 1] = ((real32_T)c_B[3] - Kinv[i + 1] * a22_tmp) / a22;
  Kinv[i + 1] -= Kinv[r2 + 1] * rtb_measure_disturbance;
  for (i = 0; i < 6; i++) {
    for (r2 = 0; r2 < 6; r2++) {
      a22_tmp = 0.0F;
      for (r1 = 0; r1 < 6; r1++) {
        a22_tmp += b_A[6 * r1 + i] * rtDW.LastPcov_PreviousInput[6 * r2 + r1];
      }

      L_tmp[i + 6 * r2] = a22_tmp;
    }

    for (r2 = 0; r2 < 2; r2++) {
      a22_tmp = 0.0F;
      for (r1 = 0; r1 < 6; r1++) {
        a22_tmp += L_tmp[6 * r1 + i] * c_A_tmp[6 * r2 + r1];
      }

      Cm_0[i + 6 * r2] = CovMat[((r2 + 6) << 3) + i] + a22_tmp;
    }

    a22_tmp = Cm_0[i + 6];
    rtb_measure_disturbance = Cm_0[i];
    L_1[i] = a22_tmp * Kinv[1] + rtb_measure_disturbance * Kinv[0];
    L_1[i + 6] = a22_tmp * Kinv[3] + rtb_measure_disturbance * Kinv[2];
    xk[i] = rtDW.last_x_PreviousInput[i];
  }

  /* SignalConversion generated from: '<S33>/ SFunction ' incorporates:
   *  Inport: '<Root>/lateral_deviation'
   *  Inport: '<Root>/relative_yaw_angle'
   *  MATLAB Function: '<S32>/FixedHorizonOptimizer'
   */
  tmp[0] = rtU.lateral_deviation * 0.1F;
  tmp[1] = rtU.relative_yaw_angle;

  /* MATLAB Function: '<S32>/FixedHorizonOptimizer' incorporates:
   *  Memory: '<S4>/LastPcov'
   */
  for (i = 0; i < 2; i++) {
    rtb_measure_disturbance = 0.0F;
    for (r2 = 0; r2 < 6; r2++) {
      rtb_measure_disturbance += Cm[(r2 << 1) + i] * xk[r2];
    }

    y_innov[i] = tmp[i] - (((real32_T)Dvm[i + 2] * v_idx_1 + (real32_T)Dvm[i] *
      CovMat_0) + rtb_measure_disturbance);
  }

  for (i = 0; i < 6; i++) {
    for (r2 = 0; r2 < 2; r2++) {
      a22_tmp = 0.0F;
      for (r1 = 0; r1 < 6; r1++) {
        a22_tmp += rtDW.LastPcov_PreviousInput[6 * r1 + i] * c_A_tmp[6 * r2 + r1];
      }

      tmp_0[i + 6 * r2] = a22_tmp;
    }

    a22_tmp = tmp_0[i + 6];
    rtb_measure_disturbance = tmp_0[i];
    xk_0[i] = ((a22_tmp * Kinv[1] + rtb_measure_disturbance * Kinv[0]) *
               y_innov[0] + (a22_tmp * Kinv[3] + rtb_measure_disturbance * Kinv
                [2]) * y_innov[1]) + xk[i];
  }

  memset(&b_tmp[0], 0, 48U * sizeof(real32_T));
  memset(&tmp_1[0], 0, 496U * sizeof(real32_T));
  tmp[0] = 0.36F;
  tmp[1] = 0.16F;
  for (i = 0; i < 30; i++) {
    tmp_2[i] = 0.0F;
    tmp_3[i] = 1.0F;
  }

  /* Memory: '<S4>/Memory' */
  for (i = 0; i < 8; i++) {
    tmp_6[i] = rtDW.Memory_PreviousInput[i];
  }

  /* End of Memory: '<S4>/Memory' */

  /* MATLAB Function: '<S32>/FixedHorizonOptimizer' */
  for (i = 0; i < 8; i++) {
    t_0[i] = t[i];
  }

  memcpy(&w_0[0], &w[0], 25U * sizeof(real32_T));
  memcpy(&y_0[0], &y[0], 40U * sizeof(real32_T));

  /* Update for Memory: '<S4>/Memory' incorporates:
   *  MATLAB Function: '<S32>/FixedHorizonOptimizer'
   *  UnitDelay: '<S4>/last_mv'
   */
  mpcblock_optimizer(rseq, vseq, xk_0, rtDW.last_mv_DSTATE, tmp_6, b_b, b_tmp,
                     t_0, tmp_1, tmp_2, 0.0F, w_0, y_0, tmp, ab, tmp_3, b_A, Bu,
                     Bv, b_C, Dv, b_Mrows, &rtb_measure_disturbance, rtb_useq,
                     &a22, rtDW.Memory_PreviousInput);

  /* MATLAB Function: '<S32>/FixedHorizonOptimizer' */
  for (i = 0; i < 6; i++) {
    for (r2 = 0; r2 < 6; r2++) {
      a22_tmp = 0.0F;
      for (r1 = 0; r1 < 6; r1++) {
        a22_tmp += L_tmp[6 * r1 + i] * b_A[6 * r1 + r2];
      }

      r1 = 6 * r2 + i;
      L_tmp_0[r1] = a22_tmp;
      L_tmp_1[r1] = Cm_0[i + 6] * L_1[r2 + 6] + Cm_0[i] * L_1[r2];
    }
  }

  for (i = 0; i < 6; i++) {
    for (r2 = 0; r2 < 6; r2++) {
      r1 = 6 * i + r2;
      L_tmp[r1] = CovMat[(i << 3) + r2] + (L_tmp_0[r1] - L_tmp_1[r1]);
    }
  }

  /* Outport: '<Root>/steering_angle' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
   */
  rtY.steering_angle = rtDW.DiscreteTimeIntegrator_DSTATE;

  /* Update for UnitDelay: '<S4>/last_mv' incorporates:
   *  MATLAB Function: '<S32>/FixedHorizonOptimizer'
   */
  rtDW.last_mv_DSTATE = rtb_measure_disturbance;
  for (i = 0; i < 6; i++) {
    /* MATLAB Function: '<S32>/FixedHorizonOptimizer' */
    a22_tmp = 0.0F;
    for (r2 = 0; r2 < 6; r2++) {
      /* MATLAB Function: '<S32>/FixedHorizonOptimizer' */
      r1 = 6 * i + r2;
      CovMat_tmp = 6 * r2 + i;

      /* Update for Memory: '<S4>/LastPcov' incorporates:
       *  MATLAB Function: '<S32>/FixedHorizonOptimizer'
       */
      rtDW.LastPcov_PreviousInput[r1] = (L_tmp[r1] + L_tmp[CovMat_tmp]) * 0.5F;

      /* MATLAB Function: '<S32>/FixedHorizonOptimizer' */
      a22_tmp += b_A[CovMat_tmp] * xk[r2];
    }

    /* Update for Memory: '<S4>/last_x' incorporates:
     *  MATLAB Function: '<S32>/FixedHorizonOptimizer'
     */
    rtDW.last_x_PreviousInput[i] = ((Bv[i + 6] * v_idx_1 + Bv[i] * CovMat_0) +
      (Bu[i] * rtb_measure_disturbance + a22_tmp)) + (L_1[i + 6] * y_innov[1] +
      L_1[i] * y_innov[0]);
  }

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' incorporates:
   *  MATLAB Function: '<S32>/FixedHorizonOptimizer'
   */
  rtDW.DiscreteTimeIntegrator_DSTATE += 0.05F * rtb_measure_disturbance;
  if (rtDW.DiscreteTimeIntegrator_DSTATE > 0.52F) {
    rtDW.DiscreteTimeIntegrator_DSTATE = 0.52F;
  } else if (rtDW.DiscreteTimeIntegrator_DSTATE < -0.52F) {
    rtDW.DiscreteTimeIntegrator_DSTATE = -0.52F;
  }

  /* End of Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' */
}

/* Model initialize function */
void LateralController_initialize(void)
{
  /* Registration code */

  /* Initialize DataMapInfo substructure containing ModelMap for C API */
  LateralController_InitializeDataMapInfo();

  /* InitializeConditions for Memory: '<S4>/LastPcov' */
  memcpy(&rtDW.LastPcov_PreviousInput[0], &rtConstP.LastPcov_InitialCondition[0],
         36U * sizeof(real32_T));
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
