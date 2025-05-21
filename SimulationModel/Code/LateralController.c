/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LateralController.c
 *
 * Code generated for Simulink model 'LateralController'.
 *
 * Model version                  : 1.22
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Wed May 21 19:29:22 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "LateralController.h"
#include "rtwtypes.h"
#include <string.h>
#include <math.h>
#include <stddef.h>

/* Named constants for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
#define RMDscale                       (0.0285714287F)
#define Wdu                            (0.01F)
#define Wu                             (0.0F)
#define degrees                        (4)
#define ny                             (3)
#define p                              (20)
#define NumBitsPerChar                 8U

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
static real32_T norm_a(const real32_T x[25]);
static void xzgetrf(real32_T A[25], int32_T ipiv[5], int32_T *info);
static void xtrsm(const real32_T A[25], real32_T B_0[25]);
static void inv(const real32_T x[25], real32_T y[25]);
static void mpower(const real32_T a[25], real32_T b, real32_T c[25]);
static real32_T log2_a(real32_T x);
static void getExpmParams(const real32_T A[25], real32_T A2[25], real32_T A4[25],
  real32_T A6[25], int32_T *m, real32_T *s);
static void padeApproximation(const real32_T A[25], const real32_T A2[25], const
  real32_T A4[25], const real32_T A6[25], int32_T m, real32_T F[25]);
static void recomputeBlockDiag(const real32_T A[25], real32_T F[25], const
  int32_T blockFormat[4]);
static real32_T xnrm2_c(int32_T n, const real32_T x[25], int32_T ix0);
static void xzsyhetrd(real32_T A[25], real32_T D[5], real32_T E[4], real32_T
                      tau[4]);
static real32_T computeNormMTridiagonal(int32_T n, const real32_T d[5], const
  real32_T e[4], int32_T i0);
static void xzlascl(real32_T cfrom, real32_T cto, int32_T m, int32_T n, real32_T
                    A[5], int32_T iA0, int32_T lda);
static void xzlascl_b(real32_T cfrom, real32_T cto, int32_T m, int32_T n,
                      real32_T A[4], int32_T iA0, int32_T lda);
static void xzlartg(real32_T f, real32_T g, real32_T *cs, real32_T *sn, real32_T
                    *r);
static void rotateRight_k(int32_T m, int32_T n, real32_T z[25], int32_T iz0,
  int32_T ldz, const real32_T cs[8], int32_T ic0, int32_T is0);
static void xdlaev2(real32_T a, real32_T b, real32_T c, real32_T *rt1, real32_T *
                    rt2, real32_T *cs1, real32_T *sn1);
static void rotateRight(int32_T m, int32_T n, real32_T z[25], int32_T iz0,
  int32_T ldz, const real32_T cs[8], int32_T ic0, int32_T is0);
static int32_T xzsteqr(real32_T d[5], real32_T e[4], real32_T z[25]);
static void xsyheev(real32_T A[25], int32_T *info, real32_T W[5]);
static int32_T xpotrf(real32_T b_A[16]);
static real32_T minimum(const real32_T x[4]);
static void trisolve(const real32_T b_A[16], real32_T b_B[16]);
static real32_T norm(const real32_T x[4]);
static real32_T maximum(const real32_T x[4]);
static real32_T xnrm2(int32_T n, const real32_T x[16], int32_T ix0);
static void xgemv(int32_T b_m, int32_T n, const real32_T b_A[16], int32_T ia0,
                  const real32_T x[16], int32_T ix0, real32_T y[4]);
static void xgerc(int32_T b_m, int32_T n, real32_T alpha1, int32_T ix0, const
                  real32_T y[4], real32_T b_A[16], int32_T ia0);
static real32_T KWIKfactor(const real32_T b_Ac[184], const int32_T iC[46],
  int32_T nA, const real32_T b_Linv[16], real32_T RLinv[16], real32_T b_D[16],
  real32_T b_H[16], int32_T n);
static void DropConstraint(int32_T kDrop, boolean_T iA[46], int32_T *nA, int32_T
  iC[46]);
static void qpkwik(const real32_T b_Linv[16], const real32_T b_Hinv[16], const
                   real32_T f[4], const real32_T b_Ac[184], const real32_T b[46],
                   boolean_T iA[46], int32_T maxiter, real32_T FeasTol, real32_T
                   x[4], real32_T lambda[46], int32_T *status);
static void mpcblock_optimizer(const real32_T rseq[60], const real32_T vseq[42],
  const real32_T x[5], real32_T old_u, const boolean_T iA[46], const real32_T
  b_Mlim[46], real32_T b_Mx[230], real32_T b_Mu1[46], real32_T b_Mv[1932], const
  real32_T b_utarget[20], real32_T b_uoff, real32_T b_H[16], real32_T b_Ac[184],
  const real32_T b_Wy[3], const real32_T b_Jm[60], const real32_T b_I1[20],
  const real32_T b_A[25], const real32_T Bu[105], const real32_T Bv[210], const
  real32_T b_C[15], const real32_T Dv[126], const int32_T b_Mrows[46], real32_T *
  u, real32_T useq[21], real32_T *status, boolean_T iAout[46]);
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);

#define NOT_USING_NONFINITE_LITERALS   1

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static void rt_InitInfAndNaN(size_t realSize);
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;

/*
 * Initialize rtInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T inf = 0.0;
  if (bitsPerReal == 32U) {
    inf = rtGetInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0x7FF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    inf = tmpVal.fltVal;
  }

  return inf;
}

/*
 * Initialize rtInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetInfF(void)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

/*
 * Initialize rtMinusInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetMinusInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T minf = 0.0;
  if (bitsPerReal == 32U) {
    minf = rtGetMinusInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    minf = tmpVal.fltVal;
  }

  return minf;
}

/*
 * Initialize rtMinusInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetMinusInfF(void)
{
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

/*
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetNaN(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T nan = 0.0;
  if (bitsPerReal == 32U) {
    nan = rtGetNaNF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF80000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    nan = tmpVal.fltVal;
  }

  return nan;
}

/*
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetNaNF(void)
{
  IEEESingle nanF = { { 0.0F } };

  nanF.wordL.wordLuint = 0xFFC00000U;
  return nanF.wordL.wordLreal;
}

/*
 * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
 * generated code. NaN is initialized as non-signaling. Assumes IEEE.
 */
static void rt_InitInfAndNaN(size_t realSize)
{
  (void) (realSize);
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  boolean_T result = (boolean_T) 0;
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  if (bitsPerReal == 32U) {
    result = rtIsNaNF((real32_T)value);
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.fltVal = value;
    result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) == 0x7FF00000 &&
                         ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                          (tmpVal.bitVal.words.wordL != 0) ));
  }

  return result;
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

static int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static real32_T norm_a(const real32_T x[25])
{
  int32_T i;
  int32_T j;
  real32_T y;
  boolean_T exitg1;
  y = 0.0F;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 5)) {
    real32_T s;
    s = 0.0F;
    for (i = 0; i < 5; i++) {
      s += fabsf(x[5 * j + i]);
    }

    if (rtIsNaNF(s)) {
      y = (rtNaNF);
      exitg1 = true;
    } else {
      if (s > y) {
        y = s;
      }

      j++;
    }
  }

  return y;
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

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void xzgetrf(real32_T A[25], int32_T ipiv[5], int32_T *info)
{
  int32_T iy;
  int32_T j;
  int32_T k;
  for (j = 0; j < 5; j++) {
    ipiv[j] = j + 1;
  }

  *info = 0;
  for (j = 0; j < 4; j++) {
    int32_T b_ix;
    int32_T ix;
    int32_T jj;
    real32_T smax;
    jj = j * 6;
    iy = 4 - j;
    b_ix = 0;
    smax = fabsf(A[jj]);
    for (k = 2; k <= iy + 1; k++) {
      real32_T s;
      s = fabsf(A[(jj + k) - 1]);
      if (s > smax) {
        b_ix = k - 1;
        smax = s;
      }
    }

    if (A[jj + b_ix] != 0.0F) {
      if (b_ix != 0) {
        iy = j + b_ix;
        ipiv[j] = iy + 1;
        b_ix = j;
        for (k = 0; k < 5; k++) {
          smax = A[b_ix];
          A[b_ix] = A[iy];
          A[iy] = smax;
          b_ix += 5;
          iy += 5;
        }
      }

      iy = (jj - j) + 5;
      for (k = jj + 2; k <= iy; k++) {
        A[k - 1] /= A[jj];
      }
    } else {
      *info = j + 1;
    }

    b_ix = 3 - j;
    ix = jj + 7;
    for (k = 0; k <= b_ix; k++) {
      smax = A[(k * 5 + jj) + 5];
      if (smax != 0.0F) {
        int32_T c;
        c = (ix - j) + 3;
        for (iy = ix; iy <= c; iy++) {
          A[iy - 1] += A[((jj + iy) - ix) + 1] * -smax;
        }
      }

      ix += 5;
    }
  }

  if ((*info == 0) && (!(A[24] != 0.0F))) {
    *info = 5;
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void xtrsm(const real32_T A[25], real32_T B_0[25])
{
  int32_T i;
  int32_T j;
  int32_T k;
  for (j = 0; j < 5; j++) {
    int32_T jBcol;
    jBcol = 5 * j;
    for (k = 4; k >= 0; k--) {
      int32_T kAcol;
      int32_T tmp_0;
      real32_T tmp;
      kAcol = 5 * k;
      tmp_0 = k + jBcol;
      tmp = B_0[tmp_0];
      if (tmp != 0.0F) {
        B_0[tmp_0] = tmp / A[k + kAcol];
        for (i = 0; i < k; i++) {
          int32_T tmp_1;
          tmp_1 = i + jBcol;
          B_0[tmp_1] -= A[i + kAcol] * B_0[tmp_0];
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void inv(const real32_T x[25], real32_T y[25])
{
  int32_T ipiv[5];
  int32_T i;
  int32_T info;
  int32_T j;
  int32_T pipk;
  int32_T y_tmp;
  int32_T y_tmp_0;
  real32_T b_x[25];
  int8_T p_0[5];
  for (pipk = 0; pipk < 25; pipk++) {
    y[pipk] = 0.0F;
    b_x[pipk] = x[pipk];
  }

  xzgetrf(b_x, ipiv, &info);
  for (pipk = 0; pipk < 5; pipk++) {
    p_0[pipk] = (int8_T)(pipk + 1);
  }

  if (ipiv[0] > 1) {
    pipk = p_0[ipiv[0] - 1];
    p_0[ipiv[0] - 1] = p_0[0];
    p_0[0] = (int8_T)pipk;
  }

  if (ipiv[1] > 2) {
    pipk = p_0[ipiv[1] - 1];
    p_0[ipiv[1] - 1] = p_0[1];
    p_0[1] = (int8_T)pipk;
  }

  if (ipiv[2] > 3) {
    pipk = p_0[ipiv[2] - 1];
    p_0[ipiv[2] - 1] = p_0[2];
    p_0[2] = (int8_T)pipk;
  }

  if (ipiv[3] > 4) {
    pipk = p_0[ipiv[3] - 1];
    p_0[ipiv[3] - 1] = p_0[3];
    p_0[3] = (int8_T)pipk;
  }

  for (info = 0; info < 5; info++) {
    y_tmp = (p_0[info] - 1) * 5;
    y[info + y_tmp] = 1.0F;
    for (j = info + 1; j < 6; j++) {
      pipk = (y_tmp + j) - 1;
      if (y[pipk] != 0.0F) {
        for (i = j + 1; i < 6; i++) {
          y_tmp_0 = (y_tmp + i) - 1;
          y[y_tmp_0] -= b_x[((j - 1) * 5 + i) - 1] * y[pipk];
        }
      }
    }
  }

  xtrsm(b_x, y);
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void mpower(const real32_T a[25], real32_T b, real32_T c[25])
{
  int32_T b_n;
  int32_T exitg1;
  int32_T i;
  int32_T i_0;
  int32_T n;
  int32_T nb;
  int32_T nbitson;
  real32_T aBuffer[25];
  real32_T b_a[25];
  real32_T cBuffer[25];
  real32_T cBuffer_0[25];
  real32_T cBuffer_1;
  real32_T e;
  real32_T ed2;
  boolean_T aBufferInUse;
  boolean_T firstmult;
  boolean_T lsb;
  if (floorf(b) == b) {
    e = fabsf(b);
    if (e < 2.14748365E+9F) {
      memcpy(&b_a[0], &a[0], 25U * sizeof(real32_T));
      n = (int32_T)e;
      b_n = (int32_T)e;
      nbitson = 0;
      nb = -2;
      while (b_n > 0) {
        nb++;
        if (((uint32_T)b_n & 1U) != 0U) {
          nbitson++;
        }

        b_n >>= 1;
      }

      if ((int32_T)e <= 2) {
        if (b == 2.0F) {
          for (nbitson = 0; nbitson < 5; nbitson++) {
            for (i = 0; i < 5; i++) {
              e = 0.0F;
              for (i_0 = 0; i_0 < 5; i_0++) {
                e += a[5 * i_0 + i] * a[5 * nbitson + i_0];
              }

              c[i + 5 * nbitson] = e;
            }
          }
        } else if (b == 1.0F) {
          memcpy(&c[0], &a[0], 25U * sizeof(real32_T));
        } else if (b == -1.0F) {
          inv(a, c);
        } else if (b == -2.0F) {
          for (nbitson = 0; nbitson < 5; nbitson++) {
            for (i = 0; i < 5; i++) {
              e = 0.0F;
              for (i_0 = 0; i_0 < 5; i_0++) {
                e += a[5 * i_0 + i] * a[5 * nbitson + i_0];
              }

              b_a[i + 5 * nbitson] = e;
            }
          }

          inv(b_a, c);
        } else {
          firstmult = false;
          for (n = 0; n < 25; n++) {
            if (!firstmult) {
              firstmult = rtIsNaNF(a[n]);
            }
          }

          if (firstmult) {
            for (nbitson = 0; nbitson < 25; nbitson++) {
              c[nbitson] = (rtNaNF);
            }
          } else {
            memset(&c[0], 0, 25U * sizeof(real32_T));
            for (n = 0; n < 5; n++) {
              c[n + 5 * n] = 1.0F;
            }
          }
        }
      } else {
        firstmult = true;
        aBufferInUse = false;
        lsb = (((uint32_T)nbitson & 1U) != 0U);
        lsb = ((lsb && (b < 0.0F)) || ((!lsb) && (b >= 0.0F)));
        for (b_n = 0; b_n <= nb; b_n++) {
          if (((uint32_T)n & 1U) != 0U) {
            if (firstmult) {
              firstmult = false;
              if (lsb) {
                if (aBufferInUse) {
                  memcpy(&cBuffer[0], &aBuffer[0], 25U * sizeof(real32_T));
                } else {
                  memcpy(&cBuffer[0], &b_a[0], 25U * sizeof(real32_T));
                }
              } else if (aBufferInUse) {
                memcpy(&c[0], &aBuffer[0], 25U * sizeof(real32_T));
              } else {
                memcpy(&c[0], &b_a[0], 25U * sizeof(real32_T));
              }
            } else {
              if (aBufferInUse) {
                if (lsb) {
                  for (nbitson = 0; nbitson < 5; nbitson++) {
                    for (i = 0; i < 5; i++) {
                      e = 0.0F;
                      for (i_0 = 0; i_0 < 5; i_0++) {
                        e += cBuffer[5 * i_0 + nbitson] * aBuffer[5 * i + i_0];
                      }

                      c[nbitson + 5 * i] = e;
                    }
                  }
                } else {
                  for (nbitson = 0; nbitson < 5; nbitson++) {
                    for (i = 0; i < 5; i++) {
                      e = 0.0F;
                      for (i_0 = 0; i_0 < 5; i_0++) {
                        e += c[5 * i_0 + nbitson] * aBuffer[5 * i + i_0];
                      }

                      cBuffer[nbitson + 5 * i] = e;
                    }
                  }
                }
              } else if (lsb) {
                for (nbitson = 0; nbitson < 5; nbitson++) {
                  for (i = 0; i < 5; i++) {
                    e = 0.0F;
                    for (i_0 = 0; i_0 < 5; i_0++) {
                      e += cBuffer[5 * i_0 + nbitson] * b_a[5 * i + i_0];
                    }

                    c[nbitson + 5 * i] = e;
                  }
                }
              } else {
                for (nbitson = 0; nbitson < 5; nbitson++) {
                  for (i = 0; i < 5; i++) {
                    e = 0.0F;
                    for (i_0 = 0; i_0 < 5; i_0++) {
                      e += c[5 * i_0 + nbitson] * b_a[5 * i + i_0];
                    }

                    cBuffer[nbitson + 5 * i] = e;
                  }
                }
              }

              lsb = !lsb;
            }
          }

          n >>= 1;
          if (aBufferInUse) {
            for (nbitson = 0; nbitson < 5; nbitson++) {
              for (i = 0; i < 5; i++) {
                ed2 = 0.0F;
                for (i_0 = 0; i_0 < 5; i_0++) {
                  ed2 += aBuffer[5 * i_0 + nbitson] * aBuffer[5 * i + i_0];
                }

                b_a[nbitson + 5 * i] = ed2;
              }
            }
          } else {
            for (nbitson = 0; nbitson < 5; nbitson++) {
              for (i = 0; i < 5; i++) {
                e = 0.0F;
                for (i_0 = 0; i_0 < 5; i_0++) {
                  e += b_a[5 * i_0 + nbitson] * b_a[5 * i + i_0];
                }

                aBuffer[nbitson + 5 * i] = e;
              }
            }
          }

          aBufferInUse = !aBufferInUse;
        }

        if (firstmult) {
          if (b < 0.0F) {
            if (aBufferInUse) {
              inv(aBuffer, c);
            } else {
              inv(b_a, c);
            }
          } else if (aBufferInUse) {
            memcpy(&c[0], &aBuffer[0], 25U * sizeof(real32_T));
          } else {
            memcpy(&c[0], &b_a[0], 25U * sizeof(real32_T));
          }
        } else if (b < 0.0F) {
          for (nbitson = 0; nbitson < 5; nbitson++) {
            for (i = 0; i < 5; i++) {
              e = 0.0F;
              ed2 = 0.0F;
              for (i_0 = 0; i_0 < 5; i_0++) {
                cBuffer_1 = c[5 * i_0 + i];
                n = 5 * nbitson + i_0;
                e += aBuffer[n] * cBuffer_1;
                ed2 += b_a[n] * cBuffer_1;
              }

              n = 5 * nbitson + i;
              cBuffer_0[n] = ed2;
              cBuffer[n] = e;
            }
          }

          if (aBufferInUse) {
            memcpy(&b_a[0], &cBuffer[0], 25U * sizeof(real32_T));
          } else {
            memcpy(&b_a[0], &cBuffer_0[0], 25U * sizeof(real32_T));
          }

          inv(b_a, c);
        } else {
          for (nbitson = 0; nbitson < 5; nbitson++) {
            for (i = 0; i < 5; i++) {
              e = 0.0F;
              ed2 = 0.0F;
              for (i_0 = 0; i_0 < 5; i_0++) {
                cBuffer_1 = cBuffer[5 * i_0 + i];
                n = 5 * nbitson + i_0;
                e += aBuffer[n] * cBuffer_1;
                ed2 += b_a[n] * cBuffer_1;
              }

              n = 5 * nbitson + i;
              c[n] = ed2;
              cBuffer_0[n] = e;
            }
          }

          if (aBufferInUse) {
            memcpy(&c[0], &cBuffer_0[0], 25U * sizeof(real32_T));
          }
        }
      }
    } else {
      memcpy(&b_a[0], &a[0], 25U * sizeof(real32_T));
      if (!rtIsInfF(b)) {
        firstmult = true;
        do {
          exitg1 = 0;
          ed2 = floorf(e / 2.0F);
          if (2.0F * ed2 != e) {
            if (firstmult) {
              memcpy(&c[0], &b_a[0], 25U * sizeof(real32_T));
              firstmult = false;
            } else {
              for (nbitson = 0; nbitson < 5; nbitson++) {
                for (i = 0; i < 5; i++) {
                  e = 0.0F;
                  for (i_0 = 0; i_0 < 5; i_0++) {
                    e += c[5 * i_0 + nbitson] * b_a[5 * i + i_0];
                  }

                  cBuffer[nbitson + 5 * i] = e;
                }
              }

              memcpy(&c[0], &cBuffer[0], 25U * sizeof(real32_T));
            }
          }

          if (ed2 == 0.0F) {
            exitg1 = 1;
          } else {
            e = ed2;
            for (nbitson = 0; nbitson < 5; nbitson++) {
              for (i = 0; i < 5; i++) {
                ed2 = 0.0F;
                for (i_0 = 0; i_0 < 5; i_0++) {
                  ed2 += b_a[5 * i_0 + nbitson] * b_a[5 * i + i_0];
                }

                aBuffer[nbitson + 5 * i] = ed2;
              }
            }

            memcpy(&b_a[0], &aBuffer[0], 25U * sizeof(real32_T));
          }
        } while (exitg1 == 0);

        if (b < 0.0F) {
          memcpy(&b_a[0], &c[0], 25U * sizeof(real32_T));
          inv(b_a, c);
        }
      } else {
        for (nbitson = 0; nbitson < 25; nbitson++) {
          c[nbitson] = (rtNaNF);
        }
      }
    }
  } else {
    for (nbitson = 0; nbitson < 25; nbitson++) {
      c[nbitson] = (rtNaNF);
    }
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static real32_T log2_a(real32_T x)
{
  int32_T inte;
  real32_T f;
  if (x == 0.0F) {
    f = (rtMinusInfF);
  } else if (x < 0.0F) {
    f = (rtNaNF);
  } else if ((!rtIsInfF(x)) && (!rtIsNaNF(x))) {
    real32_T t;
    t = frexpf(x, &inte);
    if (t == 0.5F) {
      f = (real32_T)inte - 1.0F;
    } else if ((inte == 1) && (t < 0.75F)) {
      f = logf(2.0F * t) / 0.693147182F;
    } else {
      f = logf(t) / 0.693147182F + (real32_T)inte;
    }
  } else {
    f = x;
  }

  return f;
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void getExpmParams(const real32_T A[25], real32_T A2[25], real32_T A4[25],
  real32_T A6[25], int32_T *m, real32_T *s)
{
  int32_T c_s;
  int32_T d_s;
  int32_T i;
  int32_T i_0;
  int32_T k;
  real32_T T[25];
  real32_T b_y[25];
  real32_T tmp[25];
  real32_T A4_0;
  real32_T d6;
  real32_T eta1;
  static const real32_T b[3] = { 0.425873F, 1.8801527F, 3.92572474F };

  boolean_T exitg1;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T guard3;
  boolean_T guard4;
  *s = 0.0F;
  for (k = 0; k < 5; k++) {
    for (i = 0; i < 5; i++) {
      d6 = 0.0F;
      for (i_0 = 0; i_0 < 5; i_0++) {
        d6 += A[5 * i_0 + i] * A[5 * k + i_0];
      }

      A2[i + 5 * k] = d6;
    }
  }

  for (k = 0; k < 5; k++) {
    for (i = 0; i < 5; i++) {
      A4_0 = 0.0F;
      for (i_0 = 0; i_0 < 5; i_0++) {
        A4_0 += A2[5 * i_0 + k] * A2[5 * i + i_0];
      }

      A4[k + 5 * i] = A4_0;
    }

    for (i = 0; i < 5; i++) {
      d6 = 0.0F;
      for (i_0 = 0; i_0 < 5; i_0++) {
        d6 += A4[5 * i_0 + k] * A2[5 * i + i_0];
      }

      A6[k + 5 * i] = d6;
    }
  }

  d6 = rt_powf_snf(norm_a(A6), 0.166666672F);
  eta1 = fmaxf(rt_powf_snf(norm_a(A4), 0.25F), d6);
  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  if (eta1 <= 0.0149558522F) {
    for (k = 0; k < 25; k++) {
      tmp[k] = 0.192850113F * fabsf(A[k]);
    }

    mpower(tmp, 7.0F, b_y);
    if (fmaxf(ceilf(log2_a(norm_a(b_y) / norm_a(A) * 2.0F / 1.1920929E-7F) /
                    6.0F), 0.0F) == 0.0F) {
      *m = 3;
    } else {
      guard4 = true;
    }
  } else {
    guard4 = true;
  }

  if (guard4) {
    if (eta1 <= 0.253939837F) {
      for (k = 0; k < 25; k++) {
        tmp[k] = 0.123218715F * fabsf(A[k]);
      }

      mpower(tmp, 11.0F, b_y);
      if (fmaxf(ceilf(log2_a(norm_a(b_y) / norm_a(A) * 2.0F / 1.1920929E-7F) /
                      10.0F), 0.0F) == 0.0F) {
        *m = 5;
      } else {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }
  }

  if (guard3) {
    mpower(A4, 2.0F, b_y);
    eta1 = rt_powf_snf(norm_a(b_y), 0.125F);
    d6 = fmaxf(d6, eta1);
    if (d6 <= 0.950417876F) {
      for (k = 0; k < 25; k++) {
        tmp[k] = 0.0904753208F * fabsf(A[k]);
      }

      mpower(tmp, 15.0F, b_y);
      if (fmaxf(ceilf(log2_a(norm_a(b_y) / norm_a(A) * 2.0F / 1.1920929E-7F) /
                      14.0F), 0.0F) == 0.0F) {
        *m = 7;
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  }

  if (guard2) {
    if (d6 <= 2.09784794F) {
      for (k = 0; k < 25; k++) {
        tmp[k] = 0.0714677349F * fabsf(A[k]);
      }

      mpower(tmp, 19.0F, b_y);
      if (fmaxf(ceilf(log2_a(norm_a(b_y) / norm_a(A) * 2.0F / 1.1920929E-7F) /
                      18.0F), 0.0F) == 0.0F) {
        *m = 9;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    for (k = 0; k < 5; k++) {
      for (i = 0; i < 5; i++) {
        A4_0 = 0.0F;
        for (i_0 = 0; i_0 < 5; i_0++) {
          A4_0 += A4[5 * i_0 + i] * A6[5 * k + i_0];
        }

        b_y[i + 5 * k] = A4_0;
      }
    }

    *s = fmaxf(ceilf(log2_a(fminf(d6, fmaxf(eta1, rt_powf_snf(norm_a(b_y), 0.1F)))
      / 5.37192059F)), 0.0F);
    d6 = rt_powf_snf(2.0F, *s);
    for (k = 0; k < 25; k++) {
      A4_0 = A[k] / d6;
      T[k] = A4_0;
      tmp[k] = 0.050315544F * fabsf(A4_0);
    }

    mpower(tmp, 27.0F, b_y);
    *s += fmaxf(ceilf(log2_a(norm_a(b_y) / norm_a(T) * 2.0F / 1.1920929E-7F) /
                      26.0F), 0.0F);
    if (rtIsInfF(*s)) {
      d6 = norm_a(A) / 5.37192059F;
      if ((!rtIsInfF(d6)) && (!rtIsNaNF(d6))) {
        d6 = frexpf(d6, &c_s);
      } else {
        c_s = 0;
      }

      *s = (real32_T)c_s;
      if (d6 == 0.5F) {
        *s = (real32_T)c_s - 1.0F;
      }
    }

    *m = 13;
  }

  d6 = norm_a(A);
  c_s = 0;
  k = 7;
  if (d6 <= 3.92572474F) {
    d_s = 0;
    exitg1 = false;
    while ((!exitg1) && (d_s < 3)) {
      if (d6 <= b[d_s]) {
        k = (d_s << 1) + 3;
        exitg1 = true;
      } else {
        d_s++;
      }
    }
  } else {
    d6 /= 3.92572474F;
    if ((!rtIsInfF(d6)) && (!rtIsNaNF(d6))) {
      d6 = frexpf(d6, &d_s);
    } else {
      d_s = 0;
    }

    c_s = d_s;
    if (d6 == 0.5F) {
      c_s = d_s - 1;
    }
  }

  if (c_s <= *s + 2.0F) {
    *s = (real32_T)c_s;
    *m = k;
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void padeApproximation(const real32_T A[25], const real32_T A2[25], const
  real32_T A4[25], const real32_T A6[25], int32_T m, real32_T F[25])
{
  int32_T ipiv[5];
  int32_T F_tmp;
  int32_T b_i;
  int32_T e_k;
  int32_T ip;
  int32_T ipiv_0;
  int32_T j;
  int32_T kAcol;
  real32_T A6_0[25];
  real32_T V[25];
  real32_T d;
  switch (m) {
   case 3:
    memcpy(&F[0], &A2[0], 25U * sizeof(real32_T));
    for (e_k = 0; e_k < 5; e_k++) {
      F_tmp = 5 * e_k + e_k;
      F[F_tmp] += 60.0F;
    }

    for (ipiv_0 = 0; ipiv_0 < 5; ipiv_0++) {
      for (e_k = 0; e_k < 5; e_k++) {
        d = 0.0F;
        for (F_tmp = 0; F_tmp < 5; F_tmp++) {
          d += A[5 * F_tmp + e_k] * F[5 * ipiv_0 + F_tmp];
        }

        A6_0[e_k + 5 * ipiv_0] = d;
      }
    }

    for (ipiv_0 = 0; ipiv_0 < 25; ipiv_0++) {
      F[ipiv_0] = A6_0[ipiv_0];
      V[ipiv_0] = 12.0F * A2[ipiv_0];
    }

    d = 120.0F;
    break;

   case 5:
    for (ipiv_0 = 0; ipiv_0 < 25; ipiv_0++) {
      F[ipiv_0] = 420.0F * A2[ipiv_0] + A4[ipiv_0];
    }

    for (e_k = 0; e_k < 5; e_k++) {
      F_tmp = 5 * e_k + e_k;
      F[F_tmp] += 15120.0F;
    }

    for (ipiv_0 = 0; ipiv_0 < 5; ipiv_0++) {
      for (e_k = 0; e_k < 5; e_k++) {
        d = 0.0F;
        for (F_tmp = 0; F_tmp < 5; F_tmp++) {
          d += A[5 * F_tmp + e_k] * F[5 * ipiv_0 + F_tmp];
        }

        A6_0[e_k + 5 * ipiv_0] = d;
      }
    }

    for (ipiv_0 = 0; ipiv_0 < 25; ipiv_0++) {
      F[ipiv_0] = A6_0[ipiv_0];
      V[ipiv_0] = 30.0F * A4[ipiv_0] + 3360.0F * A2[ipiv_0];
    }

    d = 30240.0F;
    break;

   case 7:
    for (ipiv_0 = 0; ipiv_0 < 25; ipiv_0++) {
      F[ipiv_0] = (1512.0F * A4[ipiv_0] + A6[ipiv_0]) + 277200.0F * A2[ipiv_0];
    }

    for (e_k = 0; e_k < 5; e_k++) {
      F_tmp = 5 * e_k + e_k;
      F[F_tmp] += 8.64864E+6F;
    }

    for (ipiv_0 = 0; ipiv_0 < 5; ipiv_0++) {
      for (e_k = 0; e_k < 5; e_k++) {
        d = 0.0F;
        for (F_tmp = 0; F_tmp < 5; F_tmp++) {
          d += A[5 * F_tmp + e_k] * F[5 * ipiv_0 + F_tmp];
        }

        A6_0[e_k + 5 * ipiv_0] = d;
      }
    }

    for (ipiv_0 = 0; ipiv_0 < 25; ipiv_0++) {
      F[ipiv_0] = A6_0[ipiv_0];
      V[ipiv_0] = (56.0F * A6[ipiv_0] + 25200.0F * A4[ipiv_0]) + 1.99584E+6F *
        A2[ipiv_0];
    }

    d = 1.729728E+7F;
    break;

   case 9:
    for (ipiv_0 = 0; ipiv_0 < 5; ipiv_0++) {
      for (e_k = 0; e_k < 5; e_k++) {
        d = 0.0F;
        for (F_tmp = 0; F_tmp < 5; F_tmp++) {
          d += A6[5 * F_tmp + e_k] * A2[5 * ipiv_0 + F_tmp];
        }

        V[e_k + 5 * ipiv_0] = d;
      }
    }

    for (ipiv_0 = 0; ipiv_0 < 25; ipiv_0++) {
      F[ipiv_0] = ((3960.0F * A6[ipiv_0] + V[ipiv_0]) + 2.16216E+6F * A4[ipiv_0])
        + 3.027024E+8F * A2[ipiv_0];
    }

    for (e_k = 0; e_k < 5; e_k++) {
      F_tmp = 5 * e_k + e_k;
      F[F_tmp] += 8.82161254E+9F;
    }

    for (ipiv_0 = 0; ipiv_0 < 5; ipiv_0++) {
      for (e_k = 0; e_k < 5; e_k++) {
        d = 0.0F;
        for (F_tmp = 0; F_tmp < 5; F_tmp++) {
          d += A[5 * F_tmp + e_k] * F[5 * ipiv_0 + F_tmp];
        }

        A6_0[e_k + 5 * ipiv_0] = d;
      }
    }

    for (ipiv_0 = 0; ipiv_0 < 25; ipiv_0++) {
      F[ipiv_0] = A6_0[ipiv_0];
      V[ipiv_0] = ((90.0F * V[ipiv_0] + 110880.0F * A6[ipiv_0]) + 3.027024E+7F *
                   A4[ipiv_0]) + 2.0756736E+9F * A2[ipiv_0];
    }

    d = 1.76432251E+10F;
    break;

   default:
    for (ipiv_0 = 0; ipiv_0 < 25; ipiv_0++) {
      F[ipiv_0] = (3.35221289E+10F * A6[ipiv_0] + 1.05594707E+13F * A4[ipiv_0])
        + 1.18735378E+15F * A2[ipiv_0];
    }

    for (e_k = 0; e_k < 5; e_k++) {
      F_tmp = 5 * e_k + e_k;
      F[F_tmp] += 3.23823762E+16F;
    }

    for (ipiv_0 = 0; ipiv_0 < 25; ipiv_0++) {
      V[ipiv_0] = (16380.0F * A4[ipiv_0] + A6[ipiv_0]) + 4.08408E+7F * A2[ipiv_0];
    }

    for (ipiv_0 = 0; ipiv_0 < 5; ipiv_0++) {
      for (e_k = 0; e_k < 5; e_k++) {
        d = 0.0F;
        for (F_tmp = 0; F_tmp < 5; F_tmp++) {
          d += A6[5 * F_tmp + ipiv_0] * V[5 * e_k + F_tmp];
        }

        F_tmp = 5 * e_k + ipiv_0;
        A6_0[F_tmp] = F[F_tmp] + d;
      }
    }

    for (ipiv_0 = 0; ipiv_0 < 5; ipiv_0++) {
      for (e_k = 0; e_k < 5; e_k++) {
        d = 0.0F;
        for (F_tmp = 0; F_tmp < 5; F_tmp++) {
          d += A[5 * F_tmp + e_k] * A6_0[5 * ipiv_0 + F_tmp];
        }

        F[e_k + 5 * ipiv_0] = d;
      }
    }

    for (ipiv_0 = 0; ipiv_0 < 25; ipiv_0++) {
      A6_0[ipiv_0] = (182.0F * A6[ipiv_0] + 960960.0F * A4[ipiv_0]) +
        1.32324198E+9F * A2[ipiv_0];
    }

    for (ipiv_0 = 0; ipiv_0 < 5; ipiv_0++) {
      for (e_k = 0; e_k < 5; e_k++) {
        d = 0.0F;
        for (F_tmp = 0; F_tmp < 5; F_tmp++) {
          d += A6[5 * F_tmp + ipiv_0] * A6_0[5 * e_k + F_tmp];
        }

        F_tmp = 5 * e_k + ipiv_0;
        V[F_tmp] = ((A6[F_tmp] * 6.70442586E+11F + d) + A4[F_tmp] *
                    1.29060194E+14F) + A2[F_tmp] * 7.77177E+15F;
      }
    }

    d = 6.47647525E+16F;
    break;
  }

  for (e_k = 0; e_k < 5; e_k++) {
    F_tmp = 5 * e_k + e_k;
    V[F_tmp] += d;
  }

  for (e_k = 0; e_k < 25; e_k++) {
    d = F[e_k];
    V[e_k] -= d;
    F[e_k] = 2.0F * d;
  }

  xzgetrf(V, ipiv, &e_k);
  for (e_k = 0; e_k < 4; e_k++) {
    ipiv_0 = ipiv[e_k];
    if (e_k + 1 != ipiv_0) {
      for (j = 0; j < 5; j++) {
        ip = 5 * j + e_k;
        d = F[ip];
        F_tmp = (5 * j + ipiv_0) - 1;
        F[ip] = F[F_tmp];
        F[F_tmp] = d;
      }
    }
  }

  for (e_k = 0; e_k < 5; e_k++) {
    ip = 5 * e_k;
    for (j = 0; j < 5; j++) {
      kAcol = 5 * j;
      ipiv_0 = j + ip;
      if (F[ipiv_0] != 0.0F) {
        for (b_i = j + 2; b_i < 6; b_i++) {
          F_tmp = (b_i + ip) - 1;
          F[F_tmp] -= V[(b_i + kAcol) - 1] * F[ipiv_0];
        }
      }
    }
  }

  xtrsm(V, F);
  for (e_k = 0; e_k < 5; e_k++) {
    F_tmp = 5 * e_k + e_k;
    F[F_tmp]++;
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void recomputeBlockDiag(const real32_T A[25], real32_T F[25], const
  int32_T blockFormat[4])
{
  real32_T avg;
  real32_T expa11;
  real32_T expa22;
  real32_T x;
  switch (blockFormat[0]) {
   case 0:
    break;

   case 1:
    expa11 = expf(A[0]);
    expa22 = expf(A[6]);
    avg = (A[0] + A[6]) / 2.0F;
    if (fmaxf(avg, fabsf(A[0] - A[6]) / 2.0F) < 88.7228394F) {
      x = (A[6] - A[0]) / 2.0F;
      if (x == 0.0F) {
        x = 1.0F;
      } else {
        x = sinhf(x) / x;
      }

      avg = A[5] * expf(avg) * x;
    } else {
      avg = (expa22 - expa11) * A[5] / (A[6] - A[0]);
    }

    F[0] = expa11;
    F[5] = avg;
    F[6] = expa22;
    break;

   case 2:
    expa11 = sqrtf(fabsf(A[1] * A[5]));
    expa22 = expf(A[0]);
    if (expa11 == 0.0F) {
      avg = 1.0F;
    } else {
      avg = sinf(expa11) / expa11;
    }

    F[0] = expa22 * cosf(expa11);
    F[1] = expa22 * A[1] * avg;
    F[5] = expa22 * A[5] * avg;
    F[6] = F[0];
    break;
  }

  switch (blockFormat[1]) {
   case 0:
    break;

   case 1:
    expa11 = expf(A[6]);
    expa22 = expf(A[12]);
    avg = (A[6] + A[12]) / 2.0F;
    if (fmaxf(avg, fabsf(A[6] - A[12]) / 2.0F) < 88.7228394F) {
      x = (A[12] - A[6]) / 2.0F;
      if (x == 0.0F) {
        x = 1.0F;
      } else {
        x = sinhf(x) / x;
      }

      avg = A[11] * expf(avg) * x;
    } else {
      avg = (expa22 - expa11) * A[11] / (A[12] - A[6]);
    }

    F[6] = expa11;
    F[11] = avg;
    F[12] = expa22;
    break;

   case 2:
    expa11 = sqrtf(fabsf(A[7] * A[11]));
    expa22 = expf(A[6]);
    if (expa11 == 0.0F) {
      avg = 1.0F;
    } else {
      avg = sinf(expa11) / expa11;
    }

    F[6] = expa22 * cosf(expa11);
    F[7] = expa22 * A[7] * avg;
    F[11] = expa22 * A[11] * avg;
    F[12] = F[6];
    break;
  }

  switch (blockFormat[2]) {
   case 0:
    break;

   case 1:
    expa11 = expf(A[12]);
    expa22 = expf(A[18]);
    avg = (A[12] + A[18]) / 2.0F;
    if (fmaxf(avg, fabsf(A[12] - A[18]) / 2.0F) < 88.7228394F) {
      x = (A[18] - A[12]) / 2.0F;
      if (x == 0.0F) {
        x = 1.0F;
      } else {
        x = sinhf(x) / x;
      }

      avg = A[17] * expf(avg) * x;
    } else {
      avg = (expa22 - expa11) * A[17] / (A[18] - A[12]);
    }

    F[12] = expa11;
    F[17] = avg;
    F[18] = expa22;
    break;

   case 2:
    expa11 = sqrtf(fabsf(A[13] * A[17]));
    expa22 = expf(A[12]);
    if (expa11 == 0.0F) {
      avg = 1.0F;
    } else {
      avg = sinf(expa11) / expa11;
    }

    F[12] = expa22 * cosf(expa11);
    F[13] = expa22 * A[13] * avg;
    F[17] = expa22 * A[17] * avg;
    F[18] = F[12];
    break;
  }

  switch (blockFormat[3]) {
   case 0:
    break;

   case 1:
    expa11 = expf(A[18]);
    expa22 = expf(A[24]);
    avg = (A[18] + A[24]) / 2.0F;
    if (fmaxf(avg, fabsf(A[18] - A[24]) / 2.0F) < 88.7228394F) {
      x = (A[24] - A[18]) / 2.0F;
      if (x == 0.0F) {
        x = 1.0F;
      } else {
        x = sinhf(x) / x;
      }

      avg = A[23] * expf(avg) * x;
    } else {
      avg = (expa22 - expa11) * A[23] / (A[24] - A[18]);
    }

    F[18] = expa11;
    F[23] = avg;
    F[24] = expa22;
    break;

   case 2:
    expa11 = sqrtf(fabsf(A[19] * A[23]));
    expa22 = expf(A[18]);
    if (expa11 == 0.0F) {
      avg = 1.0F;
    } else {
      avg = sinf(expa11) / expa11;
    }

    F[18] = expa22 * cosf(expa11);
    F[19] = expa22 * A[19] * avg;
    F[23] = expa22 * A[23] * avg;
    F[24] = F[18];
    break;
  }

  if (blockFormat[3] == 0) {
    F[24] = expf(A[24]);
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static real32_T xnrm2_c(int32_T n, const real32_T x[25], int32_T ix0)
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

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void xzsyhetrd(real32_T A[25], real32_T D[5], real32_T E[4], real32_T
                      tau[4])
{
  int32_T b_ix;
  int32_T b_ix_tmp;
  int32_T i;
  int32_T iv;
  int32_T iy;
  int32_T iy_tmp;
  int32_T knt;
  int32_T tau_tmp_tmp;
  int32_T temp2_tmp_tmp_tmp;
  real32_T tau_tmp;
  real32_T taui;
  real32_T temp2;
  real32_T xnorm;
  for (i = 0; i < 4; i++) {
    temp2_tmp_tmp_tmp = 5 * i + i;
    temp2 = A[temp2_tmp_tmp_tmp + 1];
    if (i + 3 <= 5) {
      b_ix = i + 3;
    } else {
      b_ix = 5;
    }

    iv = i * 5 + b_ix;
    taui = 0.0F;
    xnorm = xnrm2_c(3 - i, A, iv);
    if (xnorm != 0.0F) {
      xnorm = rt_hypotf_snf(temp2, xnorm);
      if (temp2 >= 0.0F) {
        xnorm = -xnorm;
      }

      if (fabsf(xnorm) < 9.86076132E-32F) {
        knt = 0;
        b_ix_tmp = (iv - i) + 2;
        do {
          knt++;
          for (iy = iv; iy <= b_ix_tmp; iy++) {
            A[iy - 1] *= 1.01412048E+31F;
          }

          xnorm *= 1.01412048E+31F;
          temp2 *= 1.01412048E+31F;
        } while ((fabsf(xnorm) < 9.86076132E-32F) && (knt < 20));

        xnorm = rt_hypotf_snf(temp2, xnrm2_c(3 - i, A, iv));
        if (temp2 >= 0.0F) {
          xnorm = -xnorm;
        }

        taui = (xnorm - temp2) / xnorm;
        temp2 = 1.0F / (temp2 - xnorm);
        for (iy = iv; iy <= b_ix_tmp; iy++) {
          A[iy - 1] *= temp2;
        }

        for (iv = 0; iv < knt; iv++) {
          xnorm *= 9.86076132E-32F;
        }

        temp2 = xnorm;
      } else {
        taui = (xnorm - temp2) / xnorm;
        temp2 = 1.0F / (temp2 - xnorm);
        iy = (iv - i) + 2;
        for (knt = iv; knt <= iy; knt++) {
          A[knt - 1] *= temp2;
        }

        temp2 = xnorm;
      }
    }

    E[i] = temp2;
    if (taui != 0.0F) {
      A[temp2_tmp_tmp_tmp + 1] = 1.0F;
      for (iv = i + 1; iv < 5; iv++) {
        tau[iv - 1] = 0.0F;
      }

      iy_tmp = 3 - i;
      b_ix_tmp = 5 - i;
      for (iv = 0; iv <= iy_tmp; iv++) {
        iy = i + iv;
        xnorm = A[(5 * i + iy) + 1] * taui;
        temp2 = 0.0F;
        b_ix = (iy + 1) * 5 + i;
        tau[iy] += A[(b_ix + iv) + 1] * xnorm;
        for (knt = iv + 2; knt < b_ix_tmp; knt++) {
          tau_tmp_tmp = i + knt;
          tau_tmp = A[b_ix + knt];
          tau[tau_tmp_tmp - 1] += tau_tmp * xnorm;
          temp2 += A[5 * i + tau_tmp_tmp] * tau_tmp;
        }

        tau[iy] += taui * temp2;
      }

      iv = temp2_tmp_tmp_tmp + 1;
      xnorm = 0.0F;
      b_ix = i;
      iy = temp2_tmp_tmp_tmp + 1;
      for (knt = 0; knt <= iy_tmp; knt++) {
        xnorm += tau[b_ix] * A[iy];
        b_ix++;
        iy++;
      }

      xnorm *= -0.5F * taui;
      if (!(xnorm == 0.0F)) {
        iy = i;
        b_ix = 4 - i;
        for (knt = 0; knt < b_ix; knt++) {
          tau[iy] += xnorm * A[iv];
          iv++;
          iy++;
        }
      }

      for (iv = 0; iv <= iy_tmp; iv++) {
        iy = i + iv;
        xnorm = A[(5 * i + iy) + 1];
        temp2 = tau[iy];
        tau_tmp = temp2 * xnorm;
        b_ix = (iy + 1) * 5;
        tau_tmp_tmp = b_ix + i;
        A[(iy + b_ix) + 1] = (A[(tau_tmp_tmp + iv) + 1] - tau_tmp) - tau_tmp;
        for (knt = iv + 2; knt < b_ix_tmp; knt++) {
          iy = i + knt;
          A[iy + b_ix] = (A[tau_tmp_tmp + knt] - tau[iy - 1] * xnorm) - A[5 * i
            + iy] * temp2;
        }
      }
    }

    A[temp2_tmp_tmp_tmp + 1] = E[i];
    D[i] = A[temp2_tmp_tmp_tmp];
    tau[i] = taui;
  }

  D[4] = A[24];
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static real32_T computeNormMTridiagonal(int32_T n, const real32_T d[5], const
  real32_T e[4], int32_T i0)
{
  real32_T y;
  if (n <= 0) {
    y = 0.0F;
  } else {
    int32_T i;
    boolean_T exitg1;
    y = fabsf(d[(i0 + n) - 2]);
    i = -1;
    exitg1 = false;
    while ((!exitg1) && (i + 1 <= n - 2)) {
      int32_T anorm_tmp;
      real32_T anorm;
      anorm_tmp = i0 + i;
      anorm = fabsf(d[anorm_tmp]);
      if (rtIsNaNF(anorm)) {
        y = (rtNaNF);
        exitg1 = true;
      } else {
        if (anorm > y) {
          y = anorm;
        }

        anorm = fabsf(e[anorm_tmp]);
        if (rtIsNaNF(anorm)) {
          y = (rtNaNF);
          exitg1 = true;
        } else {
          if (anorm > y) {
            y = anorm;
          }

          i++;
        }
      }
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void xzlascl(real32_T cfrom, real32_T cto, int32_T m, int32_T n, real32_T
                    A[5], int32_T iA0, int32_T lda)
{
  int32_T b_i;
  int32_T j;
  real32_T cfromc;
  real32_T ctoc;
  boolean_T notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    real32_T cfrom1;
    real32_T cto1;
    real32_T mul;
    cfrom1 = cfromc * 1.97215226E-31F;
    cto1 = ctoc / 5.0706024E+30F;
    if ((fabsf(cfrom1) > fabsf(ctoc)) && (ctoc != 0.0F)) {
      mul = 1.97215226E-31F;
      cfromc = cfrom1;
    } else if (fabsf(cto1) > fabsf(cfromc)) {
      mul = 5.0706024E+30F;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    for (j = 0; j < n; j++) {
      int32_T offset;
      offset = (j * lda + iA0) - 2;
      for (b_i = 0; b_i < m; b_i++) {
        int32_T tmp;
        tmp = (b_i + offset) + 1;
        A[tmp] *= mul;
      }
    }
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void xzlascl_b(real32_T cfrom, real32_T cto, int32_T m, int32_T n,
                      real32_T A[4], int32_T iA0, int32_T lda)
{
  int32_T b_i;
  int32_T j;
  real32_T cfromc;
  real32_T ctoc;
  boolean_T notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    real32_T cfrom1;
    real32_T cto1;
    real32_T mul;
    cfrom1 = cfromc * 1.97215226E-31F;
    cto1 = ctoc / 5.0706024E+30F;
    if ((fabsf(cfrom1) > fabsf(ctoc)) && (ctoc != 0.0F)) {
      mul = 1.97215226E-31F;
      cfromc = cfrom1;
    } else if (fabsf(cto1) > fabsf(cfromc)) {
      mul = 5.0706024E+30F;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    for (j = 0; j < n; j++) {
      int32_T offset;
      offset = (j * lda + iA0) - 2;
      for (b_i = 0; b_i < m; b_i++) {
        int32_T tmp;
        tmp = (b_i + offset) + 1;
        A[tmp] *= mul;
      }
    }
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void xzlartg(real32_T f, real32_T g, real32_T *cs, real32_T *sn, real32_T
                    *r)
{
  real32_T f1;
  f1 = fabsf(f);
  *r = fabsf(g);
  if (g == 0.0F) {
    *cs = 1.0F;
    *sn = 0.0F;
    *r = f;
  } else if (f == 0.0F) {
    *cs = 0.0F;
    if (g >= 0.0F) {
      *sn = 1.0F;
    } else {
      *sn = -1.0F;
    }
  } else if ((f1 > 1.08420217E-19F) && (f1 < 6.5219088E+18F) && (*r >
              1.08420217E-19F) && (*r < 6.5219088E+18F)) {
    *r = sqrtf(f * f + g * g);
    *cs = f1 / *r;
    if (!(f >= 0.0F)) {
      *r = -*r;
    }

    *sn = g / *r;
  } else {
    real32_T fs;
    real32_T gs;
    f1 = fminf(8.50705917E+37F, fmaxf(1.17549435E-38F, fmaxf(f1, *r)));
    fs = f / f1;
    gs = g / f1;
    *r = sqrtf(fs * fs + gs * gs);
    *cs = fabsf(fs) / *r;
    if (!(f >= 0.0F)) {
      *r = -*r;
    }

    *sn = gs / *r;
    *r *= f1;
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void rotateRight_k(int32_T m, int32_T n, real32_T z[25], int32_T iz0,
  int32_T ldz, const real32_T cs[8], int32_T ic0, int32_T is0)
{
  int32_T b_j;
  int32_T j;
  for (b_j = 0; b_j <= n - 2; b_j++) {
    int32_T offsetj;
    int32_T offsetjp1;
    real32_T ctemp;
    real32_T stemp;
    ctemp = cs[(ic0 + b_j) - 1];
    stemp = cs[(is0 + b_j) - 1];
    offsetj = (b_j * ldz + iz0) - 2;
    offsetjp1 = ((b_j + 1) * ldz + iz0) - 2;
    if ((ctemp != 1.0F) || (stemp != 0.0F)) {
      for (j = 0; j < m; j++) {
        int32_T temp_tmp;
        int32_T tmp;
        real32_T temp;
        temp_tmp = (j + offsetjp1) + 1;
        temp = z[temp_tmp];
        tmp = (j + offsetj) + 1;
        z[temp_tmp] = ctemp * temp - z[tmp] * stemp;
        z[tmp] = z[tmp] * ctemp + stemp * temp;
      }
    }
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void xdlaev2(real32_T a, real32_T b, real32_T c, real32_T *rt1, real32_T *
                    rt2, real32_T *cs1, real32_T *sn1)
{
  int32_T sgn1;
  int32_T sgn2;
  real32_T ab;
  real32_T acmn;
  real32_T acmx;
  real32_T adf;
  real32_T df;
  real32_T sm;
  real32_T tb;
  sm = a + c;
  df = a - c;
  adf = fabsf(df);
  tb = b + b;
  ab = fabsf(tb);
  if (fabsf(a) > fabsf(c)) {
    acmx = a;
    acmn = c;
  } else {
    acmx = c;
    acmn = a;
  }

  if (adf > ab) {
    real32_T b_a;
    b_a = ab / adf;
    adf *= sqrtf(b_a * b_a + 1.0F);
  } else if (adf < ab) {
    adf /= ab;
    adf = sqrtf(adf * adf + 1.0F) * ab;
  } else {
    adf = ab * 1.41421354F;
  }

  if (sm < 0.0F) {
    *rt1 = (sm - adf) * 0.5F;
    sgn1 = -1;
    *rt2 = acmx / *rt1 * acmn - b / *rt1 * b;
  } else if (sm > 0.0F) {
    *rt1 = (sm + adf) * 0.5F;
    sgn1 = 1;
    *rt2 = acmx / *rt1 * acmn - b / *rt1 * b;
  } else {
    *rt1 = 0.5F * adf;
    *rt2 = -0.5F * adf;
    sgn1 = 1;
  }

  if (df >= 0.0F) {
    df += adf;
    sgn2 = 1;
  } else {
    df -= adf;
    sgn2 = -1;
  }

  if (fabsf(df) > ab) {
    tb = -tb / df;
    *sn1 = 1.0F / sqrtf(tb * tb + 1.0F);
    *cs1 = tb * *sn1;
  } else if (ab == 0.0F) {
    *cs1 = 1.0F;
    *sn1 = 0.0F;
  } else {
    tb = -df / tb;
    *cs1 = 1.0F / sqrtf(tb * tb + 1.0F);
    *sn1 = tb * *cs1;
  }

  if (sgn1 == sgn2) {
    tb = *cs1;
    *cs1 = -*sn1;
    *sn1 = tb;
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void rotateRight(int32_T m, int32_T n, real32_T z[25], int32_T iz0,
  int32_T ldz, const real32_T cs[8], int32_T ic0, int32_T is0)
{
  int32_T b_i;
  int32_T j;
  for (j = n - 1; j >= 1; j--) {
    int32_T offsetj;
    int32_T offsetjp1;
    real32_T ctemp;
    real32_T stemp;
    ctemp = cs[(ic0 + j) - 2];
    stemp = cs[(is0 + j) - 2];
    offsetj = ((j - 1) * ldz + iz0) - 2;
    offsetjp1 = (j * ldz + iz0) - 2;
    if ((ctemp != 1.0F) || (stemp != 0.0F)) {
      for (b_i = 0; b_i < m; b_i++) {
        int32_T temp_tmp;
        int32_T tmp;
        real32_T temp;
        temp_tmp = (b_i + offsetjp1) + 1;
        temp = z[temp_tmp];
        tmp = (b_i + offsetj) + 1;
        z[temp_tmp] = ctemp * temp - z[tmp] * stemp;
        z[tmp] = z[tmp] * ctemp + stemp * temp;
      }
    }
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static int32_T xzsteqr(real32_T d[5], real32_T e[4], real32_T z[25])
{
  int32_T c_i;
  int32_T exitg1;
  int32_T exitg3;
  int32_T exitg4;
  int32_T i;
  int32_T info;
  int32_T iscale;
  int32_T l;
  int32_T l1;
  int32_T lend;
  int32_T lsv;
  int32_T m;
  int32_T tst_tmp_tmp;
  real32_T work[8];
  real32_T b;
  real32_T b_s;
  real32_T c;
  real32_T e_0;
  real32_T g;
  real32_T p_1;
  real32_T r;
  real32_T s;
  real32_T tst;
  boolean_T exitg2;
  info = 0;
  for (i = 0; i < 8; i++) {
    work[i] = 0.0F;
  }

  i = 0;
  l1 = 1;
  do {
    exitg1 = 0;
    if (l1 > 5) {
      for (c_i = 0; c_i < 4; c_i++) {
        m = c_i;
        p_1 = d[c_i];
        for (i = c_i + 2; i < 6; i++) {
          tst = d[i - 1];
          if (tst < p_1) {
            m = i - 1;
            p_1 = tst;
          }
        }

        if (m != c_i) {
          d[m] = d[c_i];
          d[c_i] = p_1;
          l1 = c_i * 5;
          m *= 5;
          for (i = 0; i < 5; i++) {
            lsv = l1 + i;
            tst = z[lsv];
            l = m + i;
            z[lsv] = z[l];
            z[l] = tst;
          }
        }
      }

      exitg1 = 1;
    } else {
      if (l1 > 1) {
        e[l1 - 2] = 0.0F;
      }

      m = l1;
      exitg2 = false;
      while ((!exitg2) && (m < 5)) {
        tst = fabsf(e[m - 1]);
        if (tst == 0.0F) {
          exitg2 = true;
        } else if (tst <= sqrtf(fabsf(d[m - 1])) * sqrtf(fabsf(d[m])) *
                   1.1920929E-7F) {
          e[m - 1] = 0.0F;
          exitg2 = true;
        } else {
          m++;
        }
      }

      l = l1;
      lsv = l1;
      lend = m;
      l1 = m + 1;
      if (m == l) {
      } else {
        tst_tmp_tmp = m - l;
        tst = computeNormMTridiagonal(tst_tmp_tmp + 1, d, e, l);
        iscale = 0;
        if (tst == 0.0F) {
        } else if (rtIsInfF(tst) || rtIsNaNF(tst)) {
          for (i = 0; i < 5; i++) {
            d[i] = (rtNaNF);
          }

          for (l = 0; l < 25; l++) {
            z[l] = (rtNaNF);
          }

          exitg1 = 1;
        } else {
          if (tst > 3.07445744E+18F) {
            iscale = 1;
            xzlascl(tst, 3.07445744E+18F, tst_tmp_tmp + 1, 1, d, l, 5);
            xzlascl_b(tst, 3.07445744E+18F, tst_tmp_tmp, 1, e, l, 5);
          } else if (tst < 7.62939453E-6F) {
            iscale = 2;
            xzlascl(tst, 7.62939453E-6F, tst_tmp_tmp + 1, 1, d, l, 5);
            xzlascl_b(tst, 7.62939453E-6F, tst_tmp_tmp, 1, e, l, 5);
          }

          if (fabsf(d[m - 1]) < fabsf(d[l - 1])) {
            lend = lsv;
            l = m;
          }

          if (lend > l) {
            do {
              exitg4 = 0;
              if (l != lend) {
                m = l;
                exitg2 = false;
                while ((!exitg2) && (m < lend)) {
                  g = fabsf(e[m - 1]);
                  if (g * g <= fabsf(d[m - 1]) * 1.42108547E-14F * fabsf(d[m]) +
                      1.17549435E-38F) {
                    exitg2 = true;
                  } else {
                    m++;
                  }
                }
              } else {
                m = lend;
              }

              if (m < lend) {
                e[m - 1] = 0.0F;
              }

              if (m == l) {
                l++;
                if (l > lend) {
                  exitg4 = 1;
                }
              } else if (l + 1 == m) {
                xdlaev2(d[l - 1], e[l - 1], d[l], &d[l - 1], &g, &work[l - 1],
                        &s);
                d[l] = g;
                work[l + 3] = s;
                rotateRight(5, 2, z, (l - 1) * 5 + 1, 5, work, l, l + 4);
                e[l - 1] = 0.0F;
                l += 2;
                if (l > lend) {
                  exitg4 = 1;
                }
              } else if (i == 150) {
                exitg4 = 1;
              } else {
                i++;
                p_1 = d[l - 1];
                c = e[l - 1];
                g = (d[l] - p_1) / (c * 2.0F);
                s = rt_hypotf_snf(g, 1.0F);
                if (!(g >= 0.0F)) {
                  s = -s;
                }

                g = (d[m - 1] - p_1) + c / (g + s);
                s = 1.0F;
                c = 1.0F;
                p_1 = 0.0F;
                for (c_i = m - 1; c_i >= l; c_i--) {
                  e_0 = e[c_i - 1];
                  b = c * e_0;
                  xzlartg(g, s * e_0, &c, &b_s, &r);
                  s = b_s;
                  if (m - 1 != c_i) {
                    e[c_i] = r;
                  }

                  g = d[c_i] - p_1;
                  r = (d[c_i - 1] - g) * b_s + 2.0F * c * b;
                  p_1 = b_s * r;
                  d[c_i] = g + p_1;
                  g = c * r - b;
                  work[c_i - 1] = c;
                  work[c_i + 3] = -b_s;
                }

                rotateRight(5, (m - l) + 1, z, (l - 1) * 5 + 1, 5, work, l, l +
                            4);
                d[l - 1] -= p_1;
                e[l - 1] = g;
              }
            } while (exitg4 == 0);
          } else {
            do {
              exitg3 = 0;
              if (l != lend) {
                m = l;
                exitg2 = false;
                while ((!exitg2) && (m > lend)) {
                  g = fabsf(e[m - 2]);
                  if (g * g <= fabsf(d[m - 1]) * 1.42108547E-14F * fabsf(d[m - 2])
                      + 1.17549435E-38F) {
                    exitg2 = true;
                  } else {
                    m--;
                  }
                }
              } else {
                m = lend;
              }

              if (m > lend) {
                e[m - 2] = 0.0F;
              }

              if (m == l) {
                l--;
                if (l < lend) {
                  exitg3 = 1;
                }
              } else if (l - 1 == m) {
                xdlaev2(d[l - 2], e[l - 2], d[l - 1], &d[l - 2], &g, &work[m - 1],
                        &s);
                d[l - 1] = g;
                work[m + 3] = s;
                rotateRight_k(5, 2, z, (l - 2) * 5 + 1, 5, work, m, m + 4);
                e[l - 2] = 0.0F;
                l -= 2;
                if (l < lend) {
                  exitg3 = 1;
                }
              } else if (i == 150) {
                exitg3 = 1;
              } else {
                i++;
                p_1 = d[l - 1];
                c = e[l - 2];
                g = (d[l - 2] - p_1) / (c * 2.0F);
                s = rt_hypotf_snf(g, 1.0F);
                if (!(g >= 0.0F)) {
                  s = -s;
                }

                g = (d[m - 1] - p_1) + c / (g + s);
                s = 1.0F;
                c = 1.0F;
                p_1 = 0.0F;
                for (c_i = m; c_i < l; c_i++) {
                  e_0 = e[c_i - 1];
                  b = c * e_0;
                  xzlartg(g, s * e_0, &c, &b_s, &r);
                  s = b_s;
                  if (c_i != m) {
                    e[c_i - 2] = r;
                  }

                  g = d[c_i - 1] - p_1;
                  r = (d[c_i] - g) * b_s + 2.0F * c * b;
                  p_1 = b_s * r;
                  d[c_i - 1] = g + p_1;
                  g = c * r - b;
                  work[c_i - 1] = c;
                  work[c_i + 3] = b_s;
                }

                rotateRight_k(5, (l - m) + 1, z, (m - 1) * 5 + 1, 5, work, m, m
                              + 4);
                d[l - 1] -= p_1;
                e[l - 2] = g;
              }
            } while (exitg3 == 0);
          }

          switch (iscale) {
           case 1:
            xzlascl(3.07445744E+18F, tst, tst_tmp_tmp + 1, 1, d, lsv, 5);
            xzlascl_b(3.07445744E+18F, tst, tst_tmp_tmp, 1, e, lsv, 5);
            break;

           case 2:
            xzlascl(7.62939453E-6F, tst, tst_tmp_tmp + 1, 1, d, lsv, 5);
            xzlascl_b(7.62939453E-6F, tst, tst_tmp_tmp, 1, e, lsv, 5);
            break;
          }

          if (i >= 150) {
            if (e[0] != 0.0F) {
              info = 1;
            }

            if (e[1] != 0.0F) {
              info++;
            }

            if (e[2] != 0.0F) {
              info++;
            }

            if (e[3] != 0.0F) {
              info++;
            }

            exitg1 = 1;
          }
        }
      }
    }
  } while (exitg1 == 0);

  return info;
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void xsyheev(real32_T A[25], int32_T *info, real32_T W[5])
{
  int32_T b_ia;
  int32_T coltop;
  int32_T e_i;
  int32_T exitg1;
  int32_T f;
  int32_T iaii;
  int32_T itau;
  int32_T jy;
  int32_T lastc;
  real32_T work[5];
  real32_T e[4];
  real32_T tau[4];
  real32_T absx;
  real32_T anrm;
  real32_T cfrom1;
  real32_T cto1;
  real32_T ctoc;
  real32_T mul;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T iscale;
  boolean_T notdone;
  *info = 0;
  anrm = 0.0F;
  e_i = 0;
  exitg2 = false;
  while ((!exitg2) && (e_i < 5)) {
    itau = 0;
    do {
      exitg1 = 0;
      if (itau <= e_i) {
        absx = fabsf(A[5 * e_i + itau]);
        if (rtIsNaNF(absx)) {
          anrm = (rtNaNF);
          exitg1 = 1;
        } else {
          if (absx > anrm) {
            anrm = absx;
          }

          itau++;
        }
      } else {
        e_i++;
        exitg1 = 2;
      }
    } while (exitg1 == 0);

    if (exitg1 == 1) {
      exitg2 = true;
    }
  }

  if (rtIsInfF(anrm) || rtIsNaNF(anrm)) {
    for (e_i = 0; e_i < 5; e_i++) {
      W[e_i] = (rtNaNF);
    }

    for (coltop = 0; coltop < 25; coltop++) {
      A[coltop] = (rtNaNF);
    }
  } else {
    iscale = false;
    guard1 = false;
    if ((anrm > 0.0F) && (anrm < 3.14018486E-16F)) {
      iscale = true;
      anrm = 3.14018486E-16F / anrm;
      guard1 = true;
    } else if (anrm > 3.18452578E+15F) {
      iscale = true;
      anrm = 3.18452578E+15F / anrm;
      guard1 = true;
    }

    if (guard1) {
      absx = 1.0F;
      ctoc = anrm;
      notdone = true;
      while (notdone) {
        cfrom1 = absx * 1.97215226E-31F;
        cto1 = ctoc / 5.0706024E+30F;
        if ((fabsf(cfrom1) > ctoc) && (ctoc != 0.0F)) {
          mul = 1.97215226E-31F;
          absx = cfrom1;
        } else if (cto1 > fabsf(absx)) {
          mul = 5.0706024E+30F;
          ctoc = cto1;
        } else {
          mul = ctoc / absx;
          notdone = false;
        }

        for (coltop = 0; coltop < 25; coltop++) {
          A[coltop] *= mul;
        }
      }
    }

    xzsyhetrd(A, W, e, tau);
    for (e_i = 3; e_i >= 0; e_i--) {
      coltop = (e_i + 1) * 5;
      A[coltop] = 0.0F;
      for (itau = e_i + 3; itau < 6; itau++) {
        A[(itau + coltop) - 1] = A[(5 * e_i + itau) - 1];
      }
    }

    A[0] = 1.0F;
    A[1] = 0.0F;
    A[2] = 0.0F;
    A[3] = 0.0F;
    A[4] = 0.0F;
    for (e_i = 0; e_i < 5; e_i++) {
      work[e_i] = 0.0F;
    }

    for (e_i = 3; e_i >= 0; e_i--) {
      iaii = (e_i * 5 + e_i) + 12;
      if (e_i + 1 < 4) {
        A[iaii - 6] = 1.0F;
        if (tau[e_i] != 0.0F) {
          itau = 4 - e_i;
          lastc = iaii - e_i;
          while ((itau > 0) && (A[lastc - 3] == 0.0F)) {
            itau--;
            lastc--;
          }

          lastc = 2 - e_i;
          exitg2 = false;
          while ((!exitg2) && (lastc + 1 > 0)) {
            coltop = lastc * 5 + iaii;
            b_ia = coltop;
            do {
              exitg1 = 0;
              if (b_ia <= (coltop + itau) - 1) {
                if (A[b_ia - 1] != 0.0F) {
                  exitg1 = 1;
                } else {
                  b_ia++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          itau = 0;
          lastc = -1;
        }

        if (itau > 0) {
          if (lastc + 1 != 0) {
            memset(&work[0], 0, (uint32_T)(lastc + 1) * sizeof(real32_T));
            jy = 5 * lastc + iaii;
            for (coltop = iaii; coltop <= jy; coltop += 5) {
              absx = 0.0F;
              f = (coltop + itau) - 1;
              for (b_ia = coltop; b_ia <= f; b_ia++) {
                absx += A[((iaii + b_ia) - coltop) - 6] * A[b_ia - 1];
              }

              b_ia = div_nde_s32_floor(coltop - iaii, 5);
              work[b_ia] += absx;
            }
          }

          if (!(-tau[e_i] == 0.0F)) {
            jy = iaii;
            for (coltop = 0; coltop <= lastc; coltop++) {
              absx = work[coltop];
              if (absx != 0.0F) {
                absx *= -tau[e_i];
                f = (itau + jy) - 1;
                for (b_ia = jy; b_ia <= f; b_ia++) {
                  A[b_ia - 1] += A[((iaii + b_ia) - jy) - 6] * absx;
                }
              }

              jy += 5;
            }
          }
        }

        lastc = (iaii - e_i) - 2;
        for (itau = iaii - 4; itau <= lastc; itau++) {
          A[itau - 1] *= -tau[e_i];
        }
      }

      A[iaii - 6] = 1.0F - tau[e_i];
      for (itau = 0; itau < e_i; itau++) {
        A[(iaii - itau) - 7] = 0.0F;
      }
    }

    *info = xzsteqr(W, e, A);
    if (*info != 0) {
      for (e_i = 0; e_i < 5; e_i++) {
        W[e_i] = (rtNaNF);
      }

      for (coltop = 0; coltop < 25; coltop++) {
        A[coltop] = (rtNaNF);
      }
    } else if (iscale) {
      anrm = 1.0F / anrm;
      for (e_i = 0; e_i < 5; e_i++) {
        W[e_i] *= anrm;
      }
    }
  }
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static int32_T xpotrf(real32_T b_A[16])
{
  int32_T b_k;
  int32_T info;
  int32_T j;
  int32_T jm1;
  boolean_T exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 4)) {
    int32_T idxAjj;
    real32_T c;
    real32_T ssq;
    idxAjj = (j << 2) + j;
    ssq = 0.0F;
    if (j >= 1) {
      for (b_k = 0; b_k < j; b_k++) {
        c = b_A[(b_k << 2) + j];
        ssq += c * c;
      }
    }

    ssq = b_A[idxAjj] - ssq;
    if (ssq > 0.0F) {
      ssq = sqrtf(ssq);
      b_A[idxAjj] = ssq;
      if (j + 1 < 4) {
        if (j != 0) {
          int32_T b_iy;
          b_iy = (((j - 1) << 2) + j) + 2;
          for (b_k = j + 2; b_k <= b_iy; b_k += 4) {
            int32_T d;
            jm1 = b_k - j;
            c = -b_A[(((jm1 - 2) >> 2) << 2) + j];
            d = jm1 + 2;
            for (jm1 = b_k; jm1 <= d; jm1++) {
              int32_T tmp;
              tmp = ((idxAjj + jm1) - b_k) + 1;
              b_A[tmp] += b_A[jm1 - 1] * c;
            }
          }
        }

        ssq = 1.0F / ssq;
        jm1 = (idxAjj - j) + 4;
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
static real32_T minimum(const real32_T x[4])
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
    while ((!exitg1) && (k < 5)) {
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
    for (k = idx + 1; k < 5; k++) {
      real32_T x_0;
      x_0 = x[k - 1];
      if (ex > x_0) {
        ex = x_0;
      }
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void trisolve(const real32_T b_A[16], real32_T b_B[16])
{
  int32_T b_k;
  int32_T i;
  int32_T j;
  for (j = 0; j < 4; j++) {
    int32_T jBcol;
    jBcol = j << 2;
    for (b_k = 0; b_k < 4; b_k++) {
      int32_T b_B_tmp;
      int32_T kAcol;
      real32_T b_B_0;
      kAcol = b_k << 2;
      b_B_tmp = b_k + jBcol;
      b_B_0 = b_B[b_B_tmp];
      if (b_B_0 != 0.0F) {
        b_B[b_B_tmp] = b_B_0 / b_A[b_k + kAcol];
        for (i = b_k + 2; i < 5; i++) {
          int32_T tmp;
          tmp = (i + jBcol) - 1;
          b_B[tmp] -= b_A[(i + kAcol) - 1] * b_B[b_B_tmp];
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static real32_T norm(const real32_T x[4])
{
  real32_T absxk;
  real32_T scale;
  real32_T t;
  real32_T y;
  scale = 1.29246971E-26F;
  absxk = fabsf(x[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = fabsf(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabsf(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabsf(x[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrtf(y);
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static real32_T maximum(const real32_T x[4])
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
    while ((!exitg1) && (k < 5)) {
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
    for (k = idx + 1; k < 5; k++) {
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
static real32_T xnrm2(int32_T n, const real32_T x[16], int32_T ix0)
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

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void xgemv(int32_T b_m, int32_T n, const real32_T b_A[16], int32_T ia0,
                  const real32_T x[16], int32_T ix0, real32_T y[4])
{
  int32_T b_iy;
  int32_T ia;
  if ((b_m != 0) && (n != 0)) {
    int32_T b;
    if (n - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)n * sizeof(real32_T));
    }

    b = ((n - 1) << 2) + ia0;
    for (b_iy = ia0; b_iy <= b; b_iy += 4) {
      int32_T d;
      real32_T c;
      c = 0.0F;
      d = (b_iy + b_m) - 1;
      for (ia = b_iy; ia <= d; ia++) {
        c += x[((ix0 + ia) - b_iy) - 1] * b_A[ia - 1];
      }

      ia = (b_iy - ia0) >> 2;
      y[ia] += c;
    }
  }
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void xgerc(int32_T b_m, int32_T n, real32_T alpha1, int32_T ix0, const
                  real32_T y[4], real32_T b_A[16], int32_T ia0)
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

      jA += 4;
    }
  }
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static real32_T KWIKfactor(const real32_T b_Ac[184], const int32_T iC[46],
  int32_T nA, const real32_T b_Linv[16], real32_T RLinv[16], real32_T b_D[16],
  real32_T b_H[16], int32_T n)
{
  int32_T b_coltop;
  int32_T b_lastv;
  int32_T coltop;
  int32_T exitg1;
  int32_T ii;
  int32_T k_i;
  int32_T knt;
  real32_T Q[16];
  real32_T R[16];
  real32_T TL[16];
  real32_T b_A[16];
  real32_T tau[4];
  real32_T work[4];
  real32_T Status;
  real32_T atmp;
  real32_T b_A_0;
  real32_T beta1;
  boolean_T exitg2;
  Status = 1.0F;
  memset(&RLinv[0], 0, sizeof(real32_T) << 4U);
  for (k_i = 0; k_i < nA; k_i++) {
    ii = iC[k_i];
    for (b_coltop = 0; b_coltop < 4; b_coltop++) {
      RLinv[b_coltop + (k_i << 2)] = ((b_Ac[ii - 1] * b_Linv[b_coltop] +
        b_Linv[b_coltop + 4] * b_Ac[ii + 45]) + b_Linv[b_coltop + 8] * b_Ac[ii +
        91]) + b_Linv[b_coltop + 12] * b_Ac[ii + 137];
    }
  }

  memcpy(&b_A[0], &RLinv[0], sizeof(real32_T) << 4U);
  tau[0] = 0.0F;
  work[0] = 0.0F;
  tau[1] = 0.0F;
  work[1] = 0.0F;
  tau[2] = 0.0F;
  work[2] = 0.0F;
  tau[3] = 0.0F;
  work[3] = 0.0F;
  for (k_i = 0; k_i < 4; k_i++) {
    ii = (k_i << 2) + k_i;
    if (k_i + 1 < 4) {
      atmp = b_A[ii];
      b_lastv = ii + 2;
      tau[k_i] = 0.0F;
      beta1 = xnrm2(3 - k_i, b_A, ii + 2);
      if (beta1 != 0.0F) {
        b_A_0 = b_A[ii];
        beta1 = rt_hypotf_snf(b_A_0, beta1);
        if (b_A_0 >= 0.0F) {
          beta1 = -beta1;
        }

        if (fabsf(beta1) < 9.86076132E-32F) {
          knt = 0;
          coltop = (ii - k_i) + 4;
          do {
            knt++;
            for (b_coltop = b_lastv; b_coltop <= coltop; b_coltop++) {
              b_A[b_coltop - 1] *= 1.01412048E+31F;
            }

            beta1 *= 1.01412048E+31F;
            atmp *= 1.01412048E+31F;
          } while ((fabsf(beta1) < 9.86076132E-32F) && (knt < 20));

          beta1 = rt_hypotf_snf(atmp, xnrm2(3 - k_i, b_A, ii + 2));
          if (atmp >= 0.0F) {
            beta1 = -beta1;
          }

          tau[k_i] = (beta1 - atmp) / beta1;
          atmp = 1.0F / (atmp - beta1);
          for (b_coltop = b_lastv; b_coltop <= coltop; b_coltop++) {
            b_A[b_coltop - 1] *= atmp;
          }

          for (b_lastv = 0; b_lastv < knt; b_lastv++) {
            beta1 *= 9.86076132E-32F;
          }

          atmp = beta1;
        } else {
          tau[k_i] = (beta1 - b_A_0) / beta1;
          atmp = 1.0F / (b_A_0 - beta1);
          b_coltop = (ii - k_i) + 4;
          for (knt = b_lastv; knt <= b_coltop; knt++) {
            b_A[knt - 1] *= atmp;
          }

          atmp = beta1;
        }
      }

      b_A[ii] = 1.0F;
      if (tau[k_i] != 0.0F) {
        b_lastv = 4 - k_i;
        knt = (ii - k_i) + 3;
        while ((b_lastv > 0) && (b_A[knt] == 0.0F)) {
          b_lastv--;
          knt--;
        }

        knt = 3 - k_i;
        exitg2 = false;
        while ((!exitg2) && (knt > 0)) {
          b_coltop = (((knt - 1) << 2) + ii) + 4;
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
        xgemv(b_lastv, knt, b_A, ii + 5, b_A, ii + 1, work);
        xgerc(b_lastv, knt, -tau[k_i], ii + 1, work, b_A, ii + 5);
      }

      b_A[ii] = atmp;
    } else {
      tau[3] = 0.0F;
    }
  }

  for (k_i = 0; k_i < 4; k_i++) {
    for (ii = 0; ii <= k_i; ii++) {
      b_lastv = k_i << 2;
      R[ii + b_lastv] = b_A[b_lastv + ii];
    }

    for (ii = k_i + 2; ii < 5; ii++) {
      R[(ii + (k_i << 2)) - 1] = 0.0F;
    }

    work[k_i] = 0.0F;
  }

  for (k_i = 3; k_i >= 0; k_i--) {
    b_lastv = ((k_i << 2) + k_i) + 5;
    if (k_i + 1 < 4) {
      b_A[b_lastv - 5] = 1.0F;
      if (tau[k_i] != 0.0F) {
        knt = 4 - k_i;
        b_coltop = b_lastv - k_i;
        while ((knt > 0) && (b_A[b_coltop - 2] == 0.0F)) {
          knt--;
          b_coltop--;
        }

        b_coltop = 3 - k_i;
        exitg2 = false;
        while ((!exitg2) && (b_coltop > 0)) {
          coltop = ((b_coltop - 1) << 2) + b_lastv;
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
        xgemv(knt, b_coltop, b_A, b_lastv, b_A, b_lastv - 4, work);
        xgerc(knt, b_coltop, -tau[k_i], b_lastv - 4, work, b_A, b_lastv);
      }

      b_coltop = b_lastv - k_i;
      for (knt = b_lastv - 3; knt < b_coltop; knt++) {
        b_A[knt - 1] *= -tau[k_i];
      }
    }

    b_A[b_lastv - 5] = 1.0F - tau[k_i];
    for (knt = 0; knt < k_i; knt++) {
      b_A[(b_lastv - knt) - 6] = 0.0F;
    }
  }

  for (k_i = 0; k_i < 4; k_i++) {
    ii = k_i << 2;
    Q[ii] = b_A[ii];
    Q[ii + 1] = b_A[ii + 1];
    Q[ii + 2] = b_A[ii + 2];
    Q[ii + 3] = b_A[ii + 3];
  }

  k_i = 0;
  do {
    exitg1 = 0;
    if (k_i <= nA - 1) {
      if (fabsf(R[(k_i << 2) + k_i]) < 1.0E-12F) {
        Status = -2.0F;
        exitg1 = 1;
      } else {
        k_i++;
      }
    } else {
      for (k_i = 0; k_i < n; k_i++) {
        for (ii = 0; ii < n; ii++) {
          b_lastv = k_i << 2;
          b_coltop = ii << 2;
          TL[k_i + b_coltop] = ((b_Linv[b_lastv + 1] * Q[b_coltop + 1] +
            b_Linv[b_lastv] * Q[b_coltop]) + b_Linv[b_lastv + 2] * Q[b_coltop +
                                2]) + b_Linv[b_lastv + 3] * Q[b_coltop + 3];
        }
      }

      memset(&RLinv[0], 0, sizeof(real32_T) << 4U);
      for (k_i = nA; k_i >= 1; k_i--) {
        b_coltop = (k_i - 1) << 2;
        knt = (k_i + b_coltop) - 1;
        RLinv[knt] = 1.0F;
        for (ii = k_i; ii <= nA; ii++) {
          coltop = (((ii - 1) << 2) + k_i) - 1;
          RLinv[coltop] /= R[knt];
        }

        if (k_i > 1) {
          for (ii = 0; ii <= k_i - 2; ii++) {
            for (b_lastv = k_i; b_lastv <= nA; b_lastv++) {
              knt = (b_lastv - 1) << 2;
              coltop = knt + ii;
              RLinv[coltop] -= RLinv[(knt + k_i) - 1] * R[b_coltop + ii];
            }
          }
        }
      }

      for (k_i = 0; k_i < n; k_i++) {
        for (ii = k_i + 1; ii <= n; ii++) {
          b_coltop = ((ii - 1) << 2) + k_i;
          b_H[b_coltop] = 0.0F;
          for (b_lastv = nA + 1; b_lastv <= n; b_lastv++) {
            knt = (b_lastv - 1) << 2;
            b_H[b_coltop] -= TL[(knt + ii) - 1] * TL[knt + k_i];
          }

          b_H[(ii + (k_i << 2)) - 1] = b_H[b_coltop];
        }
      }

      for (k_i = 0; k_i < nA; k_i++) {
        for (ii = 0; ii < n; ii++) {
          b_coltop = (k_i << 2) + ii;
          b_D[b_coltop] = 0.0F;
          for (b_lastv = k_i + 1; b_lastv <= nA; b_lastv++) {
            knt = (b_lastv - 1) << 2;
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
static void DropConstraint(int32_T kDrop, boolean_T iA[46], int32_T *nA, int32_T
  iC[46])
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
static void qpkwik(const real32_T b_Linv[16], const real32_T b_Hinv[16], const
                   real32_T f[4], const real32_T b_Ac[184], const real32_T b[46],
                   boolean_T iA[46], int32_T maxiter, real32_T FeasTol, real32_T
                   x[4], real32_T lambda[46], int32_T *status)
{
  int32_T iC[46];
  int32_T U_tmp;
  int32_T U_tmp_0;
  int32_T b_exponent;
  int32_T exitg1;
  int32_T exitg3;
  int32_T exponent;
  int32_T i;
  int32_T iC_0;
  int32_T iSave;
  int32_T nA;
  int32_T tmp;
  real32_T cTol[46];
  real32_T RLinv[16];
  real32_T U[16];
  real32_T b_D[16];
  real32_T b_H[16];
  real32_T Opt[8];
  real32_T Rhs[8];
  real32_T r[4];
  real32_T z[4];
  real32_T Xnorm0;
  real32_T cMin;
  real32_T cVal;
  real32_T cVal_tmp;
  real32_T cVal_tmp_0;
  real32_T rMin;
  real32_T t;
  boolean_T ColdReset;
  boolean_T DualFeasible;
  boolean_T cTolComputed;
  boolean_T exitg2;
  boolean_T exitg4;
  boolean_T guard1;
  boolean_T guard2;
  x[0] = 0.0F;
  x[1] = 0.0F;
  x[2] = 0.0F;
  x[3] = 0.0F;
  *status = 1;
  r[0] = 0.0F;
  r[1] = 0.0F;
  r[2] = 0.0F;
  r[3] = 0.0F;
  rMin = 0.0F;
  cTolComputed = false;
  for (i = 0; i < 46; i++) {
    lambda[i] = 0.0F;
    cTol[i] = 1.0F;
    iC[i] = 0;
  }

  nA = 0;
  for (tmp = 0; tmp < 46; tmp++) {
    if (iA[tmp]) {
      nA++;
      iC[nA - 1] = tmp + 1;
    }
  }

  guard1 = false;
  if (nA > 0) {
    for (i = 0; i < 8; i++) {
      Opt[i] = 0.0F;
    }

    Rhs[0] = f[0];
    Rhs[4] = 0.0F;
    Rhs[1] = f[1];
    Rhs[5] = 0.0F;
    Rhs[2] = f[2];
    Rhs[6] = 0.0F;
    Rhs[3] = f[3];
    Rhs[7] = 0.0F;
    DualFeasible = false;
    tmp = (int32_T)roundf(0.3F * (real32_T)nA);
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
            memset(&iC[0], 0, 46U * sizeof(int32_T));
            for (i = 0; i < 46; i++) {
              iA[i] = false;
            }

            ColdReset = true;
          }
        } else {
          for (i = 0; i < nA; i++) {
            Rhs[i + 4] = b[iC[i] - 1];
            for (iSave = i + 1; iSave <= nA; iSave++) {
              U_tmp_0 = ((i << 2) + iSave) - 1;
              U[U_tmp_0] = 0.0F;
              for (iC_0 = 0; iC_0 < nA; iC_0++) {
                U_tmp = iC_0 << 2;
                U[U_tmp_0] += RLinv[(U_tmp + iSave) - 1] * RLinv[U_tmp + i];
              }

              U[i + ((iSave - 1) << 2)] = U[U_tmp_0];
            }
          }

          for (i = 0; i < 4; i++) {
            Opt[i] = ((b_H[i + 4] * Rhs[1] + b_H[i] * Rhs[0]) + b_H[i + 8] *
                      Rhs[2]) + b_H[i + 12] * Rhs[3];
            for (iSave = 0; iSave < nA; iSave++) {
              Opt[i] += b_D[(iSave << 2) + i] * Rhs[iSave + 4];
            }
          }

          for (i = 0; i < nA; i++) {
            iSave = i << 2;
            Opt[i + 4] = ((b_D[iSave + 1] * Rhs[1] + b_D[iSave] * Rhs[0]) +
                          b_D[iSave + 2] * Rhs[2]) + b_D[iSave + 3] * Rhs[3];
            for (iSave = 0; iSave < nA; iSave++) {
              Opt[i + 4] += U[(iSave << 2) + i] * Rhs[iSave + 4];
            }
          }

          Xnorm0 = -1.0E-12F;
          i = -1;
          for (iSave = 0; iSave < nA; iSave++) {
            cMin = Opt[iSave + 4];
            lambda[iC[iSave] - 1] = cMin;
            if ((cMin < Xnorm0) && (iSave + 1 <= nA)) {
              i = iSave;
              Xnorm0 = cMin;
            }
          }

          if (i + 1 <= 0) {
            DualFeasible = true;
            x[0] = Opt[0];
            x[1] = Opt[1];
            x[2] = Opt[2];
            x[3] = Opt[3];
          } else {
            (*status)++;
            if (tmp <= 5) {
              iC_0 = 5;
            } else {
              iC_0 = tmp;
            }

            if (*status > iC_0) {
              nA = 0;
              memset(&iC[0], 0, 46U * sizeof(int32_T));
              for (i = 0; i < 46; i++) {
                iA[i] = false;
              }

              ColdReset = true;
            } else {
              lambda[iC[i] - 1] = 0.0F;
              DropConstraint(i + 1, iA, &nA, iC);
            }
          }
        }
      } else {
        if (nA <= 0) {
          memset(&lambda[0], 0, 46U * sizeof(real32_T));
          Xnorm0 = f[1];
          cMin = f[0];
          cVal = f[2];
          t = f[3];
          for (tmp = 0; tmp < 4; tmp++) {
            x[tmp] = ((-b_Hinv[tmp + 4] * Xnorm0 + -b_Hinv[tmp] * cMin) +
                      -b_Hinv[tmp + 8] * cVal) + -b_Hinv[tmp + 12] * t;
          }
        }

        exitg3 = 1;
      }
    } while (exitg3 == 0);

    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    Xnorm0 = f[1];
    cMin = f[0];
    cVal = f[2];
    t = f[3];
    for (tmp = 0; tmp < 4; tmp++) {
      x[tmp] = ((-b_Hinv[tmp + 4] * Xnorm0 + -b_Hinv[tmp] * cMin) + -b_Hinv[tmp
                + 8] * cVal) + -b_Hinv[tmp + 12] * t;
    }

    guard1 = true;
  }

  if (guard1) {
    Xnorm0 = norm(x);
    exitg2 = false;
    while ((!exitg2) && (*status <= maxiter)) {
      cMin = -FeasTol;
      tmp = -1;
      for (i = 0; i < 46; i++) {
        if (!cTolComputed) {
          z[0] = fabsf(b_Ac[i] * x[0]);
          z[1] = fabsf(b_Ac[i + 46] * x[1]);
          z[2] = fabsf(b_Ac[i + 92] * x[2]);
          z[3] = fabsf(b_Ac[i + 138] * x[3]);
          cTol[i] = fmaxf(cTol[i], maximum(z));
        }

        if (!iA[i]) {
          cVal = ((((b_Ac[i + 46] * x[1] + b_Ac[i] * x[0]) + b_Ac[i + 92] * x[2])
                   + b_Ac[i + 138] * x[3]) - b[i]) / cTol[i];
          if (cVal < cMin) {
            cMin = cVal;
            tmp = i;
          }
        }
      }

      cTolComputed = true;
      if (tmp + 1 <= 0) {
        exitg2 = true;
      } else if (*status == maxiter) {
        *status = 0;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0;
          if ((tmp + 1 > 0) && (*status <= maxiter)) {
            guard2 = false;
            if (nA == 0) {
              for (iC_0 = 0; iC_0 < 4; iC_0++) {
                z[iC_0] = ((b_Hinv[iC_0 + 4] * b_Ac[tmp + 46] + b_Hinv[iC_0] *
                            b_Ac[tmp]) + b_Hinv[iC_0 + 8] * b_Ac[tmp + 92]) +
                  b_Hinv[iC_0 + 12] * b_Ac[tmp + 138];
              }

              guard2 = true;
            } else {
              cMin = KWIKfactor(b_Ac, iC, nA, b_Linv, RLinv, b_D, b_H, degrees);
              if (cMin <= 0.0F) {
                *status = -2;
                exitg1 = 1;
              } else {
                for (iC_0 = 0; iC_0 < 16; iC_0++) {
                  U[iC_0] = -b_H[iC_0];
                }

                for (iC_0 = 0; iC_0 < 4; iC_0++) {
                  z[iC_0] = ((U[iC_0 + 4] * b_Ac[tmp + 46] + U[iC_0] * b_Ac[tmp])
                             + U[iC_0 + 8] * b_Ac[tmp + 92]) + U[iC_0 + 12] *
                    b_Ac[tmp + 138];
                }

                for (i = 0; i < nA; i++) {
                  iSave = i << 2;
                  r[i] = ((b_D[iSave + 1] * b_Ac[tmp + 46] + b_D[iSave] *
                           b_Ac[tmp]) + b_D[iSave + 2] * b_Ac[tmp + 92]) +
                    b_D[iSave + 3] * b_Ac[tmp + 138];
                }

                guard2 = true;
              }
            }

            if (guard2) {
              i = 0;
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
                    if ((i == 0) || (cVal < rMin)) {
                      rMin = cVal;
                      i = iSave + 1;
                    }
                  }
                }

                if (i > 0) {
                  cMin = rMin;
                  DualFeasible = false;
                }
              }

              t = b_Ac[tmp + 46];
              cVal_tmp = b_Ac[tmp + 92];
              cVal_tmp_0 = b_Ac[tmp + 138];
              cVal = ((t * z[1] + z[0] * b_Ac[tmp]) + cVal_tmp * z[2]) +
                cVal_tmp_0 * z[3];
              if (cVal <= 0.0F) {
                cVal = 0.0F;
                ColdReset = true;
              } else {
                cVal = (b[tmp] - (((t * x[1] + b_Ac[tmp] * x[0]) + cVal_tmp * x
                                   [2]) + cVal_tmp_0 * x[3])) / cVal;
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
                  if ((iC_0 <= 46) && (lambda[iC_0 - 1] < 0.0F)) {
                    lambda[iC_0 - 1] = 0.0F;
                  }
                }

                lambda[tmp] += t;
                frexpf(1.0F, &exponent);
                if (fabsf(t - cMin) < 1.1920929E-7F) {
                  DropConstraint(i, iA, &nA, iC);
                }

                if (!ColdReset) {
                  x[0] += t * z[0];
                  x[1] += t * z[1];
                  x[2] += t * z[2];
                  x[3] += t * z[3];
                  frexpf(1.0F, &b_exponent);
                  if (fabsf(t - cVal) < 1.1920929E-7F) {
                    if (nA == degrees) {
                      *status = -1;
                      exitg1 = 1;
                    } else {
                      nA++;
                      iC[nA - 1] = tmp + 1;
                      i = nA - 1;
                      exitg4 = false;
                      while ((!exitg4) && (i + 1 > 1)) {
                        iC_0 = iC[i - 1];
                        if (iC[i] > iC_0) {
                          exitg4 = true;
                        } else {
                          iSave = iC[i];
                          iC[i] = iC_0;
                          iC[i - 1] = iSave;
                          i--;
                        }
                      }

                      iA[tmp] = true;
                      tmp = -1;
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
              for (tmp = 0; tmp < 46; tmp++) {
                cTol[tmp] = fmaxf(fabsf(b[tmp]), 1.0F);
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
static void mpcblock_optimizer(const real32_T rseq[60], const real32_T vseq[42],
  const real32_T x[5], real32_T old_u, const boolean_T iA[46], const real32_T
  b_Mlim[46], real32_T b_Mx[230], real32_T b_Mu1[46], real32_T b_Mv[1932], const
  real32_T b_utarget[20], real32_T b_uoff, real32_T b_H[16], real32_T b_Ac[184],
  const real32_T b_Wy[3], const real32_T b_Jm[60], const real32_T b_I1[20],
  const real32_T b_A[25], const real32_T Bu[105], const real32_T Bv[210], const
  real32_T b_C[15], const real32_T Dv[126], const int32_T b_Mrows[46], real32_T *
  u, real32_T useq[21], real32_T *status, boolean_T iAout[46])
{
  int32_T CA_tmp;
  int32_T Tries;
  int32_T b_Jm_tmp;
  int32_T i;
  int32_T i1;
  real32_T c_Sx[300];
  real32_T WySuJm[180];
  real32_T c_SuJm[180];
  real32_T CA_0[126];
  real32_T Sum_0[60];
  real32_T WduJm[60];
  real32_T WuI2Jm[60];
  real32_T c_Su1[60];
  real32_T b_Mlim_0[46];
  real32_T b_Mlim_1[46];
  real32_T b_Mv_0[46];
  real32_T L[16];
  real32_T CA[15];
  real32_T CA_1[15];
  real32_T b_C_0[6];
  real32_T varargin_1[4];
  real32_T zopt[4];
  real32_T Sum[3];
  real32_T WuI2Jm_0;
  real32_T b_Jm_0;
  real32_T normH;
  real32_T s;
  int16_T ixw;
  int8_T a[400];
  int8_T b[16];
  int8_T rows[3];
  int8_T kidx;
  int8_T rows_0;
  int8_T rows_1;
  static const int8_T c_A[400] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
    0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
    1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T guard2;
  memset(&useq[0], 0, 21U * sizeof(real32_T));
  for (i = 0; i < 46; i++) {
    iAout[i] = false;
  }

  for (i1 = 0; i1 < 3; i1++) {
    Sum[i1] = 0.0F;
    for (Tries = 0; Tries < 5; Tries++) {
      normH = 0.0F;
      for (CA_tmp = 0; CA_tmp < 5; CA_tmp++) {
        normH += b_C[3 * CA_tmp + i1] * b_A[5 * Tries + CA_tmp];
      }

      CA_tmp = 3 * Tries + i1;
      CA[CA_tmp] = normH;
      Sum[i1] += b_C[CA_tmp] * Bu[Tries];
    }

    for (Tries = 0; Tries < 2; Tries++) {
      normH = 0.0F;
      for (CA_tmp = 0; CA_tmp < 5; CA_tmp++) {
        normH += b_C[3 * CA_tmp + i1] * Bv[5 * Tries + CA_tmp];
      }

      b_C_0[i1 + 3 * Tries] = normH;
    }
  }

  for (i1 = 0; i1 < 2; i1++) {
    rtDW.c_Hv[60 * i1] = b_C_0[3 * i1];
    i = (i1 + 2) * 60;
    rtDW.c_Hv[i] = Dv[3 * i1];
    Tries = 3 * i1 + 1;
    rtDW.c_Hv[60 * i1 + 1] = b_C_0[Tries];
    rtDW.c_Hv[i + 1] = Dv[Tries];
    Tries = 3 * i1 + 2;
    rtDW.c_Hv[60 * i1 + 2] = b_C_0[Tries];
    rtDW.c_Hv[i + 2] = Dv[Tries];
  }

  for (i1 = 0; i1 < 38; i1++) {
    i = (i1 + 4) * 60;
    rtDW.c_Hv[i] = 0.0F;
    rtDW.c_Hv[i + 1] = 0.0F;
    rtDW.c_Hv[i + 2] = 0.0F;
  }

  for (i1 = 0; i1 < 42; i1++) {
    memset(&rtDW.c_Hv[i1 * 60 + 3], 0, 57U * sizeof(real32_T));
  }

  for (i1 = 0; i1 < 5; i1++) {
    c_Sx[60 * i1] = CA[3 * i1];
    c_Sx[60 * i1 + 1] = CA[3 * i1 + 1];
    c_Sx[60 * i1 + 2] = CA[3 * i1 + 2];
    memset(&c_Sx[i1 * 60 + 3], 0, 57U * sizeof(real32_T));
  }

  c_Su1[0] = Sum[0];
  c_Su1[1] = Sum[1];
  c_Su1[2] = Sum[2];
  memset(&c_Su1[3], 0, 57U * sizeof(real32_T));
  rtDW.Su[0] = Sum[0];
  rtDW.Su[1] = Sum[1];
  rtDW.Su[2] = Sum[2];
  for (i1 = 0; i1 < 19; i1++) {
    i = (i1 + 1) * 60;
    rtDW.Su[i] = 0.0F;
    rtDW.Su[i + 1] = 0.0F;
    rtDW.Su[i + 2] = 0.0F;
  }

  for (i1 = 0; i1 < 20; i1++) {
    memset(&rtDW.Su[i1 * 60 + 3], 0, 57U * sizeof(real32_T));
  }

  for (i = 0; i < 19; i++) {
    kidx = (int8_T)((i + 1) * 3 + 1);
    for (i1 = 0; i1 < 3; i1++) {
      rows_0 = (int8_T)(i1 + kidx);
      rows[i1] = rows_0;
      normH = 0.0F;
      for (Tries = 0; Tries < 5; Tries++) {
        normH += CA[3 * Tries + i1] * Bu[Tries];
      }

      normH += Sum[i1];
      Sum[i1] = normH;
      c_Su1[rows_0 - 1] = normH;
      Sum_0[i1] = normH;
    }

    rows_0 = rows[0];
    kidx = rows[1];
    rows_1 = rows[2];
    for (i1 = 0; i1 < 19; i1++) {
      Tries = (i1 + 1) * 3;
      Sum_0[Tries] = rtDW.Su[(60 * i1 + rows_0) - 4];
      Sum_0[Tries + 1] = rtDW.Su[(60 * i1 + kidx) - 4];
      Sum_0[Tries + 2] = rtDW.Su[(60 * i1 + rows_1) - 4];
    }

    rows_0 = rows[0];
    kidx = rows[1];
    rows_1 = rows[2];
    for (i1 = 0; i1 < 20; i1++) {
      rtDW.Su[(rows_0 + 60 * i1) - 1] = Sum_0[3 * i1];
      rtDW.Su[(kidx + 60 * i1) - 1] = Sum_0[3 * i1 + 1];
      rtDW.Su[(rows_1 + 60 * i1) - 1] = Sum_0[3 * i1 + 2];
    }

    for (i1 = 0; i1 < 3; i1++) {
      for (Tries = 0; Tries < 2; Tries++) {
        normH = 0.0F;
        for (CA_tmp = 0; CA_tmp < 5; CA_tmp++) {
          normH += CA[3 * CA_tmp + i1] * Bv[5 * Tries + CA_tmp];
        }

        b_C_0[i1 + 3 * Tries] = normH;
      }
    }

    for (i1 = 0; i1 < 2; i1++) {
      CA_0[3 * i1] = b_C_0[3 * i1];
      CA_tmp = 3 * i1 + 1;
      CA_0[CA_tmp] = b_C_0[CA_tmp];
      CA_tmp = 3 * i1 + 2;
      CA_0[CA_tmp] = b_C_0[CA_tmp];
    }

    rows_0 = rows[0];
    kidx = rows[1];
    rows_1 = rows[2];
    for (i1 = 0; i1 < 40; i1++) {
      CA_tmp = (i1 + 2) * 3;
      CA_0[CA_tmp] = rtDW.c_Hv[(60 * i1 + rows_0) - 4];
      CA_0[CA_tmp + 1] = rtDW.c_Hv[(60 * i1 + kidx) - 4];
      CA_0[CA_tmp + 2] = rtDW.c_Hv[(60 * i1 + rows_1) - 4];
    }

    rows_0 = rows[0];
    kidx = rows[1];
    rows_1 = rows[2];
    for (i1 = 0; i1 < 42; i1++) {
      rtDW.c_Hv[(rows_0 + 60 * i1) - 1] = CA_0[3 * i1];
      rtDW.c_Hv[(kidx + 60 * i1) - 1] = CA_0[3 * i1 + 1];
      rtDW.c_Hv[(rows_1 + 60 * i1) - 1] = CA_0[3 * i1 + 2];
    }

    for (i1 = 0; i1 < 3; i1++) {
      for (Tries = 0; Tries < 5; Tries++) {
        normH = 0.0F;
        for (CA_tmp = 0; CA_tmp < 5; CA_tmp++) {
          normH += CA[3 * CA_tmp + i1] * b_A[5 * Tries + CA_tmp];
        }

        CA_1[i1 + 3 * Tries] = normH;
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      CA[i1] = CA_1[i1];
    }

    rows_0 = rows[0];
    kidx = rows[1];
    rows_1 = rows[2];
    for (i1 = 0; i1 < 5; i1++) {
      c_Sx[(rows_0 + 60 * i1) - 1] = CA[3 * i1];
      c_Sx[(kidx + 60 * i1) - 1] = CA[3 * i1 + 1];
      c_Sx[(rows_1 + 60 * i1) - 1] = CA[3 * i1 + 2];
    }
  }

  for (i1 = 0; i1 < 3; i1++) {
    for (Tries = 0; Tries < 60; Tries++) {
      s = 0.0F;
      for (CA_tmp = 0; CA_tmp < 20; CA_tmp++) {
        s += rtDW.Su[60 * CA_tmp + Tries] * b_Jm[20 * i1 + CA_tmp];
      }

      c_SuJm[Tries + 60 * i1] = s;
    }
  }

  if (b_Mrows[0] > 0) {
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 46)) {
      if (b_Mrows[i] <= 60) {
        Tries = b_Mrows[i];
        b_Ac[i] = -c_SuJm[Tries - 1];
        b_Ac[i + 46] = -c_SuJm[Tries + 59];
        b_Ac[i + 92] = -c_SuJm[Tries + 119];
        Tries = b_Mrows[i];
        for (i1 = 0; i1 < 5; i1++) {
          b_Mx[i + 46 * i1] = -c_Sx[(60 * i1 + Tries) - 1];
        }

        b_Mu1[i] = -c_Su1[b_Mrows[i] - 1];
        Tries = b_Mrows[i];
        for (i1 = 0; i1 < 42; i1++) {
          b_Mv[i + 46 * i1] = -rtDW.c_Hv[(60 * i1 + Tries) - 1];
        }

        i++;
      } else if (b_Mrows[i] <= 120) {
        Tries = b_Mrows[i];
        b_Ac[i] = c_SuJm[Tries - 61];
        b_Ac[i + 46] = c_SuJm[Tries - 1];
        b_Ac[i + 92] = c_SuJm[Tries + 59];
        Tries = b_Mrows[i];
        for (i1 = 0; i1 < 5; i1++) {
          b_Mx[i + 46 * i1] = c_Sx[(60 * i1 + Tries) - 61];
        }

        b_Mu1[i] = c_Su1[b_Mrows[i] - 61];
        Tries = b_Mrows[i];
        for (i1 = 0; i1 < 42; i1++) {
          b_Mv[i + 46 * i1] = rtDW.c_Hv[(60 * i1 + Tries) - 61];
        }

        i++;
      } else {
        exitg1 = true;
      }
    }
  }

  i = -1;
  for (Tries = 0; Tries < 20; Tries++) {
    for (i1 = 0; i1 < 20; i1++) {
      a[(i + i1) + 1] = c_A[20 * Tries + i1];
    }

    i += 20;
  }

  for (i1 = 0; i1 < 3; i1++) {
    for (Tries = 0; Tries < 20; Tries++) {
      normH = 0.0F;
      for (CA_tmp = 0; CA_tmp < 20; CA_tmp++) {
        normH += (real32_T)a[20 * CA_tmp + Tries] * b_Jm[20 * i1 + CA_tmp];
      }

      Sum_0[Tries + 20 * i1] = normH;
    }
  }

  ixw = 1;
  for (i = 0; i < 60; i++) {
    normH = b_Wy[ixw - 1];
    WySuJm[i] = normH * c_SuJm[i];
    WySuJm[i + 60] = c_SuJm[i + 60] * normH;
    WySuJm[i + 120] = c_SuJm[i + 120] * normH;
    ixw++;
    if (ixw > 3) {
      ixw = 1;
    }

    WuI2Jm[i] = Sum_0[i] * Wu;
    WduJm[i] = b_Jm[i] * Wdu;
  }

  for (i1 = 0; i1 < 3; i1++) {
    for (Tries = 0; Tries < 3; Tries++) {
      s = 0.0F;
      for (CA_tmp = 0; CA_tmp < 60; CA_tmp++) {
        s += c_SuJm[60 * i1 + CA_tmp] * WySuJm[60 * Tries + CA_tmp];
      }

      normH = 0.0F;
      b_Jm_0 = 0.0F;
      for (CA_tmp = 0; CA_tmp < 20; CA_tmp++) {
        i = 20 * i1 + CA_tmp;
        b_Jm_tmp = 20 * Tries + CA_tmp;
        b_Jm_0 += b_Jm[i] * WduJm[b_Jm_tmp];
        normH += Sum_0[i] * WuI2Jm[b_Jm_tmp];
      }

      b_H[i1 + (Tries << 2)] = (s + b_Jm_0) + normH;
    }

    normH = 0.0F;
    for (Tries = 0; Tries < 60; Tries++) {
      normH += WySuJm[60 * i1 + Tries] * c_Su1[Tries];
    }

    s = 0.0F;
    for (Tries = 0; Tries < 20; Tries++) {
      s += WuI2Jm[20 * i1 + Tries] * b_I1[Tries];
    }

    Sum[i1] = normH + s;
  }

  for (i1 = 0; i1 < 60; i1++) {
    WuI2Jm[i1] = -WuI2Jm[i1];
  }

  for (i1 = 0; i1 < 5; i1++) {
    for (Tries = 0; Tries < 3; Tries++) {
      normH = 0.0F;
      for (CA_tmp = 0; CA_tmp < 60; CA_tmp++) {
        normH += c_Sx[60 * i1 + CA_tmp] * WySuJm[60 * Tries + CA_tmp];
      }

      CA[i1 + 5 * Tries] = normH;
    }
  }

  for (i1 = 0; i1 < 42; i1++) {
    for (Tries = 0; Tries < 3; Tries++) {
      s = 0.0F;
      for (CA_tmp = 0; CA_tmp < 60; CA_tmp++) {
        s += rtDW.c_Hv[60 * i1 + CA_tmp] * WySuJm[60 * Tries + CA_tmp];
      }

      CA_0[i1 + 42 * Tries] = s;
    }
  }

  for (i1 = 0; i1 < 180; i1++) {
    WySuJm[i1] = -WySuJm[i1];
  }

  i = 0;
  memcpy(&L[0], &b_H[0], sizeof(real32_T) << 4U);
  Tries = xpotrf(L);
  guard1 = false;
  if (Tries == 0) {
    varargin_1[0] = L[0];
    varargin_1[1] = L[5];
    varargin_1[2] = L[10];
    varargin_1[3] = L[15];
    if (minimum(varargin_1) > 1.49011612E-7F) {
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    normH = 0.0F;
    Tries = 0;
    exitg2 = false;
    while ((!exitg2) && (Tries < 4)) {
      s = ((fabsf(b_H[Tries + 4]) + fabsf(b_H[Tries])) + fabsf(b_H[Tries + 8]))
        + fabsf(b_H[Tries + 12]);
      if (rtIsNaNF(s)) {
        normH = (rtNaNF);
        exitg2 = true;
      } else {
        if (s > normH) {
          normH = s;
        }

        Tries++;
      }
    }

    if (normH >= 1.0E+10F) {
      i = 2;
    } else {
      Tries = 0;
      exitg1 = false;
      while ((!exitg1) && (Tries <= 4)) {
        normH = rt_powf_snf(10.0F, (real32_T)Tries) * 1.49011612E-7F;
        for (i1 = 0; i1 < 16; i1++) {
          b[i1] = 0;
        }

        b[0] = 1;
        b[5] = 1;
        b[10] = 1;
        b[15] = 1;
        for (i1 = 0; i1 < 16; i1++) {
          s = normH * (real32_T)b[i1] + b_H[i1];
          b_H[i1] = s;
          L[i1] = s;
        }

        i = xpotrf(L);
        guard2 = false;
        if (i == 0) {
          varargin_1[0] = L[0];
          varargin_1[1] = L[5];
          varargin_1[2] = L[10];
          varargin_1[3] = L[15];
          if (minimum(varargin_1) > 1.49011612E-7F) {
            i = 1;
            exitg1 = true;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }

        if (guard2) {
          i = 3;
          Tries++;
        }
      }
    }
  }

  if (i > 1) {
    *u = old_u + b_uoff;
    for (i = 0; i < 21; i++) {
      useq[i] = *u;
    }

    *status = -2.0F;
  } else {
    for (i1 = 0; i1 < 16; i1++) {
      b[i1] = 0;
    }

    b[0] = 1;
    b[5] = 1;
    b[10] = 1;
    b[15] = 1;
    for (i = 0; i < 4; i++) {
      CA_tmp = i << 2;
      b_H[CA_tmp] = b[CA_tmp];
      b_H[CA_tmp + 1] = b[CA_tmp + 1];
      b_H[CA_tmp + 2] = b[CA_tmp + 2];
      b_H[CA_tmp + 3] = b[CA_tmp + 3];
      varargin_1[i] = 0.0F;
    }

    trisolve(L, b_H);
    for (i = 0; i < 3; i++) {
      normH = 0.0F;
      for (i1 = 0; i1 < 5; i1++) {
        normH += CA[5 * i + i1] * x[i1];
      }

      b_Jm_0 = 0.0F;
      for (i1 = 0; i1 < 60; i1++) {
        b_Jm_0 += WySuJm[60 * i + i1] * rseq[i1];
      }

      s = 0.0F;
      for (i1 = 0; i1 < 42; i1++) {
        s += CA_0[42 * i + i1] * vseq[i1];
      }

      WuI2Jm_0 = 0.0F;
      for (i1 = 0; i1 < 20; i1++) {
        WuI2Jm_0 += WuI2Jm[20 * i + i1] * b_utarget[i1];
      }

      varargin_1[i] = (((normH + b_Jm_0) + Sum[i] * old_u) + s) + WuI2Jm_0;
    }

    for (i = 0; i < 46; i++) {
      iAout[i] = iA[i];
      normH = 0.0F;
      for (i1 = 0; i1 < 5; i1++) {
        normH += b_Mx[46 * i1 + i] * x[i1];
      }

      b_Mlim_0[i] = (b_Mlim[i] + normH) + b_Mu1[i] * old_u;
      normH = 0.0F;
      for (i1 = 0; i1 < 42; i1++) {
        normH += b_Mv[46 * i1 + i] * vseq[i1];
      }

      b_Mv_0[i] = normH;
    }

    for (i1 = 0; i1 < 4; i1++) {
      for (Tries = 0; Tries < 4; Tries++) {
        CA_tmp = i1 << 2;
        i = Tries << 2;
        L[i1 + i] = ((b_H[CA_tmp + 1] * b_H[i + 1] + b_H[CA_tmp] * b_H[i]) +
                     b_H[CA_tmp + 2] * b_H[i + 2]) + b_H[CA_tmp + 3] * b_H[i + 3];
      }
    }

    for (i1 = 0; i1 < 46; i1++) {
      b_Mlim_1[i1] = -(b_Mlim_0[i1] + b_Mv_0[i1]);
    }

    qpkwik(b_H, L, varargin_1, b_Ac, b_Mlim_1, iAout, 200, 1.0E-6F, zopt,
           b_Mlim_0, &i);
    if (((int32_T)(real32_T)i < 0) || ((int32_T)(real32_T)i == 0)) {
      zopt[0] = 0.0F;
    }

    *status = (real32_T)i;
    *u = (old_u + zopt[0]) + b_uoff;
  }
}

/* Model step function */
void LateralController_step(void)
{
  int32_T blockFormat[4];
  int32_T Kinv_tmp;
  int32_T Kinv_tmp_0;
  int32_T c_j;
  int32_T e_j;
  int32_T i;
  int32_T rtemp;
  real32_T Cm[315];
  real32_T Bv[210];
  real32_T Dv[126];
  real32_T Bu[105];
  real32_T CovMat[64];
  real32_T rseq[60];
  real32_T b_tmp[56];
  real32_T b_Mlim[46];
  real32_T vseq[42];
  real32_T b_B[40];
  real32_T A[25];
  real32_T A2[25];
  real32_T A4[25];
  real32_T A6[25];
  real32_T M_exp[25];
  real32_T rtb_useq[21];
  real32_T tmp_1[20];
  real32_T tmp_2[20];
  real32_T Cm_0[15];
  real32_T L[15];
  real32_T b_C[15];
  real32_T c_A_tmp[15];
  real32_T tmp_0[15];
  real32_T Kinv[9];
  real32_T c_A[9];
  real32_T b_b[6];
  real32_T rtb_xest[5];
  real32_T xk[5];
  real32_T tmp[3];
  real32_T y_innov[3];
  real32_T A_0;
  real32_T Kinv_tmp_1;
  real32_T Kinv_tmp_2;
  real32_T Kinv_tmp_3;
  real32_T M_exp_0;
  real32_T exptj;
  real32_T rtb_refrence_yaw_rate;
  real32_T u;
  int8_T Dvm[126];
  int8_T b_D[24];
  int8_T c_B[9];
  int8_T UnknownIn[7];
  int8_T b_b_tmp[6];
  boolean_T recomputeDiags;
  static const int8_T c[6] = { 0, 0, 1, 0, -1, 0 };

  static const real32_T c_0[25] = { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.04F, 1.0F,
    0.0F, 0.0F, 0.0F, 0.0422857143F, 0.114285715F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F };

  static const real32_T d[40] = { 0.000415238086F, 0.00114285713F, 0.02F, 0.0F,
    0.0F, -0.014F, -0.7F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.02F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.02F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };

  static const real32_T e[15] = { 20.0F, 0.0F, 0.0F, 0.0F, 0.4F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F };

  static const int8_T f[24] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 1, 0, 0, 0, 1 };

  static const int8_T g[6] = { 1, 1, 1, 35, 35, 35 };

  static const int8_T tmp_3[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const real32_T h[9] = { 0.05F, 2.5F, 1.0F, 0.05F, 2.5F, 1.0F, 0.05F,
    2.5F, 1.0F };

  static const int8_T n[46] = { 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30,
    30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30,
    30, 30, 30, 30, 30, 30, 30, 30, 30, 15, 15, 15, 15, 15, 15 };

  static const real32_T w[230] = { -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    -0.0F, -0.0F, -0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    -0.0F, -0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F,
    -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F,
    -1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
    1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F,
    -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, 1.0F, 1.0F,
    1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
    1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };

  static const real32_T x[46] = { -0.02F, -0.04F, -0.06F, -0.08F, -0.1F, -0.12F,
    -0.14F, -0.16F, -0.18F, -0.2F, -0.22F, -0.24F, -0.26F, -0.28F, -0.3F, -0.32F,
    -0.34F, -0.36F, -0.38F, -0.4F, 0.02F, 0.04F, 0.06F, 0.08F, 0.1F, 0.12F,
    0.14F, 0.16F, 0.18F, 0.2F, 0.22F, 0.24F, 0.26F, 0.28F, 0.3F, 0.32F, 0.34F,
    0.36F, 0.38F, 0.4F, -1.0F, -1.0F, -1.0F, 1.0F, 1.0F, 1.0F };

  static const real32_T y[16] = { 141.606766F, 122.726585F, 105.539963F, 0.0F,
    122.726585F, 106.474976F, 91.6404419F, 0.0F, 105.539963F, 91.6404419F,
    78.9674454F, 0.0F, 0.0F, 0.0F, 0.0F, 100000.0F };

  static const real32_T bb[184] = { -0.02F, -0.04F, -0.06F, -0.08F, -0.1F,
    -0.12F, -0.14F, -0.16F, -0.18F, -0.2F, -0.22F, -0.24F, -0.26F, -0.28F, -0.3F,
    -0.32F, -0.34F, -0.36F, -0.38F, -0.4F, 0.02F, 0.04F, 0.06F, 0.08F, 0.1F,
    0.12F, 0.14F, 0.16F, 0.18F, 0.2F, 0.22F, 0.24F, 0.26F, 0.28F, 0.3F, 0.32F,
    0.34F, 0.36F, 0.38F, 0.4F, -1.0F, -1.0F, -1.0F, 1.0F, 1.0F, 1.0F, -0.0F,
    -0.02F, -0.04F, -0.06F, -0.08F, -0.1F, -0.12F, -0.14F, -0.16F, -0.18F, -0.2F,
    -0.22F, -0.24F, -0.26F, -0.28F, -0.3F, -0.32F, -0.34F, -0.36F, -0.38F, 0.0F,
    0.02F, 0.04F, 0.06F, 0.08F, 0.1F, 0.12F, 0.14F, 0.16F, 0.18F, 0.2F, 0.22F,
    0.24F, 0.26F, 0.28F, 0.3F, 0.32F, 0.34F, 0.36F, 0.38F, -0.0F, -1.0F, -1.0F,
    0.0F, 1.0F, 1.0F, -0.0F, -0.0F, -0.02F, -0.04F, -0.06F, -0.08F, -0.1F,
    -0.12F, -0.14F, -0.16F, -0.18F, -0.2F, -0.22F, -0.24F, -0.26F, -0.28F, -0.3F,
    -0.32F, -0.34F, -0.36F, 0.0F, 0.0F, 0.02F, 0.04F, 0.06F, 0.08F, 0.1F, 0.12F,
    0.14F, 0.16F, 0.18F, 0.2F, 0.22F, 0.24F, 0.26F, 0.28F, 0.3F, 0.32F, 0.34F,
    0.36F, -0.0F, -0.0F, -1.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
    1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
    1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
    1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F };

  static const real32_T cb[3] = { 1.0F, 100.0F, 0.0F };

  static const real32_T db[60] = { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F };

  static const int32_T b_Mrows[46] = { 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33,
    36, 39, 42, 45, 48, 51, 54, 57, 60, 63, 66, 69, 72, 75, 78, 81, 84, 87, 90,
    93, 96, 99, 102, 105, 108, 111, 114, 117, 120, 121, 122, 123, 141, 142, 143
  };

  int32_T exitg1;
  real32_T w_0[230];
  real32_T bb_0[184];
  real32_T x_0[46];
  real32_T y_0[16];
  boolean_T tmp_4[46];
  boolean_T exitg2;

  /* Product: '<S1>/Product' incorporates:
   *  Inport: '<Root>/curvature'
   *  Inport: '<Root>/velocity'
   */
  rtb_refrence_yaw_rate = rtU.velocity_sim * rtU.curvature;

  /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
   *  Constant: '<S1>/Constant2'
   *  Constant: '<S1>/L'
   *  Inport: '<Root>/velocity'
   */
  A[0] = 0.0F;
  A[5] = rtU.velocity_sim * 0.02F;
  A[10] = rtU.velocity_sim * 0.02F;
  A[1] = 0.0F;
  A[6] = 0.0F;
  A[11] = rtU.velocity_sim / 0.35F * 0.02F;
  A[2] = 0.0F;
  A[7] = 0.0F;
  A[12] = 0.0F;
  for (i = 0; i < 2; i++) {
    c_j = (i + 3) * 5;
    A[c_j] = (real32_T)c[3 * i] * 0.02F;
    A[c_j + 1] = (real32_T)c[3 * i + 1] * 0.02F;
    A[c_j + 2] = (real32_T)c[3 * i + 2] * 0.02F;
  }

  for (i = 0; i < 5; i++) {
    A[5 * i + 3] = 0.0F;
    A[5 * i + 4] = 0.0F;
  }

  recomputeDiags = true;
  for (e_j = 0; e_j < 25; e_j++) {
    if (recomputeDiags) {
      A_0 = A[e_j];
      if (rtIsInfF(A_0) || rtIsNaNF(A_0)) {
        recomputeDiags = false;
      }
    }
  }

  if (!recomputeDiags) {
    for (i = 0; i < 25; i++) {
      M_exp[i] = (rtNaNF);
    }
  } else {
    e_j = 0;
    exitg2 = false;
    while ((!exitg2) && (e_j < 5)) {
      c_j = 0;
      do {
        exitg1 = 0;
        if (c_j < 5) {
          if ((c_j != e_j) && (!(A[5 * e_j + c_j] == 0.0F))) {
            recomputeDiags = false;
            exitg1 = 1;
          } else {
            c_j++;
          }
        } else {
          e_j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }

    if (recomputeDiags) {
      memset(&M_exp[0], 0, 25U * sizeof(real32_T));
      for (e_j = 0; e_j < 5; e_j++) {
        rtemp = 5 * e_j + e_j;
        M_exp[rtemp] = expf(A[rtemp]);
      }
    } else {
      recomputeDiags = true;
      e_j = 0;
      exitg2 = false;
      while ((!exitg2) && (e_j < 5)) {
        c_j = 0;
        do {
          exitg1 = 0;
          if (c_j <= e_j) {
            if (!(A[5 * e_j + c_j] == A[5 * c_j + e_j])) {
              recomputeDiags = false;
              exitg1 = 1;
            } else {
              c_j++;
            }
          } else {
            e_j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }

      if (recomputeDiags) {
        xsyheev(A, &e_j, xk);
        for (e_j = 0; e_j < 5; e_j++) {
          exptj = expf(xk[e_j]);
          for (c_j = 0; c_j < 5; c_j++) {
            rtemp = 5 * e_j + c_j;
            M_exp[rtemp] = A[rtemp] * exptj;
          }
        }

        for (i = 0; i < 5; i++) {
          for (Kinv_tmp = 0; Kinv_tmp < 5; Kinv_tmp++) {
            M_exp_0 = 0.0F;
            for (rtemp = 0; rtemp < 5; rtemp++) {
              M_exp_0 += M_exp[5 * rtemp + i] * A[5 * rtemp + Kinv_tmp];
            }

            A4[i + 5 * Kinv_tmp] = M_exp_0;
          }
        }

        memcpy(&M_exp[0], &A4[0], 25U * sizeof(real32_T));
        for (i = 0; i < 5; i++) {
          for (Kinv_tmp = 0; Kinv_tmp < 5; Kinv_tmp++) {
            rtemp = 5 * i + Kinv_tmp;
            A4[rtemp] = (M_exp[5 * Kinv_tmp + i] + M_exp[rtemp]) / 2.0F;
          }
        }

        memcpy(&M_exp[0], &A4[0], 25U * sizeof(real32_T));
      } else {
        recomputeDiags = true;
        e_j = 3;
        while (recomputeDiags && (e_j <= 5)) {
          c_j = e_j;
          while (recomputeDiags && (c_j <= 5)) {
            recomputeDiags = (A[((e_j - 3) * 5 + c_j) - 1] == 0.0F);
            c_j++;
          }

          e_j++;
        }

        if (recomputeDiags) {
          rtemp = 1;
          exitg2 = false;
          while ((!exitg2) && (rtemp - 1 < 4)) {
            i = (rtemp - 1) * 5 + rtemp;
            u = A[i];
            if (u != 0.0F) {
              if ((rtemp != 4) && (A[(5 * rtemp + rtemp) + 1] != 0.0F)) {
                recomputeDiags = false;
                exitg2 = true;
              } else {
                Kinv_tmp = 5 * rtemp + rtemp;
                if (A[i - 1] != A[Kinv_tmp]) {
                  recomputeDiags = false;
                  exitg2 = true;
                } else {
                  A_0 = A[Kinv_tmp - 1];
                  if (rtIsNaNF(u)) {
                    M_exp_0 = (rtNaNF);
                  } else if (u < 0.0F) {
                    M_exp_0 = -1.0F;
                  } else {
                    M_exp_0 = (real32_T)(u > 0.0F);
                  }

                  if (rtIsNaNF(A_0)) {
                    exptj = (rtNaNF);
                  } else if (A_0 < 0.0F) {
                    exptj = -1.0F;
                  } else {
                    exptj = (real32_T)(A_0 > 0.0F);
                  }

                  if (M_exp_0 * exptj != -1.0F) {
                    recomputeDiags = false;
                    exitg2 = true;
                  } else {
                    rtemp++;
                  }
                }
              }
            } else {
              rtemp++;
            }
          }
        }

        getExpmParams(A, A2, A4, A6, &e_j, &exptj);
        if (exptj != 0.0F) {
          A_0 = rt_powf_snf(2.0F, exptj);
          for (i = 0; i < 25; i++) {
            A[i] /= A_0;
          }

          A_0 = rt_powf_snf(2.0F, 2.0F * exptj);
          for (i = 0; i < 25; i++) {
            A2[i] /= A_0;
          }

          A_0 = rt_powf_snf(2.0F, 4.0F * exptj);
          for (i = 0; i < 25; i++) {
            A4[i] /= A_0;
          }

          A_0 = rt_powf_snf(2.0F, 6.0F * exptj);
          for (i = 0; i < 25; i++) {
            A6[i] /= A_0;
          }
        }

        if (recomputeDiags) {
          blockFormat[0] = 0;
          blockFormat[1] = 0;
          blockFormat[2] = 0;
          blockFormat[3] = 0;
          c_j = 0;
          while (c_j + 1 < 4) {
            M_exp_0 = A[(5 * c_j + c_j) + 1];
            if (M_exp_0 != 0.0F) {
              blockFormat[c_j] = 2;
              blockFormat[c_j + 1] = 0;
              c_j += 2;
            } else if ((M_exp_0 == 0.0F) && (A[((c_j + 1) * 5 + c_j) + 2] ==
                        0.0F)) {
              blockFormat[c_j] = 1;
              c_j++;
            } else {
              blockFormat[c_j] = 0;
              c_j++;
            }
          }

          if (A[19] != 0.0F) {
            blockFormat[3] = 2;
          } else {
            switch (blockFormat[2]) {
             case 0:
              blockFormat[3] = 1;
              break;

             case 1:
              blockFormat[3] = 1;
              break;
            }
          }
        }

        padeApproximation(A, A2, A4, A6, e_j, M_exp);
        if (recomputeDiags) {
          recomputeBlockDiag(A, M_exp, blockFormat);
        }

        e_j = (int32_T)exptj;
        for (c_j = 0; c_j < e_j; c_j++) {
          for (i = 0; i < 5; i++) {
            for (Kinv_tmp = 0; Kinv_tmp < 5; Kinv_tmp++) {
              M_exp_0 = 0.0F;
              for (rtemp = 0; rtemp < 5; rtemp++) {
                M_exp_0 += M_exp[5 * rtemp + i] * M_exp[5 * Kinv_tmp + rtemp];
              }

              A4[i + 5 * Kinv_tmp] = M_exp_0;
            }
          }

          memcpy(&M_exp[0], &A4[0], 25U * sizeof(real32_T));
          if (recomputeDiags) {
            for (i = 0; i < 25; i++) {
              A[i] *= 2.0F;
            }

            recomputeBlockDiag(A, M_exp, blockFormat);
          }
        }
      }
    }
  }

  /* MATLAB Function: '<S32>/FixedHorizonOptimizer' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  Memory: '<S4>/LastPcov'
   *  Memory: '<S4>/last_x'
   */
  memset(&Bu[0], 0, 105U * sizeof(real32_T));
  memset(&Bv[0], 0, 210U * sizeof(real32_T));
  memset(&Dv[0], 0, 126U * sizeof(real32_T));
  memset(&Dvm[0], 0, 126U * sizeof(int8_T));
  memset(&Cm[0], 0, 315U * sizeof(real32_T));
  memcpy(&A[0], &c_0[0], 25U * sizeof(real32_T));
  memcpy(&b_B[0], &d[0], 40U * sizeof(real32_T));
  for (i = 0; i < 15; i++) {
    b_C[i] = e[i];
  }

  for (i = 0; i < 24; i++) {
    b_D[i] = f[i];
  }

  for (i = 0; i < 6; i++) {
    b_b_tmp[i] = g[i];
  }

  for (i = 0; i < 2; i++) {
    /* MATLAB Function: '<S1>/MATLAB Function' */
    c_j = (i + 3) * 5;
    b_b[3 * i] = (real32_T)b_b_tmp[3 * i] * M_exp[c_j];
    e_j = 3 * i + 1;
    b_b[e_j] = M_exp[c_j + 1] * (real32_T)b_b_tmp[e_j];
    e_j = 3 * i + 2;
    b_b[e_j] = M_exp[c_j + 2] * (real32_T)b_b_tmp[e_j];
  }

  for (i = 0; i < 3; i++) {
    b_C[3 * i] = (real32_T)tmp_3[3 * i] / h[3 * i];
    A[5 * i] = M_exp[5 * i];

    /* MATLAB Function: '<S1>/MATLAB Function' */
    c_j = 3 * i + 1;
    b_C[c_j] = (real32_T)tmp_3[c_j] / h[c_j];

    /* MATLAB Function: '<S1>/MATLAB Function' */
    c_j = 5 * i + 1;
    A[c_j] = M_exp[c_j];

    /* MATLAB Function: '<S1>/MATLAB Function' */
    c_j = 3 * i + 2;
    b_C[c_j] = (real32_T)tmp_3[c_j] / h[c_j];

    /* MATLAB Function: '<S1>/MATLAB Function' */
    c_j = 5 * i + 2;
    A[c_j] = M_exp[c_j];
    b_B[i] = b_b[i];
    b_B[i + 5] = b_b[i + 3];
    b_D[i + 3] = 0;
  }

  for (i = 0; i < 5; i++) {
    Bu[i] = b_B[i];
  }

  for (i = 0; i < 10; i++) {
    Bv[i] = b_B[i + 5];
  }

  for (i = 0; i < 5; i++) {
    Cm[3 * i] = b_C[3 * i];
    c_j = 3 * i + 1;
    Cm[c_j] = b_C[c_j];
    c_j = 3 * i + 2;
    Cm[c_j] = b_C[c_j];
  }

  for (i = 0; i < 2; i++) {
    c_j = (i + 1) * 3;
    Dvm[3 * i] = b_D[c_j];
    Dvm[3 * i + 1] = b_D[c_j + 1];
    Dvm[3 * i + 2] = b_D[c_j + 2];
  }

  for (i = 0; i < 7; i++) {
    UnknownIn[i] = 0;
  }

  UnknownIn[0] = 1;
  UnknownIn[1] = 2;
  for (i = 0; i < 5; i++) {
    UnknownIn[i + 2] = (int8_T)(i + 4);
  }

  for (i = 0; i < 7; i++) {
    for (Kinv_tmp = 0; Kinv_tmp < 5; Kinv_tmp++) {
      b_tmp[Kinv_tmp + (i << 3)] = b_B[(UnknownIn[i] - 1) * 5 + Kinv_tmp];
    }

    c_j = (UnknownIn[i] - 1) * 3;
    e_j = i << 3;
    b_tmp[e_j + 5] = b_D[c_j];
    b_tmp[e_j + 6] = b_D[c_j + 1];
    b_tmp[e_j + 7] = b_D[c_j + 2];
  }

  for (i = 0; i < 8; i++) {
    for (Kinv_tmp = 0; Kinv_tmp < 8; Kinv_tmp++) {
      A_0 = 0.0F;
      for (rtemp = 0; rtemp < 7; rtemp++) {
        c_j = rtemp << 3;
        A_0 += b_tmp[c_j + i] * b_tmp[c_j + Kinv_tmp];
      }

      CovMat[i + (Kinv_tmp << 3)] = A_0;
    }
  }

  for (i = 0; i < 2; i++) {
    c_j = (i + 1) * 3;
    Dv[3 * i] = b_D[c_j];
    Dv[3 * i + 1] = b_D[c_j + 1];
    Dv[3 * i + 2] = b_D[c_j + 2];
  }

  for (i = 0; i < 46; i++) {
    b_Mlim[i] = n[i];
  }

  Bv[5] = 0.0F;
  Bv[6] = 0.0F;
  Bv[7] = 0.0F;
  memset(&vseq[0], 0, 42U * sizeof(real32_T));
  for (e_j = 0; e_j < 21; e_j++) {
    vseq[(e_j << 1) + 1] = 1.0F;
  }

  for (c_j = 0; c_j < 20; c_j++) {
    rseq[c_j * ny] = 0.0F;
    rseq[c_j * ny + 1] = 0.0F;
    rseq[c_j * ny + 2] = 0.0F;
  }

  for (c_j = 0; c_j < 21; c_j++) {
    vseq[c_j << 1] = RMDscale * rtb_refrence_yaw_rate;
  }

  A_0 = vseq[0];
  u = vseq[1];
  for (i = 0; i < 9; i++) {
    c_B[i] = 0;
  }

  for (c_j = 0; c_j < 3; c_j++) {
    c_B[c_j + 3 * c_j] = 1;
    for (i = 0; i < 5; i++) {
      e_j = 3 * i + c_j;
      c_A_tmp[i + 5 * c_j] = Cm[e_j];
      rtb_refrence_yaw_rate = 0.0F;
      for (Kinv_tmp = 0; Kinv_tmp < 5; Kinv_tmp++) {
        rtb_refrence_yaw_rate += Cm[3 * Kinv_tmp + c_j] *
          rtDW.LastPcov_PreviousInput[5 * i + Kinv_tmp];
      }

      Cm_0[e_j] = rtb_refrence_yaw_rate;
    }
  }

  for (i = 0; i < 3; i++) {
    for (Kinv_tmp = 0; Kinv_tmp < 3; Kinv_tmp++) {
      M_exp_0 = 0.0F;
      for (rtemp = 0; rtemp < 5; rtemp++) {
        M_exp_0 += Cm_0[3 * rtemp + i] * c_A_tmp[5 * Kinv_tmp + rtemp];
      }

      c_A[i + 3 * Kinv_tmp] = CovMat[(((Kinv_tmp + 5) << 3) + i) + 5] + M_exp_0;
    }
  }

  e_j = 0;
  c_j = 1;
  i = 2;
  rtb_refrence_yaw_rate = fabsf(c_A[0]);
  exptj = fabsf(c_A[1]);
  if (exptj > rtb_refrence_yaw_rate) {
    rtb_refrence_yaw_rate = exptj;
    e_j = 1;
    c_j = 0;
  }

  if (fabsf(c_A[2]) > rtb_refrence_yaw_rate) {
    e_j = 2;
    c_j = 1;
    i = 0;
  }

  c_A[c_j] /= c_A[e_j];
  c_A[i] /= c_A[e_j];
  c_A[c_j + 3] -= c_A[e_j + 3] * c_A[c_j];
  c_A[i + 3] -= c_A[e_j + 3] * c_A[i];
  c_A[c_j + 6] -= c_A[e_j + 6] * c_A[c_j];
  c_A[i + 6] -= c_A[e_j + 6] * c_A[i];
  if (fabsf(c_A[i + 3]) > fabsf(c_A[c_j + 3])) {
    rtemp = c_j;
    c_j = i;
    i = rtemp;
  }

  c_A[i + 3] /= c_A[c_j + 3];
  c_A[i + 6] -= c_A[i + 3] * c_A[c_j + 6];
  Kinv[3 * e_j] = (real32_T)c_B[0] / c_A[e_j];
  rtb_refrence_yaw_rate = c_A[e_j + 3];
  Kinv[3 * c_j] = (real32_T)c_B[3] - Kinv[3 * e_j] * rtb_refrence_yaw_rate;
  M_exp_0 = c_A[e_j + 6];
  Kinv[3 * i] = (real32_T)c_B[6] - Kinv[3 * e_j] * M_exp_0;
  exptj = c_A[c_j + 3];
  Kinv[3 * c_j] /= exptj;
  Kinv_tmp_1 = c_A[c_j + 6];
  Kinv[3 * i] -= Kinv[3 * c_j] * Kinv_tmp_1;
  Kinv_tmp_2 = c_A[i + 6];
  Kinv[3 * i] /= Kinv_tmp_2;
  Kinv_tmp_3 = c_A[i + 3];
  Kinv[3 * c_j] -= Kinv[3 * i] * Kinv_tmp_3;
  Kinv[3 * e_j] -= Kinv[3 * i] * c_A[i];
  Kinv[3 * e_j] -= Kinv[3 * c_j] * c_A[c_j];
  Kinv_tmp = 3 * e_j + 1;
  Kinv[Kinv_tmp] = (real32_T)c_B[1] / c_A[e_j];
  rtemp = 3 * c_j + 1;
  Kinv[rtemp] = (real32_T)c_B[4] - Kinv[Kinv_tmp] * rtb_refrence_yaw_rate;
  Kinv_tmp_0 = 3 * i + 1;
  Kinv[Kinv_tmp_0] = (real32_T)c_B[7] - Kinv[Kinv_tmp] * M_exp_0;
  Kinv[rtemp] /= exptj;
  Kinv[Kinv_tmp_0] -= Kinv[rtemp] * Kinv_tmp_1;
  Kinv[Kinv_tmp_0] /= Kinv_tmp_2;
  Kinv[rtemp] -= Kinv[Kinv_tmp_0] * Kinv_tmp_3;
  Kinv[Kinv_tmp] -= Kinv[Kinv_tmp_0] * c_A[i];
  Kinv[Kinv_tmp] -= Kinv[rtemp] * c_A[c_j];
  Kinv_tmp = 3 * e_j + 2;
  Kinv[Kinv_tmp] = (real32_T)c_B[2] / c_A[e_j];
  rtemp = 3 * c_j + 2;
  Kinv[rtemp] = (real32_T)c_B[5] - Kinv[Kinv_tmp] * rtb_refrence_yaw_rate;
  Kinv_tmp_0 = 3 * i + 2;
  Kinv[Kinv_tmp_0] = (real32_T)c_B[8] - Kinv[Kinv_tmp] * M_exp_0;
  Kinv[rtemp] /= exptj;
  Kinv[Kinv_tmp_0] -= Kinv[rtemp] * Kinv_tmp_1;
  Kinv[Kinv_tmp_0] /= Kinv_tmp_2;
  Kinv[rtemp] -= Kinv[Kinv_tmp_0] * Kinv_tmp_3;
  Kinv[Kinv_tmp] -= Kinv[Kinv_tmp_0] * c_A[i];
  Kinv[Kinv_tmp] -= Kinv[rtemp] * c_A[c_j];
  for (i = 0; i < 5; i++) {
    for (Kinv_tmp = 0; Kinv_tmp < 5; Kinv_tmp++) {
      M_exp_0 = 0.0F;
      for (rtemp = 0; rtemp < 5; rtemp++) {
        M_exp_0 += A[5 * rtemp + i] * rtDW.LastPcov_PreviousInput[5 * Kinv_tmp +
          rtemp];
      }

      M_exp[i + 5 * Kinv_tmp] = M_exp_0;
    }

    for (Kinv_tmp = 0; Kinv_tmp < 3; Kinv_tmp++) {
      M_exp_0 = 0.0F;
      for (rtemp = 0; rtemp < 5; rtemp++) {
        M_exp_0 += M_exp[5 * rtemp + i] * c_A_tmp[5 * Kinv_tmp + rtemp];
      }

      Cm_0[i + 5 * Kinv_tmp] = CovMat[((Kinv_tmp + 5) << 3) + i] + M_exp_0;
    }

    rtb_refrence_yaw_rate = Cm_0[i + 5];
    M_exp_0 = Cm_0[i];
    exptj = Cm_0[i + 10];
    for (Kinv_tmp = 0; Kinv_tmp < 3; Kinv_tmp++) {
      L[i + 5 * Kinv_tmp] = (Kinv[3 * Kinv_tmp + 1] * rtb_refrence_yaw_rate +
        Kinv[3 * Kinv_tmp] * M_exp_0) + Kinv[3 * Kinv_tmp + 2] * exptj;
    }

    xk[i] = Bu[i] * 0.0F + rtDW.last_x_PreviousInput[i];
  }

  /* SignalConversion generated from: '<S33>/ SFunction ' incorporates:
   *  Inport: '<Root>/lateral_deviation'
   *  Inport: '<Root>/relative_yaw_angle'
   *  MATLAB Function: '<S32>/FixedHorizonOptimizer'
   *  UnitDelay: '<S1>/Unit Delay'
   */
  tmp[0] = rtU.lateral_deviation * 20.0F;
  tmp[1] = rtU.relative_yaw_angle * 0.4F;
  tmp[2] = rtDW.UnitDelay_DSTATE;

  /* MATLAB Function: '<S32>/FixedHorizonOptimizer' incorporates:
   *  Memory: '<S4>/LastPcov'
   */
  for (i = 0; i < 3; i++) {
    rtb_refrence_yaw_rate = 0.0F;
    for (Kinv_tmp = 0; Kinv_tmp < 5; Kinv_tmp++) {
      rtb_refrence_yaw_rate += Cm[3 * Kinv_tmp + i] * xk[Kinv_tmp];
    }

    y_innov[i] = tmp[i] - (((real32_T)Dvm[i + 3] * u + (real32_T)Dvm[i] * A_0) +
      rtb_refrence_yaw_rate);
  }

  for (i = 0; i < 5; i++) {
    for (Kinv_tmp = 0; Kinv_tmp < 3; Kinv_tmp++) {
      M_exp_0 = 0.0F;
      for (rtemp = 0; rtemp < 5; rtemp++) {
        M_exp_0 += rtDW.LastPcov_PreviousInput[5 * rtemp + i] * c_A_tmp[5 *
          Kinv_tmp + rtemp];
      }

      tmp_0[i + 5 * Kinv_tmp] = M_exp_0;
    }

    M_exp_0 = 0.0F;
    exptj = tmp_0[i + 5];
    rtb_refrence_yaw_rate = tmp_0[i];
    Kinv_tmp_1 = tmp_0[i + 10];
    for (Kinv_tmp = 0; Kinv_tmp < 3; Kinv_tmp++) {
      M_exp_0 += ((Kinv[3 * Kinv_tmp + 1] * exptj + Kinv[3 * Kinv_tmp] *
                   rtb_refrence_yaw_rate) + Kinv[3 * Kinv_tmp + 2] * Kinv_tmp_1)
        * y_innov[Kinv_tmp];
    }

    rtb_xest[i] = xk[i] + M_exp_0;
  }

  memset(&rtDW.fv[0], 0, 1932U * sizeof(real32_T));
  for (i = 0; i < 20; i++) {
    tmp_1[i] = 0.0F;
    tmp_2[i] = 1.0F;
  }

  /* Memory: '<S4>/Memory' */
  for (i = 0; i < 46; i++) {
    tmp_4[i] = rtDW.Memory_PreviousInput[i];
  }

  /* End of Memory: '<S4>/Memory' */

  /* MATLAB Function: '<S32>/FixedHorizonOptimizer' */
  memcpy(&w_0[0], &w[0], 230U * sizeof(real32_T));
  memcpy(&x_0[0], &x[0], 46U * sizeof(real32_T));
  memcpy(&y_0[0], &y[0], sizeof(real32_T) << 4);
  memcpy(&bb_0[0], &bb[0], 184U * sizeof(real32_T));

  /* Update for Memory: '<S4>/Memory' incorporates:
   *  MATLAB Function: '<S32>/FixedHorizonOptimizer'
   *  UnitDelay: '<S4>/last_mv'
   */
  mpcblock_optimizer(rseq, vseq, rtb_xest, rtDW.last_mv_DSTATE, tmp_4, b_Mlim,
                     w_0, x_0, rtDW.fv, tmp_1, 0.0F, y_0, bb_0, cb, db, tmp_2, A,
                     Bu, Bv, b_C, Dv, b_Mrows, &rtb_refrence_yaw_rate, rtb_useq,
                     &exptj, rtDW.Memory_PreviousInput);

  /* MATLAB Function: '<S32>/FixedHorizonOptimizer' */
  for (i = 0; i < 5; i++) {
    for (Kinv_tmp = 0; Kinv_tmp < 5; Kinv_tmp++) {
      M_exp_0 = 0.0F;
      for (rtemp = 0; rtemp < 5; rtemp++) {
        M_exp_0 += M_exp[5 * rtemp + i] * A[5 * rtemp + Kinv_tmp];
      }

      rtemp = 5 * Kinv_tmp + i;
      A4[rtemp] = M_exp_0;
      A6[rtemp] = (Cm_0[i + 5] * L[Kinv_tmp + 5] + Cm_0[i] * L[Kinv_tmp]) +
        Cm_0[i + 10] * L[Kinv_tmp + 10];
    }
  }

  for (i = 0; i < 5; i++) {
    for (Kinv_tmp = 0; Kinv_tmp < 5; Kinv_tmp++) {
      c_j = 5 * i + Kinv_tmp;
      A2[c_j] = CovMat[(i << 3) + Kinv_tmp] + (A4[c_j] - A6[c_j]);
    }
  }

  /* Switch: '<S1>/Switch' incorporates:
   *  RelationalOperator: '<S1>/IsNaN'
   */
  if (rtIsNaNF(rtb_xest[2])) {
    /* Outport: '<Root>/steering_angle' incorporates:
     *  Constant: '<S1>/Constant1'
     */
    rtY.steering_angle = 0.0F;
  } else {
    /* Outport: '<Root>/steering_angle' */
    rtY.steering_angle = rtb_xest[2];
  }

  /* End of Switch: '<S1>/Switch' */

  /* Update for UnitDelay: '<S4>/last_mv' incorporates:
   *  MATLAB Function: '<S32>/FixedHorizonOptimizer'
   */
  rtDW.last_mv_DSTATE = rtb_refrence_yaw_rate;

  /* Update for UnitDelay: '<S1>/Unit Delay' */
  rtDW.UnitDelay_DSTATE = rtb_xest[2];
  for (i = 0; i < 5; i++) {
    /* MATLAB Function: '<S32>/FixedHorizonOptimizer' */
    M_exp_0 = 0.0F;
    for (Kinv_tmp = 0; Kinv_tmp < 5; Kinv_tmp++) {
      rtemp = 5 * Kinv_tmp + i;
      M_exp_0 += A[rtemp] * xk[Kinv_tmp];
      c_j = 5 * i + Kinv_tmp;

      /* Update for Memory: '<S4>/LastPcov' */
      rtDW.LastPcov_PreviousInput[c_j] = (A2[c_j] + A2[rtemp]) * 0.5F;
    }

    /* Update for Memory: '<S4>/last_x' incorporates:
     *  MATLAB Function: '<S32>/FixedHorizonOptimizer'
     */
    rtDW.last_x_PreviousInput[i] = ((Bv[i + 5] * u + Bv[i] * A_0) + (Bu[i] *
      rtb_refrence_yaw_rate + M_exp_0)) + ((L[i + 5] * y_innov[1] + L[i] *
      y_innov[0]) + L[i + 10] * y_innov[2]);
  }
}

/* Model initialize function */
void LateralController_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* InitializeConditions for Memory: '<S4>/LastPcov' */
  memcpy(&rtDW.LastPcov_PreviousInput[0], &rtConstP.LastPcov_InitialCondition[0],
         25U * sizeof(real32_T));
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
