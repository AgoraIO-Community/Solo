#ifndef AGR_Sate_TABLES_QMF
#define AGR_Sate_TABLES_QMF

#include "SKP_Silk_typedef.h"
#include "AGR_BWE_defines.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef FIXED_POINT
extern const spx_word16_t h0[64];
#else
extern const SKP_float AGR_Sate_qmf_coeffs[QMF_ORDER];
extern const SKP_int16 QMF_Coeffs_Q16[QMF_ORDER];
#endif
#ifdef __cplusplus
}
#endif

#endif
