#ifndef AGR_Sate_TABLES_QMF
#define AGR_Sate_TABLES_QMF

#include "SKP_Silk_typedef.h"
#include "AGR_BWE_defines.h"
#include "AGR_BWE_arch.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef FIXED_POINT
extern const spx_word16_t AGR_Sate_qmf_coeffs_fix[64];
#else
extern const SKP_float AGR_Sate_qmf_coeffs[QMF_ORDER];
#endif
#ifdef __cplusplus
}
#endif

#endif