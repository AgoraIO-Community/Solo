#ifndef AGR_Sate_TABLES_HIGHBAND_COEFF_H
#define AGR_Sate_TABLES_HIGHBAND_COEFF_H

#include "AGR_BWE_defines.h"
#include "SKP_Silk_typedef.h"

#ifdef __cplusplus
extern "C"
{
#endif

//tables for NLSF quantization
extern const SKP_float AGR_Sate_highband_lsp_cdbk1[HB_LSP_CB1 * HB_LPC_ORDER];
extern const SKP_float AGR_Sate_highband_lsp_cdbk2[HB_LSP_CB2 * HB_LPC_ORDER];

//tables for gain quantization
extern const SKP_float AGR_Sate_highband_gain_cdbk[HB_GAIN_CB];;

#ifdef __cplusplus
}
#endif

#endif
