#include "AGR_BWE_main_FIX.h"
#include "SKP_Silk_main_FIX.h"

SKP_int32 AGR_Sate_find_HB_LPC_FIX(
    AGR_Sate_encoder_hb_state_FIX      *psEnc,              /* I/O  Encoder state FLP       */
    AGR_Sate_HB_encoder_control_FIX    *hbEncCtrl,          /* I/O  HB Encoder control FLP  */
    SKP_int32                       hb_subfr_length,    /* I    subframe length         */
    SKP_int32                       hb_lpc_order,       /* I    high band lpc order     */
    SKP_int32                       first               /* I                            */
)
{
    SKP_int32 k;
	SKP_int32 lsp_idx;

    SKP_int32 NLSF_Q15[MAX_LPC_ORDER];
    const SKP_int16 *x_ptr_fix;
    SKP_int16       *x_pre_ptr_fix, LPC_in_pre_fix[NB_SUBFR * MAX_LPC_ORDER + 1280];

	/* Create signal with prepended subframes */
	x_ptr_fix = psEnc->x_hb_buf_fix + hbEncCtrl->BWE_FrameSize - hb_lpc_order;
	x_pre_ptr_fix = LPC_in_pre_fix; // each contains LPC ORDER + SUBFRAME LENGTH

	// copy unscaled high band signals to LPC_in_pre buffer
	for( k = 0; k < NB_SUBFR; k++ ) {
        SKP_memcpy(x_pre_ptr_fix, x_ptr_fix, sizeof(SKP_int16) * (hb_subfr_length + hb_lpc_order));
		x_pre_ptr_fix += hb_subfr_length + hb_lpc_order;
		x_ptr_fix     += hb_subfr_length;
	}

    /* LPC_in_pre contains unfiltered input for HB signal */
    SKP_Silk_find_LPC_FIX(NLSF_Q15, &psEnc->HB_NLSFInterpCoef_Q2, psEnc->HB_prev_NLSFq_Q15,
                            0, hb_lpc_order, LPC_in_pre_fix, hb_subfr_length + hb_lpc_order);

	// use interp flag is disabled [ psEnc->sCmn.useInterpolatedNLSFs * ( 1 - psEnc->sCmn.first_frame_after_reset ) ]
  
	lsp_idx = AGR_Sate_lsp_quant_highband(NLSF_Q15,hb_lpc_order);

    if (first)
        SKP_memcpy(psEnc->HB_prev_NLSFq_Q15, NLSF_Q15, hb_lpc_order * sizeof(SKP_int32));

	for (k = 0; k < NB_SUBFR; k ++) {
        SKP_memcpy(&psEnc->interp_NLSF_Q15[k], NLSF_Q15, sizeof(SKP_int32) * hb_lpc_order);
	}

    /* Copy to prediction struct for use in next frame for fluctuation reduction */
    SKP_memcpy(psEnc->HB_prev_NLSFq_Q15, NLSF_Q15, hb_lpc_order * sizeof(SKP_int32));

	return lsp_idx;
}

