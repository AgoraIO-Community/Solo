#include "AGR_BWE_main_FLP.h"


SKP_int32 AGR_Sate_find_HB_LPC_FLP(
    AGR_Sate_encoder_hb_state_FLP      *psEnc,              /* I/O  Encoder state FLP       */
    AGR_Sate_HB_encoder_control_FLP    *hbEncCtrl,          /* I/O  HB Encoder control FLP  */
    SKP_int32                       hb_subfr_length,    /* I    subframe length         */
    SKP_int32                       hb_lpc_order,       /* I    high band lpc order     */
    SKP_int32                       first               /* I                            */
)
{
    SKP_int32       i,k;
    SKP_float       NLSF[ MAX_LPC_ORDER ];
    const SKP_float *x_ptr;
    SKP_float       *x_pre_ptr, LPC_in_pre[ NB_SUBFR * MAX_LPC_ORDER + 1280 ];
	SKP_float        alpha;
	SKP_int32        lsp_idx;

	/* Create signal with prepended subframes */
	x_ptr = psEnc->x_hb_buf + hbEncCtrl->BWE_FrameSize - hb_lpc_order;
	x_pre_ptr = LPC_in_pre; // each contains LPC ORDER + SUBFRAME LENGTH

	// copy unscaled high band signals to LPC_in_pre buffer
	for( k = 0; k < NB_SUBFR; k++ ) {
		SKP_Silk_scale_copy_vector_FLP(x_pre_ptr, x_ptr, 1, hb_subfr_length + hb_lpc_order);
		x_pre_ptr += hb_subfr_length + hb_lpc_order;
		x_ptr     += hb_subfr_length;
	}

    /* LPC_in_pre contains unfiltered input for HB signal */
	SKP_Silk_find_LPC_FLP(NLSF, &psEnc->HB_NLSFInterpCoef_Q2, psEnc->HB_prev_NLSFq,
							0, hb_lpc_order,  LPC_in_pre, hb_subfr_length + hb_lpc_order );
	// use interp flag is disabled [ psEnc->sCmn.useInterpolatedNLSFs * ( 1 - psEnc->sCmn.first_frame_after_reset ) ]

	// Quantize HB_LSP here ...
	lsp_idx = AGR_Sate_lsp_quant_highband(NLSF,hb_lpc_order);//quantized NLSF

	if (first)
		memcpy( psEnc->HB_prev_NLSFq, NLSF, hb_lpc_order * sizeof( SKP_float ) );

	for (k = 0; k < NB_SUBFR; k ++) {
		alpha = (1.0f + k) /NB_SUBFR;
		for (i = 0; i < hb_lpc_order; i++) {
			// use full frame NLSF is slightly better than interpolation
			//psEnc->interp_NLSF[k][i] = (1-alpha) * psEnc->HB_prev_NLSFq[i] + alpha * NLSF[i];
			psEnc->interp_NLSF[k][i] =  NLSF[i];
		}
	}

    /* Copy to prediction struct for use in next frame for fluctuation reduction */
    memcpy( psEnc->HB_prev_NLSFq, NLSF, hb_lpc_order * sizeof( SKP_float ) );

	/* convert NLSF to HB Pred Coef, disable interpolation mode for the time being */
    SKP_Silk_NLSF2A_stable_FLP(hbEncCtrl->BWE_PredCoef[1], NLSF, hb_lpc_order);
	memcpy(hbEncCtrl->BWE_PredCoef[0], hbEncCtrl->BWE_PredCoef[1], hb_lpc_order * sizeof(SKP_float));

	return lsp_idx;
}

