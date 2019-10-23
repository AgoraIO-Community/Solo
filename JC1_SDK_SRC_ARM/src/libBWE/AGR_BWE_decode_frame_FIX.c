#include "AGR_BWE_main_FIX.h"
#include "AGR_BWE_tables_qmf.h"
#include "AGR_BWE_tables_highband_coeff.h"
#include "AGR_BWE_bits.h"
#include "SKP_Silk_SigProc_FIX.h"

void AGR_Sate_PLC_interp_HB_parameters_fix(
    AGR_Sate_decoder_hb_state_FIX      *psHBDec,             /* I/O  Decoder state  */
    SKP_int16                       pGain_fix[],         /* I/O  high band gain */
    SKP_int32                       pLSP_fix[],          /* I/O  lsp coeffs     */
    SKP_int32                       order,               /* I    lpc order      */
    SKP_int32                       nlost                /* I    lost frames    */
)
{
    SKP_int32 i, k;

    SKP_int16 last_gain_fix, next_gain_fix;
    SKP_int32 alpha_fix;
    SKP_int32 last_lsp_fix[MAX_LPC_ORDER], next_lsp_fix[MAX_LPC_ORDER];

    last_gain_fix = psHBDec->HB_prev_Gain_fix;
    next_gain_fix = pGain_fix[0];
    if (psHBDec->hb_lossCnt == 0) {
        psHBDec->hb_gIncr_fix = (next_gain_fix - last_gain_fix) / (nlost * NB_SUBFR + 1);
    }

    for (k = 0; k < NB_SUBFR; k++) {
        pGain_fix[k] = last_gain_fix + psHBDec->hb_gIncr_fix*(k + psHBDec->hb_lossCnt*NB_SUBFR + 1);
    }

    SKP_memcpy(last_lsp_fix, psHBDec->HB_prev_NLSFq_fix, order * sizeof(SKP_int32));
    SKP_memcpy(next_lsp_fix, pLSP_fix, order * sizeof(SKP_int32));

    alpha_fix = ((psHBDec->hb_lossCnt + 1) << 16) / (nlost + 1);
    for (i = 0; i < order; i++) {
        pLSP_fix[i] = SKP_SMULWW(last_lsp_fix[i], (65535 - alpha_fix)) + SKP_SMULWW(next_lsp_fix[i], alpha_fix);
    }
}

SKP_int32 AGR_Bwe_decode_frame_FIX(
	AGR_Sate_HB_decoder_control_FIX *hbDecCtrl,
	AGR_Sate_decoder_hb_state_FIX *psHBDec,
	NovaBits *bits,                                     /* I    bitstream operator      */
	SKP_int16 *OutHigh,
	SKP_int32 *residue_Q10,
	SKP_int32 lostflag                                  /* I    lost falg               */
	)
{
	SKP_int32 hb_lostflag;
	SKP_int32 sub, hb_idx;

	SKP_int32 QHB_LSP_fix[MAX_LPC_ORDER];
	SKP_int16 lpc_fix[MAX_LPC_ORDER];
	SKP_int16 QGain_transfer_fix[HB_SUBFR];
	SKP_int16 *p_out_fix;

	hb_lostflag = lostflag;
	// In case of packet loss, interpolate LSP & gain
	if ((lostflag == 1)|| (hb_lostflag == 2)){
		// if no look-ahead packet is available in jitter buffer, use prev gain and shape
		SKP_memcpy(QHB_LSP_fix, psHBDec->HB_prev_NLSFq_fix, hbDecCtrl->BWE_LPCOrder * sizeof(SKP_int32));
		for (sub = 0; sub<HB_SUBFR; sub++) {
			QGain_transfer_fix[sub] = psHBDec->HB_prev_Gain_fix;
		}
		memset(residue_Q10, 0, sizeof(SKP_int32)*MAX_INTERNAL_FRAME_SIZE);
		psHBDec->hb_lossCnt++;
	}else {
		psHBDec->hb_gIncr_fix = 0;
		// no packet loss
		hb_idx = AGR_Sate_bits_unpack_unsigned(bits, 12);
		AGR_Sate_lsp_dequant_highband(QHB_LSP_fix, hb_idx, hbDecCtrl->BWE_LPCOrder);
		for (sub = 0; sub<HB_SUBFR; sub++) {
			hb_idx = AGR_Sate_bits_unpack_unsigned(bits, 5);
			QGain_transfer_fix[sub] = AGR_Sate_highband_gain_cdbk_fix[hb_idx];
		}

		if (hbDecCtrl->first) {
			SKP_memcpy(psHBDec->HB_prev_NLSFq_fix, QHB_LSP_fix, hbDecCtrl->BWE_LPCOrder * sizeof(SKP_int32));
			psHBDec->HB_prev_Gain_fix = QGain_transfer_fix[HB_SUBFR - 1];
		}

		if ((hb_lostflag == 1)|| (hb_lostflag == 2)){
			// temp solution, use prev parameters, easy for evaluation.
			for (sub = 0; sub<HB_SUBFR; sub++) {
				QGain_transfer_fix[sub] = psHBDec->HB_prev_Gain_fix;
			}
			SKP_memcpy(psHBDec->HB_prev_NLSFq_fix, QHB_LSP_fix, hbDecCtrl->BWE_LPCOrder * sizeof(SKP_int32));
		}

		psHBDec->hb_lossCnt = 0;
	}

	p_out_fix = OutHigh;
	for (sub = 0; sub<HB_SUBFR; sub++)
	{
		// exciation modulation by spectrum folding
		SKP_Silk_NLSF2A_stable(lpc_fix, QHB_LSP_fix, hbDecCtrl->BWE_LPCOrder);

		// resynthesize high band signal with folded exc * encoder gain + encoder lpc
		AGR_Sate_LPC_synthesis_filter_fix(&residue_Q10[sub*hbDecCtrl->BWE_SubFrameSize], lpc_fix, -FOLDING_GAIN_FIX * QGain_transfer_fix[sub],
			psHBDec->HB_synth_state, p_out_fix, hbDecCtrl->BWE_SubFrameSize, hbDecCtrl->BWE_LPCOrder);

		p_out_fix += hbDecCtrl->BWE_SubFrameSize;
	}

	if ((lostflag == 0) || (lostflag == 4) || (lostflag == 3)){
		psHBDec->HB_prev_Gain_fix = QGain_transfer_fix[NB_SUBFR - 1];
		SKP_memcpy(psHBDec->HB_prev_NLSFq_fix, QHB_LSP_fix, hbDecCtrl->BWE_LPCOrder * sizeof(SKP_int32));
	}

	hbDecCtrl->first = 0;
    
    return 0;

}

SKP_int32 hb_lostflag = 0; // effective only if LB is not lost, i.e., lostflag = 0;
SKP_int32 AGR_Sate_decode_process(
    SATEDecCtl *sateCtl,                                /* I/O  SATE Decoder state      */
    NovaBits *bits,                                     /* I    bitstream operator      */
    SKP_int16 *vout,                                    /* O    output signal           */
    void *skdecCtrl, 
	void *hbdecCtrl, 	
    SKP_int16 nBytes[],                                 /* I    input bitstram size     */
    SKP_int32 lostflag                                  /* I    lost falg               */
)
{
	void *psDec;
	AGR_Sate_decoder_hb_state_FIX *psHBDec;
	SKP_SILK_SDK_DecControlStruct *DecControl;
	AGR_Sate_HB_decoder_control_FIX *hbDecCtrl;

	SKP_int32 sub, ret;
	SKP_uint8 payloadToDec[MAX_BYTES_PER_FRAME * MAX_INPUT_FRAMES]; ///* silk max payload size */
	SKP_int32 res_Q10[MAX_INTERNAL_FRAME_SIZE * 2];
	SKP_int16 OutLow[MAX_INTERNAL_FRAME_SIZE], *pOutLow;
	SKP_int16 OutHigh[MAX_INTERNAL_FRAME_SIZE], *pOutHigh;
	SKP_int16 nBytesIn[6] = { 0, 0, 0, 0, 0, 0 };
	SKP_int32 *p_res_Q10 = res_Q10;
	SKP_uint8 *pstream = payloadToDec;
	SKP_int16 nSample;	
    
	psDec = (void *)sateCtl->stDec;
	psHBDec = sateCtl->stHBDec;
	DecControl = (SKP_SILK_SDK_DecControlStruct*)&sateCtl->decControl;
	hbDecCtrl = (AGR_Sate_HB_decoder_control_FIX*)&sateCtl->HBdecControl;
	memset(res_Q10, 0, sizeof(SKP_int32)*MAX_INTERNAL_FRAME_SIZE);

	/*-----------------------------Low band processing -----start-------------------------------------*/
	if(bits){
		if (lostflag == 2){/*only received md1 stream, there isn't HB steam in md1 stream*/
			nBytes[0] = (bits->nbBits >> 3);
		}else{
			nBytes[0] = (bits->nbBits >> 3) - (hbDecCtrl->QMF_HB_FrameSize / hbDecCtrl->BWE_FrameSize)*HB_BYTE;
		}
        if (nBytes[1]){
			nBytes[1] = nBytes[1] - (hbDecCtrl->QMF_HB_FrameSize / hbDecCtrl->BWE_FrameSize)*HB_BYTE;
        }
		memcpy(payloadToDec, bits->chars, nBytes[0] * sizeof(SKP_uint8));
		bits->charPtr = nBytes[0];
	}else{
		nBytes[0] = 0;
	}

	pstream = payloadToDec;

	nBytes[0] -= nBytes[1];
	nBytesIn[0] = nBytes[0];
	nBytesIn[1] = nBytes[1];


	pOutLow = OutLow;
	for (sub = 0; sub < HB2LB_NUM; sub++){
		nSample = hbDecCtrl->SATE_FrameSize;
		ret = SKP_Silk_SDK_Decode(psDec, DecControl, lostflag, pstream, nBytes, pOutLow, &nSample);

        if (ret < 0){
			return ret;
        }

		SKP_Silk_SDK_Get_Decoder_Residue(psDec, p_res_Q10);
		pOutLow += hbDecCtrl->SATE_FrameSize;
		p_res_Q10 += hbDecCtrl->SATE_FrameSize;
	}

	pOutHigh = OutHigh;
	p_res_Q10 = res_Q10;
	for (sub = 0; sub < hbDecCtrl->QMF_HB_FrameSize; sub += hbDecCtrl->BWE_FrameSize){
		AGR_Bwe_decode_frame_FIX(hbDecCtrl, psHBDec, bits, pOutHigh, p_res_Q10, lostflag);
		pOutHigh += hbDecCtrl->BWE_FrameSize;
		p_res_Q10 += hbDecCtrl->BWE_FrameSize;
	}
	/*----------------------------Silk High band processing -----end-------------------------------------*/
	AGR_Sate_qmf_synth(OutLow, OutHigh, AGR_Sate_qmf_coeffs_fix, vout, hbDecCtrl->JC1_FrameSize, QMF_ORDER, hbDecCtrl->g0_mem, hbDecCtrl->g1_mem, NULL);

	return 0;
}
