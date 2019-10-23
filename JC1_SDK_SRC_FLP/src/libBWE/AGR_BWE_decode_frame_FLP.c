#include "AGR_BWE_main_FLP.h"
#include "AGR_BWE_tables_qmf.h"
#include "AGR_BWE_tables_highband_coeff.h"
#include "AGR_BWE_bits.h"

void AGR_Sate_PLC_interp_HB_parameters(
	AGR_Sate_decoder_hb_state_FLP      *psHBDec,             /* I/O  Decoder state  */
	SKP_float                       pGain[],             /* I/O  high band gain */
	SKP_float                       pLSP[],              /* I/O  lsp coeffs     */
	SKP_int32                       order,               /* I    lpc order      */
	SKP_int32                       nlost                /* I    lost frames    */
)
{
	SKP_int32 i, k;
	SKP_float last_gain, next_gain, alpha;
	SKP_float last_lsp[MAX_LPC_ORDER], next_lsp[MAX_LPC_ORDER];

	// --------------------------------update gain------------------------------------------
	last_gain = psHBDec->HB_prev_Gain;
	next_gain = pGain[0];
	if (psHBDec->hb_lossCnt == 0) {
		psHBDec->hb_gIncr = (next_gain - last_gain) / (nlost * NB_SUBFR + 1);
	}

	for (k = 0; k < NB_SUBFR; k++) {
		pGain[k] = (last_gain + psHBDec->hb_gIncr*(k + psHBDec->hb_lossCnt*NB_SUBFR + 1));
	}

	// --------------------------------update shape------------------------------------------
	memcpy(last_lsp, psHBDec->HB_prev_NLSFq, order * sizeof(SKP_float));
	memcpy(next_lsp, pLSP, order * sizeof(SKP_float));

	alpha = (SKP_float)(psHBDec->hb_lossCnt + 1) / (nlost + 1);
	for (i = 0; i < order; i++) {
		pLSP[i] = (1 - alpha) * last_lsp[i] + alpha * next_lsp[i];
	}

}


SKP_int32 AGR_Bwe_decode_frame_FLP(
	AGR_Sate_HB_decoder_control_FLP *hbDecCtrl,
	AGR_Sate_decoder_hb_state_FLP *psHBDec,
	NovaBits *bits,                                     /* I    bitstream operator      */
	SKP_float *OutHigh,
	SKP_int32 *residue_Q10,
	SKP_int32 lostflag                                  /* I    lost falg               */
	)
{
	SKP_float *p_out;
	SKP_float QHB_LSP[16];
	SKP_float  res_f[MAX_INTERNAL_FRAME_SIZE];
	SKP_int i, hb_lostflag, sub, hb_idx;
	SKP_float  HB_PredCoef[MAX_LPC_ORDER];
	SKP_float  HB_LPCRes[MAX_SUBFR_SIZE];
	SKP_float QGain_transfer[HB_SUBFR];

	for (i = 0; i<hbDecCtrl->BWE_FrameSize; i++){
		res_f[i] = (SKP_float)(residue_Q10[i] >>10);
	}


	/*-----------------------------Low band processing ---------end------------------------------------*/

	/*----------------------------Silk High band processing -----start-------------------------------------*/

	hb_lostflag = lostflag;
	// In case of packet loss, interpolate LSP & gain
	if ((lostflag == 1)|| (hb_lostflag == 2)){
		// if no look-ahead packet is available in jitter buffer, use prev gain and shape
		memcpy(QHB_LSP, psHBDec->HB_prev_NLSFq, hbDecCtrl->BWE_LPCOrder * sizeof(SKP_float));
		for (sub = 0; sub<HB_SUBFR; sub++) {
			QGain_transfer[sub] = psHBDec->HB_prev_Gain;
		}
		memset(residue_Q10, 0, sizeof(int)*MAX_INTERNAL_FRAME_SIZE);
		psHBDec->hb_lossCnt++;
	}else {
		psHBDec->hb_gIncr = 0;
		// no packet loss
		hb_idx = AGR_Sate_bits_unpack_unsigned(bits, 12);
		AGR_Sate_lsp_dequant_highband(QHB_LSP, hb_idx, hbDecCtrl->BWE_LPCOrder);

		for (sub = 0; sub<HB_SUBFR; sub++) {
			hb_idx = AGR_Sate_bits_unpack_unsigned(bits, 5);
			QGain_transfer[sub] = AGR_Sate_highband_gain_cdbk[hb_idx];
		}

		if (hbDecCtrl->first) {
			memcpy(psHBDec->HB_prev_NLSFq, QHB_LSP, hbDecCtrl->BWE_LPCOrder * sizeof(SKP_float));
			psHBDec->HB_prev_Gain = QGain_transfer[HB_SUBFR - 1];
		}

		if ((hb_lostflag == 1)|| (hb_lostflag == 2)){
			// temp solution, use prev parameters, easy for evaluation.
			for (sub = 0; sub<HB_SUBFR; sub++) {
				QGain_transfer[sub] = psHBDec->HB_prev_Gain;
			}
			memcpy(QHB_LSP, psHBDec->HB_prev_NLSFq, hbDecCtrl->BWE_LPCOrder * sizeof(SKP_float));
		}

		psHBDec->hb_lossCnt = 0;
	}

	p_out = OutHigh;
	for (sub = 0; sub<HB_SUBFR; sub++){
		// exciation modulation by spectrum folding
		for (i = 0; i < hbDecCtrl->BWE_SubFrameSize; i++) {
			HB_LPCRes[i] = (SKP_float)(-FOLDING_GAIN * QGain_transfer[sub] * res_f[sub*hbDecCtrl->BWE_SubFrameSize + i]);
		}
		// Convert to LPC for residual energy evaluation 
		SKP_Silk_NLSF2A_stable_FLP(HB_PredCoef, QHB_LSP, hbDecCtrl->BWE_LPCOrder);

		// resynthesize high band signal with folded exc * encoder gain + encoder lpc
		AGR_Sate_LPC_synthesizer(p_out, HB_LPCRes, psHBDec->sLPC, HB_PredCoef, hbDecCtrl->BWE_LPCOrder, hbDecCtrl->BWE_SubFrameSize);

		/* Copy last P samples to be used in the next sub frame */
		memcpy(psHBDec->sLPC, &psHBDec->sLPC[hbDecCtrl->BWE_SubFrameSize], MAX_LPC_ORDER * sizeof(SKP_float));

		p_out += hbDecCtrl->BWE_SubFrameSize;
	}

	if ((lostflag == 0) || (lostflag == 4) || (lostflag == 3)){
		psHBDec->HB_prev_Gain = QGain_transfer[HB_SUBFR - 1]; // store last available gain for HB PLC
		memcpy(psHBDec->HB_prev_NLSFq, QHB_LSP, hbDecCtrl->BWE_LPCOrder * sizeof(SKP_float)); // store last available shape for HB PLC
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
	AGR_Sate_decoder_hb_state_FLP *psHBDec;
	SKP_SILK_SDK_DecControlStruct *DecControl;
	AGR_Sate_HB_decoder_control_FLP *hbDecCtrl;

	SKP_int32 i, sub, ret;
	SKP_uint8 payloadToDec[MAX_PAYLOAD_SIZE]; ///* silk max payload size */
	SKP_int16 lowout[MAX_INTERNAL_FRAME_SIZE];
	SKP_int32 res_Q10[MAX_INTERNAL_FRAME_SIZE];
	SKP_float out[MAX_INTERNAL_FRAME_SIZE];
	SKP_float OutLow[MAX_INTERNAL_FRAME_SIZE];
	SKP_float OutHigh[MAX_INTERNAL_FRAME_SIZE],*pOutHigh;
	SKP_int32 tempint;
	SKP_uint8 *pstream = payloadToDec;
	SKP_int16 nBytesIn[6] = { 0, 0, 0, 0, 0, 0 };
	SKP_int16 nSample;	
	SKP_int32 *p_res_Q10 = res_Q10;
	SKP_int16 *p_lowout = lowout;

	psDec = (void *)sateCtl->stDec;
	psHBDec = sateCtl->stHBDec;
	DecControl = (SKP_SILK_SDK_DecControlStruct*)&sateCtl->decControl;
	hbDecCtrl = (AGR_Sate_HB_decoder_control_FLP*)&sateCtl->HBdecControl;
	memset(res_Q10, 0, sizeof(SKP_int32)*MAX_INTERNAL_FRAME_SIZE);
	memset(lowout, 0, sizeof(SKP_int16)*MAX_INTERNAL_FRAME_SIZE);

	/*-----------------------------Low band processing -----start-------------------------------------*/
	if(bits){
		if (lostflag == 2){/*only received md1 stream, there isn't HB steam in md1 stream*/
			nBytes[0] = (bits->nbBits >> 3);
		}else{
			nBytes[0] = (bits->nbBits >> 3) - (hbDecCtrl->QMF_HB_FrameSize / hbDecCtrl->BWE_FrameSize)*HB_BYTE;
		}
		if (nBytes[1]) {
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

	for (sub = 0; sub < hbDecCtrl->QMF_LB_FrameSize; sub += hbDecCtrl->SATE_FrameSize) {

		nSample = hbDecCtrl->SATE_FrameSize;
		ret = SKP_Silk_SDK_Decode(psDec, DecControl, lostflag, pstream, nBytes, p_lowout, &nSample);

		if (ret < 0) {
		    return ret;
	    }

		SKP_Silk_SDK_Get_Decoder_Residue(psDec, p_res_Q10);
		p_lowout  += hbDecCtrl->SATE_FrameSize;
		p_res_Q10 += hbDecCtrl->SATE_FrameSize;
	}

	for (i = 0; i< hbDecCtrl->QMF_LB_FrameSize; i++){
		OutLow[i] = (SKP_float)lowout[i];
	}
	
	pOutHigh = OutHigh;
	p_res_Q10 = res_Q10;
	for (sub = 0; sub < hbDecCtrl->QMF_HB_FrameSize; sub += hbDecCtrl->BWE_FrameSize){
		AGR_Bwe_decode_frame_FLP(hbDecCtrl,psHBDec,bits,pOutHigh,p_res_Q10,lostflag);
		pOutHigh += hbDecCtrl->BWE_FrameSize;
		p_res_Q10 += hbDecCtrl->BWE_FrameSize;
	}
	
	/*----------------------------Silk High band processing -----end-------------------------------------*/

	AGR_Sate_qmf_synth(OutLow, OutHigh, AGR_Sate_qmf_coeffs, out, hbDecCtrl->JC1_FrameSize, QMF_ORDER, hbDecCtrl->g0_mem, hbDecCtrl->g1_mem, NULL);

	for (i = 0; i<hbDecCtrl->JC1_FrameSize; i++){
		tempint = (SKP_int32)out[i];
		if (tempint > 32767)
			tempint = 32767;
		else if (tempint < -32768)
			tempint = -32768;
		vout[i] = (SKP_int16)tempint;
	}

	return 0;
}