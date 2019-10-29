#include "AGR_BWE_main_FIX.h"
#include "AGR_BWE_tables_qmf.h"
#include "AGR_BWE_tables_highband_coeff.h"
#include "AGR_BWE_bits.h"
#include "SKP_Silk_SigProc_FIX.h"


SKP_int32 AGR_Bwe_encode_frame_FIX(
	AGR_Sate_HB_encoder_control_FIX *hbEncCtrl,
	AGR_Sate_encoder_hb_state_FIX *psHBEnc,
    NovaBits    *bits,                                   /* I   bitstream operator       */
	SKP_int16   *high,
	SKP_int32   *residue,
    SKP_int16   *nBytesOut      /* I/O: Number of bytes in outData (input: Max bytes)   */
)
{
	SKP_int16 *p_hb_fix;
	SKP_int16 S[MAX_LPC_ORDER];
	SKP_int32 res_nrg0, res_nrg1;
	SKP_int64 res_nrg2, res_nrg3;
	SKP_int16 gain_fix;
	SKP_int32 hb_lsp_idx, hb_gain_idx[NB_SUBFR];

	SKP_int16 HB_PredCoef_fix[MAX_LPC_ORDER];
	SKP_int16 exc_fix[MAX_SUBFR_SIZE];
	SKP_int32 i, sub, tmp;

	// copy qmf output to encoder state buf x_hb_buf
	// delay 5ms to sync with low band silk encode.
    memcpy(psHBEnc->x_hb_buf_fix + hbEncCtrl->BWE_FrameSize + hbEncCtrl->lb_Delay * hbEncCtrl->hb_KHz,
		high, sizeof(SKP_int16)*hbEncCtrl->BWE_FrameSize);

	//calculate high band LPC & LSP
	hb_lsp_idx = AGR_Sate_find_HB_LPC_FIX(psHBEnc, hbEncCtrl, hbEncCtrl->BWE_LPCFrameSize, hbEncCtrl->BWE_LPCOrder, hbEncCtrl->first);

    p_hb_fix = psHBEnc->x_hb_buf_fix + hbEncCtrl->BWE_FrameSize;
	for (sub = 0; sub < HB_SUBFR; sub++) {
		// Convert to LPC for residual energy 
        SKP_Silk_NLSF2A_stable(HB_PredCoef_fix, psHBEnc->interp_NLSF_Q15[sub/2], hbEncCtrl->BWE_LPCOrder);

		SKP_memset(S, 0, hbEncCtrl->BWE_LPCOrder * sizeof(SKP_int16));
		// get residual from LPC analysis
        SKP_Silk_LPC_analysis_filter(p_hb_fix, HB_PredCoef_fix, S, exc_fix, hbEncCtrl->BWE_SubFrameSize, hbEncCtrl->BWE_LPCOrder);

		res_nrg0 = 0;
		res_nrg1 = 0;
		res_nrg2 = 0;
		res_nrg3 = 0;
        for (i = 0; i < hbEncCtrl->BWE_SubFrameSize; i++){
        	res_nrg0 = (SKP_ADD64((res_nrg0), SKP_MUL((SKP_int16)exc_fix[i], (SKP_int16)exc_fix[i])));
            res_nrg2 = SKP_SMLABB(res_nrg2, exc_fix[i], exc_fix[i]);//q0
			tmp = (SKP_int32)(residue[sub*hbEncCtrl->BWE_SubFrameSize + i] >> 10);
        	res_nrg3 = (SKP_ADD64((res_nrg3), SKP_MUL(tmp, tmp)));
            res_nrg1 = SKP_SMLABB(res_nrg1, tmp, tmp);//q0
        }

        res_nrg0 = SKP_Silk_SQRT_APPROX(res_nrg0);
        res_nrg1 = SKP_Silk_SQRT_APPROX(res_nrg1);
        gain_fix = (SKP_int16)(SKP_LSHIFT32((res_nrg0 + 1), 4) / (res_nrg1 + 1));

        hb_gain_idx[sub] = AGR_Sate_gain_quant_highband_fix(gain_fix, AGR_Sate_highband_gain_cdbk_fix, HB_GAIN_CB);
		p_hb_fix += hbEncCtrl->BWE_SubFrameSize;
	}
    
    if (bits){
        AGR_Sate_bits_pack(bits, hb_lsp_idx, 12);
        for (i = 0; i < NB_SUBFR; i++){
            AGR_Sate_bits_pack(bits, hb_gain_idx[i], 5);
        }
    }
    
	/* Update input buffer */
    memmove(psHBEnc->x_hb_buf_fix, &psHBEnc->x_hb_buf_fix[hbEncCtrl->BWE_FrameSize],
        (hbEncCtrl->BWE_FrameSize + hbEncCtrl->lb_Delay  * hbEncCtrl->hb_KHz) * sizeof(SKP_int16));

	hbEncCtrl->first = 0;

	nBytesOut[0] += HB_BYTE;

	return 0;

}




SKP_int32 AGR_Sate_encode_process(
    SATEEncCtl *sateCtl,                                /* I/O SATE Encoder state       */
    const SKP_int16  *vin,                                    /* I   input signal             */
    NovaBits   *bits,                                   /* I   bitstream operator       */
	void       *skctrl, 
	void       *hbctrl,
    SKP_int16 *nBytesOut                                /* I   encoded bits             */
)
{
	void *psEnc;
	AGR_Sate_encoder_hb_state_FIX *psHBEnc;
	SKP_SILK_SDK_EncControlStruct  *SilkEncControl;
	AGR_Sate_HB_encoder_control_FIX *hbEncCtrl;

	SKP_int16 lowshort[MAX_INTERNAL_HALF_FRAME_SIZE], *phigh;
    SKP_int16 highshort[MAX_INTERNAL_HALF_FRAME_SIZE],*plow;
	SKP_uint8 payload[MAX_PAYLOAD_SIZE],*pstream; ///* silk max payload size */
	SKP_uint8 payload1[MAX_PAYLOAD_SIZE], *pstream1; ///* silk max payload size */
	SKP_uint8 payload2[MAX_PAYLOAD_SIZE], *pstream2; ///* silk max payload size */
	SKP_int32 res_Q10[MAX_INTERNAL_FRAME_SIZE],*p_res_Q10;
	SKP_int32 sub, ret;
	SKP_int16 nBytes[6] = { 0, 0, 0, 0, 0, 0 };
	SKP_int16 nHBBytes[1] = { 0 };


	psEnc = (void*)(sateCtl->stEnc);
	psHBEnc = (AGR_Sate_encoder_hb_state_FIX*)(sateCtl->stHBEnc);
	SilkEncControl = (SKP_SILK_SDK_EncControlStruct*)(&sateCtl->encControl);
	hbEncCtrl = (AGR_Sate_HB_encoder_control_FIX*)(&sateCtl->HBencControl);

	/* High-band buffering / sync with low band */
	/* Compute the two sub-bands by filtering with QMF h0*/
	AGR_Sate_qmf_decomp(vin, AGR_Sate_qmf_coeffs_fix, lowshort, highshort, hbEncCtrl->JC1_FrameSize, QMF_ORDER, hbEncCtrl->h0_mem, NULL);
    
	pstream = payload;
	pstream1 = payload1;
	pstream2 = payload2;
	
	nBytesOut[0] = 0;
	nBytesOut[1] = 0;

	plow = lowshort;
	p_res_Q10 = res_Q10;

	for (sub = 0; sub < hbEncCtrl->QMF_LB_FrameSize ; sub+= hbEncCtrl->SATE_FrameSize){
		nBytes[0] = MAX_PAYLOAD_SIZE;//silk MAX_BYTES_PER_FRAME * MAX_INPUT_FRAMES;

		/* Silk Encoder */
		ret = SKP_Silk_SDK_Encode(psEnc, SilkEncControl, plow, hbEncCtrl->SATE_FrameSize, pstream, &nBytes[0]);

		SKP_Silk_SDK_Get_Encoder_Residue(psEnc, p_res_Q10);
		plow += hbEncCtrl->SATE_FrameSize;
		p_res_Q10 += hbEncCtrl->SATE_FrameSize;
	}

	nBytesOut[0] = nBytes[0];
	nBytesOut[1] = nBytes[1];
	pstream = payload;


	nBytesOut[0] += nBytesOut[1];
	
	//|--LB(byte aligned)--|--HB(byte aligned)--|
	memcpy(&(bits->chars[0]), payload, nBytesOut[0]);
	bits->nbBits = (nBytesOut[0]) << 3;
	bits->charPtr = nBytesOut[0];
	bits->chars[bits->charPtr] = 0;
	bits->bitPtr = 0;
	phigh = highshort;
	p_res_Q10 = res_Q10;
	
	for (sub = 0; sub < hbEncCtrl->QMF_HB_FrameSize; sub += hbEncCtrl->BWE_FrameSize){
		AGR_Bwe_encode_frame_FIX(hbEncCtrl, psHBEnc, bits, phigh, p_res_Q10, nHBBytes);
		phigh += hbEncCtrl->BWE_FrameSize;
		p_res_Q10 += hbEncCtrl->BWE_FrameSize;
	}
	
	if(nBytesOut[0]){
		if(nHBBytes[0] > 0){
			nBytesOut[0] += nHBBytes[0];
			nBytesOut[1] += nHBBytes[0];
		}
	}else{
		/*DTX data*/
	}
	/*----------------------------Silk High Band processing -----end-------------------------------------*/
	return 1;
}




	
