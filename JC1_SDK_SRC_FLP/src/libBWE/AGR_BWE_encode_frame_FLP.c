


#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "AGR_BWE_main_FLP.h"
#include "AGR_BWE_tables_qmf.h"
#include "AGR_BWE_tables_highband_coeff.h"
#include "AGR_BWE_bits.h"



SKP_int32 AGR_Bwe_encode_frame_FLP(
	AGR_Sate_HB_encoder_control_FLP *hbEncCtrl,
	AGR_Sate_encoder_hb_state_FLP *psHBEnc,
    NovaBits    *bits,                                   /* I   bitstream operator       */
	SKP_float   *high,
	SKP_int32   *residue,
    SKP_int16   *nBytesOut      /* I/O: Number of bytes in outData (input: Max bytes)   */
	
)
{
    SKP_float res_f[MAX_INTERNAL_FRAME_SIZE];
	SKP_int   i,sub;
    SKP_float gain, energy_lb, energy_hb;
	SKP_int32 hb_lsp_idx, hb_gain_idx[NB_SUBFR];
	SKP_float *pHBBuf;
    SKP_float  HB_PredCoef[MAX_LPC_ORDER];
    SKP_float  HB_LPCRes[MAX_SUBFR_SIZE];
	
    for (i = 0; i < hbEncCtrl->QMF_HB_FrameSize; i++){
        res_f[i] = (SKP_float)(residue[i]>>10);
    }

	/*-----------------------------Low band processing -------end-----------------------------------*/

	/*----------------------------Silk High band processing -----start-------------------------------------*/

	// copy qmf output to encoder state buf x_hb_buf
	// delay 5ms to sync with low band silk encode.
	memcpy(psHBEnc->x_hb_buf + hbEncCtrl->BWE_FrameSize + hbEncCtrl->lb_Delay * hbEncCtrl->hb_KHz,
		high, sizeof(SKP_float)*hbEncCtrl->BWE_FrameSize);

	//calculate high band LPC & LSP
	hb_lsp_idx = AGR_Sate_find_HB_LPC_FLP(psHBEnc, hbEncCtrl, hbEncCtrl->BWE_LPCFrameSize, hbEncCtrl->BWE_LPCOrder, hbEncCtrl->first);

    pHBBuf = psHBEnc->x_hb_buf + hbEncCtrl->BWE_FrameSize;

	for (sub = 0; sub < HB_SUBFR; sub++ ) {
		// Convert to LPC for residual energy 
		SKP_Silk_NLSF2A_stable_FLP(HB_PredCoef, psHBEnc->interp_NLSF[sub / 2], hbEncCtrl->BWE_LPCOrder);

		// get residual from LPC analysis
		SKP_Silk_LPC_analysis_filter_FLP(HB_LPCRes, HB_PredCoef, pHBBuf, hbEncCtrl->BWE_SubFrameSize, hbEncCtrl->BWE_LPCOrder);

		energy_hb = (SKP_float)sqrt(0.01 + SKP_Silk_energy_FLP(HB_LPCRes, hbEncCtrl->BWE_SubFrameSize) / hbEncCtrl->BWE_SubFrameSize);

		energy_lb = (SKP_float)sqrt(0.01 + SKP_Silk_energy_FLP(&res_f[sub * hbEncCtrl->BWE_SubFrameSize], hbEncCtrl->BWE_SubFrameSize) / hbEncCtrl->BWE_SubFrameSize);

		gain =(SKP_float) ((0.01 + energy_hb) / (0.01 + energy_lb));

        hb_gain_idx[sub] = AGR_Sate_gain_quant_highband(gain, AGR_Sate_highband_gain_cdbk, HB_GAIN_CB);	

		pHBBuf += hbEncCtrl->BWE_SubFrameSize;
	}


    if (bits){
        AGR_Sate_bits_pack(bits, hb_lsp_idx, 12);
        for (i = 0; i < NB_SUBFR; i++){
                AGR_Sate_bits_pack(bits, hb_gain_idx[i], 5);
        }
    }
    
	/* Update input buffer */
	memmove(psHBEnc->x_hb_buf, &psHBEnc->x_hb_buf[hbEncCtrl->BWE_FrameSize],
		(hbEncCtrl->BWE_FrameSize + hbEncCtrl->lb_Delay  * hbEncCtrl->hb_KHz) * sizeof(SKP_float));

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
	AGR_Sate_encoder_hb_state_FLP *psHBEnc;
	SKP_SILK_SDK_EncControlStruct  *SilkEncControl;
	AGR_Sate_HB_encoder_control_FLP *hbEncCtrl;

	SKP_float  in[MAX_INTERNAL_FRAME_SIZE];
	SKP_int16  lowshort[MAX_INTERNAL_HALF_FRAME_SIZE],*plow;
	SKP_uint8 payload[MAX_PAYLOAD_SIZE],*pstream; ///* silk max payload size */
	SKP_uint8 payload1[MAX_PAYLOAD_SIZE], *pstream1; ///* silk max payload size */
	SKP_uint8 payload2[MAX_PAYLOAD_SIZE], *pstream2; ///* silk max payload size */
	SKP_int32  res[MAX_INTERNAL_FRAME_SIZE],*pres;

	SKP_int32 i, sub, ret, tempint;
	SKP_float *low, *high;
	SKP_int16 nBytes[6] = { 0,0,0,0,0,0 };
	SKP_int16 nHBBytes[1] = { 0 };
	SKP_float InHigh[MAX_INTERNAL_HALF_FRAME_SIZE];
	SKP_float InLow[MAX_INTERNAL_HALF_FRAME_SIZE], *pInLow;

	psEnc = (void*)(sateCtl->stEnc);
	psHBEnc = (AGR_Sate_encoder_hb_state_FLP*)(sateCtl->stHBEnc);
	SilkEncControl = (SKP_SILK_SDK_EncControlStruct*)(&sateCtl->encControl);
	hbEncCtrl = (AGR_Sate_HB_encoder_control_FLP*)(&sateCtl->HBencControl);

	for (i = 0; i<hbEncCtrl->JC1_FrameSize; i++){
		in[i] = (SKP_float)vin[i];
	}
	low = InLow;
	high = InHigh;

	/* High-band buffering / sync with low band */
	/* Compute the two sub-bands by filtering with QMF h0*/
	AGR_Sate_qmf_decomp(in, AGR_Sate_qmf_coeffs, low, high, hbEncCtrl->JC1_FrameSize, QMF_ORDER, hbEncCtrl->h0_mem, NULL);
    
	pstream = payload;
	pstream1 = payload1;
	pstream2 = payload2;

	/*-----------------------------Low band processing -----start-------------------------------------*/
	for (i = 0; i< (hbEncCtrl->JC1_FrameSize >> 1); i++){
		tempint  = (SKP_int32)low[i];
		if(tempint > 32767)
			tempint = 32767;
		else if(tempint < -32768)
			tempint = -32768;
		lowshort[i] = (SKP_int16)tempint;
	}
	
	nBytesOut[0] = 0;
	nBytesOut[1] = 0;

	plow = lowshort;
	pres = res;

	pInLow = InLow;

	for (sub = 0; sub < hbEncCtrl->QMF_LB_FrameSize ; sub+= hbEncCtrl->SATE_FrameSize){
		nBytes[0] = MAX_PAYLOAD_SIZE;//silk MAX_BYTES_PER_FRAME * MAX_INPUT_FRAMES;
		/* Silk Encoder */
		ret = SKP_Silk_SDK_Encode(psEnc, SilkEncControl, plow, (SKP_int16)hbEncCtrl->SATE_FrameSize, pstream, &nBytes[0]);
		
		SKP_Silk_SDK_Get_Encoder_Residue(psEnc, pres);
		plow += hbEncCtrl->SATE_FrameSize;
		pres += hbEncCtrl->SATE_FrameSize;
		
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

	pres = res;
	
	for (sub = 0; sub < hbEncCtrl->QMF_HB_FrameSize; sub += hbEncCtrl->BWE_FrameSize){
		AGR_Bwe_encode_frame_FLP(hbEncCtrl,psHBEnc,bits,high,res,nHBBytes);
		high += hbEncCtrl->BWE_FrameSize;
		pres += hbEncCtrl->BWE_FrameSize;
	}
	
	if(nBytesOut[0]){
		if(nHBBytes[0] > 0){
			nBytesOut[0] += nHBBytes[0];
			nBytesOut[1] += nHBBytes[0];
		}
	}
	else{
		/*DTX data*/
	}
	/*----------------------------Silk High Band processing -----end-------------------------------------*/
	return 1;
}




	
