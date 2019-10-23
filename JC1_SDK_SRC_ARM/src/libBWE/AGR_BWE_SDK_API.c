
#include <stdlib.h>
#include <stdio.h>
#include "SKP_Silk_SDK_API.h"
#include "AGR_JC1_SDK_API.h"
#include "SKP_Silk_control.h"
#include "AGR_BWE_main_FIX.h"
#include "AGR_BWE_defines.h"


void *AGR_Sate_Encoder_Init(
    USER_Ctrl_enc *enc_Ctrl         /* I/O  Encoder state */
)
{
	SATEEncCtl *pSATEEncControl;

	SKP_SILK_SDK_EncControlStruct * pSilkEncControl; // Struct for input to encoder 
	AGR_Sate_HB_encoder_control_FIX *pHBencControl;
	
	SKP_int32 packetLoss_perc,INBandFEC_enabled,complexity;
	SKP_int32 bwe_framesize_ms=40,sate_framesize_ms=20;


	pSATEEncControl = (SATEEncCtl *)malloc(sizeof(SATEEncCtl));
	
    if(pSATEEncControl==NULL) {
        SKP_assert(0);
    }
	
	pSilkEncControl = &(pSATEEncControl->encControl);
	pHBencControl = &(pSATEEncControl->HBencControl);
	
	memset(pSATEEncControl,0,sizeof(SATEEncCtl));

	if (enc_Ctrl->targetRate_bps <= 0) {
		enc_Ctrl->targetRate_bps = 15600;
	}

	
	packetLoss_perc = 0;
	INBandFEC_enabled = 0;
	complexity = 2;
	
	/*
		  joint disable
		  20ms 2MD
		  40ms 2MD
		  joint mode 0	reserve for low band multiframe joint coding 
		  joint mode 1	high band multiframe joint coding method 0
		  joint mode 2	reserve for high band multiframe joint coding method 1 (LPC and Res)
		  joint mode 3	reserve for low band and high band all multiframe joint coding 
						reserve for 1. low band multiframe joint coding is OK
									2. select a better joint method in 0 or 1 of high band
	
	*/

	if(enc_Ctrl->joint_enable){
		switch (enc_Ctrl->joint_mode){
		case 0:
			bwe_framesize_ms  = 20;
			sate_framesize_ms = 20;
			fprintf(stderr,"Unsupport : joint mode 0	reserve for low band multiframe joint coding\n");
			break;
		case 1:
			bwe_framesize_ms  = 40;
			sate_framesize_ms = 20;
			break;
		case 2:
			fprintf(stderr,"Unsupport : joint mode 2	reserve for high band multiframe joint coding method 1 (LPC and Res)\n");
			break;
		case 3:
			fprintf(stderr,"Unsupport : joint mode 3	reserve for low band and high band all multiframe joint coding \n");
			break;
		default:
			printf("Error in setting joint mode! It must be 0, 1, 2, 3\n");
			return NULL;
			break;
		}
	} else {
		bwe_framesize_ms  = 20;
		sate_framesize_ms = 20;
	}
	
	if(pSATEEncControl!=NULL) {
		AGR_Sate_spsk_encoder_init(pSATEEncControl);
		if(enc_Ctrl->samplerate == 32000) {
			pHBencControl->hb_KHz = 16;
			pSilkEncControl->API_sampleRate = 16000;
			pSilkEncControl->maxInternalSampleRate = 16000;
		} else {
			pHBencControl->hb_KHz = 8;
			pSilkEncControl->API_sampleRate = 8000;
			pSilkEncControl->maxInternalSampleRate = 8000;
		}

		if(enc_Ctrl->dtx_enable == 0) {
			pSilkEncControl->useDTX = 0;
		} else {
			pSilkEncControl->useDTX = 1;
		}


		pHBencControl->JC1_FrameSize = ((enc_Ctrl->samplerate) * enc_Ctrl->framesize_ms / 1000);
		pHBencControl->QMF_HB_FrameSize = ((enc_Ctrl->samplerate / 2) * enc_Ctrl->framesize_ms / 1000);
		pHBencControl->QMF_LB_FrameSize = ((enc_Ctrl->samplerate / 2) * enc_Ctrl->framesize_ms / 1000);
		pHBencControl->SATE_FrameSize = ((enc_Ctrl->samplerate / 2) * sate_framesize_ms / 1000);
		pHBencControl->BWE_FrameSize = ((enc_Ctrl->samplerate / 2) * bwe_framesize_ms / 1000);
			

		pHBencControl->first = 1;
		pHBencControl->BWE_LPCOrder = 8;
		pHBencControl->BWE_LPCFrameSize = (((enc_Ctrl->samplerate / 2) * 10) / 1000);
		pHBencControl->BWE_SubFrameSize = ( pHBencControl->BWE_FrameSize / HB_SUBFR );
		
		pSilkEncControl->packetSize   = ( enc_Ctrl->framesize_ms * pSilkEncControl->API_sampleRate ) / 1000;
		pSilkEncControl->packetLossPercentage  = packetLoss_perc;
		pSilkEncControl->useInBandFEC  = INBandFEC_enabled;
		pSilkEncControl->complexity  = complexity;
		pSilkEncControl->bitRate = enc_Ctrl->targetRate_bps - ((1600 * 20) / bwe_framesize_ms);
		pSilkEncControl->useMDIndex = enc_Ctrl->useMDIndex;
			
		AGR_Sate_bits_init(&pSATEEncControl->bits);//bitstream buffer clean
	}

	return (void *)pSATEEncControl;
}


SKP_int32 AGR_Sate_Encoder_Encode(
    void *SATEEnc_State,            /* I/O  Encoder state       */
    const SKP_int16 *AGR_Sate_PCM,       /* I    Input signal        */
    SKP_uint8* AGR_Sate_Bit,            /* O    Encoded stream      */
    SKP_int32 AGR_Sate_Buf_Size,        /* I    Maximum output bits */
    SKP_int16 *nBytesOut            /* O    Output size         */
)
{
	int nbBytes;
	SATEEncCtl *pSATEEncControl;

    if(SATEEnc_State==NULL){
		return -1;
    }
	pSATEEncControl = (SATEEncCtl*)SATEEnc_State; 

	AGR_Sate_encode_process(pSATEEncControl, AGR_Sate_PCM, &pSATEEncControl->bits, (void*)&pSATEEncControl->encControl, (void*)&pSATEEncControl->HBencControl, nBytesOut);

	AGR_Sate_bits_insert_terminator(&pSATEEncControl->bits);

	nbBytes = AGR_Sate_bits_write(&pSATEEncControl->bits, &AGR_Sate_Bit[0], AGR_Sate_Buf_Size);

	return nbBytes;
}


int AGR_Sate_Encoder_Uninit(void *SATEEnc_State)
{
	SATEEncCtl *pSATEEncControl;

    if(SATEEnc_State==NULL){
		return -1;
    }

	pSATEEncControl = (SATEEncCtl*)SATEEnc_State; 

	AGR_Sate_spsk_encoder_uninit(pSATEEncControl);
	AGR_Sate_bits_destroy(&(pSATEEncControl->bits));
	free(pSATEEncControl);

	return 0;
}

void *AGR_Sate_Decoder_Init(
    USER_Ctrl_dec *dec_Ctrl         /* I/O  Decoder state       */
)
{
	SATEDecCtl *pSATEDecControl;
	SKP_SILK_SDK_DecControlStruct * pSilkDecControl; // Struct for input to encoder 
	AGR_Sate_HB_decoder_control_FIX *pHBdecControl;
	SKP_int32 bwe_framesize_ms=40,sate_framesize_ms=20;

	pSATEDecControl = (SATEDecCtl *)malloc(sizeof(SATEDecCtl));
	memset(pSATEDecControl, 0, sizeof(SATEDecCtl));
		
	if(pSATEDecControl==NULL){
        SKP_assert(0);
    }
	
	pSilkDecControl = &(pSATEDecControl->decControl);
	pHBdecControl = &(pSATEDecControl->HBdecControl);
	

	if (pSATEDecControl != NULL)
	{  
		AGR_Sate_spsk_decoder_init(pSATEDecControl);

		if(dec_Ctrl->samplerate==32000) {
			pSilkDecControl->API_sampleRate = 16000;
		} else {
			pSilkDecControl->API_sampleRate = 8000;
		}

		if(dec_Ctrl->joint_enable) {
			switch (dec_Ctrl->joint_mode){
			case 0:
				bwe_framesize_ms  = 20;
				sate_framesize_ms = 20;
				break;
			case 1:
				bwe_framesize_ms  = 40;
				sate_framesize_ms = 20;
				break;
			case 2:
				fprintf(stderr,"Unsupport : joint mode 2	reserve for high band multiframe joint coding method 1 (LPC and Res)\n");
				break;
			case 3:
				fprintf(stderr,"Unsupport : joint mode 3	reserve for low band and high band all multiframe joint coding \n");
				break;
			default:
				fprintf(stderr,"Error in setting joint mode! It must be 0, 1, 2, 3\n");
				return NULL;
				break;
			}
		} else {
			bwe_framesize_ms  = 20;
			sate_framesize_ms = 20;
		}		
		
		pSilkDecControl->framesPerPacket = 1;
		pSilkDecControl->useMDIndex = dec_Ctrl->useMDIndex;

		pHBdecControl->first             = 1;
		pHBdecControl->BWE_LPCOrder      = 8;
		pHBdecControl->JC1_FrameSize     = ((dec_Ctrl->samplerate) * dec_Ctrl->framesize_ms/ 1000);
		pHBdecControl->QMF_HB_FrameSize  = ((dec_Ctrl->samplerate/2) * dec_Ctrl->framesize_ms/ 1000);
		pHBdecControl->QMF_LB_FrameSize  = ((dec_Ctrl->samplerate/2) * dec_Ctrl->framesize_ms/ 1000);
		pHBdecControl->SATE_FrameSize = ((dec_Ctrl->samplerate / 2) *  sate_framesize_ms / 1000);
		pHBdecControl->BWE_FrameSize = ((dec_Ctrl->samplerate / 2) * bwe_framesize_ms / 1000);
		pHBdecControl->BWE_SubFrameSize  = ( pHBdecControl->BWE_FrameSize / HB_SUBFR );
			

		AGR_Sate_bits_init(&pSATEDecControl->bits);//bitstream buffer clean
	}

	return (void *)pSATEDecControl;
}



SKP_int32 AGR_Sate_Decoder_Decode(
    void *SATEDec_State,            /* I/O  Decoder state       */
    SKP_int16 *AGR_Sate_PCM,            /* O    Output signal       */
    SKP_int16 *nSamplesOut,         /* O    Decoded frame size  */
    const SKP_uint8* AGR_Sate_Bit,      /* I    Input bits          */
    SKP_int16 nBytes[],             /* I    Input bits size     */
    SKP_int32 lostflag              /* I    lost flag           */
)
{
	int ret;
	SATEDecCtl *pSATEDecState;

    if(SATEDec_State==NULL){
        return -1;
    }
		
	pSATEDecState=(SATEDecCtl*)SATEDec_State; 

	AGR_Sate_bits_reset(&pSATEDecState->bits);
    if (nBytes[0]<= 0){
        return -1;
    }
		
	memcpy(&(pSATEDecState->bits.chars[0]), AGR_Sate_Bit, nBytes[0]);
	pSATEDecState->bits.nbBits = nBytes[0] << 3;

	ret = AGR_Sate_decode_process(pSATEDecState, &pSATEDecState->bits, AGR_Sate_PCM, (void*)&pSATEDecState->decControl, (void*)&pSATEDecState->HBdecControl, nBytes,lostflag);
	
	*nSamplesOut = (short)pSATEDecState->HBdecControl.JC1_FrameSize;

	return ret;
}


SKP_int32 AGR_Sate_Decoder_Uninit(
    void *SATEDec_State             /* I/O  Decoder state       */
)
{
	SATEDecCtl *pSATEDecState;

    if(SATEDec_State==NULL){
		return -1;
    }
	pSATEDecState = (SATEDecCtl*)SATEDec_State; 
    AGR_Sate_spsk_decoder_uninit(pSATEDecState);
	AGR_Sate_bits_destroy(&(pSATEDecState->bits));
	free(pSATEDecState);

	return 0;
}

