#ifndef AGR_Sate_STRUCTS_H
#define AGR_Sate_STRUCTS_H

#include "SKP_Silk_SDK_API.h"
#include "AGR_BWE_bits.h"
#include "AGR_BWE_defines.h"

#ifdef __cplusplus
extern "C"
{
#endif

/********************************/
/* Encoder state FLP            */
/********************************/
typedef struct {
    //float							x_hb_buf [2*MAX_FRAME_LENGTH + LA_SHAPE_MAX];/* Buffer for high band lpc analysis */
    SKP_float	x_hb_buf[2 * 1280 + 480];/* Buffer for high band lpc analysis */
    SKP_int32	HB_NLSFInterpCoef_Q2;
    SKP_float   HB_prev_NLSFq[MAX_LPC_ORDER];
    SKP_float	HB_NLSF[MAX_LPC_ORDER];
    SKP_float	exc_rms[NB_SUBFR];
    SKP_float	interp_NLSF[NB_SUBFR][MAX_LPC_ORDER];
} AGR_Sate_encoder_hb_state_FLP;

typedef struct {
    SKP_int32	hb_lossCnt;
    SKP_float   hb_gIncr;
    SKP_int32   HB_NLSFInterpCoef_Q2;
    SKP_float   HB_prev_NLSFq[MAX_LPC_ORDER];
    SKP_float   HB_prev_Gain;
    SKP_float   HB_NLSF[MAX_LPC_ORDER];
    SKP_float   sLPC[1280 / NB_SUBFR + MAX_LPC_ORDER];
} AGR_Sate_decoder_hb_state_FLP;


/**********************************/
/* High Band Encoder control FLP      */
/**********************************/
typedef struct {
    SKP_int32     hb_KHz;
    SKP_int32     lb_Delay;
    SKP_float     h0_mem[QMF_ORDER];	
    SKP_float     BWE_PredCoef[2][MAX_LPC_ORDER];  /* holds interpolated and final coefficients */
    SKP_int32     BWE_LPCOrder;
	SKP_int32     BWE_LPCFrameSize;
	SKP_int32     BWE_SubFrameSize;
    SKP_int32     first;
    SKP_int32     JC1_FrameSize;
	SKP_int32     QMF_HB_FrameSize;
	SKP_int32     QMF_LB_FrameSize;
	SKP_int32     SATE_FrameSize;
	SKP_int32     BWE_FrameSize;
	SKP_int32     bitstream_format;
} AGR_Sate_HB_encoder_control_FLP; 

/**********************************/
/* High Band Decoder control FLP      */
/**********************************/
typedef struct {
	SKP_float   g0_mem[QMF_ORDER];
	SKP_float   g1_mem[QMF_ORDER];
    SKP_int32     first;
    SKP_int32     BWE_LPCOrder;
	SKP_int32     BWE_SubFrameSize;
    SKP_int32     JC1_FrameSize;
	SKP_int32     QMF_HB_FrameSize;
	SKP_int32     QMF_LB_FrameSize;
	SKP_int32     SATE_FrameSize;
	SKP_int32     BWE_FrameSize;
} AGR_Sate_HB_decoder_control_FLP;

typedef struct {
    void *stEnc;
    void *stHBEnc;
    SKP_SILK_SDK_EncControlStruct  encControl;
    AGR_Sate_HB_encoder_control_FLP HBencControl;
    NovaBits bits;
} SATEEncCtl;


typedef struct {
    void *stDec;
    void *stHBDec;
    SKP_SILK_SDK_DecControlStruct decControl;
    AGR_Sate_HB_decoder_control_FLP HBdecControl;
    NovaBits bits;
} SATEDecCtl;

#ifdef __cplusplus
}
#endif 

#endif