#ifndef AGR_Sate_MAIN_FLP_H
#define AGR_Sate_MAIN_FLP_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "SKP_Silk_SDK_API.h"
#include "AGR_BWE_bits.h"
#include "AGR_BWE_defines.h"
#include "AGR_BWE_structs.h"
#include "AGR_BWE_arch.h"

#ifdef __cplusplus
extern "C"
{
#endif

SKP_int32 AGR_Sate_spsk_encoder_init(
    SATEEncCtl *novaCtl                                 /* I/O  SATE Encoder state      */   
);
SKP_int32 AGR_Sate_spsk_decoder_init(
    SATEDecCtl *novaCtl                                 /* I/O  SATE Decoder state      */
);

void AGR_Sate_spsk_encoder_uninit(
    SATEEncCtl *st                                      /* I/O  SATE Encoder state      */
);
void AGR_Sate_spsk_decoder_uninit(
    SATEDecCtl *st                                      /* I/O  SATE Decoder state      */
);

SKP_int32 AGR_Sate_encode_process(
	SATEEncCtl *novaCtl,                                /* I/O SATE Encoder state       */
	const SKP_int16  *vin,                                    /* I   input signal             */
	NovaBits   *bits,                                   /* I   bitstream operator       */
	void       *skctrl,
	void       *hbctrl,
	SKP_int16 *nBytesOut                                /* I   encoded bits             */
);
SKP_int32 AGR_Sate_decode_process(
	SATEDecCtl *novaCtl,                                /* I/O  SATE Decoder state      */
	NovaBits *bits,                                     /* I    bitstream operator      */
	SKP_int16 *vout,                                    /* O    output signal           */
	void *skdecCtrl,
	void *hbdecCtrl,
	SKP_int16 nBytes[],                                 /* I    input bitstram size     */
	SKP_int32 lostflag                                  /* I    lost falg               */
);

SKP_int32 AGR_Sate_find_HB_LPC_FLP(
    AGR_Sate_encoder_hb_state_FLP      *psEnc,              /* I/O  Encoder state FLP       */
    AGR_Sate_HB_encoder_control_FLP    *hbEncCtrl,          /* I/O  HB Encoder control FLP  */
    SKP_int32                       hb_subfr_length,    /* I    subframe length         */
	SKP_int32                       hb_lpc_order,       /* I    high band lpc order     */ 
	SKP_int32                       first               /* I/O                          */    
);


void AGR_Sate_LPC_synthesizer(
    SKP_float	*output,                                /* O    output signal           */
    SKP_float	*ipexc,                                 /* I    excitation signal       */
    SKP_float	*sLPC,                                  /* I/O  state vector            */
    SKP_float	*a_tmp,                                 /* I    filter coefficients     */
    SKP_int32	LPC_order,                              /* I    filter order            */
    SKP_int32	subfr_length                            /* I    signal length           */
);

   /* Quantizes high-band LSPs with 12 bits */
SKP_int32 AGR_Sate_lsp_quant_highband(
    SKP_float *lsp,                                     /* I/O  lsp coefficients        */           
    SKP_int32 order                                     /* I    lpc order               */
);
void AGR_Sate_lsp_dequant_highband(
    SKP_float *qlsp,                                    /* O    output lsp coefficients */
    SKP_int32 idx,                                      /* I    quantized index         */ 
    SKP_int32 order                                     /* I    lpc order               */
);

/* Quantizes high-band GAINs with 5*4 bits */
SKP_int32 AGR_Sate_gain_quant_highband(
          SKP_float gain,                               /* I unquantized gain           */
    const SKP_float *cdbk,                              /* I code book                  */
          SKP_int32 nVect                               /* I book size                  */
);

void AGR_Sate_qmf_decomp(
    const spx_word16_t *xx,                             /* I   Input signal              */   
    const spx_word16_t *aa,                             /* I   Qmf coefficients          */
          spx_word16_t *y1,                             /* O   Output low band signal    */
          spx_word16_t *y2,                             /* O   Output high band signal   */
          SKP_int32     N,                              /* I   frame size                */
          SKP_int32     M,                              /* I   Qmf order                 */
          spx_word16_t *mem,                            /* I/O Qmf state                 */
          SKP_int8     *stack
);
void AGR_Sate_qmf_synth(
    const spx_word16_t *x1,                             /* I   Low band signal           */
    const spx_word16_t *x2,                             /* I   High band signal          */
    const spx_word16_t *a,                              /* I   Qmf coefficients          */
          spx_word16_t *y,                              /* O   Synthesised signal        */
          SKP_int32     N,                              /* I   Signal size               */
          SKP_int32     M,                              /* I   Qmf order                 */
          spx_word16_t *mem1,                           /* I/O Qmf low band state        */
          spx_word16_t *mem2,                           /* I/O Qmf high band state       */
          SKP_int8     *stack
);

extern void SKP_Silk_scale_copy_vector_FLP(
	SKP_float           *data_out,
	const SKP_float     *data_in,
	SKP_float           gain,
	SKP_int             dataSize
	);

extern void SKP_Silk_find_LPC_FLP(
	SKP_float                 NLSF[],             /* O    NLSFs                                   */
	SKP_int                   *interpIndex,       /* O    NLSF interp. index for NLSF interp.     */
	const SKP_float                 prev_NLSFq[],       /* I    Previous NLSFs, for NLSF interpolation  */
	const SKP_int                   useInterpNLSFs,     /* I    Flag                                    */
	const SKP_int                   LPC_order,          /* I    LPC order                               */
	const SKP_float                 x[],                /* I    Input signal                            */
	const SKP_int                   subfr_length        /* I    Subframe length incl preceeding samples */
	);

extern void SKP_Silk_NLSF2A_stable_FLP(
	SKP_float                 *pAR,               /* O    LPC coefficients [ LPC_order ]          */
	const SKP_float                 *pNLSF,             /* I    NLSF vector      [ LPC_order ]          */
	const SKP_int                   LPC_order           /* I    LPC order                               */
	);

/* 16th order LPC analysis filter */
extern void SKP_Silk_LPC_analysis_filter_FLP(
	SKP_float                 r_LPC[],            /* O    LPC residual signal                     */
	const SKP_float                 PredCoef[],         /* I    LPC coefficients                        */
	const SKP_float                 s[],                /* I    Input signal                            */
	const SKP_int                   length,             /* I    Length of input signal                  */
	const SKP_int                   Order               /* I    LPC order                               */
	);

extern double SKP_Silk_energy_FLP(
	const SKP_float     *data,
	SKP_int             dataSize
	);
#ifdef __cplusplus
}
#endif

#endif
