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
    SATEEncCtl *sateCtl                                 /* I/O  SATE Encoder state      */   
);
SKP_int32 AGR_Sate_spsk_decoder_init(
    SATEDecCtl *sateCtl                                 /* I/O  SATE Decoder state      */
);

void AGR_Sate_spsk_encoder_uninit(
    SATEEncCtl *st                                      /* I/O  SATE Encoder state      */
);
void AGR_Sate_spsk_decoder_uninit(
    SATEDecCtl *st                                      /* I/O  SATE Decoder state      */
);

SKP_int32 AGR_Sate_encode_process(
    SATEEncCtl *sateCtl,                                /* I/O SATE Encoder state       */
    const SKP_int16  *vin,                                    /* I   input signal             */
    NovaBits   *bits,                                   /* I   bitstream operator       */
	void       *skctrl, 
	void       *hbctrl,
    SKP_int16 *nBytesOut                                /* I   encoded bits             */
);
SKP_int32 AGR_Sate_decode_process(
    SATEDecCtl *sateCtl,                                /* I/O  SATE Decoder state      */
    NovaBits *bits,                                     /* I    bitstream operator      */
    SKP_int16 *vout,                                    /* O    output signal           */
    void *skdecCtrl, 
	void *hbdecCtrl, 	
    SKP_int16 nBytes[],                                 /* I    input bitstram size     */
    SKP_int32 lostflag                                  /* I    lost falg               */
);

SKP_int32 AGR_Sate_find_HB_LPC_FIX(
    AGR_Sate_encoder_hb_state_FIX      *psEnc,              /* I/O  Encoder state FLP       */
    AGR_Sate_HB_encoder_control_FIX    *hbEncCtrl,          /* I/O  HB Encoder control FLP  */
    SKP_int32                       hb_subfr_length,    /* I    subframe length         */
	SKP_int32                       hb_lpc_order,       /* I    high band lpc order     */ 
	SKP_int32                       first               /* I/O                          */    
);


void AGR_Sate_LPC_synthesizer(
    SKP_float	*output,                                /* O    output signal           */
    SKP_float	*ipexc,                                 /* I    excitation signal       */
    SKP_float	*sLPC,                                  /* I/O  state vector            */
    SKP_float	*a_tmp,                                 /* I    filter coefficients     */
    SKP_float   gain,
    SKP_int32	LPC_order,                              /* I    filter order            */
    SKP_int32	subfr_length                            /* I    signal length           */
);
void AGR_Sate_LPC_synthesis_filter_fix(
    const SKP_int32 *in_Q10,        /* I:   excitation signal */
    const SKP_int16 *A_Q12,     /* I:   AR coefficients [Order], between -8_Q0 and 8_Q0 */
    const SKP_int32 Gain_Q16,   /* I:   gain */
    SKP_int32 *S,               /* I/O: state vector [Order] */
    SKP_int16 *out,             /* O:   output signal */
    const SKP_int32 len,        /* I:   signal length */
    const SKP_int Order         /* I:   filter order, must be even */
);

   /* Quantizes high-band LSPs with 12 bits */
SKP_int32 AGR_Sate_lsp_quant_highband(
    SKP_int32 *lsp,                                     /* I/O  lsp coefficients        */           
    SKP_int32 order                                     /* I    lpc order               */
);
void AGR_Sate_lsp_dequant_highband(
    SKP_int32 *qlsp,                                    /* O    output lsp coefficients */
    SKP_int32 idx,                                      /* I    quantized index         */ 
    SKP_int32 order                                     /* I    lpc order               */
);

SKP_int32 AGR_Sate_gain_quant_highband_fix(
    SKP_int16 gain,                               /* I unquantized gain           */
    const SKP_int16 *cdbk,                        /* I code book                  */
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
    
extern    void SKP_Silk_NLSF2A_stable(
                                SKP_int16                       pAR_Q12[ MAX_LPC_ORDER ],   /* O    Stabilized AR coefs [LPC_order]     */
                                const SKP_int                   pNLSF[ MAX_LPC_ORDER ],     /* I    NLSF vector         [LPC_order]     */
                                const SKP_int                   LPC_order                   /* I    LPC/LSF order                       */
    );
#ifdef __cplusplus
}
#endif

#endif
