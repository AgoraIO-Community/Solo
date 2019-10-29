/***********************************************************************
Copyright (c) 2006-2012, Skype Limited. All rights reserved. 
Redistribution and use in source and binary forms, with or without 
modification, (subject to the limitations in the disclaimer below) 
are permitted provided that the following conditions are met:
- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright 
notice, this list of conditions and the following disclaimer in the 
documentation and/or other materials provided with the distribution.
- Neither the name of Skype Limited, nor the names of specific 
contributors, may be used to endorse or promote products derived from 
this software without specific prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED 
BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
CONTRIBUTORS ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF 
USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************/

#include "SKP_Silk_main.h"


SKP_INLINE void SKP_Silk_nsq_scale_states(
    SKP_Silk_nsq_state  *NSQ,               /* I/O NSQ state                        */
    const SKP_int16     x[],                /* I input in Q0                        */
    SKP_int32           x_sc_Q10[],         /* O input scaled with 1/Gain           */
    SKP_int             subfr_length,       /* I length of input                    */
    const SKP_int16     sLTP[],             /* I re-whitened LTP state in Q0        */
    SKP_int32           sLTP_Q16[],         /* O LTP state matching scaled input    */
    SKP_int             subfr,              /* I subframe number                    */
    const SKP_int       LTP_scale_Q14,      /* I                                    */
    const SKP_int32     Gains_Q16[ NB_SUBFR ], /* I                                 */
    const SKP_int       pitchL[ NB_SUBFR ]  /* I                                    */
);

SKP_INLINE void SKP_Silk_nsq_scale_md_states(
    SKP_Silk_nsq_state  *NSQ,               /* I/O NSQ state                        */
    SKP_int             subfr_length,       /* I length of input                    */
    const SKP_int16     sLTP[],             /* I re-whitened LTP state in Q0        */
    SKP_int32           sLTP_Q16[],         /* O LTP state matching scaled input    */
    SKP_int             subfr,              /* I subframe number                    */
    const SKP_int       LTP_scale_Q14,      /* I                                    */
    const SKP_int32     Gains_Q16[ NB_SUBFR ], /* I                                 */
    const SKP_int       pitchL[ NB_SUBFR ]  /* I                                    */
);

SKP_INLINE void Agora_MD_Silk_noise_shape_quantizer(
    SKP_Silk_nsq_state  *NSQ,               /* I/O  NSQ state                       */
    SKP_Silk_nsq_state  NSQ_md[MAX_INTERLEAVE_NUM], /* I/O  NSQ state                           */
    SKP_int             sigtype,            /* I    Signal type                     */
    const SKP_int32     x_sc_Q10[],         /* I                                    */
    const SKP_int32     x_md_Q10[],         /* I                                    */
    SKP_int8            q[],                /* O                                    */
	SKP_int32           r[],                /* O                                    */
    SKP_int16           xq[],               /* O                                    */
    SKP_int32           sLTP_Q16[],         /* I/O  LTP state                       */
    SKP_int8           *q_md[MAX_INTERLEAVE_NUM],      /* I/O  LTP state                       */
    SKP_int16           *xq_md[MAX_INTERLEAVE_NUM],      /* I/O  LTP state                       */
    SKP_int32           *sLTP_md_Q16[MAX_INTERLEAVE_NUM],      /* I/O  LTP state                       */
    const SKP_int16     a_Q12[],            /* I    Short term prediction coefs     */
    const SKP_int16     b_Q14[],            /* I    Long term prediction coefs      */
    const SKP_int16     AR_shp_Q13[],       /* I    Noise shaping AR coefs          */
    SKP_int             lag,                /* I    Pitch lag                       */
    SKP_int32           HarmShapeFIRPacked_Q14, /* I                                */
    SKP_int             Tilt_Q14,           /* I    Spectral tilt                   */
    SKP_int32           LF_shp_Q14,         /* I                                    */
    SKP_int32           Gain_Q16,           /* I                                    */
    SKP_int             Lambda_Q10,         /* I                                    */
    SKP_int             offset_Q10,         /* I                                    */
    SKP_int             length,             /* I    Input length                    */
    SKP_int             shapingLPCOrder,    /* I    Noise shaping AR filter order   */
    SKP_int             predictLPCOrder,     /* I    Prediction filter order         */
    const SKP_int32 				MDGains_Q16,
	const SKP_int32 				DeltaGains_Q16
);

SKP_INLINE void SKP_Silk_noise_shape_quantizer(
    SKP_Silk_nsq_state  *NSQ,               /* I/O  NSQ state                       */
    SKP_Silk_nsq_state  (*NSQ_md)[MAX_INTERLEAVE_NUM],    /* I/O  NSQ state                           */
    SKP_int             sigtype,            /* I    Signal type                     */
    const SKP_int32     x_sc_Q10[],         /* I                                    */
    SKP_int8            q[],                /* O    Quantized Q0 residual           */
	SKP_int32           r[],                /* O    Unquantized residual            */
    SKP_int16           xq[],               /* O    Simulated full signal           */
    SKP_int32           sLTP_Q16[],         /* I/O  LTP state                       */
    SKP_int32           (*sLTP_md_Q16)[MAX_INTERLEAVE_NUM],      /* I/O  LTP state                       */
    const SKP_int16     a_Q12[],            /* I    Short term prediction coefs     */
    const SKP_int16     b_Q14[],            /* I    Long term prediction coefs      */
    const SKP_int16     AR_shp_Q13[],       /* I    Noise shaping AR coefs          */
    SKP_int             lag,                /* I    Pitch lag                       */
    SKP_int32           HarmShapeFIRPacked_Q14, /* I                                */
    SKP_int             Tilt_Q14,           /* I    Spectral tilt                   */
    SKP_int32           LF_shp_Q14,         /* I                                    */
    SKP_int32           Gain_Q16,           /* I                                    */
	SKP_int32           DeltaGains_Q16,
    SKP_int             Lambda_Q10,         /* I                                    */
    SKP_int             offset_Q10,         /* I                                    */
    SKP_int             length,             /* I    Input length                    */
    SKP_int             shapingLPCOrder,    /* I    Noise shaping AR filter order   */
    SKP_int             predictLPCOrder     /* I    Prediction filter order         */
);

void SKP_Silk_NSQ(
    SKP_Silk_encoder_state          *psEncC,                                    /* I/O  Encoder State                       */
    SKP_Silk_encoder_control        *psEncCtrlC,                                /* I    Encoder Control                     */
    SKP_Silk_nsq_state              *NSQ,                                       /* I/O  NSQ state                           */
    SKP_Silk_nsq_state              NSQ_md[MAX_INTERLEAVE_NUM],                 /* I/O  NSQ state                           */
    const SKP_int16                 x[],                                        /* I    prefiltered input signal            */
    SKP_int8                        q[],                                        /* O    quantized qulse signal              */
    SKP_int8                        *q_md[ MAX_INTERLEAVE_NUM ],                /* O    quantized qulse signal              */
    SKP_int32                       r[],                                        /* O    Output residual signal              */
    const SKP_int                   LSFInterpFactor_Q2,                         /* I    LSF interpolation factor in Q2      */
    const SKP_int16                 PredCoef_Q12[ 2 * MAX_LPC_ORDER ],          /* I    Short term prediction coefficients  */
    const SKP_int16                 LTPCoef_Q14[ LTP_ORDER * NB_SUBFR ],        /* I    Long term prediction coefficients   */
    const SKP_int16                 AR2_Q13[ NB_SUBFR * MAX_SHAPE_LPC_ORDER ],  /* I    Noise shaping filter                */
    const SKP_int                   HarmShapeGain_Q14[ NB_SUBFR ],              /* I    Smooth coefficients                 */
    const SKP_int                   Tilt_Q14[ NB_SUBFR ],                       /* I    Spectral tilt                       */
    const SKP_int32                 LF_shp_Q14[ NB_SUBFR ],                     /* I    Short-term shaping coefficients     */
    const SKP_int32                 Gains_Q16[ NB_SUBFR ],                      /* I    Gain for each subframe              */
	const SKP_int                   Lambda_Q10,                                 /* I    Quantization coefficient            */
    const SKP_int                   LTP_scale_Q14 ,                             /* I    LTP state scaling                   */
	const SKP_int32 				MDGains_Q16[ NB_SUBFR ],                    /* I    New gain, no use now                */
	const SKP_int32 				DeltaGains_Q16                              /* I    Gain for odd subframe               */
)
{
    SKP_int     k, lag, start_idx, LSF_interpolation_flag, i;
    const SKP_int16 *A_Q12, *B_Q14, *AR_shp_Q13;
    SKP_int16   *pxq;
    SKP_int32   sLTP_Q16[ 2 * MAX_FRAME_LENGTH ];
    SKP_int16   sLTP[     2 * MAX_FRAME_LENGTH ];
    SKP_int32   HarmShapeFIRPacked_Q14;
    SKP_int     offset_Q10;
    SKP_int32   FiltState[ MAX_LPC_ORDER ];
    SKP_int32   x_sc_Q10[ MAX_FRAME_LENGTH / NB_SUBFR ];
    SKP_int32   x_md_Q10[ MAX_FRAME_LENGTH / NB_SUBFR ];
    SKP_int16   *pxq_md[MAX_INTERLEAVE_NUM];
    SKP_int32   sLTP_md_Q16[MAX_INTERLEAVE_NUM][ 2 * MAX_FRAME_LENGTH ];
    SKP_int16   sLTP_md[MAX_INTERLEAVE_NUM][     2 * MAX_FRAME_LENGTH ];
    SKP_int32   FiltStateMD[MAX_INTERLEAVE_NUM][ MAX_LPC_ORDER ];
    SKP_int32   *psLTP_md_Q16[MAX_INTERLEAVE_NUM];
	SKP_memset(sLTP_md_Q16,0,sizeof(SKP_int32)*MAX_INTERLEAVE_NUM*2 * MAX_FRAME_LENGTH);
	SKP_memset(sLTP_md,0,sizeof(SKP_int16)*MAX_INTERLEAVE_NUM*2 * MAX_FRAME_LENGTH);
	SKP_memset(sLTP_Q16,0,sizeof(SKP_int32)*2 * MAX_FRAME_LENGTH);
	SKP_memset(sLTP,0,sizeof(SKP_int16)*2 * MAX_FRAME_LENGTH);

    NSQ->rand_seed  =  psEncCtrlC->Seed;
	
	for(i = 0; i< psEncC->interleave_num; i++){
    	NSQ_md[i].rand_seed  =  psEncCtrlC->Seed;
		psLTP_md_Q16[i]  = sLTP_md_Q16[i];
	}
    /* Set unvoiced lag to the previous one, overwrite later for voiced */
    lag             = NSQ->lagPrev;

    SKP_assert( NSQ->prev_inv_gain_Q16 != 0 );

    offset_Q10 = SKP_Silk_Quantization_Offsets_Q10[ psEncCtrlC->sigtype ][ psEncCtrlC->QuantOffsetType ];

    if( LSFInterpFactor_Q2 == ( 1 << 2 ) ) {
        LSF_interpolation_flag = 0;
    } else {
        LSF_interpolation_flag = 1;
    }

    /* Setup pointers to start of sub frame */
    NSQ->sLTP_shp_buf_idx = psEncC->frame_length;
    NSQ->sLTP_buf_idx     = psEncC->frame_length;
    pxq                   = &NSQ->xq[ psEncC->frame_length ];

	
	for(i = 0; i< psEncC->interleave_num; i++)
	{
    	NSQ_md[i].sLTP_shp_buf_idx = psEncC->frame_length;
    	NSQ_md[i].sLTP_buf_idx     = psEncC->frame_length;
    	pxq_md[i] 				   = &(NSQ_md[i].xq[ psEncC->frame_length ]);
	}
	
    for( k = 0; k < NB_SUBFR; k++ ) {
        A_Q12      = &PredCoef_Q12[ (( k >> 1 ) | ( 1 - LSF_interpolation_flag )) * MAX_LPC_ORDER ];
        B_Q14      = &LTPCoef_Q14[ k * LTP_ORDER ];
        AR_shp_Q13 = &AR2_Q13[     k * MAX_SHAPE_LPC_ORDER ];

        /* Noise shape parameters */
        SKP_assert( HarmShapeGain_Q14[ k ] >= 0 );
        HarmShapeFIRPacked_Q14  =                          SKP_RSHIFT( HarmShapeGain_Q14[ k ], 2 );
        HarmShapeFIRPacked_Q14 |= SKP_LSHIFT( ( SKP_int32 )SKP_RSHIFT( HarmShapeGain_Q14[ k ], 1 ), 16 );

        NSQ->rewhite_flag = 0;
		for(i = 0; i< psEncC->interleave_num; i++){
            NSQ_md[i].rewhite_flag = 0;
		}
        if( psEncCtrlC->sigtype == SIG_TYPE_VOICED ) {
            /* Voiced */
            lag = psEncCtrlC->pitchL[ k ];

            /* Re-whitening */
            if( ( k & ( 3 - SKP_LSHIFT( LSF_interpolation_flag, 1 ) ) ) == 0 ) {

                /* Rewhiten with new A coefs */
                start_idx = psEncC->frame_length - lag - psEncC->predictLPCOrder - LTP_ORDER / 2;
                SKP_assert( start_idx >= 0 );
                SKP_assert( start_idx <= psEncC->frame_length - psEncC->predictLPCOrder );

                SKP_memset( FiltState, 0, psEncC->predictLPCOrder * sizeof( SKP_int32 ) );
                SKP_Silk_MA_Prediction( &NSQ->xq[ start_idx + k * ( psEncC->frame_length >> 2 ) ], 
                    A_Q12, FiltState, sLTP + start_idx, psEncC->frame_length - start_idx, psEncC->predictLPCOrder );

                NSQ->rewhite_flag = 1;
                NSQ->sLTP_buf_idx = psEncC->frame_length;
				
				for(i = 0; i< psEncC->interleave_num; i++){
                	SKP_memset( FiltStateMD[i], 0, psEncC->predictLPCOrder * sizeof( SKP_int32 ) );
                	SKP_Silk_MA_Prediction( &(NSQ_md[i].xq[ start_idx + k * ( psEncC->frame_length >> 2 ) ]), 
                    	A_Q12, FiltStateMD[i], sLTP_md[i] + start_idx, psEncC->frame_length - start_idx, psEncC->predictLPCOrder );
				
                	NSQ_md[i].rewhite_flag = 1;
                	NSQ_md[i].sLTP_buf_idx = psEncC->frame_length;
				}
            }
        }

        SKP_Silk_nsq_scale_states( NSQ, x, x_sc_Q10, psEncC->subfr_length, sLTP, 
            sLTP_Q16, k, LTP_scale_Q14, Gains_Q16, psEncCtrlC->pitchL );
			
		for(i = 0; i< psEncC->interleave_num; i++){
			SKP_Silk_nsq_scale_states(&(NSQ_md[i]),x, x_md_Q10,  psEncC->subfr_length, sLTP_md[i], 
            sLTP_md_Q16[i], k, LTP_scale_Q14, Gains_Q16, psEncCtrlC->pitchL );
		}

        Agora_MD_Silk_noise_shape_quantizer( NSQ, NSQ_md, psEncCtrlC->sigtype, 
			x_sc_Q10, x_md_Q10,
			q,r, pxq, sLTP_Q16, 
			q_md, pxq_md, psLTP_md_Q16 ,
			A_Q12, B_Q14, AR_shp_Q13, lag, HarmShapeFIRPacked_Q14, Tilt_Q14[ k ], LF_shp_Q14[ k ], Gains_Q16[ k ], Lambda_Q10, 
            offset_Q10, psEncC->subfr_length, psEncC->shapingLPCOrder, psEncC->predictLPCOrder,MDGains_Q16[ k ],DeltaGains_Q16
        );

        x          += psEncC->subfr_length;
        q          += psEncC->subfr_length;
        r          += psEncC->subfr_length;
        pxq        += psEncC->subfr_length;
		
		for(i = 0; i< psEncC->interleave_num; i++){
        	q_md[i]   += psEncC->subfr_length;
            pxq_md[i] += psEncC->subfr_length;
		}
    }

    /* Update lagPrev for next frame */
    NSQ->lagPrev = psEncCtrlC->pitchL[ NB_SUBFR - 1 ];
	for(i = 0; i< psEncC->interleave_num; i++){
    	NSQ_md[i].lagPrev = psEncCtrlC->pitchL[ NB_SUBFR - 1 ];
	}

    /* Save quantized speech and noise shaping signals */
    SKP_memcpy( NSQ->xq,           &NSQ->xq[           psEncC->frame_length ], psEncC->frame_length * sizeof( SKP_int16 ) );
    SKP_memcpy( NSQ->sLTP_shp_Q10, &NSQ->sLTP_shp_Q10[ psEncC->frame_length ], psEncC->frame_length * sizeof( SKP_int32 ) );
	
	for(i = 0; i< psEncC->interleave_num; i++){
    	SKP_memcpy( NSQ_md[i].xq,           &(NSQ_md[i].xq[           psEncC->frame_length ]), psEncC->frame_length * sizeof( SKP_int16 ) );
    	SKP_memcpy( NSQ_md[i].sLTP_shp_Q10, &(NSQ_md[i].sLTP_shp_Q10[ psEncC->frame_length ]), psEncC->frame_length * sizeof( SKP_int32 ) );
	}
	
}

/**************************************************/
/* Oiriginal Silk SKP_Silk_noise_shape_quantizer  */
/**************************************************/
SKP_INLINE void SKP_Silk_noise_shape_quantizer(
    SKP_Silk_nsq_state  *NSQ,               /* I/O  NSQ state                       */
    SKP_Silk_nsq_state  (*NSQ_md)[MAX_INTERLEAVE_NUM], /* I/O  NSQ state                           */
    SKP_int             sigtype,            /* I    Signal type                     */
    const SKP_int32     x_sc_Q10[],         /* I                                    */
    SKP_int8            q[],                /* O                                    */
	SKP_int32           r[],                /* O                                    */
    SKP_int16           xq[],               /* O                                    */
    SKP_int32           sLTP_Q16[],         /* I/O  LTP state                       */
    SKP_int32           (*sLTP_md_Q16)[MAX_INTERLEAVE_NUM],      /* I/O  LTP state                       */
    const SKP_int16     a_Q12[],            /* I    Short term prediction coefs     */
    const SKP_int16     b_Q14[],            /* I    Long term prediction coefs      */
    const SKP_int16     AR_shp_Q13[],       /* I    Noise shaping AR coefs          */
    SKP_int             lag,                /* I    Pitch lag                       */
    SKP_int32           HarmShapeFIRPacked_Q14, /* I                                */
    SKP_int             Tilt_Q14,           /* I    Spectral tilt                   */
    SKP_int32           LF_shp_Q14,         /* I                                    */
    SKP_int32           Gain_Q16,           /* I                                    */
    SKP_int32           DeltaGains_Q16,           /* I                                    */
    SKP_int             Lambda_Q10,         /* I                                    */
    SKP_int             offset_Q10,         /* I                                    */
    SKP_int             length,             /* I    Input length                    */
    SKP_int             shapingLPCOrder,    /* I    Noise shaping AR filter order   */
    SKP_int             predictLPCOrder     /* I    Prediction filter order         */
)
{
    SKP_int     i, j, k;
    SKP_int32   LTP_pred_Q14, LPC_pred_Q10, n_AR_Q10, n_LTP_Q14;
    SKP_int32   n_LF_Q10, r_Q10, q_Q0, q_Q10,r_temp_Q10;
    SKP_int32   thr1_Q10, thr2_Q10, thr3_Q10;
    SKP_int32   thr1_md_Q10, thr2_md_Q10, thr3_md_Q10;
    SKP_int32   dither, exc_Q10, LPC_exc_Q10, xq_Q10;
    SKP_int32   tmp1, tmp2, sLF_AR_shp_Q10;
    SKP_int32   *psLPC_Q14, *shp_lag_ptr, *pred_lag_ptr;
    SKP_int32   q1_Q0, q1_Q10;
    SKP_int32   inv_gain_Q16;	
    SKP_int32   *psLPC_md_Q14[MAX_INTERLEAVE_NUM], *shp_lag_md_ptr[MAX_INTERLEAVE_NUM], *pred_lag_md_ptr[MAX_INTERLEAVE_NUM];

    inv_gain_Q16 = SKP_INVERSE32_varQ( SKP_max( DeltaGains_Q16, 1 ), 32 );
    inv_gain_Q16 = SKP_min( inv_gain_Q16, SKP_int32_MAX );

    shp_lag_ptr  = &NSQ->sLTP_shp_Q10[ NSQ->sLTP_shp_buf_idx - lag + HARM_SHAPE_FIR_TAPS / 2 ];
    pred_lag_ptr = &sLTP_Q16[ NSQ->sLTP_buf_idx - lag + LTP_ORDER / 2 ];
    
    /* Setup short term AR state */
    psLPC_Q14 = &NSQ->sLPC_Q14[ NSQ_LPC_BUF_LENGTH - 1 ];

	for(k = 0;k < 2; k++){
    	shp_lag_md_ptr[k]  = &(NSQ_md[k]->sLTP_shp_Q10[ NSQ_md[k]->sLTP_shp_buf_idx - lag + HARM_SHAPE_FIR_TAPS / 2 ]);
    	pred_lag_md_ptr[k] = &sLTP_md_Q16[k][ NSQ_md[k]->sLTP_buf_idx - lag + LTP_ORDER / 2 ];
    
    	/* Setup short term AR state */
    	psLPC_md_Q14[k] = &(NSQ_md[k]->sLPC_Q14[ NSQ_LPC_BUF_LENGTH - 1 ]);
	}
    /* Quantization thresholds */

    thr1_Q10 = SKP_SUB_RSHIFT32( -1536, Lambda_Q10, 1 );
    thr2_Q10 = SKP_SUB_RSHIFT32(  -512, Lambda_Q10, 1 );
    thr2_Q10 = SKP_ADD_RSHIFT32( thr2_Q10, SKP_SMULBB( offset_Q10, Lambda_Q10 ), 10 );
    thr3_Q10 = SKP_ADD_RSHIFT32(   512, Lambda_Q10, 1 );

    thr1_md_Q10 = SKP_SUB_RSHIFT32(  -1536, Lambda_Q10, 1 );
    thr2_md_Q10 = SKP_SUB_RSHIFT32(  -512, Lambda_Q10, 1 );
    thr2_md_Q10 = SKP_ADD_RSHIFT32( thr2_md_Q10, SKP_SMULBB( offset_Q10, Lambda_Q10 ), 10 );
    thr3_md_Q10 = SKP_ADD_RSHIFT32(   512, Lambda_Q10, 1 );

    for( i = 0; i < length; i++ ) {
		
        /* Generate dither */
        NSQ->rand_seed = SKP_RAND( NSQ->rand_seed );
        /* dither = rand_seed < 0 ? 0xFFFFFFFF : 0; */
        dither = SKP_RSHIFT( NSQ->rand_seed, 31 );
        
#ifdef DISABLE_DITHER		
        dither = 0;
#endif                 
        /* Short-term prediction */
        SKP_assert( ( predictLPCOrder  & 1 ) == 0 );    /* check that order is even */
        /* check that array starts at 4-byte aligned address */
        SKP_assert( ( ( SKP_int64 )( ( SKP_int8* )a_Q12 - ( SKP_int8* )0 ) & 3 ) == 0 );
        SKP_assert( predictLPCOrder >= 10 );            /* check that unrolling works */

        /* Short-term prediction */
		
		LPC_pred_Q10 = 0;
        for( j = 0; j < predictLPCOrder; j ++ ) {
            LPC_pred_Q10 = SKP_SMLAWB( LPC_pred_Q10, psLPC_Q14[ -j ], a_Q12[ j ] );
        }

		/* Long-term prediction */

		LTP_pred_Q14 = 0;
        if( sigtype == SIG_TYPE_VOICED ) {
            /* Unrolled loop */
        	for( j = 0; j < 5; j ++ ) {
            	LTP_pred_Q14 = SKP_SMLAWB( LTP_pred_Q14, pred_lag_ptr[ -j ], b_Q14[ j ] );
        	}
            pred_lag_ptr++;
        } else {
            LTP_pred_Q14 = 0;
        }

        /* Noise shape feedback */
        SKP_assert( ( shapingLPCOrder & 1 ) == 0 );   /* check that order is even */
        tmp2 = psLPC_Q14[ 0 ];
        tmp1 = NSQ->sAR2_Q14[ 0 ];
        NSQ->sAR2_Q14[ 0 ] = tmp2;
        n_AR_Q10 = SKP_SMULWB( tmp2, AR_shp_Q13[ 0 ] );
        for( j = 2; j < shapingLPCOrder; j += 2 ) {
            tmp2 = NSQ->sAR2_Q14[ j - 1 ];
            NSQ->sAR2_Q14[ j - 1 ] = tmp1;
            n_AR_Q10 = SKP_SMLAWB( n_AR_Q10, tmp1, AR_shp_Q13[ j - 1 ] );
            tmp1 = NSQ->sAR2_Q14[ j + 0 ];
            NSQ->sAR2_Q14[ j + 0 ] = tmp2;
            n_AR_Q10 = SKP_SMLAWB( n_AR_Q10, tmp2, AR_shp_Q13[ j ] );
        }
        NSQ->sAR2_Q14[ shapingLPCOrder - 1 ] = tmp1;
        n_AR_Q10 = SKP_SMLAWB( n_AR_Q10, tmp1, AR_shp_Q13[ shapingLPCOrder - 1 ] );

        n_AR_Q10 = SKP_RSHIFT( n_AR_Q10, 1 );   /* Q11 -> Q10 */
        n_AR_Q10 = SKP_SMLAWB( n_AR_Q10, NSQ->sLF_AR_shp_Q12, Tilt_Q14 );

        n_LF_Q10 = SKP_LSHIFT( SKP_SMULWB( NSQ->sLTP_shp_Q10[ NSQ->sLTP_shp_buf_idx - 1 ], LF_shp_Q14 ), 2 ); 
        n_LF_Q10 = SKP_SMLAWT( n_LF_Q10, NSQ->sLF_AR_shp_Q12, LF_shp_Q14 );

        SKP_assert( lag > 0 || sigtype == SIG_TYPE_UNVOICED );

        /* Long-term shaping */
        if( lag > 0 ) {
            /* Symmetric, packed FIR coefficients */
            n_LTP_Q14 = SKP_SMULWB( SKP_ADD32( shp_lag_ptr[ 0 ], shp_lag_ptr[ -2 ] ), HarmShapeFIRPacked_Q14 );
            n_LTP_Q14 = SKP_SMLAWT( n_LTP_Q14, shp_lag_ptr[ -1 ],                     HarmShapeFIRPacked_Q14 );
            n_LTP_Q14 = SKP_LSHIFT( n_LTP_Q14, 6 );
            shp_lag_ptr++;
        } else {
            n_LTP_Q14 = 0;
        }

		/* Input minus prediction plus noise feedback  */
        //r = x[ i ] - LTP_pred - LPC_pred + n_AR + n_Tilt + n_LF + n_LTP;
        tmp1  = SKP_SUB32( LTP_pred_Q14, n_LTP_Q14 );                       /* Add Q14 stuff */
        tmp1  = SKP_RSHIFT( tmp1, 4 );                                      /* convert to Q10  */
        tmp1  = SKP_ADD32( tmp1, LPC_pred_Q10 );                            /* add Q10 stuff */ 
        tmp1  = SKP_SUB32( tmp1, n_AR_Q10 );                                /* subtract Q10 stuff */ 
        tmp1  = SKP_SUB32( tmp1, n_LF_Q10 );                                /* subtract Q10 stuff */ 
        r_Q10 = SKP_SUB32( x_sc_Q10[ i ], tmp1 );
		r_temp_Q10 = r_Q10;

		/* Flip sign depending on dither */
		r_Q10 = ( r_Q10 ^ dither ) - dither;
		r_Q10 = SKP_SUB32( r_Q10, offset_Q10 );
		r_Q10 = SKP_LIMIT_32( r_Q10, -64 << 10, 64 << 10 );

		/* Quantize */
		q_Q0 = 0;
		q_Q10 = 0;
		q1_Q10 = q_Q10;
		if( r_Q10 < thr2_Q10 ) {
			if( r_Q10 < thr1_Q10 ) {
				q_Q0 = SKP_RSHIFT_ROUND( SKP_ADD_RSHIFT32( r_Q10, Lambda_Q10, 1 ), 10 );
				q_Q10 = SKP_LSHIFT( q_Q0, 10 );
				q1_Q10 = SKP_ADD_RSHIFT32( r_Q10, Lambda_Q10, 1 );
			} else {
				q_Q0 = -1;
				q_Q10 = -1024;
				q1_Q10 = -1024;
			}
		} else {
			if( r_Q10 > thr3_Q10 ) {
				q_Q0 = SKP_RSHIFT_ROUND( SKP_SUB_RSHIFT32( r_Q10, Lambda_Q10, 1 ), 10 );
				q_Q10 = SKP_LSHIFT( q_Q0, 10 );
				q1_Q10 = q_Q10;
			}
		}
		q[ i ] = ( SKP_int8 )q_Q0; /* No saturation needed because max is 64 */

		q1_Q0 = q_Q0;
        q_Q10 = q1_Q10;
        q[ i ] = ( SKP_int8 )q1_Q0; /* No saturation needed because max is 64 */

        /* Excitation */
        exc_Q10 = SKP_ADD32( q_Q10, offset_Q10 );
        exc_Q10 = ( exc_Q10 ^ dither ) - dither;

	
		r[i] = exc_Q10;
        /* Add predictions */
        LPC_exc_Q10 = SKP_ADD32( exc_Q10, SKP_RSHIFT_ROUND( LTP_pred_Q14, 4 ) );
        xq_Q10      = SKP_ADD32( LPC_exc_Q10, LPC_pred_Q10 );
        
        /* Scale XQ back to normal level before saving */
        xq[ i ] = ( SKP_int16 )SKP_SAT16( SKP_RSHIFT_ROUND( SKP_SMULWW( xq_Q10, Gain_Q16 ), 10 ) );
        
        
        /* Update states */
        psLPC_Q14++;
        *psLPC_Q14 = SKP_LSHIFT( xq_Q10, 4 );
        sLF_AR_shp_Q10 = SKP_SUB32( xq_Q10, n_AR_Q10 );
        NSQ->sLF_AR_shp_Q12 = SKP_LSHIFT( sLF_AR_shp_Q10, 2 );

        NSQ->sLTP_shp_Q10[ NSQ->sLTP_shp_buf_idx ] = SKP_SUB32( sLF_AR_shp_Q10, n_LF_Q10 );
        sLTP_Q16[ NSQ->sLTP_buf_idx ] = SKP_LSHIFT( LPC_exc_Q10, 6 );
        NSQ->sLTP_shp_buf_idx++;
        NSQ->sLTP_buf_idx++;

    }

    /* Update LPC synth buffer */
    SKP_memcpy( NSQ->sLPC_Q14, &NSQ->sLPC_Q14[ length ], NSQ_LPC_BUF_LENGTH * sizeof( SKP_int32 ) );
}

SKP_int32 Agora_Silk_NSQ(
	SKP_int32   		r_Q10,
	SKP_int8            q[],
	SKP_int32           dither,
    SKP_int             Lambda_Q10,             /* I                                        */
    SKP_int             offset_Q10             /* I                                        */
	)
{
	SKP_int32  q_Q0, q_Q10,q1_Q10;
	SKP_int32  thr1_Q10, thr2_Q10, thr3_Q10;
	SKP_int32  exc_Q10;

    thr1_Q10 = SKP_SUB_RSHIFT32( -1536, Lambda_Q10, 1 );
    thr2_Q10 = SKP_SUB_RSHIFT32(  -512, Lambda_Q10, 1 );
    thr2_Q10 = SKP_ADD_RSHIFT32( thr2_Q10, SKP_SMULBB( offset_Q10, Lambda_Q10 ), 10 );
    thr3_Q10 = SKP_ADD_RSHIFT32(   512, Lambda_Q10, 1 );
	/* Flip sign depending on dither */
	r_Q10 = ( r_Q10 ^ dither ) - dither;
	r_Q10 = SKP_SUB32( r_Q10, offset_Q10 );
	r_Q10 = SKP_LIMIT_32( r_Q10, -64 << 10, 64 << 10 );
	
	/* Quantize */
	q_Q0 = 0;
	q_Q10 = 0;
	
	if( r_Q10 < thr2_Q10 ) {
		if( r_Q10 < thr1_Q10 ) {
			q_Q0 = SKP_RSHIFT_ROUND( SKP_ADD_RSHIFT32( r_Q10, Lambda_Q10, 1 ), 10 );
			q_Q10 = SKP_LSHIFT( q_Q0, 10 );
			q1_Q10 = SKP_ADD_RSHIFT32( r_Q10, Lambda_Q10, 1 );
		} else {
			q_Q0 = -1;
			q_Q10 = -1024;
			q1_Q10 = -1024;
		}
	} else {
		if( r_Q10 > thr3_Q10 ) {
			q_Q0 = SKP_RSHIFT_ROUND( SKP_SUB_RSHIFT32( r_Q10, Lambda_Q10, 1 ), 10 );
			q_Q10 = SKP_LSHIFT( q_Q0, 10 );
			q1_Q10 = q_Q10;
		}
	}
	*q = ( SKP_int8 )q_Q0; /* No saturation needed because max is 64 */
	/* Excitation */
	exc_Q10 = SKP_ADD32( q_Q10, offset_Q10 );
	exc_Q10 = ( exc_Q10 ^ dither ) - dither;

	return exc_Q10;
}

SKP_int32 Agora_Silk_Rec_And_UpdateState(
	SKP_Silk_nsq_state  *NSQ, 
	SKP_int8    q,
	SKP_int32	exc_Q10,
	SKP_int32   Gain_Q16,
	SKP_int32	*psLPC_Q14,
    SKP_int32   sLTP_Q16[],         /* I/O  LTP state                       */
	SKP_int32   LTP_pred_Q14,
	SKP_int32   LPC_pred_Q10,
	SKP_int32   n_AR_Q10,
	SKP_int32   n_LF_Q10
)
{
	SKP_int32   LPC_exc_Q10;
	SKP_int32   sLF_AR_shp_Q10;
	SKP_int32   xq_Q10;
	SKP_int32   xq = 0;
	
    /* Add predictions */
    LPC_exc_Q10 = SKP_ADD32( exc_Q10, SKP_RSHIFT_ROUND( LTP_pred_Q14, 4 ) );
    xq_Q10      = SKP_ADD32( LPC_exc_Q10, LPC_pred_Q10 );
    
    /* Scale XQ back to normal level before saving */
    xq = ( SKP_int16 )SKP_SAT16( SKP_RSHIFT_ROUND( SKP_SMULWW( xq_Q10, Gain_Q16 ), 10 ) );
	
	/* Update states */
	*psLPC_Q14 = SKP_LSHIFT( xq_Q10, 4 );
	sLF_AR_shp_Q10 = SKP_SUB32( xq_Q10, n_AR_Q10 );
	NSQ->sLF_AR_shp_Q12 = SKP_LSHIFT( sLF_AR_shp_Q10, 2 );
	
	NSQ->sLTP_shp_Q10[ NSQ->sLTP_shp_buf_idx ] = SKP_SUB32( sLF_AR_shp_Q10, n_LF_Q10 );
	sLTP_Q16[ NSQ->sLTP_buf_idx ] = SKP_LSHIFT( LPC_exc_Q10, 6 );

	NSQ->sLTP_shp_buf_idx++;

	NSQ->sLTP_buf_idx++;

	/* Make dither dependent on quantized signal */

	return xq;
}


/***********************************/
/* SKP_Silk_noise_shape_quantizer  */
/***********************************/
SKP_INLINE void Agora_MD_Silk_noise_shape_quantizer(
    SKP_Silk_nsq_state  *NSQ,               /* I/O  NSQ state                       */
    SKP_Silk_nsq_state  NSQ_md[MAX_INTERLEAVE_NUM], /* I/O  NSQ state                           */
    SKP_int             sigtype,            /* I    Signal type                     */
    const SKP_int32     x_sc_Q10[],         /* I                                    */
    const SKP_int32     x_md_Q10[],         /* I                                    */
    SKP_int8            q[],                /* O                                    */
	SKP_int32           r[],                /* O                                    */
    SKP_int16           xq[],               /* O                                    */
    SKP_int32           sLTP_Q16[],         /* I/O  LTP state                       */
    SKP_int8            *q_md[MAX_INTERLEAVE_NUM],      /* I/O  LTP state                       */
    SKP_int16           *xq_md[MAX_INTERLEAVE_NUM],      /* I/O  LTP state                       */
    SKP_int32           *sLTP_md_Q16[MAX_INTERLEAVE_NUM],      /* I/O  LTP state                       */
    const SKP_int16     a_Q12[],            /* I    Short term prediction coefs     */
    const SKP_int16     b_Q14[],            /* I    Long term prediction coefs      */
    const SKP_int16     AR_shp_Q13[],       /* I    Noise shaping AR coefs          */
    SKP_int             lag,                /* I    Pitch lag                       */
    SKP_int32           HarmShapeFIRPacked_Q14, /* I                                */
    SKP_int             Tilt_Q14,           /* I    Spectral tilt                   */
    SKP_int32           LF_shp_Q14,         /* I                                    */
    SKP_int32           Gain_Q16,           /* I                                    */
    SKP_int             Lambda_Q10,         /* I                                    */
    SKP_int             offset_Q10,         /* I                                    */
    SKP_int             length,             /* I    Input length                    */
    SKP_int             shapingLPCOrder,    /* I    Noise shaping AR filter order   */
    SKP_int             predictLPCOrder,     /* I    Prediction filter order         */
    const SKP_int32 				MDGains_Q16,
	const SKP_int32 				DeltaGains_Q16
)
{
    SKP_int     i, k;
    SKP_int32   LTP_pred_Q14, LPC_pred_Q10, n_AR_Q10, n_LTP_Q14;
    SKP_int32   n_LF_Q10, r_Q10,r_temp_Q10;
    SKP_int32   thr1_Q10, thr2_Q10, thr3_Q10;
    SKP_int32   dither, exc_Q10, temp_exc_Q10;
    SKP_int32   *psLPC_Q14, *shp_lag_ptr, *pred_lag_ptr;
    SKP_int32   inv_gain_Q16;
	SKP_int32   xin_md[MAX_INTERLEAVE_NUM],exc_md_Q10[MAX_INTERLEAVE_NUM];//,r_md_Q10[MAX_INTERLEAVE_NUM];
    SKP_int32   LTP_pred_md_Q14[MAX_INTERLEAVE_NUM], LPC_pred_md_Q10[MAX_INTERLEAVE_NUM], n_AR_md_Q10[MAX_INTERLEAVE_NUM], n_LTP_md_Q14[MAX_INTERLEAVE_NUM];
    SKP_int32   *psLPC_md_Q14[MAX_INTERLEAVE_NUM], *shp_lag_md_ptr[MAX_INTERLEAVE_NUM], *pred_lag_md_ptr[MAX_INTERLEAVE_NUM];
    SKP_int32   n_LF_md_Q10[MAX_INTERLEAVE_NUM], r_md_Q10[MAX_INTERLEAVE_NUM];
	SKP_int32	dither_md[MAX_INTERLEAVE_NUM];
	SKP_int32   r_md1_Q10, r_md2_Q10;
	SKP_int32   DeltaGains_p1_Q16, DeltaGains_p2_Q16, inv_gain_p1_Q16, inv_gain_p2_Q16;

    inv_gain_Q16 = SKP_INVERSE32_varQ( SKP_max( DeltaGains_Q16, 1 ), 32 );
    inv_gain_Q16 = SKP_min( inv_gain_Q16, SKP_int32_MAX );

    shp_lag_ptr  = &NSQ->sLTP_shp_Q10[ NSQ->sLTP_shp_buf_idx - lag + HARM_SHAPE_FIR_TAPS / 2 ];
    pred_lag_ptr = &sLTP_Q16[ NSQ->sLTP_buf_idx - lag + LTP_ORDER / 2 ];
    
    /* Setup short term AR state */
    psLPC_Q14 = &NSQ->sLPC_Q14[ NSQ_LPC_BUF_LENGTH - 1 ];

	inv_gain_Q16 = SKP_INVERSE32_varQ( SKP_max( DeltaGains_Q16, 1 ), 32 );
	inv_gain_Q16 = SKP_min( inv_gain_Q16, SKP_int32_MAX );
	
	inv_gain_p1_Q16 = inv_gain_Q16;  
	inv_gain_p1_Q16 = SKP_min( inv_gain_p1_Q16, SKP_int32_MAX );
	inv_gain_p2_Q16 = (65536 - inv_gain_Q16);  
	inv_gain_p2_Q16 = SKP_min( inv_gain_p2_Q16, SKP_int32_MAX );


	DeltaGains_p1_Q16 = SKP_INVERSE32_varQ( SKP_max( inv_gain_p1_Q16, 1 ), 32 );  
	DeltaGains_p1_Q16 = SKP_min( DeltaGains_p1_Q16, SKP_int32_MAX );
	DeltaGains_p2_Q16 = SKP_INVERSE32_varQ( SKP_max( inv_gain_p2_Q16, 1 ), 32 );  
	DeltaGains_p2_Q16 = SKP_min( DeltaGains_p2_Q16, SKP_int32_MAX );

#ifdef DISABLE_OFFSET			
		offset_Q10 = 0;
#endif		

	for(k = 0;k < 2; k++){
    	shp_lag_md_ptr[k]  = &(NSQ_md[k].sLTP_shp_Q10[ NSQ_md[k].sLTP_shp_buf_idx - lag + HARM_SHAPE_FIR_TAPS / 2 ]);
    	pred_lag_md_ptr[k] = &sLTP_md_Q16[k][ NSQ_md[k].sLTP_buf_idx - lag + LTP_ORDER / 2 ];
    
    	/* Setup short term AR state */
    	psLPC_md_Q14[k] = &(NSQ_md[k].sLPC_Q14[ NSQ_LPC_BUF_LENGTH - 1 ]);
	}
    /* Quantization thresholds */

    thr1_Q10 = SKP_SUB_RSHIFT32( -1536, Lambda_Q10, 1 );
    thr2_Q10 = SKP_SUB_RSHIFT32(  -512, Lambda_Q10, 1 );
    thr2_Q10 = SKP_ADD_RSHIFT32( thr2_Q10, SKP_SMULBB( offset_Q10, Lambda_Q10 ), 10 );
    thr3_Q10 = SKP_ADD_RSHIFT32(   512, Lambda_Q10, 1 );


    for( i = 0; i < length; i++ ) {
		
        /* Generate dither */
        NSQ->rand_seed = SKP_RAND( NSQ->rand_seed );
        /* dither = rand_seed < 0 ? 0xFFFFFFFF : 0; */
        dither = SKP_RSHIFT( NSQ->rand_seed, 31 );
        
#ifdef DISABLE_DITHER		
        dither = 0;
#endif  

	for(k = 0;k < 2; k++){
        /* Generate dither */
        NSQ_md[k].rand_seed = SKP_RAND( NSQ_md[k].rand_seed );
        /* dither = rand_seed < 0 ? 0xFFFFFFFF : 0; */
        dither_md[k] = SKP_RSHIFT( NSQ_md[k].rand_seed, 31 );
        
#ifdef DISABLE_DITHER		
        dither_md[k] = 0;
#endif  
	}
		/* Short-term prediction */
		LPC_pred_Q10 = Agora_Silk_STP(psLPC_Q14,predictLPCOrder,a_Q12);

		/* Long-term prediction */
		LTP_pred_Q14 = Agora_Silk_LTP(sigtype,&pred_lag_ptr,b_Q14);
	
		/* Short-term shaping */
		n_AR_Q10	 = Agora_Silk_STS(psLPC_Q14,NSQ->sAR2_Q14,NSQ->sLF_AR_shp_Q12,shapingLPCOrder,0,AR_shp_Q13,Tilt_Q14);	
		n_LF_Q10	 = Agora_Silk_LFS(NSQ->sLTP_shp_Q10[ NSQ->sLTP_shp_buf_idx - 1 ], NSQ->sLF_AR_shp_Q12 ,n_AR_Q10	,LF_shp_Q14);	
	
        /* Long-term shaping */
		n_LTP_Q14 = Agora_Silk_LTS(lag,&shp_lag_ptr,HarmShapeFIRPacked_Q14);

		/* Input minus prediction plus noise feedback  */
        /*r = x[ i ] - LTP_pred - LPC_pred + n_AR + n_Tilt + n_LF + n_LTP*/
		r_Q10	     = Agora_Silk_DoPred_And_Shap(x_sc_Q10[ i ],LTP_pred_Q14,LPC_pred_Q10,n_LTP_Q14,n_AR_Q10,n_LF_Q10);
#define METHORD2
#ifdef METHORD2	
			
	if((i%2 == 0)){
		r_md1_Q10 = SKP_SMULWW( inv_gain_p1_Q16, r_Q10);
		r_md2_Q10 = SKP_SMULWW( inv_gain_p2_Q16, r_Q10);
	}else{
		r_md1_Q10 = SKP_SMULWW( inv_gain_p2_Q16, r_Q10);
		r_md2_Q10 = SKP_SMULWW( inv_gain_p1_Q16, r_Q10);
	}
#endif
		
		r_temp_Q10 = r_Q10;
		exc_Q10 = Agora_Silk_NSQ(r_Q10,&q[i],dither,Lambda_Q10,offset_Q10);
		
		temp_exc_Q10 = exc_Q10;
		for(k = 0;k < 2; k++){
			/* Short-term prediction */
			LPC_pred_md_Q10[k] = Agora_Silk_STP(psLPC_md_Q14[k],predictLPCOrder,a_Q12);

			/* Long-term prediction */
			LTP_pred_md_Q14[k] = Agora_Silk_LTP(sigtype,&pred_lag_md_ptr[k],b_Q14);

			/* Short-term shaping */
			n_AR_md_Q10[k]	 = Agora_Silk_STS(psLPC_md_Q14[k],NSQ_md[k].sAR2_Q14,NSQ_md[k].sLF_AR_shp_Q12,shapingLPCOrder,0,AR_shp_Q13,Tilt_Q14);	
			n_LF_md_Q10[k]	 = Agora_Silk_LFS(NSQ_md[k].sLTP_shp_Q10[ NSQ_md[k].sLTP_shp_buf_idx - 1 ], NSQ_md[k].sLF_AR_shp_Q12 ,n_AR_md_Q10[k]	,LF_shp_Q14);	
		
	        /* Long-term shaping */
			n_LTP_md_Q14[k]    = Agora_Silk_LTS(lag,&shp_lag_md_ptr[k],HarmShapeFIRPacked_Q14);

			/* Input minus prediction plus noise feedback  */
	        /*r = x[ i ] - LTP_pred - LPC_pred + n_AR + n_Tilt + n_LF + n_LTP*/
		
			xin_md[k]  = x_md_Q10[ i ];
			r_md_Q10[k]	 = Agora_Silk_DoPred(xin_md[k],LTP_pred_md_Q14[k],LPC_pred_md_Q10[k]);
			r_md_Q10[k]  = Agora_Silk_DoShap(r_md_Q10[k],n_LTP_md_Q14[k],n_AR_md_Q10[k],n_LF_md_Q10[k]);
		}	


	    exc_md_Q10[0] = Agora_Silk_NSQ(r_md1_Q10,&q_md[0][i],dither_md[0],Lambda_Q10,offset_Q10);
	    exc_md_Q10[1] = Agora_Silk_NSQ(r_md2_Q10,&q_md[1][i],dither_md[1],Lambda_Q10,offset_Q10);
	    exc_Q10 = exc_md_Q10[0] + exc_md_Q10[1];
		
	    if((i%2 == 0)){
		    exc_md_Q10[0] = SKP_SMULWW( DeltaGains_p1_Q16, exc_md_Q10[0]);
		    exc_md_Q10[1] = SKP_SMULWW( DeltaGains_p2_Q16, exc_md_Q10[1]);
	    }else{
		    exc_md_Q10[0] = SKP_SMULWW( DeltaGains_p2_Q16, exc_md_Q10[0]);
		    exc_md_Q10[1] = SKP_SMULWW( DeltaGains_p1_Q16, exc_md_Q10[1]);
	    }

        r[i] = exc_Q10;

	    psLPC_Q14++;
        xq[ i ] = Agora_Silk_Rec_And_UpdateState(NSQ,q[i],exc_Q10,Gain_Q16,psLPC_Q14,sLTP_Q16,LTP_pred_Q14,LPC_pred_Q10,n_AR_Q10,n_LF_Q10);
	    for(k = 0;k < 2; k++){
		    psLPC_md_Q14[k]++;	
	   	    xq_md[k][ i ] = Agora_Silk_Rec_And_UpdateState(&NSQ_md[k],q_md[k][i],exc_md_Q10[k],Gain_Q16,psLPC_md_Q14[k],sLTP_md_Q16[k],LTP_pred_md_Q14[k],LPC_pred_md_Q10[k],n_AR_md_Q10[k],n_LF_md_Q10[k]);
	    }	
    }
	
    /* Update LPC synth buffer */
    SKP_memcpy( NSQ->sLPC_Q14, &NSQ->sLPC_Q14[ length ], NSQ_LPC_BUF_LENGTH * sizeof( SKP_int32 ) );
	for(k = 0;k < 2; k++){
		SKP_memcpy( NSQ_md[k].sLPC_Q14, &(NSQ_md[k].sLPC_Q14[ length ]), NSQ_LPC_BUF_LENGTH * sizeof( SKP_int32 ) );
	}
}

SKP_INLINE void SKP_Silk_nsq_scale_states(
    SKP_Silk_nsq_state  *NSQ,               /* I/O NSQ state                        */
    const SKP_int16     x[],                /* I input in Q0                        */
    SKP_int32           x_sc_Q10[],         /* O input scaled with 1/Gain           */
    SKP_int             subfr_length,       /* I length of input                    */
    const SKP_int16     sLTP[],             /* I re-whitened LTP state in Q0        */
    SKP_int32           sLTP_Q16[],         /* O LTP state matching scaled input    */
    SKP_int             subfr,              /* I subframe number                    */
    const SKP_int       LTP_scale_Q14,      /* I                                    */
    const SKP_int32     Gains_Q16[ NB_SUBFR ], /* I                                 */
    const SKP_int       pitchL[ NB_SUBFR ]  /* I                                    */
)
{
    SKP_int   i, lag;
    SKP_int32 inv_gain_Q16, gain_adj_Q16, inv_gain_Q32;

    inv_gain_Q16 = SKP_INVERSE32_varQ( SKP_max( Gains_Q16[ subfr ], 1 ), 32 );
    inv_gain_Q16 = SKP_min( inv_gain_Q16, SKP_int16_MAX );
    lag          = pitchL[ subfr ];

    /* After rewhitening the LTP state is un-scaled, so scale with inv_gain_Q16 */
    if( NSQ->rewhite_flag ) {
        inv_gain_Q32 = SKP_LSHIFT( inv_gain_Q16, 16 );
        if( subfr == 0 ) {
            /* Do LTP downscaling */
            inv_gain_Q32 = SKP_LSHIFT( SKP_SMULWB( inv_gain_Q32, LTP_scale_Q14 ), 2 );
        }
        for( i = NSQ->sLTP_buf_idx - lag - LTP_ORDER / 2; i < NSQ->sLTP_buf_idx; i++ ) {
            SKP_assert( i < MAX_FRAME_LENGTH );
            sLTP_Q16[ i ] = SKP_SMULWB( inv_gain_Q32, sLTP[ i ] );
        }
    }

    /* Adjust for changing gain */
    if( inv_gain_Q16 != NSQ->prev_inv_gain_Q16 ) {
        gain_adj_Q16 = SKP_DIV32_varQ( inv_gain_Q16, NSQ->prev_inv_gain_Q16, 16 );

        /* Scale long-term shaping state */
        for( i = NSQ->sLTP_shp_buf_idx - subfr_length * NB_SUBFR; i < NSQ->sLTP_shp_buf_idx; i++ ) {
            NSQ->sLTP_shp_Q10[ i ] = SKP_SMULWW( gain_adj_Q16, NSQ->sLTP_shp_Q10[ i ] );
        }

        /* Scale long-term prediction state */
        if( NSQ->rewhite_flag == 0 ) {
            for( i = NSQ->sLTP_buf_idx - lag - LTP_ORDER / 2; i < NSQ->sLTP_buf_idx; i++ ) {
                sLTP_Q16[ i ] = SKP_SMULWW( gain_adj_Q16, sLTP_Q16[ i ] );
            }
        }

        NSQ->sLF_AR_shp_Q12 = SKP_SMULWW( gain_adj_Q16, NSQ->sLF_AR_shp_Q12 );

        /* Scale short-term prediction and shaping states */
        for( i = 0; i < NSQ_LPC_BUF_LENGTH; i++ ) {
            NSQ->sLPC_Q14[ i ] = SKP_SMULWW( gain_adj_Q16, NSQ->sLPC_Q14[ i ] );
        }
        for( i = 0; i < MAX_SHAPE_LPC_ORDER; i++ ) {
            NSQ->sAR2_Q14[ i ] = SKP_SMULWW( gain_adj_Q16, NSQ->sAR2_Q14[ i ] );
        }
    }

    /* Scale input */
    for( i = 0; i < subfr_length; i++ ) {
        x_sc_Q10[ i ] = SKP_RSHIFT( SKP_SMULBB( x[ i ], ( SKP_int16 )inv_gain_Q16 ), 6 );
    }

    /* save inv_gain */
    SKP_assert( inv_gain_Q16 != 0 );
    NSQ->prev_inv_gain_Q16 = inv_gain_Q16;
}


SKP_INLINE void SKP_Silk_nsq_scale_md_states(
    SKP_Silk_nsq_state  *NSQ,               /* I/O NSQ state                        */
    SKP_int             subfr_length,       /* I length of input                    */
    const SKP_int16     sLTP[],             /* I re-whitened LTP state in Q0        */
    SKP_int32           sLTP_Q16[],         /* O LTP state matching scaled input    */
    SKP_int             subfr,              /* I subframe number                    */
    const SKP_int       LTP_scale_Q14,      /* I                                    */
    const SKP_int32     Gains_Q16[ NB_SUBFR ], /* I                                 */
    const SKP_int       pitchL[ NB_SUBFR ]  /* I                                    */
)
{
    SKP_int   i, lag;
    SKP_int32 inv_gain_Q16, gain_adj_Q16, inv_gain_Q32;

    inv_gain_Q16 = SKP_INVERSE32_varQ( SKP_max( Gains_Q16[ subfr ], 1 ), 32 );
    inv_gain_Q16 = SKP_min( inv_gain_Q16, SKP_int16_MAX );
    lag          = pitchL[ subfr ];

    /* After rewhitening the LTP state is un-scaled, so scale with inv_gain_Q16 */
    if( NSQ->rewhite_flag ) {
        inv_gain_Q32 = SKP_LSHIFT( inv_gain_Q16, 16 );
        if( subfr == 0 ) {
            /* Do LTP downscaling */
            inv_gain_Q32 = SKP_LSHIFT( SKP_SMULWB( inv_gain_Q32, LTP_scale_Q14 ), 2 );
        }
        for( i = NSQ->sLTP_buf_idx - lag - LTP_ORDER / 2; i < NSQ->sLTP_buf_idx; i++ ) {
            SKP_assert( i < MAX_FRAME_LENGTH );
            sLTP_Q16[ i ] = SKP_SMULWB( inv_gain_Q32, sLTP[ i ] );
        }
    }

    /* Adjust for changing gain */
    if( inv_gain_Q16 != NSQ->prev_inv_gain_Q16 ) {
        gain_adj_Q16 = SKP_DIV32_varQ( inv_gain_Q16, NSQ->prev_inv_gain_Q16, 16 );

        /* Scale long-term shaping state */
        for( i = NSQ->sLTP_shp_buf_idx - subfr_length * NB_SUBFR; i < NSQ->sLTP_shp_buf_idx; i++ ) {
            NSQ->sLTP_shp_Q10[ i ] = SKP_SMULWW( gain_adj_Q16, NSQ->sLTP_shp_Q10[ i ] );
        }

        /* Scale long-term prediction state */
        if( NSQ->rewhite_flag == 0 ) {
            for( i = NSQ->sLTP_buf_idx - lag - LTP_ORDER / 2; i < NSQ->sLTP_buf_idx; i++ ) {
                sLTP_Q16[ i ] = SKP_SMULWW( gain_adj_Q16, sLTP_Q16[ i ] );
            }
        }

        NSQ->sLF_AR_shp_Q12 = SKP_SMULWW( gain_adj_Q16, NSQ->sLF_AR_shp_Q12 );

        /* Scale short-term prediction and shaping states */
        for( i = 0; i < NSQ_LPC_BUF_LENGTH; i++ ) {
            NSQ->sLPC_Q14[ i ] = SKP_SMULWW( gain_adj_Q16, NSQ->sLPC_Q14[ i ] );
        }
        for( i = 0; i < MAX_SHAPE_LPC_ORDER; i++ ) {
            NSQ->sAR2_Q14[ i ] = SKP_SMULWW( gain_adj_Q16, NSQ->sAR2_Q14[ i ] );
        }
    }

    /* save inv_gain */
    SKP_assert( inv_gain_Q16 != 0 );
    NSQ->prev_inv_gain_Q16 = inv_gain_Q16;
}

