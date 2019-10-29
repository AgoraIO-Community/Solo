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

#define OUTPUT_INDENPENT_RD

typedef struct {
    SKP_int32 RandState[ DECISION_DELAY ];
    SKP_int32 Q_Q0 [     DECISION_DELAY ];
    SKP_int32 Q_Q10[     DECISION_DELAY ];
    SKP_int32 X_Q10[     DECISION_DELAY ];
    SKP_int32 Xq_Q10[    DECISION_DELAY ];
    SKP_int32 Rd_Q10[     DECISION_DELAY ];
    SKP_int32 Pred_Q16[  DECISION_DELAY ];
    SKP_int32 Shape_Q10[ DECISION_DELAY ];
    SKP_int32 Gain_Q16[  DECISION_DELAY ];
	SKP_int32 exc_Q10[  DECISION_DELAY ]; 
    SKP_int32 sAR2_Q14[ MAX_SHAPE_LPC_ORDER ];
    SKP_int32 sLPC_Q14[ MAX_FRAME_LENGTH / NB_SUBFR + NSQ_LPC_BUF_LENGTH ];
    SKP_int32 LF_AR_Q12;
    SKP_int32 Seed;
    SKP_int32 SeedInit;
    SKP_int32 Seed2;
    SKP_int32 SeedInit2;
    SKP_int32 RD_Q10;
    SKP_int32 Q_md_Q10[4][     DECISION_DELAY ];
    SKP_int32 Xq_md_Q10[4][    DECISION_DELAY ];
    SKP_int32 RD_md_Q10[4];
} NSQ_del_dec_struct;

typedef struct {
    SKP_int32 Q_Q0;
    SKP_int32 Q_Q10;
    SKP_int32 RD_Q10;
    SKP_int32 xq_Q14;
    SKP_int32 Q_md_Q10[4];
    SKP_int32 RD_md_Q10[4];
    SKP_int32 xq_md_Q14[4];
    SKP_int32 LF_AR_Q12;
    SKP_int32 sLTP_shp_Q10;
    SKP_int32 LPC_exc_Q16;
	SKP_int32 exc_Q10; 
    SKP_int32 X_Q10;
    SKP_int32 Rd_independent_Q10;
} NSQ_sample_struct;



SKP_INLINE void SKP_Silk_copy_del_dec_state(
    NSQ_del_dec_struct  *DD_dst,                /* I    Dst del dec state                   */
    NSQ_del_dec_struct  *DD_src,                /* I    Src del dec state                   */
    SKP_int             LPC_state_idx           /* I    Index to LPC buffer                 */
);

SKP_INLINE void SKP_Silk_nsq_del_dec_scale_states(
    SKP_Silk_nsq_state  *NSQ,                   /* I/O  NSQ state                           */
    NSQ_del_dec_struct  psDelDec[],             /* I/O  Delayed decision states             */
    SKP_int             subfr_length,           /* I    Length of input                     */
    const SKP_int16     sLTP[],                 /* I    Re-whitened LTP state in Q0         */
    SKP_int32           sLTP_Q16[],             /* O    LTP state matching scaled input     */
    SKP_int             subfr,                  /* I    Subframe number                     */
    SKP_int             nStatesDelayedDecision, /* I    Number of del dec states            */
    SKP_int             smpl_buf_idx,           /* I    Index to newest samples in buffers  */
    const SKP_int       LTP_scale_Q14,          /* I    LTP state scaling                   */
    const SKP_int32     Gains_Q16[ NB_SUBFR ],  /* I    Gain                                */
    const SKP_int       pitchL[ NB_SUBFR ]      /* I    Pitch lag                           */
);

SKP_INLINE void Agora_Silk_DelDecScale(
    const SKP_int16     x[],                    /* I    Input in Q0                         */
    SKP_int32           x_sc_Q10[],             /* O    Input scaled with 1/Gain in Q10     */
    SKP_int             subfr_length,           /* I    Length of input                     */
    SKP_int             subfr,                  /* I    Subframe number                     */
    const SKP_int32     Gains_Q16[ NB_SUBFR ]   /* I    Gain                                */
	);

SKP_INLINE void SKP_Silk_md_noise_shape_quantizer_del_dec(
	SKP_Silk_nsq_state  *NSQ,                   /* I/O  NSQ state                           */
	NSQ_del_dec_struct  psDelDec[],             /* I/O  Delayed decision states             */
	SKP_int8            q[],                    /* O    Quantized Q0 residual               */
	SKP_int32           r[],                    /* O    Unquantized residual                */
	SKP_int16           xq[],                   /* O    Simulated full signal               */
	SKP_int32           prd[],                  /* O    Accumalated error                   */
	SKP_int32           ob_q_Q10[],             /* O    Quantized Q10 residual              */
	SKP_int32           sLTP_Q16[],             /* I/O  LTP filter state                    */
	SKP_int             lag,                    /* I    Pitch lag                           */
	SKP_int             *smpl_buf_idx,          /* I/O  Index to newest samples in buffers  */
	SKP_Silk_nsq_state  *NSQ_pair1,             /* I/O  NSQ state                           */
	NSQ_del_dec_struct  psDelDec_pair1[],       /* I/O  Delayed decision states             */
	SKP_int8            q_pair1[],              /* O    Quantized Q0 residual               */
	SKP_int16           xq_pair1[],             /* O    Simulated full signal               */
	SKP_int32           prd_pair1[],            /* O    Accumalated error                   */
	SKP_int32           q_Q10_pair1[],          /* O    Quantized Q10 residual              */
	SKP_int32           sLTP_pair1_Q16[],       /* I/O  LTP filter state                    */
	SKP_int             lag_pair1,              /* I    Pitch lag                           */
	SKP_int             *smpl_buf_pair1_idx,    /* I/O  Index to newest samples in buffers  */
	SKP_Silk_nsq_state  *NSQ_pair2,             /* I/O  NSQ state                           */
	NSQ_del_dec_struct  psDelDec_pair2[],       /* I/O  Delayed decision states             */
	SKP_int8            q_pair2[],              /* O    Quantized Q0 residual               */
	SKP_int16           xq_pair2[],             /* O    Simulated full signal               */
	SKP_int32           prd_pair2[],            /* O    Accumalated error                   */
	SKP_int32           q_Q10_pair2[],          /* O    Quantized Q10 residual              */
	SKP_int32           sLTP_pair2_Q16[],       /* I/O  LTP filter state                    */
	SKP_int             lag_pair2,              /* I    Pitch lag                           */
	SKP_int             *smpl_buf_pair2_idx,    /* I/O  Index to newest samples in buffers  */
	SKP_int             sigtype,                /* I    Signal type                         */
	const SKP_int32     x_Q10[],                /* I    Input signal                        */
	const SKP_int32     x_md_Q10[],             /* I    Input md signal                     */
	const SKP_int16     a_Q12[],                /* I    Short term prediction coefs         */
	const SKP_int16     b_Q14[],                /* I    Long term prediction coefs          */
	const SKP_int16     AR_shp_Q13[],           /* I    Noise shaping coefs                 */
	SKP_int32           HarmShapeFIRPacked_Q14, /* I    Subframe smoothing coefficient      */
	SKP_int             Tilt_Q14,               /* I    Spectral tilt                       */
	SKP_int32           LF_shp_Q14,             /* I    Long-term shaping factor            */
	SKP_int32           Gain_Q16,               /* I    Gain                                */
	const SKP_int       DeltaGains_Q16,         /* I    Delta gain allocated for md         */
	SKP_int             Lambda_Q10,             /* I    Quantization coefficient            */
	SKP_int             offset_Q10,             /* I    Default offset                      */
	SKP_int             length,                 /* I    Input length                        */
	SKP_int             subfr,                  /* I    Subframe number                     */
	SKP_int             shapingLPCOrder,        /* I    Shaping LPC filter order            */
	SKP_int             predictLPCOrder,        /* I    Prediction filter order             */
	SKP_int             warping_Q16,            /* I    Parameter for warped noise shaping  */
	SKP_int             nStatesDelayedDecision, /* I    Number of states in decision tree   */
	SKP_int             decisionDelay           /* I    Length of delayed decision          */
);


void Agora_Silk_Init_DelDecState(
    SKP_Silk_nsq_state  *NSQ,                   /* I/O  NSQ state                           */
    NSQ_del_dec_struct  psDelDec[],             /* I/O  Delayed decision states             */
    SKP_int32   		Seed,                   /* I    Seed for dither                     */
    SKP_int             frame_length,           /* I    Frame length                        */
    SKP_int             nStatesDelayedDecision  /* I    Num of delayed decision state       */
	)
{
	SKP_int k;
    NSQ_del_dec_struct *psDD;
    /* Initialize delayed decision states */
    SKP_memset( psDelDec, 0, nStatesDelayedDecision * sizeof( NSQ_del_dec_struct ) );
    for( k = 0; k < nStatesDelayedDecision; k++ ) {
        psDD                 = &psDelDec[ k ];
        psDD->Seed           = ( k + Seed ) & 3;
        psDD->SeedInit       = psDD->Seed;
        psDD->Seed2          = psDD->Seed;
        psDD->SeedInit2      = psDD->Seed;
        psDD->RD_Q10         = 0;
        psDD->LF_AR_Q12      = NSQ->sLF_AR_shp_Q12;
        psDD->Shape_Q10[ 0 ] = NSQ->sLTP_shp_Q10[ frame_length - 1 ];
        SKP_memcpy( psDD->sLPC_Q14, NSQ->sLPC_Q14, NSQ_LPC_BUF_LENGTH * sizeof( SKP_int32 ) );
        SKP_memcpy( psDD->sAR2_Q14, NSQ->sAR2_Q14, sizeof( NSQ->sAR2_Q14 ) );
        psDD->RD_md_Q10[0]         = 0;
        psDD->RD_md_Q10[1]         = 0;
        psDD->RD_md_Q10[2]         = 0;
        psDD->RD_md_Q10[3]         = 0;
    }
}

SKP_int32 Agora_Silk_DelDec_UpdateState_And_Output(
	SKP_Silk_nsq_state	*NSQ,
    NSQ_del_dec_struct  psDelDec[],             /* I/O  Delayed decision states             */
	SKP_int             frame_length,           /* I    Frame length                        */
    SKP_int8            q[],                    /* O    Quantized Q0 residual               */
    SKP_int32           r[],                    /* O    Unquantized residual                */
    SKP_int16           pxq[],                  /* O    Simulated full signal               */
    SKP_int32           prd[],                  /* O    Accumalated error                   */
    SKP_int32           pres_Q10[],             /* O    Quantized Q10 residual              */
    SKP_int             pitchL[ NB_SUBFR ],     /* O    Pitch for each subframe             */
    SKP_int             *sLTP_Q16,              /* O    Simulated output signal without LPC */
    SKP_int             nStatesDelayedDecision, /* I    Num of delayed decision state       */
	SKP_int             sigtype,                /* I    Signal type                         */
    SKP_int     		subfr_length,           /* I    Subframe length                     */
	SKP_int 			decisionDelay,          /* I    Length of delayed decision          */
	SKP_int             *pWinner_ind,           /* O    Best state                          */
	SKP_int 			smpl_buf_idx            /* I    Index to newest samples             */
	)
{
	    SKP_int32 i,k,Winner_ind,last_smple_idx;
        NSQ_del_dec_struct *psDD;
        SKP_int32   RDmin_Q10;
	    SKP_int32   Seed;
	
		/* Find winner */
		RDmin_Q10 = psDelDec[ 0 ].RD_Q10;
		Winner_ind = 0;
		for( k = 1; k < nStatesDelayedDecision; k++ ) {
			if( psDelDec[ k ].RD_Q10 < RDmin_Q10 ) {
				RDmin_Q10 = psDelDec[ k ].RD_Q10;
				Winner_ind = k;
			}
		}
	
		*pWinner_ind = Winner_ind;
		/* Copy final part of signals from winner state to output and long-term filter states */
		psDD = &psDelDec[ Winner_ind ];
		Seed = psDD->SeedInit2;
		last_smple_idx = smpl_buf_idx + decisionDelay;
		for( i = 0; i < decisionDelay; i++ ) {
			last_smple_idx = ( last_smple_idx - 1 ) & DECISION_DELAY_MASK;
			q[	 i - decisionDelay ] = ( SKP_int8 )( psDD->Q_Q0[ last_smple_idx ] );
			r[   i - decisionDelay ] = ( SKP_int32 )psDD->exc_Q10[ last_smple_idx ];
			pxq[ i - decisionDelay ] = ( SKP_int16 )SKP_SAT16( SKP_RSHIFT_ROUND( 
				SKP_SMULWW( psDD->Xq_Q10[ last_smple_idx ], psDD->Gain_Q16[ last_smple_idx ] ), 10 ) );
			pres_Q10[ i - decisionDelay ] = psDD->Q_Q10[ last_smple_idx ];
			NSQ->sLTP_shp_Q10[ NSQ->sLTP_shp_buf_idx - decisionDelay + i ] = psDD->Shape_Q10[ last_smple_idx ];
            sLTP_Q16[		   NSQ->sLTP_buf_idx	 - decisionDelay + i ] = psDD->Pred_Q16[  last_smple_idx ];
			prd[i - decisionDelay] = psDD->Rd_Q10[last_smple_idx];
		}
		SKP_memcpy( NSQ->sLPC_Q14, &psDD->sLPC_Q14[ subfr_length ], NSQ_LPC_BUF_LENGTH * sizeof( SKP_int32 ) );
		SKP_memcpy( NSQ->sAR2_Q14, psDD->sAR2_Q14, sizeof( psDD->sAR2_Q14 ) );
	
		/* Update states */
		NSQ->sLF_AR_shp_Q12 = psDD->LF_AR_Q12;
		NSQ->lagPrev		= pitchL[ NB_SUBFR - 1 ];
	
		/* Save quantized speech and noise shaping signals */
		SKP_memcpy( NSQ->xq,		   &NSQ->xq[		   frame_length ], frame_length * sizeof( SKP_int16 ) );
		SKP_memcpy( NSQ->sLTP_shp_Q10, &NSQ->sLTP_shp_Q10[ frame_length ], frame_length * sizeof( SKP_int32 ) );

		return Seed;
	
}

SKP_int32 Agora_Silk_DelDec_UpdateState_And_Output_Side(
	SKP_Silk_nsq_state	*NSQ,
    NSQ_del_dec_struct  psDelDec[],             /* I/O  Delayed decision states             */
	SKP_int             frame_length,           /* I    Frame length                        */
	SKP_int8            q[],                    /* O    Quantized Q0 residual               */
	SKP_int16           pxq[],                  /* O    Simulated full signal               */
	SKP_int32           prd[],                  /* O    Accumalated error                   */
	SKP_int32           pres_Q10[],             /* O    Quantized Q10 residual              */
	SKP_int             pitchL[NB_SUBFR],       /* O    Pitch for each subframe             */
	SKP_int             *sLTP_Q16,              /* O    Simulated output signal without LPC */
	SKP_int             nStatesDelayedDecision, /* I    Num of delayed decision state       */
	SKP_int             sigtype,                /* I    Signal type                         */
	SKP_int     		subfr_length,           /* I    Subframe length                     */
	SKP_int 			decisionDelay,          /* I    Length of delayed decision          */
	SKP_int             Center_Winner_ind,      /* I    The best state                      */
	SKP_int 			smpl_buf_idx            /* I    Index to newest samples             */
	)
{
	    SKP_int32 i,k,Winner_ind,last_smple_idx;
        NSQ_del_dec_struct *psDD;
        SKP_int32   RDmin_Q10;
	    SKP_int32   Seed;
	
		/* Find winner */
		RDmin_Q10 = psDelDec[ 0 ].RD_Q10;
		Winner_ind = 0;
		for( k = 1; k < nStatesDelayedDecision; k++ ) {
			if( psDelDec[ k ].RD_Q10 < RDmin_Q10 ) {
				RDmin_Q10 = psDelDec[ k ].RD_Q10;
				Winner_ind = k;
			}
		}
		
		Winner_ind = Center_Winner_ind;
		/* Copy final part of signals from winner state to output and long-term filter states */
		psDD = &psDelDec[ Winner_ind ];
		Seed = psDD->SeedInit2;
		last_smple_idx = smpl_buf_idx + decisionDelay;
		for( i = 0; i < decisionDelay; i++ ) {
			last_smple_idx = ( last_smple_idx - 1 ) & DECISION_DELAY_MASK;
			q[	 i - decisionDelay ] = ( SKP_int8 )( psDD->Q_Q0[ last_smple_idx ] );
			pxq[ i - decisionDelay ] = ( SKP_int16 )SKP_SAT16( SKP_RSHIFT_ROUND( 
				SKP_SMULWW( psDD->Xq_Q10[ last_smple_idx ], psDD->Gain_Q16[ last_smple_idx ] ), 10 ) );
			pres_Q10[ i - decisionDelay ] = psDD->Q_Q10[ last_smple_idx ];
			NSQ->sLTP_shp_Q10[ NSQ->sLTP_shp_buf_idx - decisionDelay + i ] = psDD->Shape_Q10[ last_smple_idx ];
            sLTP_Q16[		   NSQ->sLTP_buf_idx	 - decisionDelay + i ] = psDD->Pred_Q16[  last_smple_idx ];
			prd[i - decisionDelay] = psDD->Rd_Q10[last_smple_idx];
		}
		SKP_memcpy( NSQ->sLPC_Q14, &psDD->sLPC_Q14[ subfr_length ], NSQ_LPC_BUF_LENGTH * sizeof( SKP_int32 ) );
		SKP_memcpy( NSQ->sAR2_Q14, psDD->sAR2_Q14, sizeof( psDD->sAR2_Q14 ) );
	
		/* Update states */
		NSQ->sLF_AR_shp_Q12 = psDD->LF_AR_Q12;
		NSQ->lagPrev		= pitchL[ NB_SUBFR - 1 ];
	
		/* Save quantized speech and noise shaping signals */
		SKP_memcpy( NSQ->xq,		   &NSQ->xq[		   frame_length ], frame_length * sizeof( SKP_int16 ) );
		SKP_memcpy( NSQ->sLTP_shp_Q10, &NSQ->sLTP_shp_Q10[ frame_length ], frame_length * sizeof( SKP_int32 ) );

		return Seed;
	
}


/*************************************/
/*************Rewhiten****************/
/*************************************/
SKP_int32 Agora_Silk_DelDec_Rewhitening(
    SKP_Silk_encoder_state          *psEncC,           /* I    Encode state                */
    SKP_Silk_encoder_control        *psEncCtrlC,       /* I    Encoder Control             */
	SKP_Silk_nsq_state	*NSQ,					       /* I/O  NSQ state				   */
	NSQ_del_dec_struct	psDelDec[], 			       /* I/O  Delayed decision states 	   */
	SKP_int 			k,                             /* I    Subframe num                */
	SKP_int 			LSF_interpolation_flag,        /* I    Interpolate flag            */
    SKP_int16           *sLTP,                         /* O    Rewhiten output             */
	const SKP_int16     *A_Q12,                        /* O    MA prediction coefficients  */
    SKP_int8            q[],                           /* O    Quantized Q0 residual       */
    SKP_int32           r[],                           /* O    Unquantized residual        */
	SKP_int16           pxq[],                         /* O    Simulated full signal       */
	SKP_int32           prd[],                         /* O    Accumalated error           */
	SKP_int32           pres_Q10[],                    /* O    Quantized Q10 residual      */
	SKP_int 			decisionDelay,                 /* I    Length of delayed decision  */
	SKP_int 			lag,                           /* I    Lag                         */
	SKP_int32   		FiltState[],                   /* I/O  State vector                */
	SKP_int 			smpl_buf_idx                   /* I    Index to newest samples     */

)
{
	SKP_int32 i,Winner_ind = 0,last_smple_idx,start_idx;
    NSQ_del_dec_struct *psDD;
    SKP_int32   RDmin_Q10;

	NSQ->rewhite_flag = 0;
	if( psEncCtrlC->sigtype == SIG_TYPE_VOICED ) {
		/* Voiced */
		/* Re-whitening */
		if( ( k & ( 3 - SKP_LSHIFT( LSF_interpolation_flag, 1 ) ) ) == 0 ) {
			if( k == 2 ) {
				/* RESET DELAYED DECISIONS */
				/* Find winner */
				RDmin_Q10 = psDelDec[ 0 ].RD_Q10;
				Winner_ind = 0;
				for( i = 1; i < psEncC->nStatesDelayedDecision; i++ ) {
					if( psDelDec[ i ].RD_Q10 < RDmin_Q10 ) {
						RDmin_Q10 = psDelDec[ i ].RD_Q10;
						Winner_ind = i;
					}
				}
		
				for( i = 0; i < psEncC->nStatesDelayedDecision; i++ ) {
					if( i != Winner_ind ) {
						psDelDec[ i ].RD_Q10 += ( SKP_int32_MAX >> 4 );
						SKP_assert( psDelDec[ i ].RD_Q10 >= 0 );
					}
				}
			
				/* Copy final part of signals from winner state to output and long-term filter states */
				psDD = &psDelDec[ Winner_ind ];
				last_smple_idx = smpl_buf_idx + decisionDelay;
				for( i = 0; i < decisionDelay; i++ ) {
					last_smple_idx = ( last_smple_idx - 1 ) & DECISION_DELAY_MASK;
					q[	 i - decisionDelay ] = ( SKP_int8 )( psDD->Q_Q0[ last_smple_idx ] );
					r[   i - decisionDelay ] = ( SKP_int32 )psDD->exc_Q10[ last_smple_idx ];
					pxq[ i - decisionDelay ] = ( SKP_int16 )SKP_SAT16( SKP_RSHIFT_ROUND( 
						SKP_SMULWW( psDD->Xq_Q10[ last_smple_idx ], 
						psDD->Gain_Q16[ last_smple_idx ] ), 10 ) );
                    pres_Q10[ i - decisionDelay ] = psDD->Q_Q10[ last_smple_idx ];
					NSQ->sLTP_shp_Q10[ NSQ->sLTP_shp_buf_idx - decisionDelay + i ] = psDD->Shape_Q10[ last_smple_idx ];
					prd[i - decisionDelay] = psDD->Rd_Q10[last_smple_idx];

				}

			}

			/* Rewhiten with new A coefs */
			start_idx = psEncC->frame_length - lag - psEncC->predictLPCOrder - LTP_ORDER / 2;
			SKP_assert( start_idx >= 0 );
			SKP_assert( start_idx <= psEncC->frame_length - psEncC->predictLPCOrder );

			SKP_memset( FiltState, 0, psEncC->predictLPCOrder * sizeof( SKP_int32 ) );
			SKP_Silk_MA_Prediction( &NSQ->xq[ start_idx + k * psEncC->subfr_length ], 
				A_Q12, FiltState, sLTP + start_idx, psEncC->frame_length - start_idx, psEncC->predictLPCOrder );

			NSQ->sLTP_buf_idx = psEncC->frame_length;
			NSQ->rewhite_flag = 1;
		}
	}

	return Winner_ind;
}

/*************/
/*Rewhiten MD*/
/*************/
SKP_int32 Agora_Silk_DelDec_Rewhitening_Side(
	SKP_Silk_encoder_state          *psEncC,           /* I    Encode state                */
	SKP_Silk_encoder_control        *psEncCtrlC,       /* I    Encoder Control             */
	SKP_Silk_nsq_state	*NSQ,					       /* I/O  NSQ state				   */
	NSQ_del_dec_struct	psDelDec[], 			       /* I/O  Delayed decision states 	   */
	SKP_int 			k,                             /* I    Subframe num                */
	SKP_int 			LSF_interpolation_flag,        /* I    Interpolate flag            */
    SKP_int16           *sLTP,                         /* O    Rewhiten output             */
	const SKP_int16     *A_Q12,                        /* O    MA prediction coefficients  */
    SKP_int8            q[],                           /* O    Quantized Q0 residual       */
	SKP_int16           pxq[],                         /* O    Simulated full signal       */
	SKP_int32           prd[],                         /* O    Accumalated error           */
	SKP_int32           pres_Q10[],                    /* O    Quantized Q10 residual      */
	SKP_int 			decisionDelay,                 /* I    Length of delayed decision  */
	SKP_int 			lag,                           /* I    Lag                         */
	SKP_int32   		FiltState[],                   /* I/O  State vector                */
	SKP_int32   		Center_Winner_ind,             /* I    Best state                  */
	SKP_int 			smpl_buf_idx                   /* I    Index to newest samples     */
)	
{
	SKP_int32 i,Winner_ind = 0,last_smple_idx,start_idx;
    NSQ_del_dec_struct *psDD;
    SKP_int32   RDmin_Q10;

        NSQ->rewhite_flag = 0;
	if( psEncCtrlC->sigtype == SIG_TYPE_VOICED ) {
		/* Voiced */
		/* Re-whitening */
		if( ( k & ( 3 - SKP_LSHIFT( LSF_interpolation_flag, 1 ) ) ) == 0 ) {
			if( k == 2 ) {
				/* RESET DELAYED DECISIONS */
				/* Find winner */
				RDmin_Q10 = psDelDec[ 0 ].RD_Q10;
				Winner_ind = 0;
				for( i = 1; i < psEncC->nStatesDelayedDecision; i++ ) {
					if( psDelDec[ i ].RD_Q10 < RDmin_Q10 ) {
						RDmin_Q10 = psDelDec[ i ].RD_Q10;
						Winner_ind = i;
					}
				}
				Winner_ind = Center_Winner_ind;
	
				for( i = 0; i < psEncC->nStatesDelayedDecision; i++ ) {
					if( i != Winner_ind ) {
						psDelDec[ i ].RD_Q10 += ( SKP_int32_MAX >> 4 );
						SKP_assert( psDelDec[ i ].RD_Q10 >= 0 );
					}
				}
			
				/* Copy final part of signals from winner state to output and long-term filter states */
				psDD = &psDelDec[ Winner_ind ];
				last_smple_idx = smpl_buf_idx + decisionDelay;
				for( i = 0; i < decisionDelay; i++ ) {
					last_smple_idx = ( last_smple_idx - 1 ) & DECISION_DELAY_MASK;
					q[	 i - decisionDelay ] = ( SKP_int8 )( psDD->Q_Q0[ last_smple_idx ] );
					pxq[ i - decisionDelay ] = ( SKP_int16 )SKP_SAT16( SKP_RSHIFT_ROUND( 
						SKP_SMULWW( psDD->Xq_Q10[ last_smple_idx ], 
						psDD->Gain_Q16[ last_smple_idx ] ), 10 ) );
					
					pres_Q10[ i - decisionDelay ] = psDD->Q_Q10[ last_smple_idx ];

                    NSQ->sLTP_shp_Q10[ NSQ->sLTP_shp_buf_idx - decisionDelay + i ] = psDD->Shape_Q10[ last_smple_idx ];
					prd[i - decisionDelay] = psDD->Rd_Q10[last_smple_idx];

				}

			}

			/* Rewhiten with new A coefs */
			start_idx = psEncC->frame_length - lag - psEncC->predictLPCOrder - LTP_ORDER / 2;
			SKP_assert( start_idx >= 0 );
			SKP_assert( start_idx <= psEncC->frame_length - psEncC->predictLPCOrder );

			SKP_memset( FiltState, 0, psEncC->predictLPCOrder * sizeof( SKP_int32 ) );
			SKP_Silk_MA_Prediction( &NSQ->xq[ start_idx + k * psEncC->subfr_length ], 
				A_Q12, FiltState, sLTP + start_idx, psEncC->frame_length - start_idx, psEncC->predictLPCOrder );

			NSQ->sLTP_buf_idx = psEncC->frame_length;
			NSQ->rewhite_flag = 1;
		}
	}

	return Winner_ind;
}

/******************************/
/* Simulate process of decode */
/******************************/
SKP_INLINE void Agora_Silk_UndoPred_And_Shap(
    NSQ_sample_struct  *psSS,
	SKP_int32 LTP_pred_Q14,
	SKP_int32 LPC_pred_Q10,
	SKP_int32 n_LTP_Q14,
	SKP_int32 n_AR_Q10,
	SKP_int32 n_LF_Q10
	)
{
	SKP_int32 xq_Q10,LPC_exc_Q10;
	SKP_int32 sLF_AR_shp_Q10;

    /* Update states for best quantization */

    /* Add predictions */
    LPC_exc_Q10 = psSS[ 0 ].Q_Q10 + SKP_RSHIFT_ROUND( LTP_pred_Q14, 4 );
    xq_Q10      = SKP_ADD32( LPC_exc_Q10, LPC_pred_Q10 );

    /* Update states */
    sLF_AR_shp_Q10         = SKP_SUB32(  xq_Q10, n_AR_Q10 );
    psSS[ 0 ].sLTP_shp_Q10 = SKP_SUB32(  sLF_AR_shp_Q10, n_LF_Q10 );
    psSS[ 0 ].LF_AR_Q12    = SKP_LSHIFT( sLF_AR_shp_Q10, 2 );
    psSS[ 0 ].xq_Q14       = SKP_LSHIFT( xq_Q10,         4 );
    psSS[ 0 ].LPC_exc_Q16  = SKP_LSHIFT( LPC_exc_Q10,    6 );

    /* Update states for second best quantization */

    /* Add predictions */
    LPC_exc_Q10 = psSS[ 1 ].Q_Q10 + SKP_RSHIFT_ROUND( LTP_pred_Q14, 4 );
    xq_Q10      = SKP_ADD32( LPC_exc_Q10, LPC_pred_Q10 );

    /* Update states */
    sLF_AR_shp_Q10         = SKP_SUB32(  xq_Q10, n_AR_Q10 );
    psSS[ 1 ].sLTP_shp_Q10 = SKP_SUB32(  sLF_AR_shp_Q10, n_LF_Q10 );
    psSS[ 1 ].LF_AR_Q12    = SKP_LSHIFT( sLF_AR_shp_Q10, 2 );
    psSS[ 1 ].xq_Q14       = SKP_LSHIFT( xq_Q10,         4 );
    psSS[ 1 ].LPC_exc_Q16  = SKP_LSHIFT( LPC_exc_Q10,    6 );		
	
	return ;
}


/******************************/
/*** Add dither to residual ***/
/******************************/
SKP_int32 Agora_Silk_Dither(
    NSQ_del_dec_struct *psDD,
	SKP_int32   		r_Q10
	)
{
	SKP_int32   r_temp_Q10;
    SKP_int32 	dither;
    /* Generate dither */
    psDD->Seed2 = SKP_RAND( psDD->Seed2 );
    psDD->Seed = SKP_RAND( psDD->Seed );
    dither = SKP_RSHIFT( psDD->Seed2, 31 );
	r_temp_Q10 = r_Q10;
	/* Flip sign depending on dither */
	r_Q10 = ( r_Q10 ^ dither ) - dither;

	return r_Q10;
}


/***************/
/*Remove dither*/
/***************/
void Agora_Silk_UnDither(
    NSQ_del_dec_struct *psDD,
    NSQ_sample_struct  *psSS
	)
{
    SKP_int32 	dither;
	
    dither = SKP_RSHIFT( psDD->Seed2, 31 );

    psSS[ 0 ].Q_Q10 = ( psSS[ 0 ].Q_Q10 ^ dither ) - dither;
    psSS[ 1 ].Q_Q10 = ( psSS[ 1 ].Q_Q10 ^ dither ) - dither;
	psSS[ 0 ].exc_Q10	   = psSS[ 0 ].Q_Q10;	
	psSS[ 1 ].exc_Q10	   = psSS[ 1 ].Q_Q10;
}


/***************************************/
/*Choose best two states from side info*/
/***************************************/
SKP_INLINE void Agora_Silk_RDCx1(
    NSQ_del_dec_struct *psDD,
    NSQ_sample_struct  *psSS,
	SKP_int32   		r_Q10,                 /* I Residual with gain                */
	SKP_int32   		r_p_Q10,               /* I Residual without gain             */
    SKP_int32           DeltaGains_Q16,        /* I Delta gain allocated for md       */
    SKP_int             Lambda_Q10,            /* I Divided coefficient               */
    SKP_int             offset_Q10             /* I Default offset                    */
	) 
{
	SKP_int32   r_temp2_Q10, r_temp_Q10;
	SKP_int32   q1_Q10, q2_Q10,rd1_Q10,rd2_Q10;
    SKP_int32   inv_gain_Q16;
    inv_gain_Q16 = SKP_INVERSE32_varQ( SKP_max( DeltaGains_Q16, 1 ), 32 );
    inv_gain_Q16 = SKP_min( inv_gain_Q16, SKP_int32_MAX );
	r_p_Q10 = SKP_SMULWW( inv_gain_Q16, r_p_Q10);

	r_temp_Q10 = r_Q10;
	/* Flip sign depending on dither */
	r_Q10 = SKP_SUB32( r_Q10, offset_Q10 );
	r_p_Q10 = SKP_SUB32( r_p_Q10, offset_Q10 );
	r_Q10 = SKP_LIMIT_32( r_Q10, -(64 << 10), 64 << 10 );

	/* Find two quantization level candidates and measure their rate-distortion */
	if( r_Q10 < -1536 ) {
		r_temp2_Q10 = r_Q10;
		q1_Q10	= SKP_LSHIFT( SKP_RSHIFT_ROUND( r_Q10, 10 ), 10 );

		r_Q10	= SKP_SUB32( r_p_Q10, q1_Q10);
		rd1_Q10 = SKP_RSHIFT( SKP_SMLABB( SKP_MUL( -SKP_ADD32( q1_Q10, offset_Q10 ), Lambda_Q10 ), r_Q10, r_Q10 ), 10 );
	
		r_Q10 = r_temp2_Q10;
		q2_Q10	= SKP_ADD32( SKP_LSHIFT( SKP_RSHIFT_ROUND( r_Q10, 10 ), 10 ), 1024 );

		r_Q10	= SKP_SUB32( r_p_Q10, q2_Q10);
		rd2_Q10 = SKP_RSHIFT( SKP_SMLABB( SKP_MUL( -SKP_ADD32( q2_Q10, offset_Q10 ), Lambda_Q10 ), r_Q10, r_Q10 ), 10 );
	} else if( r_Q10 > 512 ) {
		r_temp2_Q10 = r_Q10;
		q1_Q10	= SKP_LSHIFT( SKP_RSHIFT_ROUND( r_Q10, 10 ), 10 );
		
		r_Q10	= SKP_SUB32( r_p_Q10, q1_Q10 );
		rd1_Q10 = SKP_RSHIFT( SKP_SMLABB( SKP_MUL( SKP_ADD32( q1_Q10, offset_Q10 ), Lambda_Q10 ), r_Q10, r_Q10 ), 10 );
		
		r_Q10 = r_temp2_Q10;
		q2_Q10	= SKP_SUB32( SKP_LSHIFT( SKP_RSHIFT_ROUND( r_Q10, 10 ), 10 ), 1024 );
		
		r_Q10	= SKP_SUB32( r_p_Q10, q2_Q10 );
		rd2_Q10 = SKP_RSHIFT( SKP_SMLABB( SKP_MUL( SKP_ADD32( q2_Q10, offset_Q10 ), Lambda_Q10 ), r_Q10, r_Q10 ), 10 );
	} else {			/* r_Q10 >= -1536 && q1_Q10 <= 512 */
		r_temp2_Q10 = r_Q10;  
		q2_Q10	= 0;
		r_Q10	= SKP_SUB32( r_p_Q10, q2_Q10);
		rd2_Q10 = SKP_RSHIFT( SKP_SMLABB( SKP_MUL( SKP_ADD32( q2_Q10, offset_Q10 ), Lambda_Q10 ), r_Q10, r_Q10 ), 10 );
	
		r_Q10 = r_temp2_Q10;
		q1_Q10	= -1024;
		r_Q10	= SKP_SUB32( r_p_Q10, q1_Q10 );
		rd1_Q10 = SKP_RSHIFT( SKP_SMLABB( SKP_MUL( -SKP_ADD32( q1_Q10, offset_Q10 ), Lambda_Q10 ), r_Q10, r_Q10 ), 10 );
	}
	
	if( rd1_Q10 < rd2_Q10 ) {
		psSS[ 0 ].RD_Q10 = SKP_ADD32( psDD->RD_Q10, rd1_Q10 ); 
		psSS[ 1 ].RD_Q10 = SKP_ADD32( psDD->RD_Q10, rd2_Q10 );
		psSS[ 0 ].Q_Q0 = ( SKP_int8 )SKP_RSHIFT( q1_Q10, 10 );
		psSS[ 1 ].Q_Q0 = ( SKP_int8 )SKP_RSHIFT( q2_Q10, 10 );
		psSS[ 0 ].Q_Q10 = q1_Q10;
		psSS[ 1 ].Q_Q10 = q2_Q10;
		psSS[ 0 ].X_Q10 = r_temp_Q10;
        psSS[ 1 ].X_Q10 = r_temp_Q10;
        psSS[ 0 ].Rd_independent_Q10 = rd1_Q10;
        psSS[ 1 ].Rd_independent_Q10 = rd2_Q10;
	} else {
		psSS[ 0 ].RD_Q10 = SKP_ADD32( psDD->RD_Q10, rd2_Q10 );
		psSS[ 1 ].RD_Q10 = SKP_ADD32( psDD->RD_Q10, rd1_Q10 );
		psSS[ 0 ].Q_Q0 = ( SKP_int8 )SKP_RSHIFT( q2_Q10, 10 );
		psSS[ 1 ].Q_Q0 = ( SKP_int8 )SKP_RSHIFT( q1_Q10, 10 );
		psSS[ 0 ].Q_Q10 = q2_Q10;
		psSS[ 1 ].Q_Q10 = q1_Q10;
		psSS[ 0 ].X_Q10 = r_temp_Q10;
        psSS[ 1 ].X_Q10 = r_temp_Q10;
        psSS[ 0 ].Rd_independent_Q10 = rd2_Q10;
        psSS[ 1 ].Rd_independent_Q10 = rd1_Q10;
	}
	
    /* Quantized excitation */
    psSS[ 0 ].Q_Q10 = SKP_ADD32( offset_Q10, psSS[ 0 ].Q_Q10 );
	
    /* Quantized excitation */
    psSS[ 1 ].Q_Q10 = SKP_ADD32( offset_Q10, psSS[ 1 ].Q_Q10 );
	
}


/*************************************/
/* Judge which state is the best one */
/*************************************/
SKP_INLINE SKP_int Agora_Silk_JudgeWinner(
NSQ_del_dec_struct  psDelDec[],             /* I/O  Delayed decision states             */
NSQ_sample_struct   psSampleState[ MAX_DEL_DEC_STATES ][ 2 ],
NSQ_del_dec_struct  psDelDec_p1[],             /* I/O  Delayed decision states             */
NSQ_sample_struct   psSampleState_p1[ MAX_DEL_DEC_STATES ][ 2 ],
NSQ_del_dec_struct  psDelDec_p2[],             /* I/O  Delayed decision states             */
NSQ_sample_struct   psSampleState_p2[ MAX_DEL_DEC_STATES ][ 2 ],
SKP_int				i,
SKP_int             last_smple_idx,          /* I    Index to newest samples in buffers  */
SKP_int  			nStatesDelayedDecision
)
{
    SKP_int k,RDmin_Q10,Winner_ind,Winner_rand_state,RDmax_Q10,RDmax_ind,RDmin_ind;
    SKP_int RandSyncCtl = 0;
    SKP_int Winner_rand_state1,Winner_rand_state2;
    SKP_int RDmin_joint_Q10;
    
    
    /* Find winner */
	RDmin_Q10 = SKP_ADD32(SKP_ADD32(psSampleState[0][0].RD_Q10, SKP_SMULWW(psSampleState_p1[0][0].RD_Q10, INTERNAL_JOINT_LAMBDA)),
		SKP_SMULWW(psSampleState_p2[0][0].RD_Q10, INTERNAL_JOINT_LAMBDA));
    Winner_ind = 0;
    for( k = 1; k < nStatesDelayedDecision; k++ ) {
		RDmin_joint_Q10 = SKP_ADD32(SKP_ADD32(psSampleState[k][0].RD_Q10, SKP_SMULWW(psSampleState_p1[k][0].RD_Q10, INTERNAL_JOINT_LAMBDA)),
			SKP_SMULWW(psSampleState_p2[k][0].RD_Q10, INTERNAL_JOINT_LAMBDA));
        if( RDmin_joint_Q10 < RDmin_Q10 ) {
            RDmin_Q10	= RDmin_joint_Q10;
            Winner_ind = k;
        }
    }

    /* Increase RD values of expired states */
    Winner_rand_state = psDelDec[ Winner_ind ].RandState[ last_smple_idx ];
    Winner_rand_state1 = psDelDec_p1[ Winner_ind ].RandState[ last_smple_idx ];
    Winner_rand_state2 = psDelDec_p2[ Winner_ind ].RandState[ last_smple_idx ];
    for( k = 0; k < nStatesDelayedDecision; k++ ) {
        if(( psDelDec[ k ].RandState[ last_smple_idx ] != Winner_rand_state )
            || ( psDelDec_p1[ k ].RandState[ last_smple_idx ] != Winner_rand_state1 )
            || ( psDelDec_p2[ k ].RandState[ last_smple_idx ] != Winner_rand_state2 )){
            RandSyncCtl++;
            psSampleState[ k ][ 0 ].RD_Q10 = SKP_ADD32( psSampleState[ k ][ 0 ].RD_Q10, ( SKP_int32_MAX >> 4 ) );
            psSampleState[ k ][ 1 ].RD_Q10 = SKP_ADD32( psSampleState[ k ][ 1 ].RD_Q10, ( SKP_int32_MAX >> 4 ) );
            if(psSampleState[ k ][ 0 ].RD_Q10 <0)
                SKP_assert( psSampleState[ k ][ 0 ].RD_Q10 >= 0 );
        }
    }
    
    
    do{
        /* Find worst in first set and best in second set */
        RDmax_Q10  = psSampleState[ 0 ][ 0 ].RD_Q10;
        RDmin_Q10  = psSampleState[ 0 ][ 1 ].RD_Q10;
        RDmax_ind = 0;
        RDmin_ind = 0;
        for( k = 1; k < nStatesDelayedDecision; k++ ) {
            /* find worst in first set */
            if( psSampleState[ k ][ 0 ].RD_Q10 > RDmax_Q10 ) {
                RDmax_Q10  = psSampleState[ k ][ 0 ].RD_Q10;
                RDmax_ind = k;
            }
            /* find best in second set */
            if( psSampleState[ k ][ 1 ].RD_Q10 < RDmin_Q10 ) {
                RDmin_Q10  = psSampleState[ k ][ 1 ].RD_Q10;
                RDmin_ind = k;
            }
        }
        
        /* Replace a state if best from second set outperforms worst in first set */
        if( RDmin_Q10 < RDmax_Q10 ) {
            SKP_Silk_copy_del_dec_state( &psDelDec[ RDmax_ind ], &psDelDec[ RDmin_ind ], i );
            SKP_memcpy( &psSampleState[ RDmax_ind ][ 0 ], &psSampleState[ RDmin_ind ][ 1 ], sizeof( NSQ_sample_struct ) );
            SKP_Silk_copy_del_dec_state( &psDelDec_p1[ RDmax_ind ], &psDelDec_p1[ RDmin_ind ], i );
            SKP_memcpy( &psSampleState_p1[ RDmax_ind ][ 0 ], &psSampleState_p1[ RDmin_ind ][ 1 ], sizeof( NSQ_sample_struct ) );
            SKP_Silk_copy_del_dec_state( &psDelDec_p2[ RDmax_ind ], &psDelDec_p2[ RDmin_ind ], i );
            SKP_memcpy( &psSampleState_p2[ RDmax_ind ][ 0 ], &psSampleState_p2[ RDmin_ind ][ 1 ], sizeof( NSQ_sample_struct ) );
        }
    }while(--RandSyncCtl > 0);

    return Winner_ind;
}



/******************************************/
/* Output the best state from Center state*/
/******************************************/
SKP_INLINE SKP_int Agora_Silk_GetWinner(
    SKP_Silk_nsq_state  *NSQ,                           /* I/O  NSQ state                           */
    NSQ_del_dec_struct  psDelDec[],                     /* I/O  Delayed decision states             */
    NSQ_sample_struct   psSampleState[ MAX_DEL_DEC_STATES ][ 2 ],
    NSQ_del_dec_struct  psDelDec_p1[],                  /* I/O  Delayed decision states             */
    NSQ_sample_struct   psSampleState_p1[ MAX_DEL_DEC_STATES ][ 2 ],
    NSQ_del_dec_struct  psDelDec_p2[],                  /* I/O  Delayed decision states             */
    NSQ_sample_struct   psSampleState_p2[ MAX_DEL_DEC_STATES ][ 2 ],
    SKP_int8            q[],                            /* O    Quantized Q0 residual               */
    SKP_int32           r[],                            /* O    Unquantized residual                */
    SKP_int16           xq[],                           /* O    Simulated full signal               */
    SKP_int32           prd[],                          /* O    Accumalated error                   */
    SKP_int32           pres_Q10[],                     /* O    Quantized Q10 residual              */
    SKP_int32           sLTP_Q16[],                     /* I/O  LTP filter state                    */
    SKP_int				i,                              /* I    Frame point                         */
    SKP_int             subfr,                          /* I    Subframe number                     */
    SKP_int             sigtype,                        /* I    Signal type                         */
    SKP_int             last_smple_idx,                 /* I    Index to newest samples in buffers  */
    SKP_int  			nStatesDelayedDecision,         /* I    Number of states of delayed decision*/
    SKP_int             decisionDelay                   /* I    Length of delayed decision          */

)
{
    SKP_int k,RDmin_Q10,Winner_ind;
    SKP_int RDmin_joint_Q10;
    
    NSQ_del_dec_struct *psDD;

    /* Find winner */
	RDmin_Q10 = SKP_ADD32(SKP_ADD32(psSampleState[0][0].RD_Q10, SKP_SMULWW(psSampleState_p1[0][0].RD_Q10, INTERNAL_JOINT_LAMBDA)), SKP_SMULWW(psSampleState_p2[0][0].RD_Q10, INTERNAL_JOINT_LAMBDA));
    
    Winner_ind = 0;
    for( k = 1; k < nStatesDelayedDecision; k++ ) {
		RDmin_joint_Q10 = SKP_ADD32(SKP_ADD32(psSampleState[k][0].RD_Q10, SKP_SMULWW(psSampleState_p1[k][0].RD_Q10, INTERNAL_JOINT_LAMBDA)), SKP_SMULWW(psSampleState_p2[k][0].RD_Q10, INTERNAL_JOINT_LAMBDA));
        if( RDmin_joint_Q10 < RDmin_Q10 ) {
            RDmin_Q10	= RDmin_joint_Q10;
            Winner_ind = k;
        }
    }

    /* Write samples from winner to output and long-term filter states */
    psDD = &psDelDec[ Winner_ind ];
    if( subfr > 0 || i >= decisionDelay ) {
        q[  i - decisionDelay ] = ( SKP_int8 )( psDD->Q_Q0[ last_smple_idx ] );
		r[  i - decisionDelay ] = ( SKP_int32 )( psDD->exc_Q10[ last_smple_idx ] );
        xq[ i - decisionDelay ] = ( SKP_int16 )SKP_SAT16( SKP_RSHIFT_ROUND(
        SKP_SMULWW( psDD->Xq_Q10[ last_smple_idx ], psDD->Gain_Q16[ last_smple_idx ] ), 10 ) );
        pres_Q10[ i - decisionDelay ] = psDD->Q_Q10[ last_smple_idx ];
        NSQ->sLTP_shp_Q10[ NSQ->sLTP_shp_buf_idx - decisionDelay ] = psDD->Shape_Q10[ last_smple_idx ];
        sLTP_Q16[          NSQ->sLTP_buf_idx     - decisionDelay ] = psDD->Pred_Q16[  last_smple_idx ];
        prd[ i - decisionDelay ] = psDD->Rd_Q10[ last_smple_idx ];
    }
    
    NSQ->sLTP_shp_buf_idx++;
    NSQ->sLTP_buf_idx++;
    return Winner_ind;
}

/******************************************/
/* Output the best state from Side state*/
/******************************************/
SKP_INLINE void Agora_Silk_GetWinner_Side(
    SKP_Silk_nsq_state  *NSQ,                   /* I/O  NSQ state                           */
    NSQ_del_dec_struct  psDelDec[],             /* I/O  Delayed decision states             */
    NSQ_sample_struct   psSampleState[ MAX_DEL_DEC_STATES ][ 2 ],
    SKP_int             center_Winner_ind,      /* I    Winner of center*/
    SKP_int8            q[],                    /* O    Quantized Q0 residual               */
    SKP_int16           xq[],                   /* O    Simulated full signal               */
    SKP_int32           prd[],                  /* O    Accumalated error                   */
    SKP_int32           pres_Q10[],             /* O    Quantized Q10 residual              */
    SKP_int32           sLTP_Q16[],             /* I/O  LTP filter state                    */
    SKP_int				i,                      /* I    Frame point                         */
    SKP_int             subfr,                  /* I    Subframe number                     */
    SKP_int             sigtype,                /* I    Signal type                         */
    SKP_int             last_smple_idx,         /* I    Index to newest samples in buffers  */
    SKP_int  			nStatesDelayedDecision, /* I    Number of states of delayed decision*/
    SKP_int             decisionDelay           /* I    Length of delayed decision          */

)
{
   SKP_int Winner_ind;
   NSQ_del_dec_struct *psDD;
   Winner_ind = center_Winner_ind;

   /* Write samples from winner to output and long-term filter states */
   psDD = &psDelDec[ Winner_ind ];
   if( subfr > 0 || i >= decisionDelay ) {
          q[  i - decisionDelay ] = ( SKP_int8 )( psDD->Q_Q0[ last_smple_idx ] );
          xq[ i - decisionDelay ] = ( SKP_int16 )SKP_SAT16( SKP_RSHIFT_ROUND( 
          SKP_SMULWW( psDD->Xq_Q10[ last_smple_idx ], psDD->Gain_Q16[ last_smple_idx ] ), 10 ) );
          pres_Q10[ i - decisionDelay ] = psDD->Q_Q10[ last_smple_idx ];
          NSQ->sLTP_shp_Q10[ NSQ->sLTP_shp_buf_idx - decisionDelay ] = psDD->Shape_Q10[ last_smple_idx ];
          sLTP_Q16[          NSQ->sLTP_buf_idx     - decisionDelay ] = psDD->Pred_Q16[  last_smple_idx ];
		  prd[i - decisionDelay] = psDD->Rd_Q10[last_smple_idx];
   }
	
    NSQ->sLTP_shp_buf_idx++;
    NSQ->sLTP_buf_idx++;
	
}

/******************************************/
/* Update state and prepare for next frame*/
/******************************************/
SKP_INLINE void Agora_Silk_Update_DelDecState(
    NSQ_del_dec_struct  psDelDec[],             /* I/O  Delayed decision states             */
    NSQ_sample_struct   psSampleState[ MAX_DEL_DEC_STATES ][ 2 ],
    SKP_int             i,                      /* I    Frame point                         */
    SKP_int32           Gain_Q16,               /* I    Gain                                */
    SKP_int             *smpl_buf_idx,          /* I    Index to newest samples in buffers  */
    SKP_int  			nStatesDelayedDecision  /* I    Number of states of delayed decision*/
	)
{
        SKP_int32 k;
        NSQ_del_dec_struct *psDD;
		NSQ_sample_struct   *psSS;

        /* Update states */
        for( k = 0; k < nStatesDelayedDecision; k++ ) {
            psDD                                     = &psDelDec[ k ];
            psSS                                     = &psSampleState[ k ][ 0 ];
            psDD->LF_AR_Q12                          = psSS->LF_AR_Q12;
            psDD->sLPC_Q14[ NSQ_LPC_BUF_LENGTH + i ] = psSS->xq_Q14;
            psDD->Xq_Q10[    *smpl_buf_idx ]         = SKP_RSHIFT( psSS->xq_Q14, 4 );
            psDD->Xq_md_Q10[0][    *smpl_buf_idx ]   = SKP_RSHIFT( psSS->xq_md_Q14[0], 4 );
            psDD->Rd_Q10[    *smpl_buf_idx ]         = psSS->Rd_independent_Q10;
            psDD->Q_Q0[     *smpl_buf_idx ]          = psSS->Q_Q0;
            psDD->Q_Q10[     *smpl_buf_idx ]         = psSS->Q_Q10;
            psDD->X_Q10[     *smpl_buf_idx ]         = psSS->X_Q10;
            psDD->Q_md_Q10[0][     *smpl_buf_idx ]   = psSS->Q_md_Q10[0];
            psDD->Pred_Q16[  *smpl_buf_idx ]         = psSS->LPC_exc_Q16;
            psDD->Shape_Q10[ *smpl_buf_idx ]         = psSS->sLTP_shp_Q10;
			psDD->Seed								 = SKP_ADD32( psDD->Seed, psSS->Q_Q0);
            psDD->RandState[ *smpl_buf_idx ]         = psDD->Seed;
            psDD->RD_Q10                             = psSS->RD_Q10;
            psDD->Gain_Q16[  *smpl_buf_idx ]         = Gain_Q16;
			psDD->exc_Q10[  *smpl_buf_idx ]          = psSS->exc_Q10;
        }
	
}

/******************************************/
/************* Update LPC *****************/
/******************************************/
SKP_INLINE void Agora_Silk_Update_DelDecLPCState(
    NSQ_del_dec_struct  psDelDec[],             /* I/O  Delayed decision states             */
    SKP_int             length,                 /* I    Input length                        */
    SKP_int  			nStatesDelayedDecision
	)
{
        SKP_int32 k;
        NSQ_del_dec_struct *psDD;
		
		/* Update LPC states */
		for( k = 0; k < nStatesDelayedDecision; k++ ) {
			psDD = &psDelDec[ k ];
			SKP_memcpy( psDD->sLPC_Q14, &psDD->sLPC_Q14[ length ], NSQ_LPC_BUF_LENGTH * sizeof( SKP_int32 ) );
		}

	return ;
}

/*********************************************************************/
/* Noise shape quantizer for one subframe , special redesign for Solo*/
/*********************************************************************/

void SKP_Silk_NSQ_del_dec(
    SKP_Silk_encoder_state          *psEncC,                                    /* I/O  Encoder State                       */
    SKP_Silk_encoder_control        *psEncCtrlC,                                /* I    Encoder Control                     */
    SKP_Silk_nsq_state              *NSQ,                                       /* I/O  NSQ state                           */
    SKP_Silk_nsq_state              NSQ_md[MAX_INTERLEAVE_NUM],                 /* I/O  NSQ state                           */
    const SKP_int16                 x[],                                        /* I    Prefiltered input signal      */
    SKP_int8                        q[],                                        /* O    Quantized pulse signal              */
    SKP_int8                        *q_md[ MAX_INTERLEAVE_NUM ],                /* O   Quantized qulse signal              */
    SKP_int32                       r[],                                        /* O    Output residual signal */
    const SKP_int                   LSFInterpFactor_Q2,                         /* I    LSF interpolation factor in Q2 */
    const SKP_int16                 PredCoef_Q12[ 2 * MAX_LPC_ORDER ],          /* I    Prediction coefs                    */
    const SKP_int16                 LTPCoef_Q14[ LTP_ORDER * NB_SUBFR ],        /* I    LT prediction coefs                 */
    const SKP_int16                 AR2_Q13[ NB_SUBFR * MAX_SHAPE_LPC_ORDER ],  /* I    Noise shaping filter                                      */
    const SKP_int                   HarmShapeGain_Q14[ NB_SUBFR ],              /* I    Smooth coefficients                                    */
    const SKP_int                   Tilt_Q14[ NB_SUBFR ],                       /* I    Spectral tilt                       */
    const SKP_int32                 LF_shp_Q14[ NB_SUBFR ],                     /* I    Short-term shaping coefficients */
    const SKP_int32                 Gains_Q16[ NB_SUBFR ],                      /* I    Gain for each subframe                                  */
	const SKP_int32 				MDGains_Q16[ NB_SUBFR ],                    /* I    New gain, no use now
        */
	const SKP_int32			        DeltaGains_Q16,                             /* I    Gain for odd subframe */
    const SKP_int                   Lambda_Q10,                                 /* I    Quantization coefficient  */
    const SKP_int                   LTP_scale_Q14                               /* I    LTP state scaling                   */
)
{
    SKP_int     k, lag,  LSF_interpolation_flag, Winner_ind, subfr;
    SKP_int     smpl_buf_idx, decisionDelay, subfr_length;
    const SKP_int16 *A_Q12, *B_Q14, *AR_shp_Q13;
    SKP_int16   *pxq;
    SKP_int32   sLTP_Q16[ 2 * MAX_FRAME_LENGTH ];
    SKP_int16   sLTP[     2 * MAX_FRAME_LENGTH ];
    SKP_int32   HarmShapeFIRPacked_Q14;
    SKP_int     offset_Q10;
    SKP_int32   FiltState[ MAX_LPC_ORDER ];
    SKP_int32   *pres_Q10_center;
    SKP_int32   x_sc_Q10[ MAX_FRAME_LENGTH / NB_SUBFR ];
    SKP_int32   x_md_Q10[ MAX_FRAME_LENGTH / NB_SUBFR ];
    NSQ_del_dec_struct psDelDec[ MAX_DEL_DEC_STATES ];
    NSQ_del_dec_struct psDelDec_p1[ MAX_DEL_DEC_STATES ];
    NSQ_del_dec_struct psDelDec_p2[ MAX_DEL_DEC_STATES ];
    SKP_int16   *pxq_p1;
    SKP_int16   *pxq_p2;
    SKP_int32   *pres_Q10_p1;
    SKP_int32   *pres_Q10_p2;
	SKP_int     lag_p1;
	SKP_int     lag_p2;
	SKP_int8     *q_p1,*q_p2; 
    SKP_int32   FiltState_p1[ MAX_LPC_ORDER ];
    SKP_int32   sLTP_Q16_p1[ 2 * MAX_FRAME_LENGTH ];
    SKP_int16   sLTP_p1[     2 * MAX_FRAME_LENGTH ];
    SKP_int32   FiltState_p2[ MAX_LPC_ORDER ];
    SKP_int32   sLTP_Q16_p2[ 2 * MAX_FRAME_LENGTH ];
    SKP_int16   sLTP_p2[     2 * MAX_FRAME_LENGTH ];
	SKP_int     smpl_buf_p1_idx;
	SKP_int     smpl_buf_p2_idx;
    SKP_int32   *prd;
    SKP_int32   *prd_p1;
    SKP_int32   *prd_p2;
    SKP_int32   rd_total[ 2 * MAX_FRAME_LENGTH ];
    SKP_int32   rd_p1[ 2 * MAX_FRAME_LENGTH ];
    SKP_int32   rd_p2[ 2 * MAX_FRAME_LENGTH ];
    
	SKP_Silk_nsq_state              *NSQ_p1 = &(NSQ_md[0]);
	SKP_Silk_nsq_state              *NSQ_p2 = &(NSQ_md[1]);

    q_p1 = q_md[ 0 ];
    q_p2 = q_md[ 1 ];
    prd    = rd_total;
    prd_p1 = rd_p1;
    prd_p2 = rd_p2;
   
    subfr_length = psEncC->frame_length / NB_SUBFR;

    /* Set unvoiced lag to the previous one, overwrite later for voiced */
    lag = NSQ->lagPrev;
	lag_p1 = NSQ_p1->lagPrev;
	lag_p2 = NSQ_p2->lagPrev;
    SKP_assert( NSQ->prev_inv_gain_Q16 != 0 );

    offset_Q10   = SKP_Silk_Quantization_Offsets_Q10[ psEncCtrlC->sigtype ][ psEncCtrlC->QuantOffsetType ];
    smpl_buf_idx = 0; /* index of oldest samples */

    decisionDelay = SKP_min_int( DECISION_DELAY, subfr_length );

    /* For voiced frames limit the decision delay to lower than the pitch lag */
    if( psEncCtrlC->sigtype == SIG_TYPE_VOICED ) {
        for( k = 0; k < NB_SUBFR; k++ ) {
            decisionDelay = SKP_min_int( decisionDelay, psEncCtrlC->pitchL[ k ] - LTP_ORDER / 2 - 1 );
        }
    } else {
        if( lag > 0 ) {
            decisionDelay = SKP_min_int( decisionDelay, lag - LTP_ORDER / 2 - 1 );
        }
    }

    if( LSFInterpFactor_Q2 == ( 1 << 2 ) ) {
        LSF_interpolation_flag = 0;
    } else {
        LSF_interpolation_flag = 1;
    }
	

    smpl_buf_p1_idx = 0; /* index of oldest samples */
    smpl_buf_p2_idx = 0; /* index of oldest samples */


	Agora_Silk_Init_DelDecState(NSQ,psDelDec,psEncCtrlC->Seed,psEncC->frame_length,psEncC->nStatesDelayedDecision);
	Agora_Silk_Init_DelDecState(NSQ_p1,psDelDec_p1,psEncCtrlC->Seed_md[0],psEncC->frame_length,psEncC->nStatesDelayedDecision);
	Agora_Silk_Init_DelDecState(NSQ_p2,psDelDec_p2,psEncCtrlC->Seed_md[1],psEncC->frame_length,psEncC->nStatesDelayedDecision);

    /* Setup pointers to start of sub frame */
    pxq                   = &NSQ->xq[ psEncC->frame_length ];
    pres_Q10_center       = &NSQ->q_Q10[ psEncC->frame_length ];
    NSQ->sLTP_shp_buf_idx = psEncC->frame_length;
    NSQ->sLTP_buf_idx     = psEncC->frame_length;
	/*First md*/
	pxq_p1                   = &NSQ_p1->xq[ psEncC->frame_length ];
	pres_Q10_p1              = &NSQ_p1->q_Q10[ psEncC->frame_length ];
    NSQ_p1->sLTP_shp_buf_idx = psEncC->frame_length;
    NSQ_p1->sLTP_buf_idx     = psEncC->frame_length;
	/*Second md*/
	pxq_p2                = &NSQ_p2->xq[ psEncC->frame_length ];
	pres_Q10_p2           = &NSQ_p2->q_Q10[ psEncC->frame_length ];
    NSQ_p2->sLTP_shp_buf_idx = psEncC->frame_length;
    NSQ_p2->sLTP_buf_idx     = psEncC->frame_length;

    subfr = 0;
    for( k = 0; k < NB_SUBFR; k++ ) {
        A_Q12      = &PredCoef_Q12[ ( ( k >> 1 ) | ( 1 - LSF_interpolation_flag ) ) * MAX_LPC_ORDER ];
        B_Q14      = &LTPCoef_Q14[ k * LTP_ORDER           ];
        AR_shp_Q13 = &AR2_Q13[     k * MAX_SHAPE_LPC_ORDER ];

        /* Noise shape parameters */
        SKP_assert( HarmShapeGain_Q14[ k ] >= 0 );
        HarmShapeFIRPacked_Q14  =                          SKP_RSHIFT( HarmShapeGain_Q14[ k ], 2 );
        HarmShapeFIRPacked_Q14 |= SKP_LSHIFT( ( SKP_int32 )SKP_RSHIFT( HarmShapeGain_Q14[ k ], 1 ), 16 );

		if( psEncCtrlC->sigtype == SIG_TYPE_VOICED ) {
            lag = psEncCtrlC->pitchL[ k ];
			lag_p1 = lag;
			lag_p2 = lag;
	        if( ( k & ( 3 - SKP_LSHIFT( LSF_interpolation_flag, 1 ) ) ) == 0 ) {
	            if( k == 2 ) {
	                subfr = 0;
				}
	        }
		}
		
		Winner_ind = Agora_Silk_DelDec_Rewhitening(psEncC,psEncCtrlC,NSQ,psDelDec,k,LSF_interpolation_flag,sLTP,A_Q12,q,r, pxq,prd,pres_Q10_center,decisionDelay,lag,FiltState,smpl_buf_idx);
		Agora_Silk_DelDec_Rewhitening_Side(psEncC,psEncCtrlC,NSQ_p1,psDelDec_p1,k,LSF_interpolation_flag,sLTP_p1,A_Q12,q_p1,pxq_p1,prd_p1,pres_Q10_p1,decisionDelay,lag_p1,FiltState_p1,Winner_ind,smpl_buf_p1_idx);
		Agora_Silk_DelDec_Rewhitening_Side(psEncC,psEncCtrlC,NSQ_p2,psDelDec_p2,k,LSF_interpolation_flag,sLTP_p2,A_Q12,q_p2,pxq_p2,prd_p2,pres_Q10_p2,decisionDelay,lag_p2,FiltState_p2,Winner_ind,smpl_buf_p2_idx);

		Agora_Silk_DelDecScale(x, x_sc_Q10,subfr_length ,k,Gains_Q16);
		
        SKP_Silk_nsq_del_dec_scale_states( NSQ, psDelDec, 
            subfr_length, sLTP, sLTP_Q16, k, psEncC->nStatesDelayedDecision, smpl_buf_idx,
            LTP_scale_Q14, Gains_Q16, psEncCtrlC->pitchL );
		
		Agora_Silk_DelDecScale(x, x_md_Q10,subfr_length ,k,Gains_Q16);
        SKP_Silk_nsq_del_dec_scale_states( NSQ_p1, psDelDec_p1, 
            subfr_length, sLTP_p1, sLTP_Q16_p1, k, psEncC->nStatesDelayedDecision, smpl_buf_p1_idx,
            LTP_scale_Q14, Gains_Q16, psEncCtrlC->pitchL );

        SKP_Silk_nsq_del_dec_scale_states( NSQ_p2, psDelDec_p2, 
            subfr_length, sLTP_p2, sLTP_Q16_p2, k, psEncC->nStatesDelayedDecision, smpl_buf_p2_idx,
            LTP_scale_Q14, Gains_Q16, psEncCtrlC->pitchL );

		SKP_Silk_md_noise_shape_quantizer_del_dec( 
		    NSQ   , psDelDec   , q   , r, pxq   , prd,   pres_Q10_center,sLTP_Q16   , lag   , &smpl_buf_idx   ,
		    NSQ_p1, psDelDec_p1, q_p1, pxq_p1, prd_p1,pres_Q10_p1, sLTP_Q16_p1, lag_p1, &smpl_buf_p1_idx,
		    NSQ_p2, psDelDec_p2, q_p2, pxq_p2, prd_p2,pres_Q10_p2, sLTP_Q16_p2, lag_p2, &smpl_buf_p2_idx,
			psEncCtrlC->sigtype, x_sc_Q10, x_md_Q10, A_Q12, B_Q14, AR_shp_Q13,HarmShapeFIRPacked_Q14, Tilt_Q14[ k ], LF_shp_Q14[ k ], Gains_Q16[ k ],DeltaGains_Q16, 
			Lambda_Q10, offset_Q10, psEncC->subfr_length, subfr, psEncC->shapingLPCOrder, psEncC->predictLPCOrder, 
			psEncC->warping_Q16, psEncC->nStatesDelayedDecision, decisionDelay );

	
		subfr++;
        x   += psEncC->subfr_length;
        q   += psEncC->subfr_length;
        r   += psEncC->subfr_length;
        pxq += psEncC->subfr_length;
        prd += psEncC->subfr_length;
        pres_Q10_center += psEncC->subfr_length;
			
        q_p1   += psEncC->subfr_length;
        pxq_p1 += psEncC->subfr_length;
        pres_Q10_p1 += psEncC->subfr_length;
        prd_p1 += psEncC->subfr_length;
		
        q_p2   += psEncC->subfr_length;
        pxq_p2 += psEncC->subfr_length;
        pres_Q10_p2 += psEncC->subfr_length;
        prd_p2 += psEncC->subfr_length;
		
    }

   psEncCtrlC->Seed = Agora_Silk_DelDec_UpdateState_And_Output(
        NSQ,psDelDec,psEncC->frame_length,q ,r, pxq ,prd,
        pres_Q10_center,psEncCtrlC->pitchL ,sLTP_Q16,psEncC->nStatesDelayedDecision,
        psEncCtrlC->sigtype,psEncC->subfr_length ,decisionDelay,&Winner_ind,smpl_buf_idx);
   

   psEncCtrlC->Seed_md[0] = Agora_Silk_DelDec_UpdateState_And_Output_Side(
        NSQ_p1,psDelDec_p1,psEncC->frame_length,q_p1 ,pxq_p1 ,prd_p1,
        pres_Q10_p1,psEncCtrlC->pitchL ,sLTP_Q16_p1,psEncC->nStatesDelayedDecision,
        psEncCtrlC->sigtype,psEncC->subfr_length ,decisionDelay,Winner_ind,smpl_buf_p1_idx);

   psEncCtrlC->Seed_md[1] = Agora_Silk_DelDec_UpdateState_And_Output_Side(
        NSQ_p2,psDelDec_p2,psEncC->frame_length,q_p2 ,pxq_p2 ,prd_p2,
        pres_Q10_p2,psEncCtrlC->pitchL ,sLTP_Q16_p2,psEncC->nStatesDelayedDecision,
        psEncCtrlC->sigtype,psEncC->subfr_length ,decisionDelay,Winner_ind,smpl_buf_p2_idx);
 
}


typedef struct {
    SKP_int32 LTP_pred_Q14;
    SKP_int32 n_LTP_Q14;
    SKP_int32 LPC_pred_Q10[MAX_DEL_DEC_STATES];
    SKP_int32 n_AR_Q10[MAX_DEL_DEC_STATES];
    SKP_int32 n_LF_Q10[MAX_DEL_DEC_STATES];
    SKP_int32 r_Q10[MAX_DEL_DEC_STATES];
} NSQ_state;


/*********************************************/
/*Select best state from several combinations*/
/*********************************************/
SKP_int32 Agora_Silk_CenterRD(
    NSQ_del_dec_struct  psDelDec,              /* I/O  Delayed decision states        */
    NSQ_sample_struct   psSampleState[ 2 ],    /* I State for each decision tree      */
    NSQ_sample_struct   psSampleState_p1[ 2 ], /* I State for each decision tree      */
    NSQ_sample_struct   psSampleState_p2[ 2 ], /* I State for each decision tree      */
    SKP_int32           res_Q10,               /* I Residual Q10                      */
    SKP_int             Lambda_Q10,            /* I Divided coefficient               */
    SKP_int             offset_Q10             /* I Default offset                    */
)
{
    SKP_int32 s, r_temp_Q10,r_Q10,rdx_min_Q10;
    SKP_int32 qx_Q10[4], rdx_Q10[4];
    SKP_int32 q1_Q10,q2_Q10,q3_Q10,q4_Q10;
    SKP_int32 rd1_Q10,rd2_Q10,rd3_Q10,rd4_Q10;
    SKP_int32 winnner_rdx_ind1,winnner_rdx_ind2;
    NSQ_sample_struct  swapSampleSate;
    
    qx_Q10[0] = q1_Q10 = (psSampleState_p1[ 0 ].Q_Q10  +  psSampleState_p2[ 0 ].Q_Q10);
    qx_Q10[1] = q2_Q10 = (psSampleState_p1[ 1 ].Q_Q10  +  psSampleState_p2[ 1 ].Q_Q10);
    
    qx_Q10[2] = q3_Q10 = (psSampleState_p1[ 0 ].Q_Q10  +  psSampleState_p2[ 1 ].Q_Q10);
    qx_Q10[3] = q4_Q10 = (psSampleState_p1[ 1 ].Q_Q10  +  psSampleState_p2[ 0 ].Q_Q10);
    
    Lambda_Q10 += LARS_LAMBDA_AGR;
    r_temp_Q10 = res_Q10;
	r_temp_Q10 = SKP_SUB32( r_temp_Q10, offset_Q10 );
	
    if(q1_Q10 < 0){
        r_Q10 = r_temp_Q10;
        r_Q10 = SKP_SUB32( r_Q10, q1_Q10 );
        rdx_Q10[0] = rd1_Q10 = SKP_RSHIFT( SKP_SMLABB( SKP_MUL( -SKP_ADD32( q1_Q10, offset_Q10 ), Lambda_Q10 ), r_Q10, r_Q10 ), 10 );
    } else{
        r_Q10 = r_temp_Q10;
        r_Q10 = SKP_SUB32( r_Q10, q1_Q10 );
        rdx_Q10[0] = rd1_Q10 = SKP_RSHIFT( SKP_SMLABB( SKP_MUL(  SKP_ADD32( q1_Q10, offset_Q10 ), Lambda_Q10 ), r_Q10, r_Q10 ), 10 );
    }
    
    if(q2_Q10 < 0){
        r_Q10 = r_temp_Q10;
        r_Q10 = SKP_SUB32( r_Q10, q2_Q10 );
        rdx_Q10[1] = rd2_Q10 = SKP_RSHIFT( SKP_SMLABB( SKP_MUL( -SKP_ADD32( q2_Q10, offset_Q10 ), Lambda_Q10 ), r_Q10, r_Q10 ), 10 );
    }else{
        r_Q10 = r_temp_Q10;
        r_Q10 = SKP_SUB32( r_Q10, q2_Q10 );
        rdx_Q10[1] = rd2_Q10 = SKP_RSHIFT( SKP_SMLABB( SKP_MUL(  SKP_ADD32( q2_Q10, offset_Q10 ), Lambda_Q10 ), r_Q10, r_Q10 ), 10 );
    }
    
    if(q3_Q10 < 0){
        r_Q10 = r_temp_Q10;
        r_Q10 = SKP_SUB32( r_Q10, q3_Q10 );
        rdx_Q10[2] = rd3_Q10 = SKP_RSHIFT( SKP_SMLABB( SKP_MUL( -SKP_ADD32( q3_Q10, offset_Q10 ), Lambda_Q10 ), r_Q10, r_Q10 ), 10 );
    } else{
        r_Q10 = r_temp_Q10;
        r_Q10 = SKP_SUB32( r_Q10, q3_Q10 );
        rdx_Q10[2] = rd3_Q10 = SKP_RSHIFT( SKP_SMLABB( SKP_MUL(  SKP_ADD32( q3_Q10, offset_Q10 ), Lambda_Q10 ), r_Q10, r_Q10 ), 10 );
    }
    
    if(q4_Q10 < 0){
        r_Q10 = r_temp_Q10;
        r_Q10 = SKP_SUB32( r_Q10, q4_Q10 );
        rdx_Q10[3] = rd4_Q10 = SKP_RSHIFT( SKP_SMLABB( SKP_MUL( -SKP_ADD32( q4_Q10, offset_Q10 ), Lambda_Q10 ), r_Q10, r_Q10 ), 10 );
    }else{
        r_Q10 = r_temp_Q10;
        r_Q10 = SKP_SUB32( r_Q10, q4_Q10 );
        rdx_Q10[3] = rd4_Q10 = SKP_RSHIFT( SKP_SMLABB( SKP_MUL(  SKP_ADD32( q4_Q10, offset_Q10 ), Lambda_Q10 ), r_Q10, r_Q10 ), 10 );
    }
    
    Lambda_Q10 -= LARS_LAMBDA_AGR;
    
    rdx_Q10[0] = SKP_ADD32(SKP_ADD32(rdx_Q10[0], SKP_SMULWW(INTERNAL_JOINT_LAMBDA,psSampleState_p1[ 0 ].Rd_independent_Q10)),SKP_SMULWW(INTERNAL_JOINT_LAMBDA,psSampleState_p2[ 0 ].Rd_independent_Q10));
    rdx_Q10[1] = SKP_ADD32(SKP_ADD32(rdx_Q10[1], SKP_SMULWW(INTERNAL_JOINT_LAMBDA,psSampleState_p1[ 1 ].Rd_independent_Q10)),SKP_SMULWW(INTERNAL_JOINT_LAMBDA,psSampleState_p2[ 1 ].Rd_independent_Q10));
    rdx_Q10[2] = SKP_ADD32(SKP_ADD32(rdx_Q10[2], SKP_SMULWW(INTERNAL_JOINT_LAMBDA,psSampleState_p1[ 0 ].Rd_independent_Q10)),SKP_SMULWW(INTERNAL_JOINT_LAMBDA,psSampleState_p2[ 1 ].Rd_independent_Q10));
    rdx_Q10[3] = SKP_ADD32(SKP_ADD32(rdx_Q10[3], SKP_SMULWW(INTERNAL_JOINT_LAMBDA,psSampleState_p1[ 1 ].Rd_independent_Q10)),SKP_SMULWW(INTERNAL_JOINT_LAMBDA,psSampleState_p2[ 0 ].Rd_independent_Q10));
    
    /* Find winner */
    rdx_min_Q10 = rdx_Q10[0];
    winnner_rdx_ind1 = 0;
    for( s = 1; s < 4; s++ ) {
        if( rdx_Q10[ s ] < rdx_min_Q10 ) {
            rdx_min_Q10   = rdx_Q10[ s ];
            winnner_rdx_ind1 = s;
        }
    }
    
    if(winnner_rdx_ind1 == 0){
        rdx_min_Q10 = rdx_Q10[1];
        winnner_rdx_ind2 = 1;
        for( s = 2; s < 4 ; s++ ) {
            if( rdx_Q10[ s ] < rdx_min_Q10 ) {
                rdx_min_Q10   = rdx_Q10[ s ];
                winnner_rdx_ind2 = s;
            }
        }
    }else {
        rdx_min_Q10 = rdx_Q10[0];
        winnner_rdx_ind2 = 0;
        for( s = 1; s < 4  ; s++ ) {
            if(( rdx_Q10[ s ] < rdx_min_Q10 ) && (s != winnner_rdx_ind1)) {
                rdx_min_Q10   = rdx_Q10[ s ];
                winnner_rdx_ind2 = s;
            }
        }
    }
    
    psSampleState[ 0 ].RD_Q10 = SKP_ADD32( psDelDec.RD_Q10, rdx_Q10[winnner_rdx_ind1] );
    psSampleState[ 1 ].RD_Q10 = SKP_ADD32( psDelDec.RD_Q10, rdx_Q10[winnner_rdx_ind2] );
    psSampleState[ 0 ].Q_Q0 = SKP_RSHIFT(qx_Q10[winnner_rdx_ind1],10);
    psSampleState[ 1 ].Q_Q0 = SKP_RSHIFT(qx_Q10[winnner_rdx_ind2],10);
    psSampleState[ 0 ].Q_Q10 = qx_Q10[winnner_rdx_ind1];
    psSampleState[ 1 ].Q_Q10 = qx_Q10[winnner_rdx_ind2];
    psSampleState[ 0 ].Rd_independent_Q10 = rdx_Q10[winnner_rdx_ind1];
    psSampleState[ 1 ].Rd_independent_Q10 = rdx_Q10[winnner_rdx_ind2];
    
    
    if((winnner_rdx_ind1 == 0)&&(winnner_rdx_ind2 == 1)) {
		/*Do not need any process*/
    }else if((winnner_rdx_ind1 == 0)&&(winnner_rdx_ind2 == 2)) {

        SKP_memcpy(&psSampleState_p1[ 1 ], &psSampleState_p1[ 0 ], sizeof(NSQ_sample_struct));
    } else if((winnner_rdx_ind1 == 0)&&(winnner_rdx_ind2 == 3)){

        SKP_memcpy(&psSampleState_p2[ 1 ], &psSampleState_p2[ 0 ], sizeof(NSQ_sample_struct));
    }else if((winnner_rdx_ind1 == 1)&&(winnner_rdx_ind2 == 0)){

        SKP_memcpy(&swapSampleSate            , &psSampleState_p1[ 0 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p1[ 0 ], &psSampleState_p1[ 1 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p1[ 1 ],             &swapSampleSate, sizeof(NSQ_sample_struct));
        SKP_memcpy(&swapSampleSate            , &psSampleState_p2[ 0 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p2[ 0 ], &psSampleState_p2[ 1 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p2[ 1 ],             &swapSampleSate, sizeof(NSQ_sample_struct));
    }else if((winnner_rdx_ind1 == 1)&&(winnner_rdx_ind2 == 2)){

        SKP_memcpy(&swapSampleSate            , &psSampleState_p1[ 0 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p1[ 0 ], &psSampleState_p1[ 1 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p1[ 1 ],             &swapSampleSate, sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p2[ 0 ], &psSampleState_p2[ 1 ], sizeof(NSQ_sample_struct));
    }else if((winnner_rdx_ind1 == 1)&&(winnner_rdx_ind2 == 3)){

        SKP_memcpy(&psSampleState_p1[ 0 ], &psSampleState_p1[ 1 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&swapSampleSate            , &psSampleState_p2[ 0 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p2[ 0 ], &psSampleState_p2[ 1 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p2[ 1 ],             &swapSampleSate, sizeof(NSQ_sample_struct));
    }else if((winnner_rdx_ind1 == 2)&&(winnner_rdx_ind2 == 0)){

        SKP_memcpy(&psSampleState_p1[ 1 ], &psSampleState_p1[ 0 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&swapSampleSate       , &psSampleState_p2[ 0 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p2[ 0 ], &psSampleState_p2[ 1 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p2[ 1 ],        &swapSampleSate, sizeof(NSQ_sample_struct));
    }else if((winnner_rdx_ind1 == 2)&&(winnner_rdx_ind2 == 1)){

        SKP_memcpy(&psSampleState_p2[ 0 ], &psSampleState_p2[ 1 ], sizeof(NSQ_sample_struct));
    } else if((winnner_rdx_ind1 == 2)&&(winnner_rdx_ind2 == 3)){

        SKP_memcpy(&swapSampleSate            , &psSampleState_p2[ 0 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p2[ 0 ], &psSampleState_p2[ 1 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p2[ 1 ],             &swapSampleSate, sizeof(NSQ_sample_struct));
    }else if((winnner_rdx_ind1 == 3)&&(winnner_rdx_ind2 == 0)){

        SKP_memcpy(&swapSampleSate            , &psSampleState_p1[ 0 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p1[ 0 ], &psSampleState_p1[ 1 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p1[ 1 ],             &swapSampleSate, sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p2[ 1 ], &psSampleState_p2[ 0 ], sizeof(NSQ_sample_struct));
    }else if((winnner_rdx_ind1 == 3)&&(winnner_rdx_ind2 == 1)){

        SKP_memcpy(&psSampleState_p1[ 0 ], &psSampleState_p1[ 1 ], sizeof(NSQ_sample_struct));
    }else if((winnner_rdx_ind1 == 3)&&(winnner_rdx_ind2 == 2)){

        SKP_memcpy(&swapSampleSate            , &psSampleState_p1[ 0 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p1[ 0 ], &psSampleState_p1[ 1 ], sizeof(NSQ_sample_struct));
        SKP_memcpy(&psSampleState_p1[ 1 ],             &swapSampleSate, sizeof(NSQ_sample_struct));
    }
    
    return 0;
}


SKP_INLINE void SKP_Silk_md_noise_shape_quantizer_del_dec(
    SKP_Silk_nsq_state  *NSQ,                   /* I/O  NSQ state                           */
    NSQ_del_dec_struct  psDelDec[],             /* I/O  Delayed decision states             */
    SKP_int8            q[],                    /* O    Quantized Q0 residual               */
    SKP_int32           r[],                    /* O    Unquantized residual                */
    SKP_int16           xq[],                   /* O    Simulated full signal               */
    SKP_int32           prd[],                  /* O    Accumalated error                   */
    SKP_int32           ob_q_Q10[],             /* O    Quantized Q10 residual              */
    SKP_int32           sLTP_Q16[],             /* I/O  LTP filter state                    */
    SKP_int             lag,                    /* I    Pitch lag                           */
    SKP_int             *smpl_buf_idx,          /* I/O  Index to newest samples in buffers  */
    SKP_Silk_nsq_state  *NSQ_pair1,             /* I/O  NSQ state                           */
    NSQ_del_dec_struct  psDelDec_pair1[],       /* I/O  Delayed decision states             */
    SKP_int8            q_pair1[],              /* O    Quantized Q0 residual               */
    SKP_int16           xq_pair1[],             /* O    Simulated full signal               */
    SKP_int32           prd_pair1[],            /* O    Accumalated error                   */
    SKP_int32           q_Q10_pair1[],          /* O    Quantized Q10 residual              */
    SKP_int32           sLTP_pair1_Q16[],       /* I/O  LTP filter state                    */
    SKP_int             lag_pair1,              /* I    Pitch lag                           */
    SKP_int             *smpl_buf_pair1_idx,    /* I/O  Index to newest samples in buffers  */
    SKP_Silk_nsq_state  *NSQ_pair2,             /* I/O  NSQ state                           */
    NSQ_del_dec_struct  psDelDec_pair2[],       /* I/O  Delayed decision states             */
    SKP_int8            q_pair2[],              /* O    Quantized Q0 residual               */
    SKP_int16           xq_pair2[],             /* O    Simulated full signal               */
    SKP_int32           prd_pair2[],            /* O    Accumalated error                   */
    SKP_int32           q_Q10_pair2[],          /* O    Quantized Q10 residual              */
    SKP_int32           sLTP_pair2_Q16[],       /* I/O  LTP filter state                    */
    SKP_int             lag_pair2,              /* I    Pitch lag                           */
    SKP_int             *smpl_buf_pair2_idx,    /* I/O  Index to newest samples in buffers  */
    SKP_int             sigtype,                /* I    Signal type                         */
    const SKP_int32     x_Q10[],                /* I    Input signal                        */
    const SKP_int32     x_md_Q10[],             /* I    Input md signal                     */
    const SKP_int16     a_Q12[],                /* I    Short term prediction coefs         */
    const SKP_int16     b_Q14[],                /* I    Long term prediction coefs          */
    const SKP_int16     AR_shp_Q13[],           /* I    Noise shaping coefs                 */
    SKP_int32           HarmShapeFIRPacked_Q14, /* I    Subframe smoothing coefficient      */
    SKP_int             Tilt_Q14,               /* I    Spectral tilt                       */
    SKP_int32           LF_shp_Q14,             /* I    Long-term shaping factor            */
    SKP_int32           Gain_Q16,               /* I    Gain                                */
	const SKP_int       DeltaGains_Q16,         /* I    Delta gain allocated for md         */
    SKP_int             Lambda_Q10,             /* I    Quantization coefficient            */
    SKP_int             offset_Q10,             /* I    Default offset                      */
    SKP_int             length,                 /* I    Input length                        */
    SKP_int             subfr,                  /* I    Subframe number                     */
    SKP_int             shapingLPCOrder,        /* I    Shaping LPC filter order            */
    SKP_int             predictLPCOrder,        /* I    Prediction filter order             */
    SKP_int             warping_Q16,            /* I    Parameter for warped noise shaping  */
    SKP_int             nStatesDelayedDecision, /* I    Number of states in decision tree   */
    SKP_int             decisionDelay           /* I    Length of delayed decision          */
)
{
    SKP_int     i, k, Winner_ind, last_smple_idx;
    SKP_int32   *pred_lag_ptr, *shp_lag_ptr;
    NSQ_sample_struct  psSampleState[ MAX_DEL_DEC_STATES ][ 2 ];
    SKP_int32   inv_gain_Q16;
    SKP_int32   last_smple_p1_idx, last_smple_p2_idx;
	SKP_int32   r_md1_Q10,r_md2_Q10;
    SKP_int32   *pred_lag_p1_ptr, *shp_lag_p1_ptr;
    SKP_int32   *pred_lag_p2_ptr, *shp_lag_p2_ptr;
	SKP_int32    DeltaGains_p1_Q16, DeltaGains_p2_Q16;
	SKP_int32    inv_gain_p1_Q16, inv_gain_p2_Q16;
	SKP_int      offset_p1_Q10, offset_p2_Q10;
    SKP_int32    DeltaGains_md_Q16[ MAX_DEL_DEC_STATES ];
    SKP_int32    inv_gain_md_Q16[ MAX_DEL_DEC_STATES ];
    
    NSQ_state center_nsq_state,  md1_nsq_state, md2_nsq_state;
    NSQ_sample_struct  psSampleState_p1[ MAX_DEL_DEC_STATES ][ 2 ];
    NSQ_sample_struct  psSampleState_p2[ MAX_DEL_DEC_STATES ][ 2 ];

#ifdef DISABLE_OFFSET			
    offset_Q10 = 0;
#endif		
	
    inv_gain_Q16 = SKP_INVERSE32_varQ( SKP_max( DeltaGains_Q16, 1 ), 32 );
    inv_gain_Q16 = SKP_min( inv_gain_Q16, SKP_int32_MAX );
	
    inv_gain_p1_Q16 = inv_gain_Q16;
    inv_gain_p2_Q16 = (65536 - inv_gain_Q16);
    
    inv_gain_md_Q16[ 0 ] = inv_gain_p1_Q16 = SKP_min( inv_gain_p1_Q16, SKP_int32_MAX );
    inv_gain_md_Q16[ 1 ] = inv_gain_p2_Q16 = SKP_min( inv_gain_p2_Q16, SKP_int32_MAX );

    DeltaGains_p1_Q16 = SKP_INVERSE32_varQ( SKP_max( inv_gain_p1_Q16, 1 ), 32 );
    DeltaGains_p2_Q16 = SKP_INVERSE32_varQ( SKP_max( inv_gain_p2_Q16, 1 ), 32 );
    
    DeltaGains_md_Q16[ 0 ] = DeltaGains_p1_Q16 = SKP_min( DeltaGains_p1_Q16, SKP_int32_MAX );
    DeltaGains_md_Q16[ 1 ] = DeltaGains_p2_Q16 = SKP_min( DeltaGains_p2_Q16, SKP_int32_MAX );
#ifdef _OFFSET_MD_	
	offset_p1_Q10 = SKP_SMULWW( inv_gain_p1_Q16, offset_Q10);
	offset_p2_Q10 = SKP_SMULWW( inv_gain_p2_Q16, offset_Q10);
#else
	offset_p1_Q10 = offset_Q10;
	offset_p2_Q10 = offset_Q10;
#endif
    shp_lag_ptr  = &NSQ->sLTP_shp_Q10[ NSQ->sLTP_shp_buf_idx - lag + HARM_SHAPE_FIR_TAPS / 2 ];
    pred_lag_ptr = &sLTP_Q16[ NSQ->sLTP_buf_idx - lag + LTP_ORDER / 2 ];

    shp_lag_p1_ptr  = &NSQ_pair1->sLTP_shp_Q10[ NSQ_pair1->sLTP_shp_buf_idx - lag_pair1 + HARM_SHAPE_FIR_TAPS / 2 ];
    pred_lag_p1_ptr = &sLTP_pair1_Q16[ NSQ_pair1->sLTP_buf_idx - lag_pair1 + LTP_ORDER / 2 ];

    shp_lag_p2_ptr  = &NSQ_pair2->sLTP_shp_Q10[ NSQ_pair2->sLTP_shp_buf_idx - lag_pair2 + HARM_SHAPE_FIR_TAPS / 2 ];
    pred_lag_p2_ptr = &sLTP_pair2_Q16[ NSQ_pair2->sLTP_buf_idx - lag_pair2 + LTP_ORDER / 2 ];
	
    for( i = 0; i < length; i++ ) {
        /* Perform common calculations used in all states */

		/* Long-term prediction */
		center_nsq_state.LTP_pred_Q14 = Agora_Silk_LTP(sigtype,&pred_lag_ptr,b_Q14);
        /* Long-term shaping */
		center_nsq_state.n_LTP_Q14 = Agora_Silk_LTS(lag,&shp_lag_ptr,HarmShapeFIRPacked_Q14);

		/* Long-term prediction */
		md1_nsq_state.LTP_pred_Q14 = Agora_Silk_LTP(sigtype,&pred_lag_p1_ptr,b_Q14);
        /* Long-term shaping */
		md1_nsq_state.n_LTP_Q14 = Agora_Silk_LTS(lag,&shp_lag_p1_ptr,HarmShapeFIRPacked_Q14);

		/* Long-term prediction */
		md2_nsq_state.LTP_pred_Q14 = Agora_Silk_LTP(sigtype,&pred_lag_p2_ptr,b_Q14);
        /* Long-term shaping */
		md2_nsq_state.n_LTP_Q14 = Agora_Silk_LTS(lag,&shp_lag_p2_ptr,HarmShapeFIRPacked_Q14);

        for( k = 0; k < nStatesDelayedDecision; k++ ) {

            /* Short-term prediction */

			center_nsq_state.LPC_pred_Q10[k] = Agora_Silk_STP(&psDelDec[ k ].sLPC_Q14[ NSQ_LPC_BUF_LENGTH - 1 + i ],predictLPCOrder,a_Q12);

            /* Short-term shaping */
			
			center_nsq_state.n_AR_Q10[k] 	 = Agora_Silk_STS(&psDelDec[ k ].sLPC_Q14[ NSQ_LPC_BUF_LENGTH - 1 + i ],psDelDec[ k ].sAR2_Q14,psDelDec[ k ].LF_AR_Q12,shapingLPCOrder,warping_Q16,AR_shp_Q13,Tilt_Q14);
			
            /* Short-term prediction */

			md1_nsq_state.LPC_pred_Q10[k] = Agora_Silk_STP(&psDelDec_pair1[ k ].sLPC_Q14[ NSQ_LPC_BUF_LENGTH - 1 + i ],predictLPCOrder,a_Q12);


            /* Short-term shaping */
			
			md1_nsq_state.n_AR_Q10[k] 	 = Agora_Silk_STS(&psDelDec_pair1[ k ].sLPC_Q14[ NSQ_LPC_BUF_LENGTH - 1 + i ],psDelDec_pair1[ k ].sAR2_Q14,psDelDec_pair1[ k ].LF_AR_Q12 ,shapingLPCOrder,warping_Q16,AR_shp_Q13,Tilt_Q14);
			
            /* Short-term prediction */

			md2_nsq_state.LPC_pred_Q10[k] = Agora_Silk_STP(&psDelDec_pair2[ k ].sLPC_Q14[ NSQ_LPC_BUF_LENGTH - 1 + i ],predictLPCOrder,a_Q12);

			
            /* Short-term shaping */
			
			md2_nsq_state.n_AR_Q10[k] 	 = Agora_Silk_STS(&psDelDec_pair2[ k ].sLPC_Q14[ NSQ_LPC_BUF_LENGTH - 1 + i ],psDelDec_pair2[ k ].sAR2_Q14,psDelDec_pair2[ k ].LF_AR_Q12 ,shapingLPCOrder,warping_Q16,AR_shp_Q13,Tilt_Q14);



			center_nsq_state.n_LF_Q10[k] = Agora_Silk_LFS(psDelDec[ k ].Shape_Q10[ *smpl_buf_idx ]             , psDelDec[ k ].LF_AR_Q12      ,
                                                        center_nsq_state.n_AR_Q10[k]   ,LF_shp_Q14);
            
			md1_nsq_state.n_LF_Q10[k]    = Agora_Silk_LFS(psDelDec_pair1[ k ].Shape_Q10[ *smpl_buf_pair1_idx ] , psDelDec_pair1[ k ].LF_AR_Q12,
                                                        md1_nsq_state.n_AR_Q10[k],LF_shp_Q14);
            
			md2_nsq_state.n_LF_Q10[k]    = Agora_Silk_LFS(psDelDec_pair2[ k ].Shape_Q10[ *smpl_buf_pair2_idx ] , psDelDec_pair2[ k ].LF_AR_Q12,
                                                        md2_nsq_state.n_AR_Q10[k],LF_shp_Q14);
			
			center_nsq_state.r_Q10[k] = Agora_Silk_DoPred_And_Shap(x_Q10[i],center_nsq_state.LTP_pred_Q14,center_nsq_state.LPC_pred_Q10[k],center_nsq_state.n_LTP_Q14,center_nsq_state.n_AR_Q10[k],center_nsq_state.n_LF_Q10[k]);
            
			md1_nsq_state.r_Q10[k]    = Agora_Silk_DoPred_And_Shap(x_md_Q10[i],md1_nsq_state.LTP_pred_Q14,md1_nsq_state.LPC_pred_Q10[k],md1_nsq_state.n_LTP_Q14,md1_nsq_state.n_AR_Q10[k],md1_nsq_state.n_LF_Q10[k]);
            
			md2_nsq_state.r_Q10[k]    = Agora_Silk_DoPred_And_Shap(x_md_Q10[i],md2_nsq_state.LTP_pred_Q14,md2_nsq_state.LPC_pred_Q10[k],md2_nsq_state.n_LTP_Q14,md2_nsq_state.n_AR_Q10[k],md2_nsq_state.n_LF_Q10[k]);
			
			center_nsq_state.r_Q10[k] 	 = Agora_Silk_Dither(&psDelDec[ k ],center_nsq_state.r_Q10[k]);
		
			/*Different subframe has different gain allocation method*/
			if((subfr%2 < 1)){
                r_md1_Q10 = SKP_SMULWW( inv_gain_p1_Q16, center_nsq_state.r_Q10[k]);
                r_md2_Q10 = SKP_SMULWW( inv_gain_p2_Q16, center_nsq_state.r_Q10[k]);
               
        		md1_nsq_state.r_Q10[k] = Agora_Silk_Dither(&psDelDec_pair1[ k ],(md1_nsq_state.r_Q10[k]));
        		md2_nsq_state.r_Q10[k] = Agora_Silk_Dither(&psDelDec_pair2[ k ],(md2_nsq_state.r_Q10[k]));
                
                Agora_Silk_RDCx1(&psDelDec_pair1[ k ],psSampleState_p1[ k ],r_md1_Q10,md1_nsq_state.r_Q10[k],DeltaGains_p1_Q16,Lambda_Q10,offset_p1_Q10);
                Agora_Silk_RDCx1(&psDelDec_pair2[ k ],psSampleState_p2[ k ],r_md2_Q10,md2_nsq_state.r_Q10[k],DeltaGains_p2_Q16,Lambda_Q10,offset_p2_Q10);
                
                Agora_Silk_CenterRD(psDelDec[ k ],psSampleState[ k ],psSampleState_p1[ k ],psSampleState_p2[ k ],center_nsq_state.r_Q10[k],Lambda_Q10,(offset_p1_Q10 + offset_p2_Q10));

                Agora_Silk_UnDither(&psDelDec_pair1[ k ],psSampleState_p1[ k ]);
                Agora_Silk_UnDither(&psDelDec_pair2[ k ],psSampleState_p2[ k ]);
                psSampleState_p1[ k ][ 0 ].Q_Q10 = SKP_SMULWW( DeltaGains_p1_Q16, psSampleState_p1[ k ][ 0 ].Q_Q10);
                psSampleState_p1[ k ][ 1 ].Q_Q10 = SKP_SMULWW( DeltaGains_p1_Q16, psSampleState_p1[ k ][ 1 ].Q_Q10);
                psSampleState_p2[ k ][ 0 ].Q_Q10 = SKP_SMULWW( DeltaGains_p2_Q16, psSampleState_p2[ k ][ 0 ].Q_Q10);
                psSampleState_p2[ k ][ 1 ].Q_Q10 = SKP_SMULWW( DeltaGains_p2_Q16, psSampleState_p2[ k ][ 1 ].Q_Q10);
            }else{
                r_md1_Q10 = SKP_SMULWW( inv_gain_p2_Q16, center_nsq_state.r_Q10[k]);
                r_md2_Q10 = SKP_SMULWW( inv_gain_p1_Q16, center_nsq_state.r_Q10[k]);
                
        		md1_nsq_state.r_Q10[k] = Agora_Silk_Dither(&psDelDec_pair1[ k ],(md1_nsq_state.r_Q10[k]));
        		md2_nsq_state.r_Q10[k] = Agora_Silk_Dither(&psDelDec_pair2[ k ],(md2_nsq_state.r_Q10[k]));
                
                Agora_Silk_RDCx1(&psDelDec_pair1[ k ],psSampleState_p1[ k ],r_md1_Q10,md1_nsq_state.r_Q10[k],DeltaGains_p2_Q16,Lambda_Q10,offset_p2_Q10);
                Agora_Silk_RDCx1(&psDelDec_pair2[ k ],psSampleState_p2[ k ],r_md2_Q10,md2_nsq_state.r_Q10[k],DeltaGains_p1_Q16,Lambda_Q10,offset_p1_Q10);
                
                Agora_Silk_CenterRD(psDelDec[ k ],psSampleState[ k ],psSampleState_p1[ k ],psSampleState_p2[ k ],center_nsq_state.r_Q10[k],Lambda_Q10,(offset_p1_Q10 + offset_p2_Q10));

                Agora_Silk_UnDither(&psDelDec_pair1[ k ],psSampleState_p1[ k ]);
                Agora_Silk_UnDither(&psDelDec_pair2[ k ],psSampleState_p2[ k ]);

                psSampleState_p1[ k ][ 0 ].Q_Q10 = SKP_SMULWW( DeltaGains_p2_Q16, psSampleState_p1[ k ][ 0 ].Q_Q10);
                psSampleState_p1[ k ][ 1 ].Q_Q10 = SKP_SMULWW( DeltaGains_p2_Q16, psSampleState_p1[ k ][ 1 ].Q_Q10);
                psSampleState_p2[ k ][ 0 ].Q_Q10 = SKP_SMULWW( DeltaGains_p1_Q16, psSampleState_p2[ k ][ 0 ].Q_Q10);
                psSampleState_p2[ k ][ 1 ].Q_Q10 = SKP_SMULWW( DeltaGains_p1_Q16, psSampleState_p2[ k ][ 1 ].Q_Q10);
            }
            
			Agora_Silk_UnDither(&psDelDec[ k ],psSampleState[ k ]);
			Agora_Silk_UndoPred_And_Shap(psSampleState[ k ]   ,center_nsq_state.LTP_pred_Q14   ,center_nsq_state.LPC_pred_Q10[ k ]   ,center_nsq_state.n_LTP_Q14   ,center_nsq_state.n_AR_Q10[ k ]   ,center_nsq_state.n_LF_Q10[ k ]);
            
			Agora_Silk_UndoPred_And_Shap(psSampleState_p1[ k ],md1_nsq_state.LTP_pred_Q14      ,md1_nsq_state.LPC_pred_Q10[ k ]      ,md1_nsq_state.n_LTP_Q14   ,md1_nsq_state.n_AR_Q10[ k ]      ,md1_nsq_state.n_LF_Q10[ k ]);
            
			Agora_Silk_UndoPred_And_Shap(psSampleState_p2[ k ],md2_nsq_state.LTP_pred_Q14      ,md2_nsq_state.LPC_pred_Q10[ k ]       ,md2_nsq_state.n_LTP_Q14   ,md2_nsq_state.n_AR_Q10[ k ]      ,md2_nsq_state.n_LF_Q10[ k ]);
		
        }

        *smpl_buf_idx  = ( *smpl_buf_idx - 1 ) & DECISION_DELAY_MASK;                   /* Index to newest samples              */
        last_smple_idx = ( *smpl_buf_idx + decisionDelay ) & DECISION_DELAY_MASK;       /* Index to decisionDelay old samples   */
		
        *smpl_buf_pair1_idx  = ( *smpl_buf_pair1_idx - 1 ) & DECISION_DELAY_MASK;                   /* Index to newest samples              */
        last_smple_p1_idx = ( *smpl_buf_pair1_idx + decisionDelay ) & DECISION_DELAY_MASK;       /* Index to decisionDelay old samples   */
		
        *smpl_buf_pair2_idx  = ( *smpl_buf_pair2_idx - 1 ) & DECISION_DELAY_MASK;                   /* Index to newest samples              */
        last_smple_p2_idx = ( *smpl_buf_pair2_idx + decisionDelay ) & DECISION_DELAY_MASK;       /* Index to decisionDelay old samples   */

 		Agora_Silk_JudgeWinner(psDelDec,psSampleState,psDelDec_pair1,psSampleState_p1,psDelDec_pair2,psSampleState_p2,i,last_smple_idx   ,nStatesDelayedDecision);
        
		Winner_ind = Agora_Silk_GetWinner(NSQ, psDelDec, psSampleState, psDelDec_pair1, psSampleState_p1, psDelDec_pair2, psSampleState_p2, q, r, xq, prd, ob_q_Q10, sLTP_Q16, i, subfr, sigtype, last_smple_idx, nStatesDelayedDecision, decisionDelay);
        
        Agora_Silk_GetWinner_Side(NSQ_pair1,psDelDec_pair1,psSampleState_p1,Winner_ind,q_pair1,xq_pair1,prd_pair1,q_Q10_pair1,sLTP_pair1_Q16,i,subfr,sigtype,last_smple_p1_idx,nStatesDelayedDecision,decisionDelay);
        Agora_Silk_GetWinner_Side(NSQ_pair2,psDelDec_pair2,psSampleState_p2,Winner_ind,q_pair2,xq_pair2,prd_pair2,q_Q10_pair2,sLTP_pair2_Q16,i,subfr,sigtype,last_smple_p2_idx,nStatesDelayedDecision,decisionDelay);
        
		Agora_Silk_Update_DelDecState(psDelDec      ,psSampleState   ,i,Gain_Q16,smpl_buf_idx      ,nStatesDelayedDecision);
		Agora_Silk_Update_DelDecState(psDelDec_pair1,psSampleState_p1,i,Gain_Q16,smpl_buf_pair1_idx,nStatesDelayedDecision);
		Agora_Silk_Update_DelDecState(psDelDec_pair2,psSampleState_p2,i,Gain_Q16,smpl_buf_pair2_idx,nStatesDelayedDecision);
    }

    /* Update LPC states */
	Agora_Silk_Update_DelDecLPCState(psDelDec      ,length,nStatesDelayedDecision);
	Agora_Silk_Update_DelDecLPCState(psDelDec_pair1,length,nStatesDelayedDecision);
	Agora_Silk_Update_DelDecLPCState(psDelDec_pair2,length,nStatesDelayedDecision);
}

SKP_INLINE void SKP_Silk_nsq_del_dec_scale_states(
    SKP_Silk_nsq_state  *NSQ,                   /* I/O  NSQ state                           */
    NSQ_del_dec_struct  psDelDec[],             /* I/O  Delayed decision states             */
    SKP_int             subfr_length,           /* I    Length of input                     */
    const SKP_int16     sLTP[],                 /* I    Re-whitened LTP state in Q0         */
    SKP_int32           sLTP_Q16[],             /* O    LTP state matching scaled input     */
    SKP_int             subfr,                  /* I    Subframe number                     */
    SKP_int             nStatesDelayedDecision, /* I    Number of del dec states            */
    SKP_int             smpl_buf_idx,           /* I    Index to newest samples in buffers  */
    const SKP_int       LTP_scale_Q14,          /* I    LTP state scaling                   */
    const SKP_int32     Gains_Q16[ NB_SUBFR ],  /* I    Gain of each subframe               */
    const SKP_int       pitchL[ NB_SUBFR ]      /* I    Pitch lag                           */
)
{
    SKP_int            i, k, lag;
    SKP_int32          inv_gain_Q16, gain_adj_Q16, inv_gain_Q32;
    NSQ_del_dec_struct *psDD;

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

        for( k = 0; k < nStatesDelayedDecision; k++ ) {
            psDD = &psDelDec[ k ];
            
            /* Scale scalar states */
            psDD->LF_AR_Q12 = SKP_SMULWW( gain_adj_Q16, psDD->LF_AR_Q12 );
            
	        /* Scale short-term prediction and shaping states */
            for( i = 0; i < NSQ_LPC_BUF_LENGTH; i++ ) {
                psDD->sLPC_Q14[ i ] = SKP_SMULWW( gain_adj_Q16, psDD->sLPC_Q14[ i ] );
            }
            for( i = 0; i < MAX_SHAPE_LPC_ORDER; i++ ) {
                psDD->sAR2_Q14[ i ] = SKP_SMULWW( gain_adj_Q16, psDD->sAR2_Q14[ i ] );
            }
            for( i = 0; i < DECISION_DELAY; i++ ) {
                psDD->Pred_Q16[  i ] = SKP_SMULWW( gain_adj_Q16, psDD->Pred_Q16[  i ] );
                psDD->Shape_Q10[ i ] = SKP_SMULWW( gain_adj_Q16, psDD->Shape_Q10[ i ] );
            }
        }
    }


    /* save inv_gain */
    SKP_assert( inv_gain_Q16 != 0 );
    NSQ->prev_inv_gain_Q16 = inv_gain_Q16;
}

/****************************************/
/*Scale the energy level of input signal*/
/****************************************/
SKP_INLINE void  Agora_Silk_DelDecScale(
    const SKP_int16     x[],                    /* I    Input in Q0                         */
    SKP_int32           x_sc_Q10[],             /* O    Input scaled with 1/Gain in Q10     */
    SKP_int             subfr_length,           /* I    Length of input                     */
    SKP_int             subfr,                  /* I    Subframe number                     */
    const SKP_int32     Gains_Q16[ NB_SUBFR ]   /* I    Gain of each subframe               */
	)
{
	int i,inv_gain_Q16;
    inv_gain_Q16 = SKP_INVERSE32_varQ( SKP_max( Gains_Q16[ subfr ], 1 ), 32 );
    inv_gain_Q16 = SKP_min( inv_gain_Q16, SKP_int16_MAX );
	/* Scale input */
	for( i = 0; i < subfr_length; i++ ) {
		x_sc_Q10[ i ] = SKP_RSHIFT( SKP_SMULBB( x[ i ], ( SKP_int16 )inv_gain_Q16 ), 6 );
	}
}

SKP_INLINE void SKP_Silk_copy_del_dec_state(
    NSQ_del_dec_struct  *DD_dst,                /* I    Dst del dec state                   */
    NSQ_del_dec_struct  *DD_src,                /* I    Src del dec state                   */
    SKP_int             LPC_state_idx           /* I    Index to LPC buffer                 */
)
{
    SKP_memcpy( DD_dst->RandState, DD_src->RandState, sizeof( DD_src->RandState ) );
    SKP_memcpy( DD_dst->Q_Q10,     DD_src->Q_Q10,     sizeof( DD_src->Q_Q10     ) );
    SKP_memcpy( DD_dst->Q_Q0,      DD_src->Q_Q0,      sizeof( DD_src->Q_Q0     ) );
    SKP_memcpy( DD_dst->X_Q10,     DD_src->X_Q10,     sizeof( DD_src->X_Q10     ) );
    SKP_memcpy( DD_dst->Rd_Q10,    DD_src->Rd_Q10,    sizeof( DD_src->Rd_Q10     ) );
    SKP_memcpy( DD_dst->Q_md_Q10[0],     DD_src->Q_md_Q10[0],     sizeof( DD_src->Q_md_Q10[0]     ) );
    SKP_memcpy( DD_dst->Pred_Q16,  DD_src->Pred_Q16,  sizeof( DD_src->Pred_Q16  ) );
    SKP_memcpy( DD_dst->Shape_Q10, DD_src->Shape_Q10, sizeof( DD_src->Shape_Q10 ) );
    SKP_memcpy( DD_dst->Xq_Q10,    DD_src->Xq_Q10,    sizeof( DD_src->Xq_Q10    ) );
    SKP_memcpy( DD_dst->Xq_md_Q10[0],    DD_src->Xq_md_Q10[0],    sizeof( DD_src->Xq_md_Q10[0]    ) );
    SKP_memcpy( DD_dst->sAR2_Q14,  DD_src->sAR2_Q14,  sizeof( DD_src->sAR2_Q14  ) );
    SKP_memcpy( &DD_dst->sLPC_Q14[ LPC_state_idx ], &DD_src->sLPC_Q14[ LPC_state_idx ], NSQ_LPC_BUF_LENGTH * sizeof( SKP_int32 ) );
	SKP_memcpy( DD_dst->exc_Q10,   DD_src->exc_Q10, 	sizeof( DD_src->exc_Q10 ) );
	DD_dst->LF_AR_Q12 = DD_src->LF_AR_Q12;
    DD_dst->Seed      = DD_src->Seed;
    DD_dst->SeedInit  = DD_src->SeedInit;
    DD_dst->Seed2      = DD_src->Seed2;
    DD_dst->SeedInit2  = DD_src->SeedInit2;
    DD_dst->RD_Q10    = DD_src->RD_Q10;
}
