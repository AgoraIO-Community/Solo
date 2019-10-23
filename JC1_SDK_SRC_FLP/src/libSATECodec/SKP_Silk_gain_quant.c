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

#define OFFSET          ( ( MIN_QGAIN_DB * 128 ) / 6 + 16 * 128 )
#define SCALE_Q16       ( ( 65536 * ( N_LEVELS_QGAIN - 1 ) ) / ( ( ( MAX_QGAIN_DB - MIN_QGAIN_DB ) * 128 ) / 6 ) )
#define INV_SCALE_Q16   ( ( 65536 * ( ( ( MAX_QGAIN_DB - MIN_QGAIN_DB ) * 128 ) / 6 ) ) / ( N_LEVELS_QGAIN - 1 ) )

#define OFFSET2          ( 16 * 128 )
#define SCALE2_Q16       ( ( 65536 * ( 8 - 1 ) ) / ( ( ( 9 - 0 ) * 128 ) / 6 ) )
#define INV_SCALE2_Q16   ( ( 65536 * ( ( ( 9 - 0 ) * 8 ) / 6 ) ) / ( 8 - 1 ) )


#define DELTA_OFFSET 32767

/* Gain scalar quantization with hysteresis, uniform on log scale */
void SKP_Silk_gains_quant(
    SKP_int                         ind[ NB_SUBFR ],        /* O    gain indices                            */
    SKP_int32                       gain_Q16[ NB_SUBFR ],   /* I/O  gains (quantized out)                   */
    SKP_int                         *prev_ind,              /* I/O  last index in previous frame            */
    const SKP_int                   conditional  ,           /* I    first gain is delta coded if 1          */

    SKP_int	*ind2,
    SKP_int32	*DeltaGains_Q16,
    SKP_int    *prev_dind,              /* I/O  last index in previous frame            */
    SKP_int	md_enable
)
{
	SKP_int k;
	SKP_int32 inv_gain_Q16;
	SKP_int AlphaDis_Q16 = 32768/DELTA_CODE_NUM;

	if(md_enable == 1){
		inv_gain_Q16 = SKP_INVERSE32_varQ( SKP_max( *DeltaGains_Q16, 1 ), 32 );

		inv_gain_Q16 = SKP_min( inv_gain_Q16, SKP_int32_MAX );
		inv_gain_Q16 -= DELTA_OFFSET;

		*ind2 = 0;
		
		for(k = 0; k < DELTA_CODE_NUM ; k++){
			if((inv_gain_Q16 > (k*AlphaDis_Q16))&&(inv_gain_Q16 <= ((k+1)*AlphaDis_Q16))){
				*ind2 = k;
				inv_gain_Q16 = ((k+1)*AlphaDis_Q16);
			}
		}	
		inv_gain_Q16 += DELTA_OFFSET;
		*DeltaGains_Q16 = SKP_INVERSE32_varQ( SKP_max( inv_gain_Q16, 1 ), 32 );
	}

    for( k = 0; k < NB_SUBFR; k++ ) {
        /* Add half of previous quantization error, convert to log scale, scale, floor() */
        ind[ k ] = SKP_SMULWB( SCALE_Q16, SKP_Silk_lin2log( gain_Q16[ k ] ) - OFFSET );
		
        /* Round towards previous quantized gain (hysteresis) */
        if( ind[ k ] < *prev_ind ) {
            ind[ k ]++;
        }

        /* Compute delta indices and limit */
        if( k == 0 && conditional == 0 ) {
            /* Full index */
            ind[ k ] = SKP_LIMIT_int( ind[ k ], 0, N_LEVELS_QGAIN - 1 );
            ind[ k ] = SKP_max_int( ind[ k ], *prev_ind + MIN_DELTA_GAIN_QUANT );
            *prev_ind = ind[ k ];
        } else {
            /* Delta index */
            ind[ k ] = SKP_LIMIT_int( ind[ k ] - *prev_ind, MIN_DELTA_GAIN_QUANT, MAX_DELTA_GAIN_QUANT );
            /* Accumulate deltas */
            *prev_ind += ind[ k ];
            /* Shift to make non-negative */
            ind[ k ] -= MIN_DELTA_GAIN_QUANT;
        }

        /* Convert to linear scale and scale */
        gain_Q16[ k ] = SKP_Silk_log2lin( SKP_min_32( SKP_SMULWB( INV_SCALE_Q16, *prev_ind ) + OFFSET, 3967 ) ); /* 3968 = 31 in Q7 */
    }
}

/* Gains scalar dequantization, uniform on log scale */
void SKP_Silk_gains_dequant(
    SKP_int32                       gain_Q16[ NB_SUBFR ],   /* O    quantized gains                         */
    const SKP_int                   ind[ NB_SUBFR ],        /* I    gain indices                            */
    SKP_int                         *prev_ind,              /* I/O  last index in previous frame            */
    const SKP_int                   conditional ,            /* I    first gain is delta coded if 1          */
    SKP_int	ind2,
    SKP_int32	*DeltaGains_Q16,
    SKP_int	md_enable
)
{
    SKP_int   k;
	SKP_int32 inv_gain_Q16;
	SKP_int AlphaDis_Q16 = 32768/DELTA_CODE_NUM;

    for( k = 0; k < NB_SUBFR; k++ ) {
        if( k == 0 && conditional == 0 ) {
            *prev_ind = ind[ k ];
        } else {
            /* Delta index */
            *prev_ind += ind[ k ] + MIN_DELTA_GAIN_QUANT;
        }

        /* Convert to linear scale and scale */
        gain_Q16[ k ] = SKP_Silk_log2lin( SKP_min_32( SKP_SMULWB( INV_SCALE_Q16, *prev_ind ) + OFFSET, 3967 ) ); /* 3968 = 31 in Q7 */
    }
	
	if(md_enable == 1)
	{
		inv_gain_Q16 = ((ind2+1)*AlphaDis_Q16);
		inv_gain_Q16 += DELTA_OFFSET;
		*DeltaGains_Q16 = SKP_INVERSE32_varQ( SKP_max( inv_gain_Q16, 1 ), 32 );
	}
}
