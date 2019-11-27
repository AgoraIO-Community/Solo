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

#include "SKP_Silk_main_FLP.h"
#include "SKP_Silk_tuning_parameters.h"

void SKP_Silk_find_LTP_FLP(
          SKP_float b[    NB_SUBFR * LTP_ORDER ],       /* O    LTP coefs                               */
          SKP_float WLTP[NB_SUBFR*LTP_ORDER*LTP_ORDER], /* O    Weight for LTP quantization             */
          SKP_float *LTPredCodGain,                     /* O    LTP coding gain                         */
    const SKP_float r_first[],                          /* I    LPC residual, signal + state for 10 ms  */
    const SKP_float r_last[],                           /* I    LPC residual, signal + state for 10 ms  */
    const SKP_int   lag[  NB_SUBFR ],                   /* I    LTP lags                                */
    const SKP_float Wght[ NB_SUBFR ],                   /* I    Weights                                 */
    const SKP_int   subfr_length,                       /* I    Subframe length                         */
    const SKP_int   mem_offset                          /* I    Number of samples in LTP memory         */
)
{
    SKP_int   i, k;
    SKP_float *b_ptr, temp, *WLTP_ptr;
    SKP_float LPC_res_nrg, LPC_LTP_res_nrg;
    SKP_float d[ NB_SUBFR ], m, g, delta_b[ LTP_ORDER ];
    SKP_float w[ NB_SUBFR ], nrg[ NB_SUBFR ], regu;
    SKP_float Rr[ LTP_ORDER ], rr[ NB_SUBFR ];
    const SKP_float *r_ptr, *lag_ptr;

    b_ptr    = b;
    WLTP_ptr = WLTP;
    r_ptr    = &r_first[ mem_offset ];
    for( k = 0; k < NB_SUBFR; k++ ) {
        if( k == ( NB_SUBFR >> 1 ) ) { /* Shift residual for last 10 ms */
            r_ptr = &r_last[ mem_offset ];
        }
        lag_ptr = r_ptr - ( lag[ k ] + LTP_ORDER / 2 );

        SKP_Silk_corrMatrix_FLP( lag_ptr, subfr_length, LTP_ORDER, WLTP_ptr );
        SKP_Silk_corrVector_FLP( lag_ptr, r_ptr, subfr_length, LTP_ORDER, Rr );

        rr[ k ] = ( SKP_float )SKP_Silk_energy_FLP( r_ptr, subfr_length );
        regu = 1.0f + rr[ k ] + 
            matrix_ptr( WLTP_ptr, 0, 0, LTP_ORDER ) + 
            matrix_ptr( WLTP_ptr, LTP_ORDER-1, LTP_ORDER-1, LTP_ORDER );
        regu *= LTP_DAMPING / 3;
        SKP_Silk_regularize_correlations_FLP( WLTP_ptr, &rr[ k ], regu, LTP_ORDER );
        SKP_Silk_solve_LDL_FLP( WLTP_ptr, LTP_ORDER, Rr, b_ptr );

        /* Calculate residual energy */
        nrg[ k ] = SKP_Silk_residual_energy_covar_FLP( b_ptr, WLTP_ptr, Rr, rr[ k ], LTP_ORDER );

        temp = Wght[ k ] / ( nrg[ k ] * Wght[ k ] + 0.01f * subfr_length );
        SKP_Silk_scale_vector_FLP( WLTP_ptr, temp, LTP_ORDER * LTP_ORDER );
        w[ k ] = matrix_ptr( WLTP_ptr, LTP_ORDER / 2, LTP_ORDER / 2, LTP_ORDER );
    
        r_ptr    += subfr_length;
        b_ptr    += LTP_ORDER;
        WLTP_ptr += LTP_ORDER * LTP_ORDER;
    }

    /* Compute LTP coding gain */
    if( LTPredCodGain != NULL ) {
        LPC_LTP_res_nrg = 1e-6f;
        LPC_res_nrg     = 0.0f;
        for( k = 0; k < NB_SUBFR; k++ ) {
            LPC_res_nrg     += rr[  k ] * Wght[ k ];
            LPC_LTP_res_nrg += nrg[ k ] * Wght[ k ];
        }
        
        SKP_assert( LPC_LTP_res_nrg > 0 );
        *LTPredCodGain = 3.0f * SKP_Silk_log2( LPC_res_nrg / LPC_LTP_res_nrg );
    }

    /* Smoothing */
    /* d = sum( B, 1 ); */
    b_ptr = b;
    for( k = 0; k < NB_SUBFR; k++ ) {
        d[ k ] = 0;
        for( i = 0; i < LTP_ORDER; i++ ) {
            d[ k ] += b_ptr[ i ];
        }
        b_ptr += LTP_ORDER;
    }
    /* m = ( w * d' ) / ( sum( w ) + 1e-3 ); */
    temp = 1e-3f;
    for( k = 0; k < NB_SUBFR; k++ ) {
        temp += w[ k ];
    }
    m = 0;
    for( k = 0; k < NB_SUBFR; k++ ) {
        m += d[ k ] * w[ k ];
    }
    m = m / temp;

    b_ptr = b;
    for( k = 0; k < NB_SUBFR; k++ ) {
        g = LTP_SMOOTHING / ( LTP_SMOOTHING + w[ k ] ) * ( m - d[ k ] );
        temp = 0;
        for( i = 0; i < LTP_ORDER; i++ ) {
            delta_b[ i ] = SKP_max_float( b_ptr[ i ], 0.1f );
            temp += delta_b[ i ];
        }
        temp = g / temp;
        for( i = 0; i < LTP_ORDER; i++ ) {
            b_ptr[ i ] = b_ptr[ i ] + delta_b[ i ] * temp;
        }
        b_ptr += LTP_ORDER;
    }
}
