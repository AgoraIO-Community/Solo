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

void SKP_Silk_LTP_analysis_filter_FLP(
          SKP_float         *LTP_res,                   /* O    LTP res NB_SUBFR*(pre_lgth+subfr_lngth) */
    const SKP_float         *x,                         /* I    Input signal, with preceeding samples   */
    const SKP_float         B[ LTP_ORDER * NB_SUBFR ],  /* I    LTP coefficients for each subframe      */
    const SKP_int           pitchL[   NB_SUBFR ],       /* I    Pitch lags                              */
    const SKP_float         invGains[ NB_SUBFR ],       /* I    Inverse quantization gains              */
    const SKP_int           subfr_length,               /* I    Length of each subframe                 */
    const SKP_int           pre_length                  /* I    Preceeding samples for each subframe    */
)
{
    const SKP_float *x_ptr, *x_lag_ptr;
    SKP_float   Btmp[ LTP_ORDER ];
    SKP_float   *LTP_res_ptr;
    SKP_float   inv_gain;
    SKP_int     k, i, j;

    x_ptr = x;
    LTP_res_ptr = LTP_res;
    for( k = 0; k < NB_SUBFR; k++ ) {
        x_lag_ptr = x_ptr - pitchL[ k ];
        inv_gain = invGains[ k ];
        for( i = 0; i < LTP_ORDER; i++ ) {
            Btmp[ i ] = B[ k * LTP_ORDER + i ];
        }
        
        /* LTP analysis FIR filter */
        for( i = 0; i < subfr_length + pre_length; i++ ) {
            LTP_res_ptr[ i ] = x_ptr[ i ];
            /* Subtract long-term prediction */ 
            for( j = 0; j < LTP_ORDER; j++ ) {
                LTP_res_ptr[ i ] -= Btmp[ j ] * x_lag_ptr[ LTP_ORDER / 2 - j ];
            }
            LTP_res_ptr[ i ] *= inv_gain;
            x_lag_ptr++;
        }

        /* Update pointers */
        LTP_res_ptr += subfr_length + pre_length; 
        x_ptr       += subfr_length;
    }
}
