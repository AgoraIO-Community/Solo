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

#include <stdlib.h>
#include "SKP_Silk_main_FLP.h"

/*******************************************/
/* LPC analysis filter                     */
/* NB! State is kept internally and the    */
/* filter always starts with zero state    */
/* first Order output samples are not set  */
/*******************************************/

void SKP_Silk_LPC_analysis_filter_FLP(
          SKP_float                 r_LPC[],            /* O    LPC residual signal                     */
    const SKP_float                 PredCoef[],         /* I    LPC coefficients                        */
    const SKP_float                 s[],                /* I    Input signal                            */
    const SKP_int                   length,             /* I    Length of input signal                  */
    const SKP_int                   Order               /* I    LPC order                               */
)
{
    SKP_assert( Order <= length );

    switch( Order ) {
        case 6:
            SKP_Silk_LPC_analysis_filter6_FLP(  r_LPC, PredCoef, s, length );
        break;

        case 8:
            SKP_Silk_LPC_analysis_filter8_FLP(  r_LPC, PredCoef, s, length );
        break;

        case 10:
            SKP_Silk_LPC_analysis_filter10_FLP( r_LPC, PredCoef, s, length );
        break;

        case 12:
            SKP_Silk_LPC_analysis_filter12_FLP( r_LPC, PredCoef, s, length );
        break;

        case 16:
            SKP_Silk_LPC_analysis_filter16_FLP( r_LPC, PredCoef, s, length );
        break;

        default:
            SKP_assert( 0 );
        break;
    }

    /* Set first LPC Order samples to zero instead of undefined */
    SKP_memset( r_LPC, 0, Order * sizeof( SKP_float ) );
}

/* 16th order LPC analysis filter, does not write first 16 samples */
void SKP_Silk_LPC_analysis_filter16_FLP(
          SKP_float                 r_LPC[],            /* O    LPC residual signal                     */
    const SKP_float                 PredCoef[],         /* I    LPC coefficients                        */
    const SKP_float                 s[],                /* I    Input signal                            */
    const SKP_int                   length              /* I    Length of input signal                  */
)
{
    SKP_int   ix = 16;
    SKP_float LPC_pred;
    const SKP_float *s_ptr;

    for ( ; ix < length; ix++) {
        s_ptr = &s[ix - 1];

        /* short-term prediction */
        LPC_pred = s_ptr[ 0 ]   * PredCoef[ 0 ]  + 
                   s_ptr[-1]  * PredCoef[ 1 ]  +
                   s_ptr[-2]  * PredCoef[ 2 ]  +
                   s_ptr[-3]  * PredCoef[ 3 ]  +
                   s_ptr[-4]  * PredCoef[ 4 ]  +
                   s_ptr[-5]  * PredCoef[ 5 ]  +
                   s_ptr[-6]  * PredCoef[ 6 ]  +
                   s_ptr[-7]  * PredCoef[ 7 ]  +
                   s_ptr[-8]  * PredCoef[ 8 ]  +
                   s_ptr[-9]  * PredCoef[ 9 ]  +
                   s_ptr[-10] * PredCoef[ 10 ] +
                   s_ptr[-11] * PredCoef[ 11 ] +
                   s_ptr[-12] * PredCoef[ 12 ] +
                   s_ptr[-13] * PredCoef[ 13 ] +
                   s_ptr[-14] * PredCoef[ 14 ] +
                   s_ptr[-15] * PredCoef[ 15 ];

        /* prediction error */
        r_LPC[ix] = s_ptr[ 1 ] - LPC_pred;
    }
}

/* 12th order LPC analysis filter, does not write first 12 samples */
void SKP_Silk_LPC_analysis_filter12_FLP(
          SKP_float                 r_LPC[],            /* O    LPC residual signal                     */
    const SKP_float                 PredCoef[],         /* I    LPC coefficients                        */
    const SKP_float                 s[],                /* I    Input signal                            */
    const SKP_int                   length              /* I    Length of input signal                  */
)
{
    SKP_int   ix = 12;
    SKP_float LPC_pred;
    const SKP_float *s_ptr;

    for ( ; ix < length; ix++) {
        s_ptr = &s[ix - 1];

        /* short-term prediction */
        LPC_pred = s_ptr[ 0 ]   * PredCoef[ 0 ]  + 
                   s_ptr[-1]  * PredCoef[ 1 ]  +
                   s_ptr[-2]  * PredCoef[ 2 ]  +
                   s_ptr[-3]  * PredCoef[ 3 ]  +
                   s_ptr[-4]  * PredCoef[ 4 ]  +
                   s_ptr[-5]  * PredCoef[ 5 ]  +
                   s_ptr[-6]  * PredCoef[ 6 ]  +
                   s_ptr[-7]  * PredCoef[ 7 ]  +
                   s_ptr[-8]  * PredCoef[ 8 ]  +
                   s_ptr[-9]  * PredCoef[ 9 ]  +
                   s_ptr[-10] * PredCoef[ 10 ] +
                   s_ptr[-11] * PredCoef[ 11 ];

        /* prediction error */
        r_LPC[ix] = s_ptr[ 1 ] - LPC_pred;
    }
}

/* 10th order LPC analysis filter, does not write first 10 samples */
void SKP_Silk_LPC_analysis_filter10_FLP(
          SKP_float                 r_LPC[],            /* O    LPC residual signal                     */
    const SKP_float                 PredCoef[],         /* I    LPC coefficients                        */
    const SKP_float                 s[],                /* I    Input signal                            */
    const SKP_int                   length              /* I    Length of input signal                  */
)
{
    SKP_int   ix = 10;
    SKP_float LPC_pred;
    const SKP_float *s_ptr;

    for ( ; ix < length; ix++) {
        s_ptr = &s[ix - 1];

        /* short-term prediction */
        LPC_pred = s_ptr[ 0 ]   * PredCoef[ 0 ]  + 
                   s_ptr[-1]  * PredCoef[ 1 ]  +
                   s_ptr[-2]  * PredCoef[ 2 ]  +
                   s_ptr[-3]  * PredCoef[ 3 ]  +
                   s_ptr[-4]  * PredCoef[ 4 ]  +
                   s_ptr[-5]  * PredCoef[ 5 ]  +
                   s_ptr[-6]  * PredCoef[ 6 ]  +
                   s_ptr[-7]  * PredCoef[ 7 ]  +
                   s_ptr[-8]  * PredCoef[ 8 ]  +
                   s_ptr[-9]  * PredCoef[ 9 ];

        /* prediction error */
        r_LPC[ix] = s_ptr[ 1 ] - LPC_pred;
    }
}

/* 8th order LPC analysis filter, does not write first 8 samples */
void SKP_Silk_LPC_analysis_filter8_FLP(
          SKP_float                 r_LPC[],            /* O    LPC residual signal                     */
    const SKP_float                 PredCoef[],         /* I    LPC coefficients                        */
    const SKP_float                 s[],                /* I    Input signal                            */
    const SKP_int                   length              /* I    Length of input signal                  */
)
{
    SKP_int   ix = 8;
    SKP_float LPC_pred;
    const SKP_float *s_ptr;

    for ( ; ix < length; ix++) {
        s_ptr = &s[ix - 1];

        /* short-term prediction */
        LPC_pred = s_ptr[  0 ] * PredCoef[ 0 ]  + 
                   s_ptr[ -1 ] * PredCoef[ 1 ]  +
                   s_ptr[ -2 ] * PredCoef[ 2 ]  +
                   s_ptr[ -3 ] * PredCoef[ 3 ]  +
                   s_ptr[ -4 ] * PredCoef[ 4 ]  +
                   s_ptr[ -5 ] * PredCoef[ 5 ]  +
                   s_ptr[ -6 ] * PredCoef[ 6 ]  +
                   s_ptr[ -7 ] * PredCoef[ 7 ];

        /* prediction error */
        r_LPC[ix] = s_ptr[ 1 ] - LPC_pred;
    }
}

/* 6th order LPC analysis filter, does not write first 6 samples */
void SKP_Silk_LPC_analysis_filter6_FLP(
          SKP_float                 r_LPC[],            /* O    LPC residual signal                     */
    const SKP_float                 PredCoef[],         /* I    LPC coefficients                        */
    const SKP_float                 s[],                /* I    Input signal                            */
    const SKP_int                   length              /* I    Length of input signal                  */
)
{
    SKP_int   ix = 6;
    SKP_float LPC_pred;
    const SKP_float *s_ptr;

    for ( ; ix < length; ix++) {
        s_ptr = &s[ix - 1];

        /* short-term prediction */
        LPC_pred = s_ptr[  0 ] * PredCoef[ 0 ]  + 
                   s_ptr[ -1 ] * PredCoef[ 1 ]  +
                   s_ptr[ -2 ] * PredCoef[ 2 ]  +
                   s_ptr[ -3 ] * PredCoef[ 3 ]  +
                   s_ptr[ -4 ] * PredCoef[ 4 ]  +
                   s_ptr[ -5 ] * PredCoef[ 5 ];

        /* prediction error */
        r_LPC[ix] = s_ptr[ 1 ] - LPC_pred;
    }
}
