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

/*                                                                      *
 * SKP_Silk_allpass_int.c                                             *
 *                                                                      *
 * First-order allpass filter with                                      *                                                                     *
 * transfer function:                                                   *
 *                                                                      *
 *         A + Z^(-1)                                                   *
 * H(z) = ------------                                                  *
 *        1 + A*Z^(-1)                                                  *
 *                                                                      *
 * Implemented using minimum multiplier filter design.                  *
 *                                                                      *
 * Reference: http://www.univ.trieste.it/~ramponi/teaching/             *
 * DSP/materiale/Ch6(2).pdf                                             *
 *                                                                      *
 * Copyright 2007 (c), Skype Limited                                    *
 * Date: 070525                                                         *
 *                                                                      */
#include "SKP_Silk_SigProc_FLP.h"

/* first-order allpass filter */
void SKP_Silk_allpass_int_FLP(
    const SKP_float     *in,        /* I:   input signal [len]          */
    SKP_float           *S,         /* I/O: state [1]                   */
    SKP_float           A,          /* I:   coefficient (0 <= A < 1)    */
    SKP_float           *out,       /* O:   output signal [len]         */
    const SKP_int32     len         /* I:   number of samples           */
)
{
    SKP_float Y2, X2, S0;
    SKP_int32 k;

    S0 = S[ 0 ];
    for ( k = len-1; k >= 0; k-- ) {
        Y2        = *in - S0;
        X2        = Y2 * A;
        (*out++)  = S0 + X2;
        S0        = (*in++) + X2;
    }
    S[ 0 ] = S0;
}
