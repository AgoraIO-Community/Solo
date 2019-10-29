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
 * SKP_Silk_lowpass_int.c                                             *
 *                                                                      *
 * First order low-pass filter, with input as SKP_int32, running at     *
 * 48 kHz                                                               *
 *                                                                      *
 * Copyright 2006 (c), Skype Limited                                    *
 * Date: 060221                                                         *
 *                                                                      */
#include "SKP_Silk_SigProc_FIX.h"

/* First order low-pass filter, with input as SKP_int32, running at 48 kHz        */
void SKP_Silk_lowpass_int(
    const SKP_int32      *in,            /* I:    Q25 48 kHz signal; length = len */
    SKP_int32            *S,             /* I/O: Q25 state; length = 1            */
    SKP_int32            *out,           /* O:    Q25 48 kHz signal; length = len */
    const SKP_int32      len             /* I:    Number of samples               */
)
{
    SKP_int        k;
    SKP_int32    in_tmp, out_tmp, state;
    
    state = S[ 0 ];
    for( k = len; k > 0; k-- ) {    
        in_tmp  = *in++;
        in_tmp -= SKP_RSHIFT( in_tmp, 2 );              /* multiply by 0.75 */
        out_tmp = state + in_tmp;                       /* zero at nyquist  */
        state   = in_tmp - SKP_RSHIFT( out_tmp, 1 );    /* pole             */
        *out++  = out_tmp;
    }
    S[ 0 ] = state;
}


