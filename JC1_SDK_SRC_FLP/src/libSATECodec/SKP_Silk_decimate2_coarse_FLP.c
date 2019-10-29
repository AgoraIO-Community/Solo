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
 * SKP_Silk_decimate2_coarse.c                                        *
 *                                                                      *
 * downsample by a factor 2, coarser                                    *
 *                                                                      */
#include "SKP_Silk_SigProc_FLP.h"

/* coefficients for coarser 2-fold resampling */
static SKP_float A20c_FLP[ 2 ] = {0.064666748046875f, 0.508514404296875f};
static SKP_float A21c_FLP[ 2 ] = {0.245666503906250f, 0.819732666015625f};

/* downsample by a factor 2, coarser */
void SKP_Silk_decimate2_coarse_FLP(
    const SKP_float     *in,        /* I:   16 kHz signal [2*len]       */
    SKP_float           *S,         /* I/O: state vector [4]            */
    SKP_float           *out,       /* O:   8 kHz signal [len]          */
    SKP_float           *scratch,   /* I:   scratch memory [3*len]      */
    const SKP_int32     len         /* I:   number of OUTPUT samples    */
)
{
    SKP_int32 k;

    /* de-interleave allpass inputs */
    for ( k = 0; k < len; k++) {
        scratch[ k ]       = in[ 2 * k ];
        scratch[ k + len ] = in[ 2 * k + 1 ];
    }

    /* allpass filters */
    SKP_Silk_allpass_int_FLP( scratch, S + 0, A21c_FLP[ 0 ], scratch + 2 * len, len );
    SKP_Silk_allpass_int_FLP( scratch + 2 * len, S + 1, A21c_FLP[ 1 ], scratch, len );
    
    SKP_Silk_allpass_int_FLP( scratch + len, S + 2, A20c_FLP[ 0 ], scratch + 2 * len, len );
    SKP_Silk_allpass_int_FLP( scratch + 2 * len, S + 3, A20c_FLP[ 1 ], scratch + len, len );
    
    /* add two allpass outputs */
    for ( k = 0; k < len; k++ ) {
        out[ k ] = 0.5f * ( scratch[ k ] + scratch[ k + len ] );
    }       
}


