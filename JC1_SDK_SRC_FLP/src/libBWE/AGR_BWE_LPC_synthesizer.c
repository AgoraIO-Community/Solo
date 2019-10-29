/***********************************************************************
Copyright (c) 2006-2011, Skype Limited. All rights reserved. 
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
#include "AGR_BWE_main_FLP.h"

void AGR_Sate_LPC_synthesizer(
    SKP_float	*output,            /* O    output signal           */
    SKP_float	*ipexc,             /* I    excitation signal       */
    SKP_float	*sLPC,              /* I/O  state vector            */
    SKP_float	*a_tmp,             /* I    filter coefficients     */
    SKP_int32	LPC_order,          /* I    filter order            */
    SKP_int32	subfr_length        /* I    signal length           */
)
{
	SKP_int32 i, j;
	SKP_float LPC_pred;

	for( i = 0; i < subfr_length; i++ ) {
		LPC_pred = 0.0;
		for( j = 0; j < LPC_order; j ++ ) 
			LPC_pred += sLPC[ MAX_LPC_ORDER + i - j - 1 ] * a_tmp[ j ] ;
	
		/* Add prediction to LPC residual */
		output[ i ] = ipexc[i] + LPC_pred;

		/* Update states */
		sLPC[ MAX_LPC_ORDER + i ] = output[i];
	}
}



