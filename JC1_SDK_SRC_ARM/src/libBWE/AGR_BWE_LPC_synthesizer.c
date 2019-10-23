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
#include "AGR_BWE_main_FIX.h"
#include "SKP_Silk_SigProc_FIX.h"

void AGR_Sate_LPC_synthesizer(
    SKP_float	*output,            /* O    output signal           */
    SKP_float	*ipexc,             /* I    excitation signal       */
    SKP_float	*sLPC,              /* I/O  state vector            */
    SKP_float	*a_tmp,             /* I    filter coefficients     */
    SKP_float   gain,
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
		output[ i ] = ipexc[i] * gain + LPC_pred;

		/* Update states */
		sLPC[ MAX_LPC_ORDER + i ] = output[i];
	}
}

void AGR_Sate_LPC_synthesis_filter_fix(
    const SKP_int32 *in_Q10,        /* I:   excitation signal */
    const SKP_int16 *A_Q12,     /* I:   AR coefficients [Order], between -8_Q0 and 8_Q0 */
    const SKP_int32 Gain_Q16,   /* I:   gain */
    SKP_int32 *S,               /* I/O: state vector [Order] */
    SKP_int16 *out,             /* O:   output signal */
    const SKP_int32 len,        /* I:   signal length */
    const SKP_int Order         /* I:   filter order, must be even */
)
{
    SKP_int   k, j, idx, Order_half = SKP_RSHIFT(Order, 1);
    SKP_int32 SA, SB, out32_Q10, out32;
#if !defined(_SYSTEM_IS_BIG_ENDIAN)
    SKP_int32 Atmp, A_align_Q12[MAX_LPC_ORDER >> 1];

    /* combine two A_Q12 values and ensure 32-bit alignment */
    for (k = 0; k < Order_half; k++) {
        idx = SKP_SMULBB(2, k);
        A_align_Q12[k] = (((SKP_int32)A_Q12[idx]) & 0x0000ffff) | SKP_LSHIFT((SKP_int32)A_Q12[idx + 1], 16);
    }
#endif

    /* Order must be even */
    SKP_assert(2 * Order_half == Order);

    /* S[] values are in Q14 */
    for (k = 0; k < len; k++) {
        SA = S[Order - 1];
        out32_Q10 = 0;
        for (j = 0; j < (Order_half - 1); j++) {
            idx = SKP_SMULBB(2, j) + 1;
#if !defined(_SYSTEM_IS_BIG_ENDIAN)
            /* multiply-add two prediction coefficients for each loop */
            /* NOTE: the code below loads two int16 values in an int32, and multiplies each using the   */
            /* SMLAWB and SMLAWT instructions. On a big-endian CPU the two int16 variables would be     */
            /* loaded in reverse order and the code will give the wrong result. In that case swapping   */
            /* the SMLAWB and SMLAWT instructions should solve the problem.                             */
            Atmp = A_align_Q12[j];
            SB = S[Order - 1 - idx];
            S[Order - 1 - idx] = SA;
            out32_Q10 = SKP_SMLAWB(out32_Q10, SA, Atmp);
            out32_Q10 = SKP_SMLAWT(out32_Q10, SB, Atmp);
            SA = S[Order - 2 - idx];
            S[Order - 2 - idx] = SB;
#else
            SB = S[Order - 1 - idx];
            S[Order - 1 - idx] = SA;
            out32_Q10 = SKP_SMLAWB(out32_Q10, SA, A_Q12[(j << 1)]);
            out32_Q10 = SKP_SMLAWB(out32_Q10, SB, A_Q12[(j << 1) + 1]);
            SA = S[Order - 2 - idx];
            S[Order - 2 - idx] = SB;
#endif
        }

#if !defined(_SYSTEM_IS_BIG_ENDIAN)
        /* unrolled loop: epilog */
        Atmp = A_align_Q12[Order_half - 1];
        SB = S[0];
        S[0] = SA;
        out32_Q10 = SKP_SMLAWB(out32_Q10, SA, Atmp);
        out32_Q10 = SKP_SMLAWT(out32_Q10, SB, Atmp);
#else
        /* unrolled loop: epilog */
        SB = S[0];
        S[0] = SA;
        out32_Q10 = SKP_SMLAWB(out32_Q10, SA, A_Q12[Order - 2]);
        out32_Q10 = SKP_SMLAWB(out32_Q10, SB, A_Q12[Order - 1]);
#endif
        /* apply gain to excitation signal and add to prediction */
        out32_Q10 = SKP_ADD_SAT32(out32_Q10, SKP_SMULWW(Gain_Q16, in_Q10[k]));

        /* scale to Q0 */
        out32 = SKP_RSHIFT_ROUND(out32_Q10, 10);

        /* saturate output */
        out[k] = (SKP_int16)SKP_SAT16(out32);

        /* move result into delay line */
        S[Order - 1] = SKP_LSHIFT_SAT32(out32_Q10, 4);
    }
}


