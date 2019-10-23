/* Copyright (C) 2002-2006 Jean-Marc Valin 
   File: filters.c
   Various analysis/synthesis filters

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:
   
   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
   
   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
   
   - Neither the name of the Xiph.org Foundation nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "AGR_BWE_main_FIX.h"
#include <math.h>
#include "string.h"

/* Decomposes a signal into low-band and high-band using a QMF */
void AGR_Sate_qmf_decomp(
    const spx_word16_t *xx,                       /* I   Input signal              */
    const spx_word16_t *aa,                       /* I   Qmf coefficients          */
    spx_word16_t *y1,                             /* O   Output low band signal    */
    spx_word16_t *y2,                             /* O   Output high band signal   */
    SKP_int32     N,                              /* I   frame size                */
    SKP_int32     M,                              /* I   Qmf order                 */
    spx_word16_t *mem,                            /* I/O Qmf state                 */
    SKP_int8     *stack
)
{
	int i,j,k,M2;
	VARDECL(spx_word16_t *a);
	VARDECL(spx_word16_t *x);
	spx_word16_t *x2;

	ALLOC(a, M, spx_word16_t);
	ALLOC(x, N+M-1, spx_word16_t);
	x2=x+M-1;
	M2=M>>1;
	for (i=0;i<M;i++)
		a[M-i-1]= aa[i];
	for (i=0;i<M-1;i++)
		x[i]=mem[M-i-2];
	for (i=0;i<N;i++)
		x[i+M-1]=SHR16(xx[i],1);
	for (i=0;i<M-1;i++)
		mem[i]=SHR16(xx[N-i-1],1);
	for (i=0,k=0;i<N;i+=2,k++)
	{
		spx_word32_t y1k=0, y2k=0;
		for (j=0;j<M2;j++)
		{
			y1k=ADD32(y1k,MULT16_16(a[j],ADD16(x[i+j],x2[i-j])));
			y2k=SUB32(y2k,MULT16_16(a[j],SUB16(x[i+j],x2[i-j])));
			j++;
			y1k=ADD32(y1k,MULT16_16(a[j],ADD16(x[i+j],x2[i-j])));
			y2k=ADD32(y2k,MULT16_16(a[j],SUB16(x[i+j],x2[i-j])));
		}
		y1[k] = EXTRACT16(SATURATE(PSHR32(y1k,15),32767));
		y2[k] = EXTRACT16(SATURATE(PSHR32(y2k,15),32767));
	}
}

/* Re-synthesised a signal from the QMF low-band and high-band signals */
/* assumptions:
all odd x[i] are zero -- well, actually they are left out of the array now
N and M are multiples of 4 */
void AGR_Sate_qmf_synth(
    const spx_word16_t *x1,                       /* I   Low band signal           */
    const spx_word16_t *x2,                       /* I   High band signal          */
    const spx_word16_t *a,                        /* I   Qmf coefficients          */
    spx_word16_t *y,                              /* O   Synthesised signal        */
    SKP_int32     N,                              /* I   Signal size               */
    SKP_int32     M,                              /* I   Qmf order                 */
    spx_word16_t *mem1,                           /* I/O Qmf low band state        */
    spx_word16_t *mem2,                           /* I/O Qmf high band state       */
    SKP_int8     *stack
)
{
	int i, j;
	int M2, N2;
	VARDECL(spx_word16_t *xx1);
	VARDECL(spx_word16_t *xx2);

	M2 = M>>1;
	N2 = N>>1;
	ALLOC(xx1, M2+N2, spx_word16_t);
	ALLOC(xx2, M2+N2, spx_word16_t);

	for (i = 0; i < N2; i++)
		xx1[i] = x1[N2-1-i];
	for (i = 0; i < M2; i++)
		xx1[N2+i] = mem1[2*i+1];
	for (i = 0; i < N2; i++)
		xx2[i] = x2[N2-1-i];
	for (i = 0; i < M2; i++)
		xx2[N2+i] = mem2[2*i+1];

	for (i = 0; i < N2; i += 2) {
		spx_sig_t y0, y1, y2, y3;
		spx_word16_t x10, x20;

		y0 = y1 = y2 = y3 = 0;
		x10 = xx1[N2-2-i];
		x20 = xx2[N2-2-i];

		for (j = 0; j < M2; j += 2) {
			spx_word16_t x11, x21;
			spx_word16_t a0, a1;

			a0 = a[2*j];
			a1 = a[2*j+1];
			x11 = xx1[N2-1+j-i];
			x21 = xx2[N2-1+j-i];

#ifdef FIXED_POINT
			/* We multiply twice by the same coef to avoid overflows */
			y0 = MAC16_16(MAC16_16(y0, a0, x11), NEG16(a0), x21);
			y1 = MAC16_16(MAC16_16(y1, a1, x11), a1, x21);
			y2 = MAC16_16(MAC16_16(y2, a0, x10), NEG16(a0), x20);
			y3 = MAC16_16(MAC16_16(y3, a1, x10), a1, x20);
#else
			y0 = ADD32(y0,MULT16_16(a0, x11-x21));
			y1 = ADD32(y1,MULT16_16(a1, x11+x21));
			y2 = ADD32(y2,MULT16_16(a0, x10-x20));
			y3 = ADD32(y3,MULT16_16(a1, x10+x20));
#endif
			a0 = a[2*j+2];
			a1 = a[2*j+3];
			x10 = xx1[N2+j-i];
			x20 = xx2[N2+j-i];

#ifdef FIXED_POINT
			/* We multiply twice by the same coef to avoid overflows */
			y0 = MAC16_16(MAC16_16(y0, a0, x10), NEG16(a0), x20);
			y1 = MAC16_16(MAC16_16(y1, a1, x10), a1, x20);
			y2 = MAC16_16(MAC16_16(y2, a0, x11), NEG16(a0), x21);
			y3 = MAC16_16(MAC16_16(y3, a1, x11), a1, x21);
#else
			y0 = ADD32(y0,MULT16_16(a0, x10-x20));
			y1 = ADD32(y1,MULT16_16(a1, x10+x20));
			y2 = ADD32(y2,MULT16_16(a0, x11-x21));
			y3 = ADD32(y3,MULT16_16(a1, x11+x21));
#endif
		}
#ifdef FIXED_POINT
		y[2*i] = EXTRACT16(SATURATE32(PSHR32(y0,15),32767));
		y[2*i+1] = EXTRACT16(SATURATE32(PSHR32(y1,15),32767));
		y[2*i+2] = EXTRACT16(SATURATE32(PSHR32(y2,15),32767));
		y[2*i+3] = EXTRACT16(SATURATE32(PSHR32(y3,15),32767));
#else
		/* Normalize up explicitly if we're in float */
		y[2*i] = 2.f*y0;
		y[2*i+1] = 2.f*y1;
		y[2*i+2] = 2.f*y2;
		y[2*i+3] = 2.f*y3;
#endif
	}

	for (i = 0; i < M2; i++)
		mem1[2*i+1] = xx1[i];
	for (i = 0; i < M2; i++)
		mem2[2*i+1] = xx2[i];
}

