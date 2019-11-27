/* Copyright (C) 2002 Jean-Marc Valin 
   File: AGR_Sate_bits.c

   Handles bit packing/unpacking

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

#include <stdio.h>
#include <stdlib.h>
#include "AGR_BWE_bits.h"
#include "AGR_BWE_arch.h"

/* Maximum size of the bit-stream (for fixed-size allocation) */
#ifndef MAX_CHARS_PER_FRAME
#define MAX_CHARS_PER_FRAME (2000/BYTES_PER_CHAR)
#endif


#if (BYTES_PER_CHAR==2)
/* Swap bytes to proper endian order (could be done externally) */
#define HTOLS(A) ((((A) >> 8)&0xff)|(((A) & 0xff)<<8))
#else
#define HTOLS(A) (A)
#endif

void AGR_Sate_bits_init(NovaBits *bits)
{
   bits->chars = (char*)calloc(MAX_CHARS_PER_FRAME, 1);
   if (!bits->chars){
      return;
   }

   bits->buf_size = MAX_CHARS_PER_FRAME;

   bits->owner = 1;

   AGR_Sate_bits_reset(bits);
}

void AGR_Sate_bits_reset(NovaBits *bits)
{
	/* We only need to clear the first byte now */
	bits->chars[0]=0;
	bits->nbBits=0;
	bits->charPtr=0;
	bits->bitPtr=0;
	bits->overflow=0;
}

void AGR_Sate_bits_pack(NovaBits *bits, int data, int nbBits)
{
	unsigned int d=data;
	char *str1 = "Buffer too small to pack bits";
	char *str2 = "Could not resize input buffer: not packing";
	char *str3 = "Do not own input buffer: not packing";

	if (bits->charPtr+((nbBits+bits->bitPtr)>>LOG2_BITS_PER_CHAR) >= bits->buf_size){

		fprintf (stderr, "notification: %s\n", str1);
		if (bits->owner){
			int new_nchars = ((bits->buf_size+5)*3)>>1;
			char *tmp = (char*)realloc(bits->chars, new_nchars);
			if (tmp){
				bits->buf_size=new_nchars;
				bits->chars=tmp;
			} else {
				fprintf (stderr, "warning: %s\n", str2);
				return;
			}
		} else {
			fprintf (stderr, "warning: %s\n", str3);
			return;
		}
	}

	while(nbBits){
		int bit;
		bit = (d>>(nbBits-1))&1;
		bits->chars[bits->charPtr] |= bit<<(BITS_PER_CHAR-1-bits->bitPtr);
		bits->bitPtr++;

		if (bits->bitPtr==BITS_PER_CHAR){
			bits->bitPtr=0;
			bits->charPtr++;
			bits->chars[bits->charPtr] = 0;
		}
		bits->nbBits++;
		nbBits--;
	}
}


void AGR_Sate_bits_insert_terminator(NovaBits *bits)
{
    if (bits->bitPtr){
		AGR_Sate_bits_pack(bits, 0, 1);
    }
    while (bits->bitPtr){
		AGR_Sate_bits_pack(bits, 1, 1);
    }
}

unsigned int AGR_Sate_bits_unpack_unsigned(NovaBits *bits, int nbBits)
{
	unsigned int d=0;
	if ((bits->charPtr<<LOG2_BITS_PER_CHAR)+bits->bitPtr+nbBits>bits->nbBits)
		bits->overflow=1;
	if (bits->overflow)
		return 0;
	while(nbBits){
		d<<=1;
		d |= (bits->chars[bits->charPtr]>>(BITS_PER_CHAR-1 - bits->bitPtr))&1;
		bits->bitPtr++;
		if (bits->bitPtr==BITS_PER_CHAR){
			bits->bitPtr=0;
			bits->charPtr++;
		}
		nbBits--;
	}
	return d;
}


int AGR_Sate_bits_write(NovaBits *bits, unsigned char *chars, int max_nbytes)
{
	int i;
	int max_nchars = max_nbytes/BYTES_PER_CHAR;
	int charPtr, bitPtr, nbBits;

	/* Insert terminator, but save the data so we can put it back after */
	bitPtr=bits->bitPtr;
	charPtr=bits->charPtr;
	nbBits=bits->nbBits;
	AGR_Sate_bits_insert_terminator(bits);
	bits->bitPtr=bitPtr;
	bits->charPtr=charPtr;
	bits->nbBits=nbBits;

    if (max_nchars > ((bits->nbBits+BITS_PER_CHAR-1)>>LOG2_BITS_PER_CHAR)){
		max_nchars = ((bits->nbBits+BITS_PER_CHAR-1)>>LOG2_BITS_PER_CHAR);
    }

    for (i=0;i<max_nchars;i++){
		chars[i]=HTOLS(bits->chars[i]);
    }
	return max_nchars*BYTES_PER_CHAR;
}

void AGR_Sate_bits_destroy(NovaBits *bits)
{
    if (bits->owner){
		free(bits->chars);
    }
	/* Will do something once the allocation is dynamic */
}

