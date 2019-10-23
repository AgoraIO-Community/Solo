/* Copyright (C) 2002 Jean-Marc Valin */
/**
   @file AGR_Sate_bits.h
   @brief Handles bit packing/unpacking
*/
/*
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

#ifndef BITS_H
#define BITS_H
/** @defgroup NovaBits NovaBits: Bit-stream manipulations
 *  This is the structure that holds the bit-stream when encoding or decoding
 * with Speex. It allows some manipulations as well.
 *  @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/** Bit-packing data structure representing (part of) a bit-stream. */
typedef struct NovaBits {
   char *chars;   /**< "raw" data */
   int   nbBits;  /**< Total number of bits stored in the stream*/
   int   charPtr; /**< Position of the byte "cursor" */
   int   bitPtr;  /**< Position of the bit "cursor" within the current char */
   int   owner;   /**< Does the struct "own" the "raw" buffer (member "chars") */
   int   overflow;/**< Set to one if we try to read past the valid data */
   int   buf_size;/**< Allocated size for buffer */
   int   reserved1; /**< Reserved for future use */
   void *reserved2; /**< Reserved for future use */
} NovaBits;


void AGR_Sate_bits_init(NovaBits *bits);

void AGR_Sate_bits_reset(NovaBits *bits);

void AGR_Sate_bits_pack(NovaBits *bits, int data, int nbBits);

void AGR_Sate_bits_insert_terminator(NovaBits *bits);

unsigned int AGR_Sate_bits_unpack_unsigned(NovaBits *bits, int nbBits);

int AGR_Sate_bits_write(NovaBits *bits, unsigned char *chars, int max_nbytes);

void AGR_Sate_bits_destroy(NovaBits *bits);


#ifdef __cplusplus
}
#endif

/* @} */
#endif
