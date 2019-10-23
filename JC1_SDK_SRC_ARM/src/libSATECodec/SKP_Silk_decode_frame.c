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


#include "SKP_Silk_main.h"
#include "SKP_Silk_PLC.h"


SKP_int AgoraSateDecodePLC(
    SKP_Silk_decoder_state          *psDec,             /* I/O  Pointer to Silk decoder state               */
    SKP_Silk_decoder_control		*psDecCtrl,
    SKP_int16                        pOut[]             /* O    Pointer to output speech frame              */
)
{

    SKP_int  L;

    L = psDec->frame_length;
    
    /* Safety checks */
    SKP_assert( L > 0 && L <= MAX_FRAME_LENGTH );


    /*************************************************************/
    /* Generate Concealment frame if packet is lost, or corrupt  */
    /*************************************************************/
    /* Handle packet loss by extrapolation */
    SKP_Silk_PLC( psDec, psDecCtrl, pOut, L, 1 );

    return 0;
}

SKP_int AgoraSateDecodeTwoDesps(
    SKP_Silk_decoder_state          *psDec,             /* I/O  Pointer to Silk decoder state               */
    SKP_Silk_decoder_control		*psDecCtrl,
    SKP_int16                       pOut[],             /* O    Pointer to output speech frame              */
    const SKP_int                   nBytes1,             /* I    Payload length                              */
    const SKP_int                   nBytes2,             /* I    Payload length                              */
    const SKP_uint8                 pCode1[],            /* I    Pointer to payload                          */
    const SKP_uint8                 pCode2[],            /* I    Pointer to payload                          */
    SKP_int							desp_type,
    SKP_int                         decBytes[]           /* O    Used bytes to decode this frame             */
)
{

    SKP_int         i,L, fs_Khz_old, ret = 0;
    SKP_int         Pulses[MAX_INTERLEAVE_NUM][ MAX_FRAME_LENGTH ];
	SKP_int32		DeltaGains_Q16;
	SKP_int32			DeltaGains_p1_Q16;
	SKP_int32			DeltaGains_p2_Q16;
	SKP_int32			inv_gain_p1_Q16;
	SKP_int32			inv_gain_p2_Q16;
	SKP_int32			inv_gain_Q16;
	SKP_int      offset_p1_Q10;
	SKP_int      offset_p2_Q10;
	SKP_int32 rand_seed,dither,offset_Q10;
	SKP_int32    q_Q10,q_p1_Q10,q_p2_Q10;

    L = psDec->frame_length;
    
    /* Safety checks */
    SKP_assert( L > 0 && L <= MAX_FRAME_LENGTH );


	/********************************************/
	/* Initialize arithmetic coder				*/
	/********************************************/
	fs_Khz_old	  = psDec->fs_kHz;
	if( psDec->nFramesDecoded == 0 ) {
		/* Initialize range decoder state */
		SKP_Silk_range_dec_init( &psDec->sMD[0].sRC, pCode1, nBytes1 );
		
		if(desp_type > 1)
			SKP_Silk_range_dec_init( &psDec->sMD[1].sRC, pCode2, nBytes2 );
	}

	/********************************************/
	/* Decode parameters and pulse signal		*/
	/********************************************/
	SKP_Silk_decode_parameters(psDec, psDecCtrl, Pulses[0], 0, 1);

    if(desp_type > 1){
		SKP_Silk_decode_parameters(psDec, psDecCtrl, Pulses[1], 1, 1);
    }

	DeltaGains_Q16 = psDecCtrl->DeltaGains_Q16;

	inv_gain_Q16 = SKP_INVERSE32_varQ( SKP_max( DeltaGains_Q16, 1 ), 32 );
	inv_gain_Q16 = SKP_min( inv_gain_Q16, SKP_int32_MAX );
	
	inv_gain_p1_Q16 = inv_gain_Q16;  
	inv_gain_p1_Q16 = SKP_min( inv_gain_p1_Q16, SKP_int32_MAX );
	inv_gain_p2_Q16 = (65536 - inv_gain_Q16);  
	inv_gain_p2_Q16 = SKP_min( inv_gain_p2_Q16, SKP_int32_MAX );


	DeltaGains_p1_Q16 = SKP_INVERSE32_varQ( SKP_max( inv_gain_p1_Q16, 1 ), 32 );  
	DeltaGains_p1_Q16 = SKP_min( DeltaGains_p1_Q16, SKP_int32_MAX );
	DeltaGains_p2_Q16 = SKP_INVERSE32_varQ( SKP_max( inv_gain_p2_Q16, 1 ), 32 );  
	DeltaGains_p2_Q16 = SKP_min( DeltaGains_p2_Q16, SKP_int32_MAX );
	
    offset_Q10 = SKP_Silk_Quantization_Offsets_Q10[ psDecCtrl->sigtype ][ psDecCtrl->QuantOffsetType ];
#ifdef _OFFSET_MD_	
	offset_p1_Q10 = SKP_SMULWW( inv_gain_p1_Q16, offset_Q10);
	offset_p2_Q10 = SKP_SMULWW( inv_gain_p2_Q16, offset_Q10);
#else
	offset_p1_Q10 = offset_Q10;
	offset_p2_Q10 = offset_Q10;
#endif
    rand_seed = psDecCtrl->Seed;
	
	if( psDec->sMD[0].sRC.error || (psDec->sMD[1].sRC.error && desp_type > 1)) {
		psDec->nBytesLeft[0] = 0;

		ret				= 1; /* PLC operation */
		/* revert fs if changed in decode_parameters */
		SKP_Silk_decoder_set_fs( psDec, fs_Khz_old );

		/* Avoid crashing */

		decBytes[0] = psDec->sMD[0].sRC.bufferLength;
        if (desp_type > 1){
			decBytes[1] =  psDec->sMD[1].sRC.bufferLength;
        }
		if( psDec->sMD[0].sRC.error == RANGE_CODER_DEC_PAYLOAD_TOO_LONG ) {
			ret = SKP_SILK_DEC_PAYLOAD_TOO_LARGE;
		} else {
			ret = SKP_SILK_DEC_PAYLOAD_ERROR;
		}
	} else {
		psDec->nFramesDecoded++;
		decBytes[0] = psDec->sMD[0].sRC.bufferLength - psDec->nBytesLeft[0];
		decBytes[1] = psDec->sMD[1].sRC.bufferLength - psDec->nBytesLeft[1];
	
		/* Update lengths. Sampling frequency could have changed */
		L = psDec->frame_length;

		/********************************************************/
		/* Run inverse NSQ										*/
		/********************************************************/

        if(desp_type == 0){
			for( i = 0; i < psDec->frame_length; i++ ) {
#ifdef MD_SUB_FRAME
				if (i %(psDec->subfr_length<<1) < psDec->subfr_length)
#else
				if (i % MD_LEN < HALF_MD_LEN)
#endif	
				{
        			rand_seed = SKP_RAND( rand_seed );
#ifdef DISABLE_DITHER
					rand_seed = 0;
#endif
        			/* dither = rand_seed < 0 ? 0xFFFFFFFF : 0; */
        			dither = SKP_RSHIFT( rand_seed, 31 );

					q_Q10 = SKP_LSHIFT( ( SKP_int32 )Pulses[0][ i ], 10 );
					
					q_Q10 = SKP_ADD32( offset_p1_Q10, q_Q10);
					
        			psDec->sMD[0].exc_Q10[ i ] = ( q_Q10 ^ dither ) - dither;
					
					psDec->exc_Q10[ i ] = SKP_SMULWW( DeltaGains_p1_Q16,psDec->sMD[0].exc_Q10[ i ]  );
				}else{
        			rand_seed = SKP_RAND( rand_seed );
#ifdef DISABLE_DITHER
					rand_seed = 0;
#endif
        			/* dither = rand_seed < 0 ? 0xFFFFFFFF : 0; */
        			dither = SKP_RSHIFT( rand_seed, 31 );

					q_Q10 = SKP_LSHIFT( ( SKP_int32 )Pulses[0][ i ], 10 );
					
					q_Q10 = SKP_ADD32( offset_p2_Q10, q_Q10);
					
        			psDec->sMD[0].exc_Q10[ i ] = ( q_Q10 ^ dither ) - dither;
					
					psDec->exc_Q10[ i ] = SKP_SMULWW( DeltaGains_p2_Q16,psDec->sMD[0].exc_Q10[ i ]  );
				}
			}
		}else if(desp_type == 1){
			for( i = 0; i < psDec->frame_length; i++ ) {
#ifdef MD_SUB_FRAME
				if (i %(psDec->subfr_length<<1) < psDec->subfr_length)
#else
				if (i % MD_LEN < HALF_MD_LEN)
#endif	
				{
        			rand_seed = SKP_RAND( rand_seed );
#ifdef DISABLE_DITHER
					rand_seed = 0;
#endif
        			/* dither = rand_seed < 0 ? 0xFFFFFFFF : 0; */
        			dither = SKP_RSHIFT( rand_seed, 31 );

					q_Q10 = SKP_LSHIFT( ( SKP_int32 )Pulses[0][ i ], 10 );
					
					q_Q10 = SKP_ADD32( offset_p2_Q10, q_Q10);
					
        			psDec->sMD[0].exc_Q10[ i ] = ( q_Q10 ^ dither ) - dither;
					
					psDec->exc_Q10[ i ] = SKP_SMULWW( DeltaGains_p2_Q16,psDec->sMD[0].exc_Q10[ i ]  );
				}else{
        			rand_seed = SKP_RAND( rand_seed );
#ifdef DISABLE_DITHER
					rand_seed = 0;
#endif
        			/* dither = rand_seed < 0 ? 0xFFFFFFFF : 0; */
        			dither = SKP_RSHIFT( rand_seed, 31 );

					q_Q10 = SKP_LSHIFT( ( SKP_int32 )Pulses[0][ i ], 10 );
					
					q_Q10 = SKP_ADD32( offset_p1_Q10, q_Q10);
					
        			psDec->sMD[0].exc_Q10[ i ] = ( q_Q10 ^ dither ) - dither;
					
					psDec->exc_Q10[ i ] = SKP_SMULWW( DeltaGains_p1_Q16,psDec->sMD[0].exc_Q10[ i ]  );
				}
			}
		}else{
			for( i = 0; i < psDec->frame_length; i++ ) {

				rand_seed = SKP_RAND( rand_seed );
#ifdef DISABLE_DITHER
				rand_seed = 0;
#endif
        			/* dither = rand_seed < 0 ? 0xFFFFFFFF : 0; */
        		dither = SKP_RSHIFT( rand_seed, 31 );

				q_p1_Q10 = SKP_LSHIFT( ( SKP_int32 )Pulses[0][ i ], 10 );

				q_p2_Q10 = SKP_LSHIFT( ( SKP_int32 )Pulses[1][ i ], 10 );

				q_Q10 = SKP_ADD32(q_p1_Q10,q_p2_Q10);
				
				q_Q10 = SKP_ADD32( (offset_p1_Q10 + offset_p2_Q10), q_Q10);
				
    			psDec->exc_Q10[ i ] = ( q_Q10 ^ dither ) - dither;
			}
		}

		SKP_Silk_decode_core( psDec, psDecCtrl, pOut );

		/********************************************************/
		/* Update PLC state 									*/
		/********************************************************/
		SKP_Silk_PLC( psDec, psDecCtrl, pOut, L, 0 );

		psDec->lossCnt = 0;
		psDec->prev_sigtype = psDecCtrl->sigtype;

		/* A frame has been decoded without errors */
		psDec->first_frame_after_reset = 0;
		ret				= 0;  
	}

	return ret;
}

/****************/
/* Decode frame */
/****************/
SKP_int SKP_Silk_decode_frame(
    SKP_Silk_decoder_state          *psDec,             /* I/O  Pointer to Silk decoder state               */
    SKP_int16                        pOut[],             /* O    Pointer to output speech frame              */
    SKP_int16                       *pN,                /* O    Pointer to size of output frame             */
    const SKP_uint8                 pCode[],            /* I    Pointer to payload                          */
    const SKP_int16                 nBytes[],             /* I    Payload length                              */
    SKP_int                         action,             /* I    Action from Jitter Buffer                   */
    SKP_int                         decBytes[]          /* O    Used bytes to decode this frame             */
)
{
    SKP_Silk_decoder_control sDecCtrl;
    SKP_int         k,L, ret = 0;
	SKP_int         pCode_offset;
    SKP_uint8       pCode_md[MAX_INTERLEAVE_NUM][MAX_BYTES_PER_FRAME];


    L = psDec->frame_length;
    sDecCtrl.LTP_scale_Q14 = 0;
    
    /* Safety checks */
    SKP_assert( L > 0 && L <= MAX_FRAME_LENGTH );

	if((psDec->md_enable)&&(action > 3)){
		pCode_offset = 0;
		SKP_memset(pCode_md,0,MAX_INTERLEAVE_NUM*MAX_BYTES_PER_FRAME*sizeof(SKP_uint8));
		for(k = 0; k < psDec->desp_num;k++){
			if(nBytes[k])
				SKP_memcpy(pCode_md[k],pCode+pCode_offset,nBytes[k]);

			pCode_offset += nBytes[k];
		}
	}else{
		SKP_memcpy(pCode_md[0],pCode,nBytes[0]);
	}
	
    /********************************************/
    /* Decode Frame if packet is not lost  */
    /********************************************/
    *decBytes = 0;

    switch( action ) {
        case 0:
 //           ret = AgoraSateDecodeCenter(psDec,&sDecCtrl,pOut,nBytes[0],pCode_md[0],decBytes);
        break;
			
        case 1:
            AgoraSateDecodePLC(psDec,&sDecCtrl,pOut);
        break;

        case 2:
            ret = AgoraSateDecodeTwoDesps(psDec,&sDecCtrl,pOut,nBytes[0],0,pCode_md[0],NULL,0 ,decBytes);
        break;

        case 3:
            ret = AgoraSateDecodeTwoDesps(psDec,&sDecCtrl,pOut,nBytes[0],0,pCode_md[0],NULL,1 ,decBytes);
        break;

        case 4:
            ret = AgoraSateDecodeTwoDesps(psDec,&sDecCtrl,pOut,nBytes[0],nBytes[1],pCode_md[0],pCode_md[1],2 ,decBytes);
        break;
/*
        case 3:
            AgoraSateDecodeFourDesps();
        break;
*/
        default:
            SKP_assert( 0 );
        break;
    }

    if(ret == 1){
        AgoraSateDecodePLC(psDec,&sDecCtrl,pOut);
    }
    /*************************/
    /* Update output buffer. */
    /*************************/
    SKP_memcpy( psDec->outBuf, pOut, L * sizeof( SKP_int16 ) );

    /****************************************************************/
    /* Ensure smooth connection of extrapolated and good frames     */
    /****************************************************************/
    SKP_Silk_PLC_glue_frames( psDec, &sDecCtrl, pOut, L );

    /************************************************/
    /* Comfort noise generation / estimation        */
    /************************************************/
    SKP_Silk_CNG( psDec, &sDecCtrl, pOut , L );

    /********************************************/
    /* HP filter output                            */
    /********************************************/
    SKP_assert( ( ( psDec->fs_kHz == 12 ) && ( L % 3 ) == 0 ) || 
                ( ( psDec->fs_kHz != 12 ) && ( L % 2 ) == 0 ) );
    SKP_Silk_biquad( pOut, psDec->HP_B, psDec->HP_A, psDec->HPState, pOut, L );

    /********************************************/
    /* set output frame length                    */
    /********************************************/
    *pN = ( SKP_int16 )L;

    /* Update some decoder state variables */
    psDec->lagPrev = sDecCtrl.pitchL[ NB_SUBFR - 1 ];


    return ret;
}
