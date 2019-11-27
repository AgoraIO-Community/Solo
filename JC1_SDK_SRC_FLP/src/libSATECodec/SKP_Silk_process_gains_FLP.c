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

#include "SKP_Silk_main_FLP.h"
#include "SKP_Silk_tuning_parameters.h"



/* Processing of gains */
void SKP_Silk_process_gains_FLP(
    SKP_Silk_encoder_state_FLP      *psEnc,             /* I/O  Encoder state FLP                       */
    SKP_Silk_encoder_control_FLP    *psEncCtrl          /* I/O  Encoder control FLP                     */
)
{
    SKP_Silk_shape_state_FLP *psShapeSt = &psEnc->sShape;
    SKP_int     k;
    SKP_int32   pGains_Q16[ NB_SUBFR ];
    SKP_int32   pMDGains_Q16[ NB_SUBFR ];
    SKP_float   s, InvMaxSqrVal, gain, quant_offset;
	SKP_int32	Delta_Gains_Q16;
	SKP_float   tmp_float;

    /* Gain reduction when LTP coding gain is high */
    if( psEncCtrl->sCmn.sigtype == SIG_TYPE_VOICED ) {
        s = 1.0f - 0.5f * SKP_sigmoid( 0.25f * ( psEncCtrl->LTPredCodGain - 12.0f ) );
        for( k = 0; k < NB_SUBFR; k++ ) {   
            psEncCtrl->Gains[ k ] *= s;
        }
    }
	
    /* Limit the quantized signal */
    InvMaxSqrVal = ( SKP_float )( pow( 2.0f, 0.33f * ( 21.0f - psEncCtrl->current_SNR_dB ) ) / psEnc->sCmn.subfr_length );

    for( k = 0; k < NB_SUBFR; k++ ) {
        /* Soft limit on ratio residual energy and squared gains */
        gain = psEncCtrl->Gains[ k ];
        gain = ( SKP_float )sqrt( gain * gain + psEncCtrl->ResNrg[ k ] * InvMaxSqrVal );
        psEncCtrl->Gains[ k ] = SKP_min_float( gain, 32767.0f );
    }

    /* Prepare gains for noise shaping quantization */
    for( k = 0; k < NB_SUBFR; k++ ) {
        pGains_Q16[ k ] = ( SKP_int32 ) ( psEncCtrl->Gains[ k ] * 65536.0f ); 
    }

	if(psEnc->sCmn.md_enable == 1){
	    tmp_float = (1.0f/(float)psEncCtrl->md_delta_gain_par);
		tmp_float = SKP_LIMIT((tmp_float * 65536.0f), 131072.0f, -131072.0f);

		Delta_Gains_Q16 = SKP_float2int( tmp_float - (0.05 * 65536.0f));

		if(psEnc->sCmn.nFramesInPayloadBuf == 0){
			psEnc->sCmn.DeltaGain0_Q16 = Delta_Gains_Q16;
		}else{
			Delta_Gains_Q16 = psEnc->sCmn.DeltaGain0_Q16;
		}

		psEncCtrl->DeltaGains = (float)Delta_Gains_Q16/ 65536.0f;

        for( k = 0; k < NB_SUBFR; k++ ) {   
            psEncCtrl->MDGains[ k ] = psEncCtrl->Gains[ k ] * psEncCtrl->DeltaGains;
        }

	    for( k = 0; k < NB_SUBFR; k++ ) {
	        pMDGains_Q16[ k ] = ( SKP_int32 ) ( psEncCtrl->MDGains[ k ] * 65536.0f ); 
	    }
		
	    SKP_Silk_gains_quant( psEncCtrl->sCmn.GainsIndices, pGains_Q16, 
	            &psShapeSt->LastGainIndex, psEnc->sCmn.nFramesInPayloadBuf,
	            &psEncCtrl->sCmn.DeltaGainsIndices,&Delta_Gains_Q16,
	            &psEnc->sCmn.prev_delta_gain_index,
	            psEnc->sCmn.md_enable
	            );
	    /* Noise shaping quantization */
		
		psEncCtrl->DeltaGains = Delta_Gains_Q16/ 65536.0f;

	    for( k = 0; k < NB_SUBFR; k++ ) {   
	        psEncCtrl->MDGains[ k ] = pMDGains_Q16[ k ] / 65536.0f;
	    }
		
	    /* Overwrite unquantized gains with quantized gains and convert back to Q0 from Q16 */
	    for( k = 0; k < NB_SUBFR; k++ ) {
	        psEncCtrl->Gains[ k ] = pGains_Q16[ k ] / 65536.0f;
	    }

	}else{/* Noise shaping quantization */
	    SKP_Silk_gains_quant( psEncCtrl->sCmn.GainsIndices, pGains_Q16, 
	            &psShapeSt->LastGainIndex, psEnc->sCmn.nFramesInPayloadBuf 
				,&psEncCtrl->sCmn.DeltaGainsIndices,NULL,NULL,psEnc->sCmn.md_enable
	            );
	    /* Overwrite unquantized gains with quantized gains and convert back to Q0 from Q16 */
	    for( k = 0; k < NB_SUBFR; k++ ) {
	        psEncCtrl->Gains[ k ] = pGains_Q16[ k ] / 65536.0f;
	    }
	}

    /* Set quantizer offset for voiced signals. Larger offset when LTP coding gain is low or tilt is high (ie low-pass) */
    if( psEncCtrl->sCmn.sigtype == SIG_TYPE_VOICED ) {
        if( psEncCtrl->LTPredCodGain + psEncCtrl->input_tilt > 1.0f ) {
            psEncCtrl->sCmn.QuantOffsetType = 0;
        } else {
            psEncCtrl->sCmn.QuantOffsetType = 1;
        }
    }

    /* Quantizer boundary adjustment */
    quant_offset = SKP_Silk_Quantization_Offsets_Q10[ psEncCtrl->sCmn.sigtype ][ psEncCtrl->sCmn.QuantOffsetType ] / 1024.0f;
    psEncCtrl->Lambda = LAMBDA_OFFSET 
                      + LAMBDA_DELAYED_DECISIONS * psEnc->sCmn.nStatesDelayedDecision
                      + LAMBDA_SPEECH_ACT        * psEnc->speech_activity 
                      + LAMBDA_INPUT_QUALITY     * psEncCtrl->input_quality   
                      + LAMBDA_CODING_QUALITY    * psEncCtrl->coding_quality
                      + LAMBDA_QUANT_OFFSET      * quant_offset;

    SKP_assert( psEncCtrl->Lambda > 0.0f );
    SKP_assert( psEncCtrl->Lambda < 2.0f );
}
