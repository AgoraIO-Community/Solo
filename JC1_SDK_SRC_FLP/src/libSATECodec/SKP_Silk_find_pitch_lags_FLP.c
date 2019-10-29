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

void SKP_Silk_find_pitch_lags_FLP(
    SKP_Silk_encoder_state_FLP      *psEnc,             /* I/O  Encoder state FLP                       */
    SKP_Silk_encoder_control_FLP    *psEncCtrl,         /* I/O  Encoder control FLP                     */
          SKP_float                 res[],              /* O    Residual                                */
    const SKP_float                 x[]                 /* I    Speech signal                           */
)
{
    SKP_Silk_predict_state_FLP *psPredSt = &psEnc->sPred;
    SKP_int   buf_len;
    SKP_float thrhld, res_nrg;
    const SKP_float *x_buf_ptr, *x_buf;
    SKP_float auto_corr[ MAX_FIND_PITCH_LPC_ORDER + 1 ];
    SKP_float A[         MAX_FIND_PITCH_LPC_ORDER ];
    SKP_float refl_coef[ MAX_FIND_PITCH_LPC_ORDER ];
    SKP_float Wsig[      FIND_PITCH_LPC_WIN_MAX ];
    SKP_float *Wsig_ptr;

    /******************************************/
    /* Setup buffer lengths etc based on Fs   */
    /******************************************/
    buf_len = 2 * psEnc->sCmn.frame_length + psEnc->sCmn.la_pitch;

    /* Safty check */
    SKP_assert( buf_len >= psPredSt->pitch_LPC_win_length );

    x_buf = x - psEnc->sCmn.frame_length;

    /******************************************/
    /* Estimate LPC AR coeficients            */
    /******************************************/
    
    /* Calculate windowed signal */
    
    /* First LA_LTP samples */
    x_buf_ptr = x_buf + buf_len - psPredSt->pitch_LPC_win_length;
    Wsig_ptr  = Wsig;
    SKP_Silk_apply_sine_window_FLP( Wsig_ptr, x_buf_ptr, 1, psEnc->sCmn.la_pitch );

    /* Middle non-windowed samples */
    Wsig_ptr  += psEnc->sCmn.la_pitch;
    x_buf_ptr += psEnc->sCmn.la_pitch;
    SKP_memcpy( Wsig_ptr, x_buf_ptr, ( psPredSt->pitch_LPC_win_length - ( psEnc->sCmn.la_pitch << 1 ) ) * sizeof( SKP_float ) );

    /* Last LA_LTP samples */
    Wsig_ptr  += psPredSt->pitch_LPC_win_length - ( psEnc->sCmn.la_pitch << 1 );
    x_buf_ptr += psPredSt->pitch_LPC_win_length - ( psEnc->sCmn.la_pitch << 1 );
    SKP_Silk_apply_sine_window_FLP( Wsig_ptr, x_buf_ptr, 2, psEnc->sCmn.la_pitch );

    /* Calculate autocorrelation sequence */
    SKP_Silk_autocorrelation_FLP( auto_corr, Wsig, psPredSt->pitch_LPC_win_length, psEnc->sCmn.pitchEstimationLPCOrder + 1 );

    /* Add white noise, as a fraction of the energy */
    auto_corr[ 0 ] += auto_corr[ 0 ] * FIND_PITCH_WHITE_NOISE_FRACTION;

    /* Calculate the reflection coefficients using Schur */
    res_nrg = SKP_Silk_schur_FLP( refl_coef, auto_corr, psEnc->sCmn.pitchEstimationLPCOrder );

    /* Prediction gain */
    psEncCtrl->predGain = auto_corr[ 0 ] / SKP_max_float( res_nrg, 1.0f );

    /* Convert reflection coefficients to prediction coefficients */
    SKP_Silk_k2a_FLP( A, refl_coef, psEnc->sCmn.pitchEstimationLPCOrder );

    /* Bandwidth expansion */
    SKP_Silk_bwexpander_FLP( A, psEnc->sCmn.pitchEstimationLPCOrder, FIND_PITCH_BANDWITH_EXPANSION );
    
    /*****************************************/
    /* LPC analysis filtering                */
    /*****************************************/
    SKP_Silk_LPC_analysis_filter_FLP( res, A, x_buf, buf_len, psEnc->sCmn.pitchEstimationLPCOrder );
    SKP_memset( res, 0, psEnc->sCmn.pitchEstimationLPCOrder * sizeof( SKP_float ) );

    /* Threshold for pitch estimator */
    thrhld  = 0.45f;
    thrhld -= 0.004f * psEnc->sCmn.pitchEstimationLPCOrder;
    thrhld -= 0.1f   * psEnc->speech_activity;
    thrhld += 0.15f  * psEnc->sCmn.prev_sigtype;
    thrhld -= 0.1f   * psEncCtrl->input_tilt;

    /*****************************************/
    /* Call Pitch estimator                  */
    /*****************************************/
    psEncCtrl->sCmn.sigtype = SKP_Silk_pitch_analysis_core_FLP( res, psEncCtrl->sCmn.pitchL, &psEncCtrl->sCmn.lagIndex, 
        &psEncCtrl->sCmn.contourIndex, &psEnc->LTPCorr, psEnc->sCmn.prevLag, psEnc->sCmn.pitchEstimationThreshold_Q16 / 65536.0f, 
        thrhld, psEnc->sCmn.fs_kHz, psEnc->sCmn.pitchEstimationComplexity );
}
