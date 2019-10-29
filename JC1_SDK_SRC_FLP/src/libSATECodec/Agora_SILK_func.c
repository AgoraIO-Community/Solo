
#include "SKP_Silk_main.h"

/**********************/
/*Long term prediction*/
/**********************/
SKP_int32 Agora_Silk_LTP(
	SKP_int             sigtype,
	SKP_int32	  *pred_lag_ptr[], 
	const SKP_int16     b_Q14[]
)
{
	SKP_int32   LTP_pred_Q14;
	SKP_int j;
	
		/* Long-term prediction */

		LTP_pred_Q14 = 0;
	if( sigtype == SIG_TYPE_VOICED ) {
		/* Unrolled loop */
		for( j = 0; j < 5; j ++ ) {
			LTP_pred_Q14 = SKP_SMLAWB( LTP_pred_Q14, (*pred_lag_ptr)[ -j ], b_Q14[ j ] );
		}
		(*pred_lag_ptr)++;
	} else {
		LTP_pred_Q14 = 0;
	}


	return   LTP_pred_Q14;
}

/**********************/
/*Long term shaping*/
/**********************/
SKP_int32 Agora_Silk_LTS(
	SKP_int             lag,
	SKP_int32	  *shp_lag_ptr[], 
	SKP_int32    HarmShapeFIRPacked_Q14
)
{
	SKP_int32   n_LTP_Q14 = 0;
        /* Long-term shaping */
        if( lag > 0 ){
            /* Symmetric, packed FIR coefficients */
            n_LTP_Q14 = SKP_SMULWB( SKP_ADD32( (*shp_lag_ptr)[ 0 ], (*shp_lag_ptr)[ -2 ] ), HarmShapeFIRPacked_Q14 );
            n_LTP_Q14 = SKP_SMLAWT( n_LTP_Q14, (*shp_lag_ptr)[ -1 ],                     HarmShapeFIRPacked_Q14 );
            n_LTP_Q14 = SKP_LSHIFT( n_LTP_Q14, 6 );
            (*shp_lag_ptr)++;
        } else {
            n_LTP_Q14 = 0;
        }

	return   n_LTP_Q14;
}

/***********************/
/*Short term prediction*/
/***********************/
SKP_int32 Agora_Silk_STP(
	SKP_int32 			*sLPC_Q14,
	SKP_int             predictLPCOrder,
	const SKP_int16     a_Q12[]
)
{
	SKP_int32   LPC_pred_Q10;
	SKP_int j;
	SKP_int32	  *psLPC_Q14; /* Pointer used in short term prediction and shaping */
    psLPC_Q14 = sLPC_Q14;
    SKP_assert( predictLPCOrder >= 10 );            /* check that unrolling works */
    SKP_assert( ( predictLPCOrder  & 1 ) == 0 );    /* check that order is even */
    SKP_assert( ( ( ( int )( ( char* )( a_Q12 ) - ( ( char* ) 0 ) ) ) & 3 ) == 0 );    /* check that array starts at 4-byte aligned address */
	/* Song-term prediction */
    LPC_pred_Q10 = 0;
	for( j = 0; j < predictLPCOrder; j ++ ) {
		LPC_pred_Q10 = SKP_SMLAWB( LPC_pred_Q10, psLPC_Q14[ -j ], a_Q12[ j ] );
	}
	return LPC_pred_Q10;
}


/***********************/
/*Short term shaping*/
/***********************/
SKP_int32 Agora_Silk_STS(
	SKP_int32 			*sLPC_Q14,
	SKP_int32 			*sAR2_Q14,
	SKP_int32           LF_AR_Q12,
	SKP_int             shapingLPCOrder,
	SKP_int             warping_Q16,
	const SKP_int16     AR_shp_Q13[],
	SKP_int   			Tilt_Q14
)
{
	SKP_int32   tmp1, tmp2,n_AR_Q10;
	SKP_int j;
	SKP_int32   psLPC_Q14;
    /* Pointer used in short term prediction and shaping */
    psLPC_Q14 = sLPC_Q14[0];
	 
	/* Noise shape feedback */
	SKP_assert( ( shapingLPCOrder & 1 ) == 0 );   /* check that order is even */
	/* Output of lowpass section */
	tmp2 = SKP_SMLAWB( psLPC_Q14, sAR2_Q14[ 0 ], warping_Q16 );
	/* Output of allpass section */
	tmp1 = SKP_SMLAWB( sAR2_Q14[ 0 ], sAR2_Q14[ 1 ] - tmp2, warping_Q16 );
	sAR2_Q14[ 0 ] = tmp2;
	n_AR_Q10 = SKP_SMULWB( tmp2, AR_shp_Q13[ 0 ] );
	/* Loop over allpass sections */
	for( j = 2; j < shapingLPCOrder; j += 2 ) {
		/* Output of allpass section */
		tmp2 = SKP_SMLAWB( sAR2_Q14[ j - 1 ], sAR2_Q14[ j + 0 ] - tmp1, warping_Q16 );
		sAR2_Q14[ j - 1 ] = tmp1;
		n_AR_Q10 = SKP_SMLAWB( n_AR_Q10, tmp1, AR_shp_Q13[ j - 1 ] );
		/* Output of allpass section */
		tmp1 = SKP_SMLAWB( sAR2_Q14[ j + 0 ], sAR2_Q14[ j + 1 ] - tmp2, warping_Q16 );
		sAR2_Q14[ j + 0 ] = tmp2;
		n_AR_Q10 = SKP_SMLAWB( n_AR_Q10, tmp2, AR_shp_Q13[ j ] );
	}
	sAR2_Q14[ shapingLPCOrder - 1 ] = tmp1;
	n_AR_Q10 = SKP_SMLAWB( n_AR_Q10, tmp1, AR_shp_Q13[ shapingLPCOrder - 1 ] );
	
	n_AR_Q10 = SKP_RSHIFT( n_AR_Q10, 1 );			/* Q11 -> Q10 */
    n_AR_Q10 = SKP_SMLAWB( n_AR_Q10, LF_AR_Q12, Tilt_Q14 );	  
	
	return   (n_AR_Q10);
}

SKP_int32 Agora_Silk_LFS(
	SKP_int32 Shape_Q10,
	SKP_int32 LF_AR_Q12,
	SKP_int32 n_AR_Q10,
	SKP_int32 LF_shp_Q14
	)
{
	SKP_int32 n_LF_Q10;
	n_LF_Q10 = SKP_LSHIFT( SKP_SMULWB( Shape_Q10, LF_shp_Q14 ), 2 ); 
	n_LF_Q10 = SKP_SMLAWT( n_LF_Q10, LF_AR_Q12, LF_shp_Q14 ); 	  

	return n_LF_Q10;
}

SKP_int32 Agora_Silk_DoPred_And_Shap(
	const SKP_int32 x_Q10,
	SKP_int32 LTP_pred_Q14,
	SKP_int32 LPC_pred_Q10,
	SKP_int32 n_LTP_Q14,
	SKP_int32 n_AR_Q10,
	SKP_int32 n_LF_Q10
	)
{
	SKP_int32 tmp1,r_Q10;
	/* Input minus prediction plus noise feedback						*/
	/* r = x[ i ] - LTP_pred - LPC_pred + n_AR + n_Tilt + n_LF + n_LTP	*/
	tmp1  = SKP_SUB32( LTP_pred_Q14, n_LTP_Q14 );						/* Add Q14 stuff */
	tmp1  = SKP_RSHIFT( tmp1, 4 );										/* convert to Q10 */
	tmp1  = SKP_ADD32( tmp1, LPC_pred_Q10 );							/* add Q10 stuff */ 
	tmp1  = SKP_SUB32( tmp1, n_AR_Q10 );								/* subtract Q10 stuff */ 
	tmp1  = SKP_SUB32( tmp1, n_LF_Q10 );								/* subtract Q10 stuff */ 
	r_Q10 = SKP_SUB32( x_Q10, tmp1 );								/* residual error Q10 */

	return r_Q10;
}

SKP_int32 Agora_Silk_DoPred(
	const SKP_int32 x_Q10,
	SKP_int32 LTP_pred_Q14,
	SKP_int32 LPC_pred_Q10
	)
{
	SKP_int32 tmp1,r_Q10;
	/* Input minus prediction plus noise feedback						*/
	/* r = x[ i ] - LTP_pred - LPC_pred + n_AR + n_Tilt + n_LF + n_LTP	*/
	tmp1  = SKP_SUB32( LTP_pred_Q14, 0 );						/* Add Q14 stuff */
	tmp1  = SKP_RSHIFT( tmp1, 4 );										/* convert to Q10 */
	tmp1  = SKP_ADD32( tmp1, LPC_pred_Q10 );							/* add Q10 stuff */ 
	r_Q10 = SKP_SUB32( x_Q10, tmp1 );								/* residual error Q10 */

	return r_Q10;
}

SKP_int32 Agora_Silk_DoShap(
	const SKP_int32 d_Q10,
	SKP_int32 n_LTP_Q14,
	SKP_int32 n_AR_Q10,
	SKP_int32 n_LF_Q10
	)
{
	SKP_int32 tmp1,r_Q10;
	/* Input minus prediction plus noise feedback						*/
	/* r = x[ i ] - LTP_pred - LPC_pred + n_AR + n_Tilt + n_LF + n_LTP	*/
	tmp1  = SKP_SUB32( 0, n_LTP_Q14 );						/* Add Q14 stuff */
	tmp1  = SKP_RSHIFT( tmp1, 4 );										/* convert to Q10 */
	tmp1  = SKP_ADD32( tmp1, 0 );							/* add Q10 stuff */ 
	tmp1  = SKP_SUB32( tmp1, n_AR_Q10 );								/* subtract Q10 stuff */ 
	tmp1  = SKP_SUB32( tmp1, n_LF_Q10 );								/* subtract Q10 stuff */ 
	r_Q10 = SKP_SUB32( d_Q10, tmp1 );								/* residual error Q10 */

	return r_Q10;
}

