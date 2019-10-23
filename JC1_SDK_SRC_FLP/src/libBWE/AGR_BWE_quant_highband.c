
#include "AGR_BWE_tables_highband_coeff.h"
#include "AGR_BWE_main_FLP.h" 

#include <stdio.h>
#include <math.h>

#define VERY_LARGE32 1e15f
/* 
R. Laroia, N. Phamdo and N. Farvardin, "Robust and Efficient Quantization of Speech LSP
Parameters Using Structured Vector Quantization", Proc. IEEE Int. Conf. Acoust., Speech,
Signal Processing, pp. 641-644, 1991.
*/

#define MIN_NDELTA                  1e-4f

#define SKP_max_float(a, b)			(((a) > (b)) ? (a) :  (b)) 

/* Laroia low complexity NLSF weights */
static void AGR_Sate_NLSF_VQ_weights_laroia_FLP( 
          SKP_float     *pXW,           /* 0: Pointer to input vector weights           [D x 1] */
    const SKP_float     *pX,            /* I: Pointer to input vector                   [D x 1] */ 
    const SKP_int32        D              /* I: Input vector dimension                            */
)
{
    SKP_int32 k;
    SKP_float tmp1, tmp2;
    
    /* Safety checks */
    SKP_assert( D > 0 );
    SKP_assert( ( D & 1 ) == 0 );
    
    /* First value */
    tmp1 = 1.0f / SKP_max_float( pX[ 0 ],           MIN_NDELTA );
    tmp2 = 1.0f / SKP_max_float( pX[ 1 ] - pX[ 0 ], MIN_NDELTA );
    pXW[ 0 ] = tmp1 + tmp2;
    
    /* Main loop */
    for( k = 1; k < D - 1; k += 2 ) {
        tmp1 = 1.0f / SKP_max_float( pX[ k + 1 ] - pX[ k ], MIN_NDELTA );
        pXW[ k ] = tmp1 + tmp2;

        tmp2 = 1.0f / SKP_max_float( pX[ k + 2 ] - pX[ k + 1 ], MIN_NDELTA );
        pXW[ k + 1 ] = tmp1 + tmp2;
    }
    
    /* Last value */
    tmp1 = 1.0f / SKP_max_float( 1.0f - pX[ D - 1 ], MIN_NDELTA );
    pXW[ D - 1 ] = tmp1 + tmp2;
}


static SKP_int32 lsp_weight_quant(SKP_float *x, SKP_float *weight, const SKP_float *cdbk, SKP_int32 nbVec, SKP_int32 nbDim)
{
    SKP_int32 i,j;
    SKP_float dist;
    SKP_float tmp;
    SKP_float best_dist = VERY_LARGE32;
    SKP_int32 best_id = 0;

	for (i=0; i<nbVec; i++){
		dist=0;
		for (j=0; j<nbDim; j++){
			tmp = x[j] - cdbk[i*nbDim+j];
			dist = dist + weight[j]*tmp*tmp;
		}
		if (dist < best_dist){
			best_dist = dist;
			best_id = i;
		}
	}

	for (j=0; j<nbDim; j++){
		x[j] = x[j] - cdbk[best_id*nbDim+j];
	}

	return best_id;
}

SKP_int32 AGR_Sate_lsp_quant_highband(
    SKP_float *lsp,                                     /* I/O  lsp coefficients        */
    SKP_int32 order                                     /* I    lpc order               */
)
{
	SKP_int32 i;
	SKP_int32 idx1,idx2,idx;
	SKP_float quant_weight1[MAX_LPC_ORDER],quant_weight2[MAX_LPC_ORDER];
	SKP_float qlsp[MAX_LPC_ORDER];

	for (i=0; i<order; i++){
		quant_weight1[i] = 1.0f;
	}

	AGR_Sate_NLSF_VQ_weights_laroia_FLP(quant_weight2,lsp,order);

	for (i=0; i<order; i++){
		qlsp[i] = lsp[i];
	}

	idx1 = lsp_weight_quant(qlsp, quant_weight1, AGR_Sate_highband_lsp_cdbk1, HB_LSP_CB1, order);

	idx2 = lsp_weight_quant(qlsp, quant_weight2, AGR_Sate_highband_lsp_cdbk2, HB_LSP_CB2, order);


	for (i=0;i<order;i++){
		lsp[i] = AGR_Sate_highband_lsp_cdbk1[idx1*order+i] + AGR_Sate_highband_lsp_cdbk2[idx2*order+i];
	}

	idx = (idx2<<8)+idx1;

	return idx;
}

void AGR_Sate_lsp_dequant_highband(
    SKP_float *qlsp,                                    /* O    output lsp coefficients */
    SKP_int32 idx,                                      /* I    quantized index         */
    SKP_int32 order                                     /* I    lpc order               */
)
{
	SKP_int32 i, idx1, idx2;

	idx1 = idx & 0x00FF;
	idx2 = (idx>>8);

	for (i=0; i<order; i++){
		qlsp[i] = AGR_Sate_highband_lsp_cdbk1[idx1*order+i] + AGR_Sate_highband_lsp_cdbk2[idx2*order+i];
	}
}


SKP_int32 AGR_Sate_gain_quant_highband(
    SKP_float gain,                               /* I unquantized gain           */
    const SKP_float *cdbk,                        /* I code book                  */
    SKP_int32 nVect                               /* I book size                  */
)
{
	SKP_int32 i,idx;
	SKP_float min_dist,dist;

	min_dist = 1e15f;
	idx = 0;
	for (i=0; i<nVect; i++){
		dist = (gain - cdbk[i]) * (gain - cdbk[i]);
		if (dist < min_dist){
			min_dist = dist;
			idx = i;
		}	
	}

	return idx;
}
