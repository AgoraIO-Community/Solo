
#include "AGR_BWE_tables_highband_coeff.h"
#include "AGR_BWE_main_FIX.h"
#include "SKP_Silk_macros.h"
#include "SKP_Silk_SigProc_FIX.h"

#include <stdio.h>
#include <math.h>

#define VERY_LARGE32 1e15f
/* 
R. Laroia, N. Phamdo and N. Farvardin, "Robust and Efficient Quantization of Speech LSP
Parameters Using Structured Vector Quantization", Proc. IEEE Int. Conf. Acoust., Speech,
Signal Processing, pp. 641-644, 1991.
*/

#define MIN_NDELTA                  1e-4f
#define SKP_min_float(a, b)			(((a) < (b)) ? (a) :  (b)) 
#define SKP_max_float(a, b)			(((a) > (b)) ? (a) :  (b)) 
#define SKP_abs_float(a)			((SKP_float)fabs(a))


static SKP_int32 lsp_quant_fix(SKP_int32 *lsp, const SKP_int16 *cdbk, SKP_int32 nbVec, SKP_int32 dim)
{
    SKP_int32 i, j, tmp;
    SKP_int32 dist, min_dist;
    SKP_int32 idx=0;

    min_dist = SKP_int32_MAX;
    for (i = 0; i < nbVec; i++)
    {
        dist = 0;
        for (j = 0; j < dim; j++)
        {
            tmp = lsp[j] - cdbk[i * dim + j];
            dist = SKP_SMLABB(dist, tmp, tmp);
        }
        if (dist < min_dist)
        {
            min_dist = dist;
            idx = i;
        }
    }

    for (i = 0; i < dim; i++)
        lsp[i] -= cdbk[idx * dim + i];

    return idx;
}


static int lsp_weight_quant_fix(SKP_int32 *x, SKP_int32 *weight, const SKP_int16 *cdbk, int nbVec, int nbDim)
{
    int i, j;
    SKP_int32 dist;
    SKP_int32 tmp;
    SKP_int32 best_dist = SKP_int32_MAX;
    int best_id = 0;
    const SKP_int16 *ptr = cdbk;
    for (i = 0; i < nbVec; i++)
    {
        dist = 0;
        for (j = 0; j < nbDim; j++)
        {
            tmp = SKP_SUB32(x[j], *ptr++);
            dist = SKP_SMLAWB(dist, SKP_SMULBB(tmp, tmp), weight[j]);
        }
        if (dist<best_dist)
        {
            best_dist = dist;
            best_id = i;
        }
    }

    for (j = 0; j<nbDim; j++)
        x[j] = SKP_SUB32(x[j], cdbk[best_id*nbDim + j]);

    return best_id;
}

SKP_int32 AGR_Sate_lsp_quant_highband(
    SKP_int32 *lsp,                                     /* I/O  lsp coefficients q15    */
    SKP_int32 order                                     /* I    lpc order               */
)
{
	SKP_int32 i;
	SKP_int32 idx1,idx2,idx;
    SKP_int32 weight[MAX_LPC_ORDER];
    
    SKP_Silk_NLSF_VQ_weights_laroia(weight, lsp, order);

    idx1 = lsp_quant_fix(lsp, AGR_Sate_highband_lsp_cdbk1_fix, HB_LSP_CB1, order);

    idx2 = lsp_weight_quant_fix(lsp, weight, AGR_Sate_highband_lsp_cdbk2_fix, HB_LSP_CB2, order);


	for (i = 0; i < order; i++)
        lsp[i] = SKP_ADD32(AGR_Sate_highband_lsp_cdbk1_fix[idx1 * order + i], 
                            AGR_Sate_highband_lsp_cdbk2_fix[idx2 * order + i]);
    
	idx = SKP_ADD32(SKP_LSHIFT32(idx2, 8), idx1);

	return idx;
}

void AGR_Sate_lsp_dequant_highband(
    SKP_int32 *qlsp,                                    /* O    output lsp coefficients */
    SKP_int32 idx,                                      /* I    quantized index         */
    SKP_int32 order                                     /* I    lpc order               */
)
{
	SKP_int32 i, idx1, idx2;

	idx1 = idx & 0x00FF;
	idx2 = (idx>>8);

	for (i=0; i<order; i++)
	{
		qlsp[i] = AGR_Sate_highband_lsp_cdbk1_fix[idx1*order+i] + AGR_Sate_highband_lsp_cdbk2_fix[idx2*order+i];
	}
}

SKP_int32 AGR_Sate_gain_quant_highband_fix(
    SKP_int16 gain,                               /* I unquantized gain           */
    const SKP_int16 *cdbk,                        /* I code book                  */
    SKP_int32 nVect                               /* I book size                  */
)
{
    SKP_int32 i, idx;
    SKP_int32 min_dist, dist;
    SKP_int16 tmp;

    min_dist = SKP_int32_MAX;
    idx = 0;
    for (i = 0; i < nVect; i++)
    {
        tmp = gain - cdbk[i];
        dist = SKP_SMULBB(tmp, tmp);
        if (dist < min_dist)
        {
            min_dist = dist;
            idx = i;
        }
    }

    return idx;
}
