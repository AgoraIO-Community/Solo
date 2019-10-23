


#include "AGR_BWE_main_FIX.h"

SKP_int32 AGR_Sate_spsk_encoder_init(
    SATEEncCtl *sateCtl                                /*  I/O  SATE Encoder state      */
)
{
	SKP_int32  encSizeBytes;
    SKP_int32  ret;
	SKP_SILK_SDK_EncControlStruct  *SilkEncControl;

    sateCtl->stHBEnc = malloc(sizeof(AGR_Sate_encoder_hb_state_FIX));

    memset(sateCtl->stHBEnc, 0, sizeof(AGR_Sate_encoder_hb_state_FIX));

	SilkEncControl = &sateCtl->encControl;

	/* Create lowband silk Encoder &init */
	ret = SKP_Silk_SDK_Get_Encoder_Size(&encSizeBytes);

	sateCtl->stEnc = malloc(encSizeBytes);
	memset(sateCtl->stEnc, 0, encSizeBytes);
	/* Reset Encoder */
	ret = SKP_Silk_SDK_InitEncoder(sateCtl->stEnc, SilkEncControl);

	sateCtl->HBencControl.lb_Delay = SKP_Silk_SDK_Get_Encoder_Delay();
	return ret;
}

SKP_int32 AGR_Sate_spsk_decoder_init(
    SATEDecCtl *sateCtl                                /*  I/O  SATE Decoder state      */
)
{
    SKP_int32 decSizeBytes;
    SKP_int32  ret;

    /* Create spsk lowband decoder */
    ret = SKP_Silk_SDK_Get_Decoder_Size(&decSizeBytes);

    sateCtl->stDec = malloc(decSizeBytes);
    sateCtl->stHBDec = malloc(sizeof(AGR_Sate_decoder_hb_state_FIX));

	memset(sateCtl->stHBDec, 0, sizeof(AGR_Sate_decoder_hb_state_FIX));

	ret = SKP_Silk_SDK_InitDecoder(sateCtl->stDec,1);

    return ret;
}

void AGR_Sate_spsk_encoder_uninit(
    SATEEncCtl *st                                      /* I/O  SATE Encoder state      */
)
{
    if (st)
    {
        if (st->stEnc)
            free(st->stEnc);
        if (st->stHBEnc)
            free(st->stHBEnc);
    }
}

void AGR_Sate_spsk_decoder_uninit(
    SATEDecCtl *st                                      /* I/O  SATE Decoder state      */
)
{
    if (st)
    {
        if (st->stDec)
            free(st->stDec);
        if (st->stHBDec)
            free(st->stHBDec);
    }
}




		

		
		
