#ifndef AGR_JC1_SDK_API_H
#define AGR_JC1_SDK_API_H

#include "SKP_Silk_typedef.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct  
{
	SKP_int32 mode;             //0,1,2;default:2
	SKP_int32 targetRate_bps;   //default:15600
	SKP_int32 samplerate;       //default:16000
	SKP_int32 dtx_enable;             //0,1 default:0
	SKP_int32 framesize_ms;           //20,40 default:40
	SKP_int32 joint_enable;           //20,40 default:40
	SKP_int32 joint_mode;             //20,40 default:40
	SKP_int32 useMDIndex;
} USER_Ctrl_enc;

typedef struct  
{
	SKP_int32 packetLoss_perc;  //0-100;default:0
	SKP_int32 samplerate;       //default:16000
	SKP_int32 framesize_ms;           //20,40 default:40
	SKP_int32 joint_enable;           //20,40 default:40
	SKP_int32 joint_mode;             //20,40 default:40
	SKP_int32 useMDIndex;
} USER_Ctrl_dec;

void *AGR_Sate_Encoder_Init(
    USER_Ctrl_enc *enc_Ctrl         /* I/O  Encoder state */
);

SKP_int32 AGR_Sate_Encoder_Encode(
    void *SATEEnc_State,            /* I/O  Encoder state       */
    const SKP_int16 *AGR_Sate_PCM,      /* I    Input signal        */
    SKP_uint8* AGR_Sate_Bit,            /* O    Encoded stream      */
    SKP_int32 AGR_Sate_Buf_Size,        /* I    Maximum output bits */
    SKP_int16 *nBytesOut            /* O    Output size         */
);

int AGR_Sate_Encoder_Uninit(
    void *SATEEnc_State             /* I/O  Encoder state       */
);

void *AGR_Sate_Decoder_Init(
    USER_Ctrl_dec *dec_Ctrl         /* I/O  Decoder state       */
);

SKP_int32 AGR_Sate_Decoder_Decode(
    void *SATEDec_State,            /* I/O  Decoder state       */
    SKP_int16 *AGR_Sate_PCM,            /* O    Output signal       */
    SKP_int16 *nSamplesOut,         /* O    Decoded frame size  */
    const SKP_uint8* AGR_Sate_Bit,      /* I    Input bits          */
    SKP_int16 nBytes[],             /* I    Input bits size     */
    SKP_int32 lostflag              /* I    lost flag           */
);

SKP_int32 AGR_Sate_Decoder_Uninit(
    void *SATEDec_State             /* I/O  Decoder state       */
);

#ifdef __cplusplus
}
#endif

#endif