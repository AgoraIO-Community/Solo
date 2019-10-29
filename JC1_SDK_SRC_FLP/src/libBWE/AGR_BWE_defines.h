#ifndef AGR_Sate_DEFINES_H
#define AGR_Sate_DEFINES_H

#ifdef __cplusplus
extern "C"
{
#endif

#define BWE_FRAMEINMS 20
#define SATE_FRAMEINMS 20
#define HB_LPC_FRAMEINMS 10
#define LB_SUB_FRAME_NUM 4
#define HB2LB_NUM 2
#define HB_SUB_FRAME_NUM LB_SUB_FRAME_NUM 
	//(HB2LB_NUM*LB_SUB_FRAME_NUM)

#define  HB_SUBFR         4
#define  HB_LPCFR         4

#ifndef NULL
#define NULL 0
#endif
#define MAX_GAIN_NUM 32	
#define MAX_PAYLOAD_SIZE 1024

#define SILK_HIGHBAND_REGEN

#define HB_LPC_ORDER                        8
#define HB_LSP_CB1                          256
#define HB_LSP_CB2                          16
#define HB_GAIN_CB		                    32

#define MAX_INTERNAL_FRAME_SIZE             1920
#define MAX_INTERNAL_HALF_FRAME_SIZE        960
#define MAX_SUBFR_SIZE                      320
#define FOLDING_GAIN	                    0.7

#define MAX_LPC_ORDER                       16
#define NB_SUBFR                            4
#define QMF_ORDER                           64
#define QMF_ORDER1                          64

#define HB_BYTE                             4

#define MAX_BYTES_PER_FRAME                 250
#define MAX_INPUT_FRAMES                    5

enum {
	AGR_SATE_ENCODE_SET_SAMPLERATE         = 1,
	AGR_SATE_ENCODE_SET_INTERNAL_SAMPLERATE ,       
	AGR_SATE_ENCODE_SET_PACKETSIZE         , 
	AGR_SATE_ENCODE_SET_PACKLOSSRATE        ,  
	AGR_SATE_ENCODE_SET_FEC         , 
	AGR_SATE_ENCODE_SET_DTX         , 
	AGR_SATE_ENCODE_SET_COMPLEXITY    ,      
	AGR_SATE_ENCODE_SET_BITRATE     , 
	AGR_SATE_ENCODE_SET_JC1_FULLFRAMESIZE,
	AGR_SATE_ENCODE_SET_QMF_LB_FRAMESIZE,
	AGR_SATE_ENCODE_SET_QMF_HB_FRAMESIZE ,
	
	AGR_SATE_ENCODE_SET_BWE_HB_FREQ,
	AGR_SATE_ENCODE_SET_BWE_FRAMESIZE,
	AGR_SATE_ENCODE_SET_BWE_FIRST_FRAME ,
	AGR_SATE_ENCODE_SET_BWE_LPCORDER,
	AGR_SATE_ENCODE_SET_BWE_LPCFRAMESIZE,
	AGR_SATE_ENCODE_SET_BWE_SUBFRAMESIZE
};

enum {
	AGR_SATE_DECODE_SET_SAMPLERATE         = 1,
	AGR_SATE_DECODE_SET_FRAMEPERPACKET,
	AGR_SATE_DECODE_SET_JC1_FULLFRAMESIZE ,
	AGR_SATE_DECODE_SET_QMF_HB_FRAMESIZE,
	AGR_SATE_DECODE_SET_QMF_LB_FRAMESIZE,
	AGR_SATE_DECODE_SET_BWE_FIRST_FRAME ,
	AGR_SATE_DECODE_SET_BWE_LPCSIZE,
	AGR_SATE_DECODE_SET_BWE_SUBFRAMESIZE
};

#ifdef __cpluscplus
}
#endif

#endif