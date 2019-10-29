
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "AGR_JC1_SDK_API.h"

#ifdef _WIN32
# define SKP_STR_CASEINSENSITIVE_COMPARE(x, y) _stricmp(x, y)
#else
# define SKP_STR_CASEINSENSITIVE_COMPARE(x, y) strcasecmp(x, y)
#endif 


#pragma comment(lib,"libSATECodec_FLP.lib")
#pragma comment(lib,"libBWE.lib")




/* PSEUDO-RANDOM GENERATOR                                                          */
/* Make sure to store the result as the seed for the next call (also in between     */
/* frames), otherwise result won't be random at all. When only using some of the    */
/* bits, take the most significant bits by right-shifting. Do not just mask off     */
/* the lowest bits.                                                                 */ 
#define SKP_RAND(seed)                   ((907633515) + (seed) * (196314165))


#define  cmdControl

/* Define codec specific settings */
#define MAX_BYTES_PER_FRAME     250 // Equals peak bitrate of 100 kbps 
#define FRAME_LENGTH_MS         20
#define MAX_API_FS_KHZ          48
#define MAX_FRAME_SIZE 1920
#define MAX_FRAME_BYTES 1024

#define MD_NUM 2

static int rand_seed = 1;

static void print_usage(char* argv[]) {
	printf( "\nusage: %s in.bit out.pcm [settings]\n", argv[ 0 ] );
	printf( "\nstream.bit   : Bitstream input to decoder" );
	printf( "\nout.pcm      : Speech output from decoder" );
	printf( "\n   settings:" );
    printf( "\n-Fs_API <Hz> : Sampling rate of output signal in Hz; default: 16000" );
	printf( "\n-loss <perc> : Simulated packet loss percentage (0-100); default: 0" );
    printf( "\n-plc_type <0/1> : plc type\n" );
	printf( "\n" );
}

int main(int argc, char **argv)
{
  FILE *fin;
  FILE *fout;
  void *stDec;
  int frame = 0,j;
  int codec_type = 0;
  short FrmBuf[MAX_FRAME_SIZE];
  size_t    counter;
  int totPackets, lost, quiet = 0, i, k;
  short nBytes[6] = { 0, 0, 0, 0, 0, 0};
  unsigned char payload[MAX_FRAME_BYTES];
  unsigned char *payloadEnd = NULL, *payloadToDec = NULL;
  int loss_prob;
  int sate_plc = 1; // 1 for SATE PLC, 0 for SILK PLC
  int dec_mode = 0,MDI_in_bitstream = 0;
  int nwrite;
  int run_count = 0;
  int lostMD[8];
  int lostcnt = 0;
  int MD_type = 0;
  short nSamplesOut;

  USER_Ctrl_dec dec_Ctrl;

#ifndef cmdControl
  
  char *InFileName = "..\\Testseq\\Ch_f1_raw.pcm";
  char *MidFileName= "..\\Testseq\\ch8kall_16k.enc";
  char *OutFileName= "..\\Testseq\\Ch_f1_scodec.dec";

#else

  char InFileName[100];
  char OutFileName[100];
  int args;

  //default
  dec_Ctrl.packetLoss_perc = 0;
  dec_Ctrl.samplerate = 16000;
  dec_Ctrl.framesize_ms = 40;
  dec_Ctrl.joint_enable = 0;
  dec_Ctrl.joint_mode = 0;


  if( argc < 3 ) {
	  print_usage( argv );
	  exit( 0 );
  }

  /* get arguments */
  args = 1;
  strcpy( InFileName, argv[ args ] );
  args++;
  strcpy( OutFileName, argv[ args ] );
  args++;
  while( args < argc ) {
	  if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-loss" ) == 0 ) {
		  sscanf( argv[ args + 1 ], "%d", &dec_Ctrl.packetLoss_perc );
		  args += 2;
	  } else if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-Fs_API" ) == 0 ) {
		  sscanf( argv[ args + 1 ], "%d", &dec_Ctrl.samplerate );
		  args += 2;
	  } else if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-plc_type" ) == 0 ) {
		  sscanf( argv[ args + 1 ], "%d", &sate_plc );
		  args += 2;
	  } else if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-joint" ) == 0 ) {
	  	  dec_Ctrl.joint_enable = 1;
		  sscanf( argv[ args + 1 ], "%d", &dec_Ctrl.joint_mode);
		  args += 2;
	  } else if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-framesize" ) == 0 ) {
		  sscanf( argv[ args + 1 ], "%d", &dec_Ctrl.framesize_ms);
		  args += 2;
	  } else if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-dec_mode" ) == 0 ) {
		  sscanf( argv[ args + 1 ], "%d", &dec_mode );
		  args += 2;
      } else if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-MDI" ) == 0 ) {
            sscanf( argv[ args + 1 ], "%d", &MDI_in_bitstream );
            args += 2;
	  } else if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-quiet" ) == 0 ) {
		  quiet = 1;
		  args++;
	  } else {
		  printf( "Error: unrecognized setting: %s\n\n", argv[ args ] );
		  print_usage( argv );
		  exit( 0 );
	  }
  }

  if((dec_mode > 0) && (dec_Ctrl.packetLoss_perc > 0)) {
      printf("dec_mode is the test for the single stream decoding\n");
      printf("loss and dec_mode can't be set at the same time\n");
      exit(0);
  }

#endif

  if (dec_Ctrl.samplerate != 16000){
	  printf("only support wideband\n");
	  exit(0);
  }

  if((fin=fopen(InFileName,"rb"))==NULL) {
      printf("inputfile %s open error!\n",InFileName);
      exit(0);
    }
  if((fout=fopen(OutFileName,"wb"))==NULL) {
      printf("Outfile %s open error!\n",OutFileName);
      exit(0);
    }

  printf("inputfile :	%s\n",InFileName);
  printf("outputfile:	%s\n",OutFileName);
  

  if((dec_Ctrl.framesize_ms != 40)&&	
  	 (dec_Ctrl.framesize_ms != 80)&&	
  	 (dec_Ctrl.framesize_ms != 120)&&	
  	 (dec_Ctrl.framesize_ms != 160)){
		fprintf(stderr," framesize (ms) must be integer times of 40\n");
		return 0;
  	}
  
  if((dec_Ctrl.samplerate != 16000)&&	
  	 (dec_Ctrl.samplerate != 32000)&&	
  	 (dec_Ctrl.samplerate != 48000)){
		fprintf(stderr," sample rate (hz) must be 16000 , 32000 or 48000\n");
		return 0;
  	}

  
  dec_Ctrl.useMDIndex = MDI_in_bitstream;
  stDec = AGR_Sate_Decoder_Init(&dec_Ctrl);
  frame =0;
  
  nwrite = dec_Ctrl.framesize_ms*dec_Ctrl.samplerate/1000;

  /* default settings */
  loss_prob = dec_Ctrl.packetLoss_perc; // Packet loss percentage
  memset(payload, 0, sizeof(unsigned char)*MAX_FRAME_BYTES);

  totPackets = 0;
  payloadEnd = payload;
  payloadToDec = payloadEnd;

  k = 0;
  while( 1 ) {
	  for (i = 0; i< MD_NUM; i++) {
		  counter = fread(&nBytes[i], sizeof(short), 1, fin);
		  if (nBytes[i] < 0 || counter < 1) {
			  goto DecEnd;
		  }
	  }


	  /*

	  |-------|-------|---------------------|---------------------|---------------------|
	  | Byte0 | Byte1 | Low Band MD1 Stream | Low Band MD2 Stream |  High Band Stream   |
	  |-------|-------|---------------------|---------------------|---------------------|

	  MD1 Steam = Low Band MD1 Stream
	  MD2 Steam = Low Band MD2 Stream + High Band Stream
	  Length£¨ MD1 Steam £© =  Length£¨ Low Band MD1 Stream £©
	  Length£¨ MD2 Steam £© =  Length£¨ Low Band MD2 Stream £© + Length£¨ High Band Stream £©
	  Byte0 = Length£¨ MD1 Steam £© + Length£¨ MD2 Steam £© 
	        = Length£¨ Low Band MD1 Stream £© + Length£¨ Low Band MD2 Stream £© + Length£¨ High Band Stream £©
	  Byte1 = Length£¨ MD2 Steam £©

	  */

	  counter = fread(payloadEnd, sizeof(unsigned char), nBytes[0], fin);
	  
	  if( ( short )counter < nBytes[0] ) {
	    	break;
	  }


	  if (run_count % MD_NUM == 0) {
		  /* Simulate losses */
		  for (j = 0; j < MD_NUM; j++){
			  rand_seed = SKP_RAND(rand_seed);
			  if ((((float)((rand_seed >> 16) + (1 << 15))) / 65535.0f >= (loss_prob / 100.0f)) && (counter > 0)) {
			  	  if(nBytes[j] == 0)
				  	lostMD[j] = 1;
				  else
				  	lostMD[j] = 0;
			  }else {
				  lostMD[j] = 1;
				  lostcnt++;
			  }
		  }
	  }

#define _SIMU1_
	  if (run_count % MD_NUM == 0){
		  if ((lostMD[0] == 0) && (lostMD[1] == 0)) {
			  lost = 0;
			  payloadToDec = payload;
			  MD_type = 2;
		  }else if ((lostMD[0] == 0) && (lostMD[1] == 1)){

#ifdef _SIMU1_
			  lost = 0;
			  payloadToDec = payload;
#ifdef _MD1_WITH_HB_
			  memmove(payload + nBytes[0] - nBytes[1], payload + nBytes[0] - 4, 4 * sizeof(unsigned char));
			  nBytes[0] = nBytes[0] - nBytes[1] + 4;
#else
			  nBytes[0] = nBytes[0] - nBytes[1];
#endif
			  nBytes[1] = 0;
			  MD_type = 0;
#else
			  lost = 0;
			  payloadToDec = payload;
			  MD_type = 2;
#endif
		  }else if ((lostMD[0] == 1) && (lostMD[1] == 0)) {/* Loss the MD1 Steam ,use the MD2 Steam  */
			  lost = 0;
			  payloadToDec = payload + nBytes[0] - nBytes[1];  
			  nBytes[0] = nBytes[1];
			  nBytes[1] = 0;
			  MD_type = 1;
		  } else if ((lostMD[0] == 1) && (lostMD[1] == 1)) {
			  lost = 1;
		  }
	  }else{
		  if ((lostMD[0] == 0) && (lostMD[1] == 0)){
			  lost = 0;
			  payloadToDec = payload;
			  MD_type = 2;
		  } else if ((lostMD[0] == 0) && (lostMD[1] == 1)){/* Loss the MD2 Steam ,use the MD1 Steam */
			  lost = 0;
			  payloadToDec = payload;
#ifdef _MD1_WITH_HB_
			  memmove(payload + nBytes[0] - nBytes[1], payload + nBytes[0] - 4, 4 * sizeof(unsigned char));
			  nBytes[0] = nBytes[0] - nBytes[1] + 4;
#else
			  nBytes[0] = nBytes[0] - nBytes[1];
#endif
			  nBytes[1] = 0;
			  MD_type = 0;
		  }else if ((lostMD[0] == 1) && (lostMD[1] == 0)){
#ifdef _SIMU1_
			  lost = 0;
			  payloadToDec = payload + nBytes[0] - nBytes[1];
			  nBytes[0] = nBytes[1];
			  nBytes[1] = 0;
			  MD_type = 1;
#else
			  lost = 0;
			  payloadToDec = payload;
			  MD_type = 2;
#endif
		  }else if ((lostMD[0] == 1) && (lostMD[1] == 1)){
			  lost = 1;
		  }
	  }

	  run_count++;

	 if(dec_mode == 1){
		  lost = 0;
		  payloadToDec = payload;
#ifdef _MD1_WITH_HB_
		  memmove(payload + nBytes[0] - nBytes[1], payload + nBytes[0] - 4, 4 * sizeof(unsigned char));
		  nBytes[0] = nBytes[0] - nBytes[1] + 4;
#else
		  nBytes[0] = nBytes[0] - nBytes[1];
#endif
		  nBytes[1] = 0;
		  MD_type = 0;
	 }else if(dec_mode == 2){
		 lost = 0;
		 payloadToDec = payload + nBytes[0] - nBytes[1];  
		 nBytes[0] = nBytes[1];
		 nBytes[1] = 0;
		 MD_type = 1;
	 }

	if (lost == 0) {
		/* Loss: Decode enough frames to cover one packet duration */
		AGR_Sate_Decoder_Decode(stDec, FrmBuf, &nSamplesOut, payloadToDec, nBytes, (MD_type + 2));
	} else {
		// lostFlag
		/*
		* 0 : std silk
		* 1 : packet is lost
		* 2 : only the first MDC is decoded
		* 3 : only the second MDC is decoded
		* 4 : both two MDCs are decoded
		*/
		/* Loss: Decode enough frames to cover one packet duration */
		AGR_Sate_Decoder_Decode(stDec, FrmBuf, &nSamplesOut, payloadToDec, nBytes, 1);
	}

	  fwrite(FrmBuf, sizeof(short), nwrite, fout);

	  totPackets++;
	  printf("%d frames processed!\r", totPackets);
  }

   DecEnd:
 
   AGR_Sate_Decoder_Uninit(stDec);

   fclose(fin);
   fclose(fout);

   return 0;
}//end main()
