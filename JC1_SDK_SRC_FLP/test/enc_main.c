#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "AGR_JC1_SDK_API.h"


#pragma comment(lib,"libSATECodec_FLP.lib")
#pragma comment(lib,"libBWE.lib")


#define MAX_FRAME_SIZE 1920
#define  MAX_FRAME_BYTES 1024

/* Define codec specific settings */
#define MAX_BYTES_PER_FRAME     250 // Equals peak bitrate of 100 kbps
#define MAX_INPUT_FRAMES        5
#define MAX_LBRR_DELAY          2
#define MAX_FRAME_LENGTH        480
#define FRAME_LENGTH_MS         20
#define MAX_API_FS_KHZ          48

#ifdef _WIN32
# define SKP_STR_CASEINSENSITIVE_COMPARE(x, y) _stricmp(x, y)
#else
# define SKP_STR_CASEINSENSITIVE_COMPARE(x, y) strcasecmp(x, y)
#endif


static void print_usage( char* argv[] ) {
	printf( "\nusage: %s in.pcm out.bit [settings]\n", argv[ 0 ] );
	printf( "\nin.pcm               : Speech input to encoder" );
	printf( "\nstream.bit           : Bitstream output from encoder" );
	printf( "\n   settings:" );
    printf( "\n-Fs_API <Hz>         : API sampling rate in Hz, default: 16000" );
    printf( "\n-framesize <ms>   : Packet interval in ms, default: 20" );
	printf( "\n-rate <bps>          : Target bitrate; default: 16000" );
	printf( "\n-mode <mode_flag>    : Set mode, 0: LC(Low Complexity,2 layers), 1: HQ(High Quanlity,2 layers), 2: HQ-Scalable(3 layers); default: 1" );
	printf( "\n-DTX <0/1>          : Set 0: DTX disable, 1: DTX enable default: 0" );
	printf( "\n-quiet               : Print only some basic values" );
    printf( "\n-MDI <flag>          : Enable Write MD Index (0/1); default: 0" );
	printf( "\n");
}

int main(int argc, char **argv)
{

	FILE *fin;
	FILE *fout;
	void *stEnc;

	int k,  totPackets, totActPackets;
	double    sumBytes, sumCoreBytes, sumActBytes, avg_rate, avg_md2_rate, act_rate, nrg, sumBytes2;//,avg_rate2;

	unsigned char cbits[MAX_FRAME_BYTES];


	int frame = 0;
	short FrmBuf[MAX_FRAME_SIZE];

	/* default settings */
	int API_fs_Hz = 16000;
	int packetSize_ms;
	int quiet = 0;
	int smplsSinceLastPacket;
	short nBytes[6] = {0,0,0,0,0,0};
	int WriteMDI = 0;
	short nBytesX;
	short nread;
	USER_Ctrl_enc enc_Ctrl;

	char InFileName[100];
	char OutFileName[100];
	int args;

	//default
	enc_Ctrl.mode = 2;
	enc_Ctrl.targetRate_bps = 13600;
	enc_Ctrl.samplerate = API_fs_Hz;
    enc_Ctrl.dtx_enable = 0;
    enc_Ctrl.joint_enable = 0;
	enc_Ctrl.joint_mode = 0;
	enc_Ctrl.framesize_ms = 40;

	if( argc < 3 ) {
		print_usage( argv );
		exit( 0 );
	}


	/* get arguments */
	args = 1;
	strcpy( InFileName, argv[ args ] );
	args++;
	strcpy( OutFileName,   argv[ args ] );
	args++;
	while( args < argc ) {
		if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-rate" ) == 0 ) {
			sscanf( argv[ args + 1 ], "%d", &enc_Ctrl.targetRate_bps );
			args += 2;
		} else if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-FS_API" ) == 0 ) {
			sscanf( argv[ args + 1 ], "%d", &enc_Ctrl.samplerate );
			args += 2;
		} else if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-mode" ) == 0 ) {
			sscanf( argv[ args + 1 ], "%d", &enc_Ctrl.mode );
			args += 2;
		} else if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-DTX" ) == 0 ) {
			sscanf( argv[ args + 1 ], "%d", &enc_Ctrl.dtx_enable );
			args += 2;
		} else if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-quiet" ) == 0 ) {
			quiet = 1;
			args++;
		} else if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-framesize" ) == 0 ) {
			sscanf( argv[ args + 1 ], "%d", &enc_Ctrl.framesize_ms);
			args += 2;
		} else if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-joint" ) == 0 ) {
			enc_Ctrl.joint_enable = 1;
			sscanf( argv[ args + 1 ], "%d", &enc_Ctrl.joint_mode );
			args += 2;
        } else if( SKP_STR_CASEINSENSITIVE_COMPARE( argv[ args ], "-MDI") == 0 ) {
            sscanf( argv[ args + 1 ], "%d", &WriteMDI );
            args += 2;
		} else {
			printf( "Error: unrecognized setting: %s\n\n", argv[ args ] );
			print_usage( argv );
			exit( 0 );
		}
	}
  /*
  		joint disable
  		20ms 2MD
  		40ms 2MD
		joint mode 0  reserve for low band multiframe joint coding
		joint mode 1  high band multiframe joint coding method 0
		joint mode 2  reserve for high band multiframe joint coding method 1 (LPC and Res)
		joint mode 3  reserve for low band and high band all multiframe joint coding
					  reserve for 1. low band multiframe joint coding is OK
					              2. select a better joint method in 0 or 1 of high band

  */

	if(enc_Ctrl.samplerate != 16000)
	{
		printf("Sate support WB only\n");
		return 0;
	}

	if((fin=fopen(InFileName,"rb"))==NULL)
	{
		printf("inputfile %s open error!\n",InFileName);
		exit(0);
	}
	if((fout=fopen(OutFileName,"wb"))==NULL)
	{
		printf("Outfile %s open error!\n",OutFileName);
		exit(0);
	}

	printf("inputfile :	%s\n",InFileName);
	printf("outputfile:	%s\n",OutFileName);

	totPackets           = 0;
	totActPackets        = 0;
	smplsSinceLastPacket = 0;
	sumBytes             = 0.0;
	sumBytes2            = 0.0;
	sumCoreBytes         = 0.0;
	sumActBytes          = 0.0;
	packetSize_ms = enc_Ctrl.framesize_ms;
	enc_Ctrl.useMDIndex            = WriteMDI;

	nread = enc_Ctrl.framesize_ms * (enc_Ctrl.samplerate/1000);

	stEnc = AGR_Sate_Encoder_Init(&enc_Ctrl);
	frame =0;

	while(fread(&FrmBuf[0],sizeof(short),nread,fin)==nread)
	{
        memset(nBytes, 0, sizeof(short) * 6);
		nBytesX = AGR_Sate_Encoder_Encode(stEnc, &FrmBuf[0], cbits, MAX_FRAME_BYTES, &nBytes[0]);

		if (nBytesX < 0) {
			printf("\nYY_SSC_Encode returned %d", nBytesX);
			break;
		}

		smplsSinceLastPacket += nread;

		if( ( ( 1000 * smplsSinceLastPacket ) / enc_Ctrl.samplerate ) == packetSize_ms ) {
			/* Sends a dummy zero size packet in case of DTX period  */
			/* to make it work with the decoder test program.        */
			/* In practice should be handled by RTP sequence numbers */
			totPackets++;
			sumBytes += nBytesX;
			sumBytes2 += nBytes[1];
			nrg = 0.0;
			for( k = 0; k < ( int )nread; k++ ) {
				nrg += FrmBuf[ k ] * (double)FrmBuf[ k ];
			}
			if( ( nrg / nread ) > 1e3 ) {
				sumActBytes += nBytesX;
				totActPackets++;
			}
			smplsSinceLastPacket = 0;
		}


		/*

		|-------|-------|---------------------|---------------------|---------------------|
		| Byte0 | Byte1 | Low Band MD1 Stream | Low Band MD2 Stream |  High Band Stream   |
		|-------|-------|---------------------|---------------------|---------------------|

		MD1 Steam = Low Band MD1 Stream
		MD2 Steam = Low Band MD2 Stream + High Band Stream
		Length(MD1 Steam) = Length(Low Band MD1 Stream)
		Length(MD2 Steam) = Length(Low Band MD2 Stream) + Length(High Band Stream)
		Byte0 = Length(MD1 Steam) + Length(MD2 Steam)
		      = Length(Low Band MD1 Stream) + Length(Low Band MD2 Stream) + Length(High Band Stream)
		Byte1 = Length(MD2 Steam)

		*/

		fwrite(&nBytes[0], sizeof(short), 1, fout);
		fwrite(&nBytes[1], sizeof(short), 1, fout);

		if (nBytes[0])
		{
			fwrite(&cbits[0], sizeof(char), nBytesX, fout);
		}

		frame++;
		printf("%d frames processed!\r",frame);
	}//end while()

	if (frame == 0)
	{
		return 1;
	}

	{
		avg_rate = 8.0 / packetSize_ms * sumBytes / totPackets;
		avg_md2_rate = 8.0 / packetSize_ms * sumBytes2 / totPackets;
		act_rate = 8.0 / packetSize_ms * sumActBytes / totActPackets;
		printf("\n\nAverage bitrate:        %.3f kbps", (avg_rate - avg_md2_rate));
		printf("\nAverage bitrate:          %.3f kbps", avg_md2_rate);
		printf("\nAverage T bitrate:        %.3f kbps", (avg_rate));
		printf("\nActive bitrate:           %.3f kbps", act_rate);
		printf("\n\n");

	}

	printf("\n");

	AGR_Sate_Encoder_Uninit(stEnc);
	fclose(fin);
	fclose(fout);

	if(access("_bitrate.txt",0))
	{
		FILE *brf;
		brf = fopen("_bitrate.txt","wb+");
		fprintf(brf,"%s\t\t\t\t Core Average\t\t\t\t Total Average Bitrate\t\t Highest Bitrate\n","file name" );
		fprintf(brf, "%s\t\t\t\t %0.12f\t\t\t\t %0.12f\t\t\t\t %0.12f\n", InFileName, (avg_rate - avg_md2_rate), avg_rate, act_rate);
	}
	else
	{
		FILE *brf;
		brf = fopen("_bitrate.txt","ab+");
		fprintf(brf, "%s\t\t\t\t %0.12f\t\t\t\t %0.12f\t\t\t\t %0.12f\n", InFileName, (avg_rate - avg_md2_rate), avg_rate, act_rate);
	}

	return 0;
}//end main()
