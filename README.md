*English|[中文](README.zh.md)*
# Introduction

Agora Solo is an open source speech codec, it was developed based on Silk with BWE(Bandwidth Extension) and MDC(Multi Description Coding). With these technologies, Solo is enable to resist weak networks at low bitrates. Solo can support Android/iOS/MacOS/Windows now. 

Figure 1. Architecture of Solo encoder：
![solo_encoder](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_encoder.png)

Figure 2. Architecture of Solo decoder：
![solo_decoder](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_decoder.png)


# Overview of key technologies

## Bandwidth expansion

The main reason for solo to use bandwidth expansion is to reduce the computational complexity. In the WB mode of Silk, all 16KHz signals will enter the subsequent processing module, while for the voice, the information above 8kHz is very small. This part of information will enter the subsequent processing module, which will bring a certain waste of computing resources. Considering MDC needs to introduce additional analysis module to process multiple bitstreams, it will introduce additional complexity, which is one of the important reasons why MDC does not land smoothly in recent years. In order to reduce the complexity, Solo divide the wideband signal into 0-8k narrowband information and 8-16k high frequency information before coding. Only the narrow-band information will enter the subsequent normal analysis and coding process, so that the subsequent calculation amount will be reduced by half, and the overall quality will not be significantly reduced thanks to the bandwidth expansion algorithm. In the high-frequency information part, Solo uses an independent analysis and coding module to compress the high-frequency information into a 1.6kbps bitstream by default. This part of high frequency information can be combined with low frequency signal in decoder to recover high frequency signal.

## MDC with delay decision

In Silk, delay decision module is a module for computing coding error. It can select the bitstream with the smallest error from multiple candidate bitstreams as the coding output. To some extent, it makes scalar quantization have the performance of vector quantization Solo uses the delay decision module to realize the analysis and construction of multi description bitstream. The MDC of solo mainly acts on the residual signal output by the filter. Solo will perform multi gain control on the residual signal according to the current signal state: after the MD gain a (0 < a < 1) is calcelated, it will be acted on the odd subframe. Meanwhile, the even subframe is multiplied by (1-a) . Here, it is recorded as residual 1 and residual 2.

Figure 3. Multiple description residual signal:
![solo_residual](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_residual.png)

Then, the two residual signals will enter into the new delay decision module. Each residual signal uses different seed and quantization methods, which can generate 8 different alternative states. When the two signals are combined, there are 64 alternative composite states. The new delay decision module will weighted sum the independent error of each residual signal and the composite error of the two residual signals then determine the best two residual signals into post coding module.

## Stream packing

Figure 4. Encoder stream integration and packing:
![solo_packing](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_packing.png)

The default configuration of Solo is to input 40ms (2 frames) each time and output two complementary multi description bitstreams. When the decoder receives any bitstream, it can decode 40ms signal. In order to facilitate the receiver to distinguish the sequence of the bitstream, the fourth bit on the right of the first byte of the bitstream is the sequence flag bit, the flag value of the first bitstream bit is 0, and the flag value of the second bitstream flag bit is 1. When the receiver processes the bitstream, it can judge the sequence of the stream according to this flag.


# Overview of integration methods
Generally speaking, the following changes are needed to migrate solo to the media engine:

## Integrate solo to engine
Developers can either integrate source code or static library to the engine. What need to do is using the AGR_JC1_SDK_API.h,which can be found in the source code. Firstly, call AGR_Sate_Encoder_Init ( ) to configure the encoding state, then use AGR_Sate_Encoder_Encode ( ) to encode audio signal. Finally, call AGR_Sate_Encoder_Uninit ( ) pair after the encoding processing is completed. Using method of decoder is familiar withencoder, just call the relevant function of decoder.

Figure 5. Functions required by encoding:
![solo_API_en](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_API_en.png)

Figure 6. Functions required by decoding:
![solo_API_de](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_API_de.png)

## Bitstream sending
The encoder inputs 40 ms signal each time, and outputs a whole bit stream including two complementary bit streams. As shown in the figure below, the blue box and orange box respectively represent two independent bitstreams in the output bitstream, each bitstream contains the information of frame 1 and frame 2. When the encoder outputs the bitstream, it will also output the length of each complementary bitstream through an array named nbytesout, nBytesout [0] represents the total length of the bitstream, nBytesout [1] represents the length of the first complementary bitstream, and the engine needs to adjust the length according to this length then divide it into two segments of bitstream and send in different packets.

Figure 7. Bitstream segmentation and contracting:
![solo_sending](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_sending.png)

## Bitstream screening and synthesis at receiver
At the receiving end, every time a packet needs to be inserted into the buffer, it needs to be determined whether the corresponding complementary packet already exists and which complementary bitstream the current packet contains.
The engine can determine which complementary bitstream the current bitstream is based on the bitstream flag bit. If the current cache queue does not have its corresponding packet, it can directly insert the packet payload into the cache queue, and update the array named nbytes according to the bitstream contained in the packet (the decoder depends on the array to determine the current packet receiving status). nBytes [0] represents the total length of the bitstream, and nBytes [1] represents the length of the first complementary bitstream.
If there are corresponding complementary packets in the current cache queue, it is necessary to synthesize them: insert the first complementary stream before the second one, and then update the nbytes array state.

Figure 8. BItstream synthesis method at the receiver:
![solo_neteq](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_neteq.png)

# Copyright declaration of Silk

Copyright (c) 2006-2012, Skype. All rights reserved. 
Redistribution and use in source and binary forms, with or without 
modification, (subject to the limitations in the disclaimer below) 
are permitted provided that the following conditions are met:
- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright 
notice, this list of conditions and the following disclaimer in the 
documentation and/or other materials provided with the distribution.
- Neither the name of Skype, nor the names of specific 
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

# License

This software is under the MIT License (MIT).

# Another words

We have set up a solo exchange post in RTC forum to facilitate everyone's exchange and discussion. [The forum link is here](https://rtcdeveloper.com/t/topic/16270). Please feel free to commit your code or give us your idea if you find some problems. Let's make Solo as a long-term evolution codec.

