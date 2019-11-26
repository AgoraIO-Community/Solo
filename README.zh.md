*[English](README.md)|中文*
# Solo概述
抗丢包音频编解码器 SOLO 已经开源，源代码已经上传至 Github。该项目由声网Agora团队自研，以 Silk 编解码器为基础，融合BWE、MDC等技术，可以帮助WebRTC应用在较高丢包率下，仍然获得较好的实时音频体验。

Solo是一款面向不稳定网络的编解码器，它以Silk为基础，融合了带宽扩展（BWE）和多描述编码（MDC）等技术，使其能在较低复杂度下拥有弱网对抗能力。

SOLO 特性：

* 兼容 WebRTC

* 以 Silk 为基础，融合了带宽扩展(BWE)和多描述编码(MDC)等技术

* 25%丢包率下，MOS 值优于 Opus


图1. Solo编码器架构：
![solo_encoder](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_encoder.png)

图2. Solo解码器架构：
![solo_decoder](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_decoder.png)

# 关键技术概述

## 带宽扩展
Solo使用带宽扩展的主要原因是希望减少计算复杂度，在Silk WB模式中，16khz的信号都会进入后续处理模块，而对于语音来说，8khz以上的信息是非常少的，这部分信息进入到后续处理模块，会带来一定的计算资源浪费。MDC因为要引入额外分析模块处理多条码流，又会引入额外的复杂度，这是MDC在近些年来落地不顺畅的重要原因之一。为了减少复杂度，我们在编码宽带信号前，将其分为0-8k的窄带信息和8-16k的高频信息。只有窄带信息会进入到后续正常分析、编码流程中，这样后续的计算量就减少了一半，同时得益于带宽扩展算法，整体质量不会有明显下降。高频信息部分，Solo使用独立的分析与编码模块，默认将高频信息压缩成1.6kbps的码流。这部分高频信息可以在解码器内结合低频信号恢复出高频信号。

## 结合delay-decision的MDC
在Silk中，delay-decision模块是一个滞后计算编码误差的模块，它可以从多个候选码流中选择误差最小的码流作为编码输出，一定程度上来说，它使得标量量化拥有了矢量量化的性能。 Solo利用delay-decision模块，实现了多描述码流的分析与构建。Solo的MDC主要作用于滤波器输出的残差信号， Solo会根据当前信号状态，对残差信号做多增益控制：计算出MD增益a（0<a<1），将a作用于奇数子帧，并将(1-a)作用于偶数子帧以产生两段互补的残差信号，这里记作residual 1和residual 2。

图3. 多描述残差信号产生:
![solo_residual](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_residual.png)

随后，这两段残差信号会进入到新的delay-decision模块中，每个残差信号使用不同的抖动和量化方法，一共可以产生8种不同的备选状态，两两组合起来共有64种备选合成状态，新的delay-decision模块会对每个残差信号的独立误差和两个残差信号的合成误差进行加权求和，决定出最佳的两个残差信号进入到编码模块。
## 输出码流组包

图4. 编码器码流整合及组包:
![solo_packing](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_packing.png)

Solo默认配置为每次输入40ms（2帧），输出两段互补的多描述码流，解码器接收到任一段码流，即可解码出40ms的信号。为了方便接收端区分码流的顺序，码流第一个字节的右数第4个bit是码流顺序标志位，第一段码流标志位的值是0，第二段码流标志位的值是1。接收端在进行码流处理时，可依据此标志位进行码流顺序判断

# 集成方法概述

一般来说，移植Solo到媒体引擎主要需要以下改动：
##    集成Solo到引擎
开发者可自行选择集成源码或静态库到引擎，只需引用开源代码里的AGR_JC1_SDK_API.h，便可以使用Solo的编码器和解码器，编码器的使用只需要先调用AGR_Sate_Encoder_Init()对编码状态进行配置，即可调用AGR_Sate_Encoder_Encode()进行编码，编码完成后调用AGR_Sate_Encoder_Uninit()对编码器进行销毁即可。解码器和编码器的使用思路一致，调用decoder相关函数即可。

图5. 调用编码器所需函数:
![solo_API_en](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_API_en.png)

图6. 调用解码器所需函数:
![solo_API_de](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_API_de.png)

##   编码端分包发送
编码器每次输入40ms信号，输出的是一整段包含两段互补码流的码流。如图所示，蓝色框和橙色框分别代表输出码流里的两段独立码流，每段码流都包含帧1和帧2的信息，编码器输出码流的同时，也会通过nBytesOut输出每段互补码流的长度，nBytesOut[0]表示码流总长度，nBytesOut[1]表示的是第一段互补码流的长度，第二段互补码流的长度是通过nBytesOut[0]-nBytesOut[1]计算出来的，引擎端需要依据此长度，将其分割成两段码流，并分包进行发送。

图7. 码流分割及发包:
![solo_sending](https://github.com/AgoraIO-Community/Solo/blob/master/imag/solo_sending.png)

##    接收端码流筛选与合成
在接收端，每当有数据包需要插入待解码缓存之前，需要判断其对应互补包是否已经存在以及当前数据包包含哪段互补码流。
引擎可依据码流标志位判断当前码流是哪段互补码流，如果如果当前缓存队列没有其对应数据包，即可直接将数据包payload插入缓存队列，同时根据该包包含的码流进行nBytes数组的更新（解码器依赖该数组判断当前包接收状态）。nBytes[0]表示码流总长度，nBytes[1]表示第一段互补码流的长度。
如果当前缓存队列中已有对应互补数据包，需要对其进行合成，并令第一段互补码流位于第二段互补码流之前，并更新nBytes数组状态。

图8. 接收端码流合成方法：
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

# 代码许可
The MIT License（MIT）

# 最后一句话
我们在RTC论坛开设了Solo交流帖便于大家讨论，如果你有任何问题或修改意见，请联系我们，以持续演进Solo。（https://rtcdeveloper.com/t/topic/16270）
