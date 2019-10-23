@if "%1"=="" goto :end 
JC1Encoder.exe  Ch_f1_raw.pcm Ch_f1_mode2.bit -mode 2 -Fs_API 16000 -DTX 1 -rate %1 -framesize 40 -joint 0
JC1Decoder.exe  Ch_f1_mode2.bit Ch_f1_decoded_sate_loss00.pcm  -Fs_API 16000 -loss 0
JC1Decoder.exe  Ch_f1_mode2.bit Ch_f1_decoded_sate_loss05.pcm  -Fs_API 16000 -loss 05
JC1Decoder.exe  Ch_f1_mode2.bit Ch_f1_decoded_sate_loss10.pcm  -Fs_API 16000 -loss 10
JC1Decoder.exe  Ch_f1_mode2.bit Ch_f1_decoded_sate_loss15.pcm  -Fs_API 16000 -loss 15
JC1Decoder.exe  Ch_f1_mode2.bit Ch_f1_decoded_sate_loss20.pcm  -Fs_API 16000 -loss 20
JC1Decoder.exe  Ch_f1_mode2.bit Ch_f1_decoded_sate_loss25.pcm  -Fs_API 16000 -loss 25

WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_sate_loss00.pcm
WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_sate_loss05.pcm
WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_sate_loss10.pcm
WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_sate_loss15.pcm
WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_sate_loss20.pcm
WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_sate_loss25.pcm

std_silk_encode.exe  Ch_f1_raw.pcm Ch_f1_silk.bit -Fs_API 16000 -rate 16000
std_silk_decode.exe  Ch_f1_silk.bit Ch_f1_decoded_silk_loss00.pcm  -Fs_API 16000  -loss 0 
std_silk_decode.exe  Ch_f1_silk.bit Ch_f1_decoded_silk_loss05.pcm  -Fs_API 16000  -loss 05
std_silk_decode.exe  Ch_f1_silk.bit Ch_f1_decoded_silk_loss10.pcm  -Fs_API 16000  -loss 10
std_silk_decode.exe  Ch_f1_silk.bit Ch_f1_decoded_silk_loss15.pcm  -Fs_API 16000  -loss 15
std_silk_decode.exe  Ch_f1_silk.bit Ch_f1_decoded_silk_loss20.pcm  -Fs_API 16000  -loss 20
std_silk_decode.exe  Ch_f1_silk.bit Ch_f1_decoded_silk_loss25.pcm  -Fs_API 16000  -loss 25

WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_silk_loss00.pcm
WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_silk_loss05.pcm
WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_silk_loss10.pcm
WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_silk_loss15.pcm
WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_silk_loss20.pcm
WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_silk_loss25.pcm
:end
@echo Test CMD: "JC1TEST.bat bitrate" bitrate is a number : 13000


