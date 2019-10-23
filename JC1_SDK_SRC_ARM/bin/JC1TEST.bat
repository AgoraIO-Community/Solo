@if "%1"=="" goto :end 
JC1Encoder.exe  %1.pcm %1_mode2.bit -mode 2 -Fs_API 16000  -rate %2 -framesize %3 -joint %4 -MDI %5
JC1Decoder.exe  %1_mode2.bit %1_decoded_sate0.pcm  -Fs_API 16000 -dec_mode 0  -framesize %3 -joint %4 -MDI %5
JC1Decoder.exe  %1_mode2.bit %1_decoded_sate1.pcm  -Fs_API 16000 -dec_mode 1  -framesize %3 -joint %4 -MDI %5
JC1Decoder.exe  %1_mode2.bit %1_decoded_sate2.pcm  -Fs_API 16000 -dec_mode 2  -framesize %3 -joint %4 -MDI %5
WB-PESQ.exe +16000  +wb  %1.pcm  %1_decoded_sate0.pcm
WB-PESQ.exe +16000  %1.pcm  %1_decoded_sate0.pcm
WB-PESQ.exe +16000  %1.pcm  %1_decoded_sate1.pcm
WB-PESQ.exe +16000  %1.pcm  %1_decoded_sate2.pcm

std_silk_encode.exe  %1.pcm %1_silk.bit -Fs_API 16000 -rate 16000
std_silk_decode.exe  %1_silk.bit %1_decoded_silk0.pcm  -Fs_API 16000 
std_silk_encode.exe  %1.pcm %1_silk.bit -Fs_API 16000 -rate 8000
std_silk_decode.exe  %1_silk.bit %1_decoded_silk1.pcm  -Fs_API 16000 

WB-PESQ.exe +16000  +wb  %1.pcm  %1_decoded_silk0.pcm
WB-PESQ.exe +16000  %1.pcm  %1_decoded_silk0.pcm
WB-PESQ.exe +16000  %1.pcm  %1_decoded_silk1.pcm
:end
@echo Test CMD: "JC1TEST.bat <file name(exclude .pcm)> <bitrate> <framesize in ms>  <joint 0 or 1>" bitrate is a number : 13000

