
JC1Encoder.exe  Ch_f1_raw.pcm Ch_f1_mode2.bit -mode 2 -Fs_API 16000  -rate 12600
JC1Decoder.exe  Ch_f1_mode2.bit Ch_f1_decoded_sate.pcm  -Fs_API 16000 
WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_sate.pcm



std_silk_encode.exe  Ch_f1_raw.pcm Ch_f1_silk.bit -Fs_API 16000 -rate 16000
std_silk_decode.exe  Ch_f1_silk.bit Ch_f1_decoded_silk.pcm  -Fs_API 16000 
WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_silk.pcm


