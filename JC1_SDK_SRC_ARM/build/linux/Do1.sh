./JC1Encoder.exe  Ch_f1_raw.pcm Ch_f1_mode2.bit -mode 2 -Fs_API 16000  -rate 13600
./JC1Decoder.exe  Ch_f1_mode2.bit Ch_f1_decoded_sate.pcm  -Fs_API 16000 
./WB-PESQ.exe +16000  +wb  Ch_f1_raw.pcm  Ch_f1_decoded_sate.pcm