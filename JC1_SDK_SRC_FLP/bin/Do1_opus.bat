opus_demo -e voip 16000 1 16000 -cvbr -inbandfec -loss 0 Ch_f1_raw.pcm out_opus16_fec0.bit
opus_demo -e voip 16000 1 16000 -cvbr -inbandfec -loss 20 Ch_f1_raw.pcm out_opus16_fec20.bit
opus_demo -e voip 16000 1 20000 -cvbr -inbandfec -loss 20 Ch_f1_raw.pcm out_opus20_fec20.bit

opus_demo -d 16000 1 -loss 0  out_opus16_fec0.bit out_opus_fec0_loss0.pcm
opus_demo -d 16000 1 -loss 5  out_opus16_fec0.bit out_opus_fec0_loss5.pcm
opus_demo -d 16000 1 -loss 10 out_opus16_fec0.bit out_opus_fec0_loss10.pcm
opus_demo -d 16000 1 -loss 15 out_opus16_fec0.bit out_opus_fec0_loss15.pcm
opus_demo -d 16000 1 -loss 20 out_opus16_fec0.bit out_opus_fec0_loss20.pcm
opus_demo -d 16000 1 -loss 25 out_opus16_fec0.bit out_opus_fec0_loss25.pcm

WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus_fec0_loss0.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus_fec0_loss5.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus_fec0_loss10.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus_fec0_loss15.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus_fec0_loss20.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus_fec0_loss25.pcm

opus_demo -d 16000 1 -loss 0  -inbandfec out_opus16_fec20.bit out_opus_fec20_loss0.pcm
opus_demo -d 16000 1 -loss 5  -inbandfec out_opus16_fec20.bit out_opus_fec20_loss5.pcm
opus_demo -d 16000 1 -loss 10 -inbandfec out_opus16_fec20.bit out_opus_fec20_loss10.pcm
opus_demo -d 16000 1 -loss 15 -inbandfec out_opus16_fec20.bit out_opus_fec20_loss15.pcm
opus_demo -d 16000 1 -loss 20 -inbandfec out_opus16_fec20.bit out_opus_fec20_loss20.pcm
opus_demo -d 16000 1 -loss 25 -inbandfec out_opus16_fec20.bit out_opus_fec20_loss25.pcm

WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus_fec20_loss0.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus_fec20_loss5.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus_fec20_loss10.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus_fec20_loss15.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus_fec20_loss20.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus_fec20_loss25.pcm


opus_demo -d 16000 1 -loss 0  -inbandfec out_opus20_fec20.bit out_opus20_fec20_loss0.pcm
opus_demo -d 16000 1 -loss 5  -inbandfec out_opus20_fec20.bit out_opus20_fec20_loss5.pcm
opus_demo -d 16000 1 -loss 10 -inbandfec out_opus20_fec20.bit out_opus20_fec20_loss10.pcm
opus_demo -d 16000 1 -loss 15 -inbandfec out_opus20_fec20.bit out_opus20_fec20_loss15.pcm
opus_demo -d 16000 1 -loss 20 -inbandfec out_opus20_fec20.bit out_opus20_fec20_loss20.pcm
opus_demo -d 16000 1 -loss 25 -inbandfec out_opus20_fec20.bit out_opus20_fec20_loss25.pcm

WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus20_fec20_loss0.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus20_fec20_loss5.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus20_fec20_loss10.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus20_fec20_loss15.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus20_fec20_loss20.pcm
WB-PESQ +wb +16000 Ch_f1_raw.pcm out_opus20_fec20_loss25.pcm