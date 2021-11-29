#!/bin/bash

MOSH_FRAME=$1
MOSH_VIDEO=$2

RES_FRAME=$(ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0 $MOSH_FRAME)
RES=$(ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0 $MOSH_VIDEO)

ffmpeg \
	-i $MOSH_VIDEO -i $MOSH_FRAME -filter_complex \
	"[1]crop=2560:1440:0:0,scale=1280:720,setsar=1:1[i0];[i0][0:v:0]concat=n=2:v=1[outv]" \
	-map "[outv]" -c:v mpeg2video -bf 0 -g 999999 out0.avi

# Just see that it would work on accelerator
#ffmpeg \
#	-init_hw_device vaapi=foo:/dev/dri/renderD128 -hwaccel vaapi -hwaccel_output_format vaapi -hwaccel_device foo \
#	-i out0.avi \
#	-vf "format=nv12|vaapi,hwupload" \
#	-c:v h264_vaapi -bf 0 -g 999999 out1.avi

ffmpeg -i out0.avi -c copy -t 0.04 -copyinkf out2.avi
ffmpeg -i out0.avi -c copy -ss 0.1 -copyinkf out3.avi
cat out2.avi out3.avi > out.avi
