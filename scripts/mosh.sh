#!/bin/bash

# Re-encode the video to contain just a single I-frame, and replace it with another image.
# Argument 1 is the frame to mosh into the video, while argument 2 is the video to re-encode and mosh.
# Only 1920x1080 video is supported. Adjust scales accordingly.

MOSH_FRAME=$1
MOSH_VIDEO=$2

RES_FRAME=$(ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0 $MOSH_FRAME)
RES=$(ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0 $MOSH_VIDEO)

ffmpeg \
	-i $MOSH_VIDEO -i $MOSH_FRAME -filter_complex \
	"[1]crop=1920:1080:0:0,scale=1920:1080,setsar=1:1[i0];[i0][0:v:0]concat=n=2:v=1[outv]" \
	-map "[outv]" -c:v libx264 -bf 0 -g 999999 out0.avi

ffmpeg -i out0.avi -c copy -t 0.01 -copyinkf out2.avi
ffmpeg -i out0.avi -c copy -ss 0.04 -copyinkf out3.avi
cat out2.avi out3.avi > out.avi
