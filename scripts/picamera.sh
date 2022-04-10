#!/bin/sh

# This script launches Raspberry Pi's camera and streams it over TCP.

G=$1

if [ -z $G ]; then
	G=240
fi

echo $G

while true; do
	libcamera-vid -t 0 --inline --listen -o tcp://0.0.0.0:33333 --rawfull --width 1920 --height 1080 --framerate 30 -g $G
	clear
	sleep 1
done
