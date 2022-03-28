#!/bin/bash

INPUT="$1"

OUTPUT="$2"

ffmpeg -i "$INPUT" -vf "fps=15,scale=720:-1:flags=lanczos" -c:v libwebp -lossless 0 -compression_level 6 -q:v 30 -loop 0 docs/resources/video/"$OUTPUT"
