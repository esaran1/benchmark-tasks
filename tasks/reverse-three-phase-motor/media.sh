#!/bin/bash
# media.sh DREAM_LOG_DIR DEMO_OUT_DIR DEST_DIR
#   DREAM_LOG_DIR: the --log-dir of the one Dream-1 run (--store-frames). DEMO_OUT_DIR: demo.py's recorded output.
#   Writes DEST/model_run.mp4 (Dream-1, left|top|right), DEST/demo.mp4 (demo, left|top|right), DEST/setup.png (the Dream
#   run's top camera, frame 0 = the starting layout as the videos begin). All H.264 yuv420p +faststart, size-checked.
set -e
source /data/IsaacLab-Arena/yam.env >/dev/null 2>&1
L=$1; D=$2; O=$3; mkdir -p $O
log=$(ls -t $L/*.json | head -1)
fr=$(dirname $log)/vids; rm -rf $fr; inspect-robots video "$log" --out $fr >/dev/null
c() { ls $fr/*$1_cam.mp4 | head -1; }
ff="-c:v libx264 -pix_fmt yuv420p -movflags +faststart -an"
ffmpeg -loglevel error -y -i "$(c left)" -i "$(c top)" -i "$(c right)" -filter_complex hstack=inputs=3 $ff $O/model_run.mp4
ffmpeg -loglevel error -y -i $D/demo_left_cam.mp4 -i $D/demo_top_cam.mp4 -i $D/demo_right_cam.mp4 -filter_complex hstack=inputs=3 $ff $O/demo.mp4
ffmpeg -loglevel error -y -i "$(c top)" -frames:v 1 $O/setup.png
for f in model_run.mp4 demo.mp4; do      # no ffprobe on this machine: parse ffmpeg -i
  echo "== $f $(du -m $O/$f | cut -f1) MB"
  ffmpeg -hide_banner -i $O/$f 2>&1 | grep -E "Duration|Stream|title|artist|comment|author|copyright|location"
done
echo "setup.png $(du -k $O/setup.png | cut -f1) KB"
echo "check: Stream must be h264 + yuv420p; no title/artist/comment/author/location tags (encoder tags are fine)"
