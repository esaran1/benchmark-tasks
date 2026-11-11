#!/bin/bash
# gworker.sh FROZEN_TASK_DIR OUT_DIR STATE_VAR "states" "strategies"
#   Runs demo.py --no-record for every (state, strategy) through the real task_score path, one GPU slot at a time,
#   skipping finished runs. FROZEN_TASK_DIR: a frozen copy of the task folder (cp -r, never edited during the batch).
#   STATE_VAR: the task's state env var (here MOTOR_STATE). Results: OUT_DIR/s{k}_{strategy}/result.json
#   Uses ~/gpu/gpu.sh (a GPU slot queue) when it exists, otherwise runs each job directly.
T=$1; O=$2; SV=$3; S=$4; V=$5; mkdir -p $O
Q=""; [ -x ~/gpu/gpu.sh ] && Q=~/gpu/gpu.sh
for k in $S; do for v in $V; do
  d=$O/s${k}_$v; [ -f $d/result.json ] && continue; mkdir -p $d
  $Q ${Q:+--tag "gap-$(basename $T)-s$k-$v"} bash -c "source /data/IsaacLab-Arena/yam.env >/dev/null 2>&1; cd $T && \
    env $SV=$k DEMO_VARIANT=$v timeout 1500 python demo.py task.py $d --no-record > $d/log.txt 2>&1"
  echo "$(date -u +%H:%M) s${k}_$v $(grep -o '"task_score": [0-9.]*' $d/result.json 2>/dev/null) $(grep -m1 -E 'Error|Traceback' $d/log.txt)" >> $O/run.out
done; done
echo "DONE $S" >> $O/run.out
