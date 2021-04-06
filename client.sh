#!/bin/bash

# CALLS THE RL AGENT

PORT=1330
SEC=6005
LOG=/home/martin/mt/sumo-rl/outputs/shield/shield_log
PARAM_D=4
WARMUP_T=0
RUN_TEST="false"

while getopts tp:l:s:d:w: option
do
case "${option}"
in
  p) PORT=${OPTARG};;
  l) LOG=${OPTARG};;
  s) SEC=${OPTARG};;
  d) PARAM_D=${OPTARG};;
  w) WARMUP_T=${OPTARG};;
  t) RUN_TEST="true";;
esac
done

BASEDIR=$(dirname "$0")
cd $BASEDIR

SOURCE_DIR=$(pwd)

mkdir -p build
cd build

WORKING_DIR=$(pwd)

# CREATE NEEDED PATHS
mkdir out
mkdir log

# CHECK BUILD DIR
cmake ..
if [ $? -ne 0 ]; then
    rm -rf *
    cmake ..
fi

# BUILD
make

# RUN
SHIELD_BIN=$WORKING_DIR/adaptiveShielding

# test
if [ "$RUN_TEST" == "true" ]; then

  TEST_REF_DIR=$SOURCE_DIR/test/data/test_exp_basic
  CHECK=$SOURCE_DIR/test/script/check.sh
  TEST_LOG=$WORKING_DIR/log/test.log
  TEST_OUT=$WORKING_DIR/log/test.out

  SUMO_CFG1=$SOURCE_DIR/data/exp_basic/one_junction.sumo.cfg

  sumo-gui -c $SUMO_CFG1 --remote-port $PORT --start -Q --no-step-log=true --time-to-teleport=-1 &

  $SHIELD_BIN -c "NONE" --hook-sumo --port $PORT -d 4 - l 0.3 -k 10 -t 6000 -o $TEST_LOG > $TEST_OUT
  if [ $? -ne 0 ]; then
    echo RUN FAIELD
    exit 1
  fi

  bash $CHECK $TEST_REF_DIR $WORKING_DIR
  if [ $? -ne 0 ]; then
    echo TEST FAIELD
    exit 1
  fi
fi

echo $SHIELD_BIN -c "NONE" --warm-up-time 900 --overwrite-controller --hook-sumo --port $PORT -d $PARAM_D -l 0.3 -k 10 -t $SEC -o $LOG

# run
$SHIELD_BIN -c "NONE" --warm-up-time $WARMUP_T --overwrite-controller --hook-sumo --port $PORT -d $PARAM_D -l 0.3 -k 10 -t $SEC -o $LOG