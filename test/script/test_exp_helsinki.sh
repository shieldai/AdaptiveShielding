#!/bin/bash

cd $(dirname "$0")

CHECK=$(pwd)/check.sh

cd ../../

SOURCE_DIR=$(pwd)
TEST_REF_DIR=$SOURCE_DIR/test/data/test_exp_helsinki

WORKING_DIR=$(pwd)/build
cd $WORKING_DIR

rm out/*
rm log/*

TEST_LOG=$WORKING_DIR/log/test.log
TEST_OUT=$WORKING_DIR/log/test.out

SUMO_CFG2=$SOURCE_DIR/data/exp_helsinki/osm.sumocfg

SHIELD_BIN=$WORKING_DIR/adaptiveShielding

$SHIELD_BIN -c $SUMO_CFG2 -w $SOURCE_DIR/data/exp_helsinki/shieldIDs.txt -i $SOURCE_DIR/data/exp_helsinki/block.txt -d 4 - l 0.3 -k 10 -t 6000 -o $TEST_LOG > $TEST_OUT
if [ $? -ne 0 ]; then
  echo RUN FAIELD
  exit 1
fi

bash $CHECK $TEST_REF_DIR $WORKING_DIR
if [ $? -ne 0 ]; then
  echo TEST FAIELD
  exit 1
fi