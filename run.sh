#!/bin/bash

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

# -----------------------------------------------------------------------
if [ "$1" == "test" ]; then
  echo "RUN TESTS"

  # REFERENCE FILES WITH (other version can output other files)
  # STORM 1.6.2
  # SUMO 1.7.0
  # docker pull mdei/shieldone:v1

  cd "$SOURCE_DIR"/test/script
  ./test_exp_basic.sh
  ./test_exp_helsinki.sh
  ./test_exp_bus.sh
  # $SOURCE_DIR/client.sh test
  exit $?
fi
# -----------------------------------------------------------------------

SUMO_CFG1=$SOURCE_DIR/data/exp_basic/one_junction.sumo.cfg
SUMO_CFG2=$SOURCE_DIR/data/exp_helsinki/osm.sumocfg
SUMO_CFG3=$SOURCE_DIR/data/exp_bus/osm.sumocfg

# RUN
SHIELD_BIN=$WORKING_DIR/adaptiveShielding

$SHIELD_BIN -c $SUMO_CFG1 -g -d 4 - l 0.3 -k 10 -t 6000 -o log/demo.log
# $SHIELD_BIN -c $SUMO_CFG2 -w $(pwd)/data/exp_helsinki/shieldIDs.txt -i $(pwd)/data/exp_helsinki/block.txt -g -d 4 - l 0.3 -k 10 -t 6000 -o log/demo.log
# $SHIELD_BIN -c $SUMO_CFG3 -w $(pwd)/data/exp_bus/shieldIDs.txt --bus -g -d 1 - l 0.3 -k 10 -t 6000 -o log/demo.log
