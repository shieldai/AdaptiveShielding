#!/bin/bash

STORM_HARDCODED=/usr/bin/storm
SUMO_HARDCODED=/usr/bin/sumo
SUMOG_HARDCODED=/usr/bin/sumo-gui

type -a storm
if [ $? -ne 0 ]; then
  echo "STROM is not available in shell"
  exit 1
fi

ls $STORM_HARDCODED
if [ $? -ne 0 ]; then
  echo "STROM is not available in the right path $STORM_HARDCODED"
  exit 1
fi

$STORM_HARDCODED --version

type -a sumo
if [ $? -ne 0 ]; then
  echo "SUMO is not available in shell"
  exit 1
fi

ls $SUMO_HARDCODED
if [ $? -ne 0 ]; then
  echo "SUMO is not available in the right path $SUMO_HARDCODED"
  exit 1
fi

$SUMO_HARDCODED --version

type -a sumo-gui
if [ $? -ne 0 ]; then
  echo "SUMO-GUI is not available in shell"
  exit 1
fi

ls $SUMOG_HARDCODED
if [ $? -ne 0 ]; then
  echo "SUMO-GUI is not available in the right path $SUMOG_HARDCODED"
  exit 1
fi

$SUMOG_HARDCODED --version