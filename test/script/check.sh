#!/bin/bash

TEST_REF_DIR=$1
WORKING_DIR=$2

TEST_REF_LOG=$TEST_REF_DIR/log/test.log
TEST_REF_OUT=$TEST_REF_DIR/log/test.out
SHIELD_REF=$TEST_REF_DIR/out

TEST_LOG=$WORKING_DIR/log/test.log
TEST_OUT=$WORKING_DIR/log/test.out
SHIELD_DIR=$WORKING_DIR/out

TEST_DIFF=$WORKING_DIR/diff

ERRORS=0

diff $TEST_LOG $TEST_REF_LOG
if [ $? -eq 0 ]; then
  echo "TEST LOG: OK"
else
  echo "TEST LOG: FAILED"
  ((ERRORS++))
fi

for file in $TEST_LOG.*; do
  diff "$file" "$TEST_REF_DIR/log/${file##*/}"
  if [ $? -eq 0 ]; then
    echo "TEST SHIELD LOG: OK"
  else
    echo "TEST SHIELD LOG: FAILED"
    ((ERRORS++))
  fi
done

diff $TEST_OUT $TEST_REF_OUT > $TEST_DIFF
if [ $? -eq 0 ]; then
  echo "TEST PROGRAM OUTPUT: OK"
else
  echo "WARNING: PROGRAM OUTPUT CHANGED (THIS IS OK)"
  # OK timestemps, performance measures, etc.
  #((ERRORS++))
fi

# IGNORE COMMANDS/TIMESTAMPS
diff -r -I '^// ' $SHIELD_DIR $SHIELD_REF
if [ $? -eq 0 ]; then
  echo "TEST GENERATED FILES: OK"
else
  echo "TEST GENERATED FILES: FAILED"
  ((ERRORS++))
fi

exit $ERRORS