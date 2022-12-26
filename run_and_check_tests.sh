#!/bin/bash
declare file="test_results"
declare regex=" tests, 0 errors, 0 failures, 0 skipped"
. /opt/ros/noetic/setup.bash
catkin_make
catkin_make run_tests
catkin_test_results > "${file}"
declare file_content=$( cat "${file}" )
if [[ " $file_content " =~ $regex ]]
  then
    echo "All Tests Passed"
    exit 0
  else
    echo "Not All Tests Passed"
    exit 1
fi
