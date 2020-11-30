#!/bin/bash

source /opt/ros/foxy/setup.bash
source ~/Documents/rmw/install/setup.bash
source ~/Documents/apex_ai_perf/perf_test_ws/install/local_setup.bash

messages=( 'Array1k' 'Array4k' 'Array16k' 'Array32k' 'Array60k' 'Array1m' 'Array2m' 'Struct16' 'Struct256' 'Struct4k' 'Struct32k' 'PointCloud512k' 'PointCloud1m' 'PointCloud2m' 'PointCloud4m' 'Range' 'NavSatFix' 'RadarDetection' 'RadarTrack' )

runtime=130
ignore=10

num_pub=1
num_sub=3

rate=1000

rmw_name=rmw_ecal_dynamic_cpp
export RMW_IMPLEMENTATION=$rmw_name
mkdir $rmw_name
cd $rmw_name
for msg in "${messages[@]}"
do
  echo
  echo
  echo ----------------------------------------------------------------------------------------------------------------
  echo Running performance test on $rmw_name for message $msg
  echo ----------------------------------------------------------------------------------------------------------------
  echo
  mkdir $msg
  cd $msg
  sleep 5
  ros2 run performance_test perf_test --communication ROS2 --max_runtime $runtime --ignore $ignore -t test_topic -l 'log' --rate $rate -p $num_pub -s $num_sub --msg $msg
  python3 ../../run_perfplot.py
  cd ..
done
cd ..

rmw_name=rmw_cyclonedds_cpp
export RMW_IMPLEMENTATION=$rmw_name
mkdir $rmw_name
cd $rmw_name
for msg in "${messages[@]}"
do
  echo
  echo
  echo ----------------------------------------------------------------------------------------------------------------
  echo Running performance test on $rmw_name for message $msg
  echo ----------------------------------------------------------------------------------------------------------------
  echo
  mkdir $msg
  cd $msg
  sleep 5
  ros2 run performance_test perf_test --communication ROS2 --max_runtime $runtime --ignore $ignore -t test_topic -l 'log' --rate $rate -p $num_pub -s $num_sub --msg $msg
  python3 ../../run_perfplot.py
  cd ..
done
cd ..

rmw_name=rmw_fastrtps_cpp
export RMW_IMPLEMENTATION=$rmw_name
mkdir $rmw_name
cd $rmw_name
for msg in "${messages[@]}"
do
  echo
  echo
  echo ----------------------------------------------------------------------------------------------------------------
  echo Running performance test on $rmw_name for message $msg
  echo ----------------------------------------------------------------------------------------------------------------
  echo
  mkdir $msg
  cd $msg
  sleep 5
  ros2 run performance_test perf_test --communication ROS2 --max_runtime $runtime --ignore $ignore -t test_topic -l 'log' --rate $rate -p $num_pub -s $num_sub --msg $msg
  python3 ../../run_perfplot.py
  cd ..
done
cd ..

