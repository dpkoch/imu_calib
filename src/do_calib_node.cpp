/**
 * \file do_calib_node.cpp
 * \author Daniel Koch <danielpkoch@gmail.com>
 *
 * Node performs accelerometer calibration and writes parameters to data file
 */

#include <ros/ros.h>

#include "accel_calib/do_calib.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_calib");

  accel_calib::DoCalib calib;
  while (ros::ok() && calib.running())
  {
    ros::spinOnce();
  }

  return 0;
}
