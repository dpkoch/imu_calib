/**
 * \file apply_calib.cpp
 * \author Daniel Koch <daniel.p.koch@gmail.com>
 *
 * Class for applying a previously computed calibration to IMU data
 */

#include "accel_calib/apply_calib.h"

namespace accel_calib
{

ApplyCalib::ApplyCalib()
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  int queue_size;
  nh_private.param<int>("queue_size", queue_size, 5);

  std::string calib_file;
  nh_private.param<std::string>("calib_file", calib_file, "imu_calib.yaml");

  raw_sub_ = nh.subscribe("raw", queue_size, &ApplyCalib::rawImuCallback, this);
  corrected_pub_ = nh.advertise<sensor_msgs::Imu>("corrected", queue_size);

  if (!calib_.loadCalib(calib_file) || !calib_.calibReady())
  {
    ROS_FATAL("Calibration could not be loaded");
    ros::shutdown();
  }
}

void ApplyCalib::rawImuCallback(sensor_msgs::Imu::ConstPtr raw)
{
  double acc_raw[3] = { raw->linear_acceleration.x,
                        raw->linear_acceleration.y,
                        raw->linear_acceleration.z };
  double acc_corrected[3];

  calib_.applyCalib(acc_raw, acc_corrected);

  sensor_msgs::Imu corrected = *raw;
  corrected.linear_acceleration.x = acc_corrected[0];
  corrected.linear_acceleration.y = acc_corrected[1];
  corrected.linear_acceleration.z = acc_corrected[2];

  corrected_pub_.publish(corrected);
}

} // namespace accel_calib
