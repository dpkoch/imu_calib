/**
 * \file apply_calib.h
 * \author Daniel Koch <daniel.p.koch@gmail.com>
 *
 * Class for applying a previously computed calibration to IMU data
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <accel_calib/accel_calib.h>

namespace accel_calib
{

class ApplyCalib
{
public:
  ApplyCalib();

private:
  AccelCalib calib_;

  ros::Subscriber raw_sub_;
  ros::Publisher corrected_pub_;

  void rawImuCallback(sensor_msgs::Imu::ConstPtr raw);
};

} // namespace accel_calib
