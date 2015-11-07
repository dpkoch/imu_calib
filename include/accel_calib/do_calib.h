/**
 * \file do_calib.h
 * \author Daniel Koch <daniel.p.koch@gmail.com>
 *
 * Class for performing IMU calibration
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <string>
#include <vector>
#include <queue>

#include <accel_calib/accel_calib.h>

namespace accel_calib
{

class DoCalib
{
public:
  DoCalib();

  bool running();

private:
  enum DoCalibState { START, SWITCHING, RECEIVING, COMPUTING, DONE };

  AccelCalib calib_;

  DoCalibState state_;

  int measurements_per_orientation_;
  int measurements_received_;

  double reference_acceleration_;
  std::string output_file_;

  std::queue<AccelCalib::Orientation> orientations_;
  AccelCalib::Orientation current_orientation_;

  std::string orientation_labels_[6];

  ros::Subscriber imu_sub_;
  void imuCallback(sensor_msgs::Imu::ConstPtr imu);
};

} // namespace accel_calib
