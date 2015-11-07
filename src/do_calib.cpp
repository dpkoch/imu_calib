/**
 * \file do_calib.cpp
 * \author Daniel Koch <daniel.p.koch@gmail.com>
 *
 * Class for performing IMU calibration
 */

#include "accel_calib/do_calib.h"

namespace accel_calib
{

DoCalib::DoCalib() :
  state_(START)
{
  ros::NodeHandle nh;
  imu_sub_ = nh.subscribe("imu", 1, &DoCalib::imuCallback, this);

  ros::NodeHandle nh_private("~");
  nh_private.param<int>("measurements", measurements_per_orientation_, 500);
  nh_private.param<double>("reference_acceleration", reference_acceleration_, 9.80665);
  nh_private.param<std::string>("output_file", output_file_, "imu_calib.yaml");

  orientations_.push(AccelCalib::XPOS);
  orientations_.push(AccelCalib::XNEG);
  orientations_.push(AccelCalib::YPOS);
  orientations_.push(AccelCalib::YNEG);
  orientations_.push(AccelCalib::ZPOS);
  orientations_.push(AccelCalib::ZNEG);

  orientation_labels_[AccelCalib::XPOS] = "X+";
  orientation_labels_[AccelCalib::XNEG] = "X-";
  orientation_labels_[AccelCalib::YPOS] = "Y+";
  orientation_labels_[AccelCalib::YNEG] = "Y-";
  orientation_labels_[AccelCalib::ZPOS] = "Z+";
  orientation_labels_[AccelCalib::ZNEG] = "Z-";
}

bool DoCalib::running()
{
  return state_ != DONE;
}

void DoCalib::imuCallback(sensor_msgs::Imu::ConstPtr imu)
{
  bool accepted;

  switch (state_)
  {
  case START:
    calib_.beginCalib(6*measurements_per_orientation_, reference_acceleration_);
    state_ = SWITCHING;
    break;


  case SWITCHING:
    if (orientations_.empty())
    {
      state_ = COMPUTING;
    }
    else
    {
      current_orientation_ = orientations_.front();

      orientations_.pop();
      measurements_received_ = 0;

      std::cout << "Orient IMU with " << orientation_labels_[current_orientation_] << " axis up and press Enter";
      std::cin.get();
      std::cout << "Recording measurements...";

      state_ = RECEIVING;
    }
    break;


  case RECEIVING:
    accepted = calib_.addMeasurement(current_orientation_,
                                     imu->linear_acceleration.x,
                                     imu->linear_acceleration.y,
                                     imu->linear_acceleration.z);

    measurements_received_ += accepted ? 1 : 0;
    if (measurements_received_ >= measurements_per_orientation_)
    {
      std::cout << " Done." << std::endl;
      state_ = SWITCHING;
    }
    break;


  case COMPUTING:
    std::cout << "Computing calibration parameters...";
    if (calib_.computeCalib())
    {
      std::cout << " Success!"  << std::endl;

      std::cout << "Saving calibration file...";
      if (calib_.saveCalib(output_file_))
      {
        std::cout << " Success!" << std::endl;
      }
      else
      {
        std::cout << " Failed." << std::endl;
      }
    }
    else
    {
      std::cout << " Failed.";
      ROS_ERROR("Calibration failed");
    }
    state_ = DONE;
    break;


  case DONE:
    break;
  }
}

} // namespace accel_calib
