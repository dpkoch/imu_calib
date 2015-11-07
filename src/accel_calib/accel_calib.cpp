/**
 * \file accel_calib.cpp
 * \author Daniel Koch <danielpkoch@gmail.com>
 *
 * Class for calculating and applying accelerometer calibration parameters
 */

#include "accel_calib/accel_calib.h"

namespace accel_calib
{

AccelCalib::AccelCalib() : calib_ready_(false) {}

AccelCalib::AccelCalib(std::string calib_file)
{
  AccelCalib();
  loadCalib(calib_file);
}

bool AccelCalib::calibReady()
{
  return calib_ready_;
}

void AccelCalib::loadCalib(std::string calib_file)
{
}

void AccelCalib::saveCalib(std::string calib_file)
{
}

void AccelCalib::beginCalib(int measurements, double reference_acceleration)
{
  reference_acceleration_ = reference_acceleration;

  num_measurements_ = measurements;
  measurements_received_ = 0;

  //TODO initialize matrices
}

void AccelCalib::addMeasurement(AccelCalib::Orientation orientation, double ax, double ay, double az)
{
}

void AccelCalib::computeCalib()
{
}

void AccelCalib::applyCalib(double raw[3], double corrected[3])
{
  Eigen::Vector3d raw_accel(raw[0], raw[1], raw[2]);

  Eigen::Vector3d corrected_accel = SM_*raw_accel - bias_;

  corrected[0] = corrected_accel(0);
  corrected[1] = corrected_accel(1);
  corrected[2] = corrected_accel(2);
}

} // namespace accel_calib
