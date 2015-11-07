/**
 * \file accel_calib.cpp
 * \author Daniel Koch <danielpkoch@gmail.com>
 *
 * Class for calculating and applying accelerometer calibration parameters
 */

#include "accel_calib/accel_calib.h"

namespace accel_calib
{

const int reference_index_[] = { 0, 0, 1, 1, 2, 2 };
const int reference_sign_[] = { 1, -1, 1, -1, 1, -1 };

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

  meas_ = Eigen::SparseMatrix<double>(3*measurements, 12);
  meas_.reserve(4*3*measurements);

  ref_ = Eigen::SparseMatrix<double>(3*measurements, 1);
  ref_.reserve(measurements);
}

bool AccelCalib::addMeasurement(AccelCalib::Orientation orientation, double ax, double ay, double az)
{
  if (measurements_received_ < num_measurements_)
  {
    for (int i = 0; i < 3; i++)
    {
      meas_.insert(3*measurements_received_ + i, 3*i) = ax;
      meas_.insert(3*measurements_received_ + i, 3*i + 1) = ay;
      meas_.insert(3*measurements_received_ + i, 3*i + 2) = az;

      meas_.insert(3*measurements_received_ + i, 9 + i) = 1.0;
    }

    ref_.insert(3*measurements_received_ + reference_index_[orientation], 0) = reference_sign_[orientation] *  reference_acceleration_;

    measurements_received_++;
    return true;
  }
  else
  {
    return false;
  }
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
