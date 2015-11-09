/**
 * \file accel_calib.cpp
 * \author Daniel Koch <danielpkoch@gmail.com>
 *
 * Class for calculating and applying accelerometer calibration parameters
 */

#include "accel_calib/accel_calib.h"

#include <yaml-cpp/yaml.h>

#include <fstream>

namespace accel_calib
{

const int AccelCalib::reference_index_[] = { 0, 0, 1, 1, 2, 2 };
const int AccelCalib::reference_sign_[] = { 1, -1, 1, -1, 1, -1 };

AccelCalib::AccelCalib() :
  calib_ready_(false),
  calib_initialized_(false) {}

AccelCalib::AccelCalib(std::string calib_file)
{
  AccelCalib();
  loadCalib(calib_file);
}

bool AccelCalib::calibReady()
{
  return calib_ready_;
}

bool AccelCalib::loadCalib(std::string calib_file)
{
  try
  {
    YAML::Node node = YAML::LoadFile(calib_file);

    assert(node["SM"].IsSequence() && node["SM"].size() == 9);
    assert(node["bias"].IsSequence() && node["bias"].size() == 3);

    for (int i = 0; i < 9; i++)
    {
      SM_(i/3,i%3) = node["SM"][i].as<double>();
    }

    for (int i = 0; i < 3; i++)
    {
      bias_(i) = node["bias"][i].as<double>();
    }

    calib_ready_ = true;
    return true;
  }
  catch (...)
  {
    return false;
  }
}

bool AccelCalib::saveCalib(std::string calib_file)
{
  if (!calib_ready_)
    return false;

  YAML::Node node;
  for (int i = 0; i < 9; i++)
  {
      node["SM"].push_back(SM_(i/3,i%3));
  }

  for (int i = 0; i < 3; i++)
  {
    node["bias"].push_back(bias_(i));
  }

  try
  {
    std::ofstream fout;
    fout.open(calib_file.c_str());
    fout << node;
    fout.close();
  }
  catch (...)
  {
    return false;
  }

  return true;
}

void AccelCalib::beginCalib(int measurements, double reference_acceleration)
{
  reference_acceleration_ = reference_acceleration;

  num_measurements_ = measurements;
  measurements_received_ = 0;

  meas_.resize(3*measurements, 12);
  meas_.reserve(4*3*measurements);

  ref_.resize(3*measurements, 1);

  memset(orientation_count_, 0, sizeof(orientation_count_));
  calib_initialized_ = true;
}

bool AccelCalib::addMeasurement(AccelCalib::Orientation orientation, double ax, double ay, double az)
{
  if (calib_initialized_ && measurements_received_ < num_measurements_)
  {
    for (int i = 0; i < 3; i++)
    {
      meas_.insert(3*measurements_received_ + i, 3*i) = ax;
      meas_.insert(3*measurements_received_ + i, 3*i + 1) = ay;
      meas_.insert(3*measurements_received_ + i, 3*i + 2) = az;

      meas_.insert(3*measurements_received_ + i, 9 + i) = -1.0;
    }

    ref_(3*measurements_received_ + reference_index_[orientation], 0) = reference_sign_[orientation] *  reference_acceleration_;

    measurements_received_++;
    orientation_count_[orientation]++;

    return true;
  }
  else
  {
    return false;
  }
}

bool AccelCalib::computeCalib()
{
  // check status
  if (measurements_received_ < 12)
    return false;

  for (int i = 0; i < 6; i++)
  {
    if (orientation_count_[i] == 0)
      return false;
  }

  // solve least squares
  Eigen::SparseQR< Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int> > solver;
  meas_.makeCompressed();
  solver.compute(meas_);

  if (solver.info() != Eigen::Success)
    return false;

  Eigen::VectorXd xhat = solver.solve(ref_);

  if (solver.info() != Eigen::Success)
    return false;

  // extract solution
  for (int i = 0; i < 9; i++)
  {
    SM_(i/3, i%3) = xhat(i);
  }

  for (int i = 0; i < 3; i++)
  {
    bias_(i) = xhat(9+i);
  }

  calib_ready_ = true;
  return true;
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
