/**
 * \file accel_calib.h
 * \author Daniel Koch <danielpkoch@gmail.com>
 *
 * Class for calculating and applying accelerometer calibration parameters
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <string>

namespace accel_calib
{

class AccelCalib
{
public:

  enum Orientation { XPOS = 0, XNEG, YPOS, YNEG, ZPOS, ZNEG };

  AccelCalib();
  AccelCalib(std::string calib_file);

  // status
  bool calibReady();

  // file I/O
  bool loadCalib(std::string calib_file);
  bool saveCalib(std::string calib_file);

  // calibration procedure
  void beginCalib(int measurements, double reference_acceleration);
  bool addMeasurement(Orientation orientation, double ax, double ay, double az);
  bool computeCalib();

  // calibration application
  void applyCalib(double raw[3], double corrected[3]);

protected:
  static const int reference_index_[6];
  static const int reference_sign_[6];
  bool calib_ready_;

  Eigen::Matrix3d SM_; //!< combined scale and misalignment parameters
  Eigen::Vector3d bias_; //!< scaled and rotated bias parameters

  double reference_acceleration_; //!< expected acceleration measurement (e.g. 1.0 for unit of g's, 9.80665 for unit of m/s^2)

  bool calib_initialized_;
  int orientation_count_[6];

  Eigen::SparseMatrix<double> meas_; //!< least squares measurements matrix
  Eigen::MatrixXd ref_; //!< least squares expected measurements vector
  int num_measurements_; //!< number of measurements expected for this calibration
  int measurements_received_; //!< number of measurements received for this calibration
};

} // namespace accel_calib
