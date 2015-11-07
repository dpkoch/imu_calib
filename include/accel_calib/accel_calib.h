/**
 * \file accel_calib.h
 * \author Daniel Koch <danielpkoch@gmail.com>
 *
 * Class for calculating and applying accelerometer calibration parameters
 */

#include <Eigen/Core>

#include <string>

namespace accel_calib
{

class AccelCalib
{
public:

  enum Orientation { XPOS, XNEG, YPOS, YNEG, ZPOS, ZNEG };

  AccelCalib();
  AccelCalib(std::string calib_file);

  // status
  bool calibReady();

  // file I/O
  void loadCalib(std::string calib_file);
  void saveCalib(std::string calib_file);

  // calibration procedure
  void beginCalib(int measurements, double reference_acceleration);
  void addMeasurement(Orientation orientation, double ax, double ay, double az);
  void computeCalib();

  // calibration application
  void applyCalib(double raw[3], double corrected[3]);

protected:
  bool calib_ready_;

  Eigen::Matrix3d SM_; //!< combined scale and misalignment parameters
  Eigen::Vector3d bias_; //!< scaled and rotated bias parameters

  double reference_acceleration_; //!< expected acceleration measurement (e.g. 1.0 for unit of g's, 9.80665 for unit of m/s^2)

  Eigen::MatrixXd meas_; //!< least squares measurements matrix
  Eigen::VectorXd ref_; //!< least squares expected measurements vector
  int num_measurements_; //!< number of measurements expected for this calibration
  int measurements_received_; //!< number of measurements received for this calibration
};

} // namespace accel_calib
