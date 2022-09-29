#ifndef UKF_H
#define UKF_H

#include <iostream>

#include "Eigen/Dense"
#include "measurement_package.h"

/// @brief Encapsulates the implementation of unscented Kalman filter. This class
/// maintains the public and internal states of UKF. Public state includes the predicted
/// mean state vector, sigma points and state covariance matrix among others. Internal
/// state includes common measurement space dimension, translated state vector, translated
/// sigma points into measurement space and measurement covariance matrix etc.
class UKF
{
public:
  /// @brief Constructor
  UKF();

  /// @brief Destructor
  virtual ~UKF();

  /// Public member functions

  /**
   * @brief Acts as an entrypoint for the sensor measurements. Before invoking
   * appropriate update method for the sensor type it first translates the `a
   * priori` predicted sigma points state into measurement space depending on
   * the sensor type. This method maintains these intermediate states using
   * internal properties.
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * @brief Predicts sigma points, the state, and the state covariance
   * matrix. This method executes the process model to derive the `a priori`
   * state.
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * @brief the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * @brief the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /// Public member attributes

  // Initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // If this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // If this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // State covariance matrix
  Eigen::MatrixXd P_;

  // Predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // Time stamp in microseconds
  long long timestamp_mis_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;

private:
  /// Private member functions

  /**
   * @brief Executes the augmentation technique in order to incorporate the
   * longitudinal and yaw acceleration noise components into state vector,
   * state covariance matrix and sigma points. This method is meant to be
   * called by Prediction method before computing `a priori` state.
   *
   * @param X_aug_out Augmented sigma point matrix from the current state.
   * Expected dimension R^(n_aug_, 2 * n_aug_ + 1)
   */
  void augmentState(Eigen::MatrixXd *X_aug_out);

  /// @brief Translates the predicted apriori sigma points to Radar
  /// measurement space.
  void translateStateToRadar();

  /// @brief Translates the predicted apriori sigma points to Lidar
  /// measurement space.
  void translateStateToLidar();

  /// Private member attributes

  /// While defining the private member attributes we would try to avoid having
  /// unnecessary memory footprints. For instance, while defining the dimensions
  /// for predicted mean state vector, sigma point matrix and state covariance
  /// matrix we can compare the Lidar and Radar measurement dimensions and take
  /// the greater of the two. While computing these vector quantities we'd then
  /// consider the appropriate measurement dimensions.

  // Measurement space dimensions
  int n_z_radar_; // 3
  int n_z_lidar_; // 2

  // Sigma points matrix for measurement space for processing Radar
  Eigen::MatrixXd Z_sig_radar_; // Should have dimension R^(n_z_radar_, 2 * n_aug_ + 1)
  // Predicted mean measurement vector for Radar
  Eigen::VectorXd z_pred_radar_; // Should have dimension R^(n_z_radar_)
  // Predicted measurement covariance matrix for Radar
  Eigen::MatrixXd S_radar_; // Should have dimension R^(n_z_radar_, n_z_radar_)

  // Linear state transformation matrix for Lidar
  Eigen::MatrixXd H_lidar_; // Should have dimension R^(n_z_lidar_, n_x_)
  // Measurement noise covariance matrix for Lidar
  Eigen::MatrixXd R_lidar_; // Should have dimension R^(n_z_lidar_, n_z_lidar_)
  // Predicted mean measurement vector for Lidar
  Eigen::VectorXd z_pred_lidar_; // Should have dimension R^(n_z_lidar_);
  // Predicted measurement covariance matrix for Lidar
  Eigen::MatrixXd S_lidar_; // Should have dimension R^(n_z_lidar_, n_z_lidar_)
};

#endif // UKF_H