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
   * @brief Acts as an entrypoint for the sensor measurements. At the start of
   * the unscented Kalman filter process cycle we initialize the UKF state in
   * this method. For the subsequent process cycles this method delegates the
   * derivation of posterior state mean and covariance matrix to internal update
   * methods depending on the sensor type.
   *
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage const &meas_package);

  /**
   * @brief Predicts sigma points, the state, and the state covariance
   * matrix. This method executes the process model to derive the apriori
   * state. This method predicts the state mean and covariance for time
   * step k+1 while the last known measurement is from previous time step
   * k.
   *
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /// Public member properties

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

  // Normalized innovation score for Lidar
  double nis_lidar_;

  // Normalized innovation score for Radar
  double nis_radar_;

private:
  /// Private member functions

  /**
   * @brief Executes the augmentation technique in order to incorporate the
   * longitudinal and yaw acceleration noise components into state mean and
   * covariance matrix and sigma points. This method is meant to be called
   * by Prediction method before computing apriori state.
   *
   * @param X_aug_out Augmented sigma point matrix from the current state.
   * Expected dimension R^(n_aug_, 2 * n_aug_ + 1)
   */
  void augmentState(Eigen::MatrixXd *X_aug_out) const;

  /**
   * @brief Updates the state and the state covariance matrix using a Lidar
   * measurement. This method derives the posterior state and covariance
   * matrix after translating the apriori state to measurement space for
   * Lidar and processing the measurement for time step k+1.
   *
   * @param meas_package The measurement at k+1
   */
  void updateLidar(MeasurementPackage const &meas_package);

  /**
   * @brief Updates the state and the state covariance matrix using a Lidar
   * measurement. This method derives the posterior state and covariance
   * matrix after translating the apriori state to measurement space for
   * Lidar and processing the measurement for time step k+1.
   *
   * NOTE: This is an experimental alternative of the UKF way of
   * dealing with the Lidar measurements. In this implementation
   * we don't make use of sigma points and it uses linear transformation
   * to derive posterior state. That means it is expected that without
   * correct tuning this method of deriving the posterior state may
   * struggle to estimate nonlinear motions components e.g., yaw angle,
   * yaw rate accurately.
   *
   * @param meas_package The measurement at k+1
   */
  void updateLidarExperimental(MeasurementPackage const &meas_package);

  /**
   * @brief Upates the state and the state covariance matrix using a radar
   * measurement. This method derives the posterior state and covariance
   * matrix after translating the apriori state to measurement space for
   * Radar and processing the measurement for time step k+1.
   *
   * @param meas_package The measurement at k+1
   */
  void updateRadar(MeasurementPackage const &meas_package);
};

#endif // UKF_H