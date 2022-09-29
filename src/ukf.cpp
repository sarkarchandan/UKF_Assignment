#include "ukf.h"

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
  /// Initialize public member attributes

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = Eigen::VectorXd(5);

  // initial covariance matrix
  P_ = Eigen::MatrixXd(5, 5);

  // TODO: Can we initialize state vector and covariance matrix from a standard
  // normal distribution ?

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3; // Starting value for longitudinal acceleration noise

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI; // Starting value for yaw acceleration noise

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
   * End DO NOT MODIFY section for measurement noise values
   */

  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Weights for sigma points
  weights_ = Eigen::VectorXd(2 * n_aug_ + 1);
  weights_[0] = lambda_ / (lambda_ + n_aug_);
  for (size_t idx = 1; idx < 2 * n_aug_ + 1; idx++)
  {
    weights_[idx] = 1 / (2 * (lambda_ + n_aug_));
  }

  // Predicted sigma points matrix R^(5x15)
  // Sigma points in this matrix represent the predicted state distribution by the
  // unscented Kalman filter at each time step.
  Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);

  /// Initialize private member attributes

  // Common measurement space dimension for Lidar and Radar.
  n_z_radar_ = 3;
  n_z_lidar_ = 2;
  // Taking the greater of the two
  n_z_common_ = n_z_lidar_ > n_z_radar_ ? n_z_lidar_ : n_z_radar_;
  // Common translated sigma point state distribution into measurement space for
  // Lidar and Radar
  Z_sig_common_ = Eigen::MatrixXd(n_z_common_, 2 * n_aug_ + 1);
  // Common translated state mean into measurement space for Lidar and Radar
  z_pred_common_ = Eigen::VectorXd(n_z_common_);
  // Common measurement covariance matrix for Lidar and Radar
  S_common_ = Eigen::MatrixXd(n_z_common_, n_z_common_);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  // TODO: This is the entry point for Update step. We need to keep in mind that
  // lidarSense / radarSense of the Tool class never calls the UpdateLidar or
  // UpdateRadar directly. They always call this method. In this method we have
  // to check the sensor_type_ of the MeasurementPackage and invoke the correct
  // Update function.
  // Intuitively, this method is similar to PredictRadarMeasurement method from
  // the lectures. And then it calls either of the Update method to implement
  // what we did in the UpdateState method.
}

void UKF::Prediction(double delta_t)
{
  /**
   * TODO: Complete this function! Estimate the object's location.
   * Modify the state vector, x_. Predict sigma points, the state,
   * and the state covariance matrix.
   */
  // TODO: In general, this method implements the process model. This does
  // what we did in the SigmaPointPrediction and PredictMeanAndCovariance
  // methods during the lecture.
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Use lidar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  // TODO: Identical to UpdateState method
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Use radar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  // Initialize and compute the cross correlation matrix
  Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z_radar_);
  Tc.fill(0.);
  for (size_t idx = 0; idx < 2 * n_aug_ + 1; idx++)
  {
    Tc += weights_[idx] * (Xsig_pred_.col(idx) - x_) * (Z_sig_common_.col(idx) - z_pred_common_).transpose();
  }
  // Compute Kalman gain
  Eigen::MatrixXd K = Tc * S_common_.inverse();
  // Update state mean and covariance matrix frm a priori to posterior
  // for current timestamp
  Eigen::VectorXd z = meas_package.raw_measurements_;
  x_ += K * (z - z_pred_common_);
  P_ -= K * S_common_ * K.transpose();
  // TODO: Compute NIS for tracking consistency
}