#include "ukf.h"

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
  /// Initialize public member attributes

  // UKF initialization flag
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // initial state vector
  // NOTE: As a starting point we initialized as random. When the first Lidar
  // measurement comes, we'd set the position_x and position_y. Until then we
  // would consider the UKF state as uninitialized.
  // The random initialization is subjected to tuning.
  x_ = Eigen::VectorXd(n_x_);

  // initial covariance matrix
  P_ = Eigen::MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1; // Starting value for longitudinal acceleration noise

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.8; // Starting value for yaw acceleration noise

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
  // unscented Kalman filter at each time step. We initialize this as 0-value matrix.
  Xsig_pred_ = Eigen::MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

  /// Initialize private member attributes

  // Common measurement space dimension for Lidar and Radar.
  n_z_radar_ = 3;
  n_z_lidar_ = 2;

  // Sigma points matrix for measurement space
  Z_sig_radar_ = Eigen::MatrixXd::Zero(n_z_radar_, 2 * n_aug_ + 1);
  // Predicted mean measurement vector
  z_pred_radar_ = Eigen::VectorXd::Zero(n_z_radar_);
  // Predicted measurement covariance matrix
  S_radar_ = Eigen::MatrixXd::Zero(n_z_radar_, n_z_radar_);

  // Sigma point matrix for measurement space for processing Lidar
  Z_sig_lidar_ = Eigen::MatrixXd::Zero(n_z_lidar_, 2 * n_aug_ + 1);
  // Predicted mean measurement vector for Lidar
  z_pred_lidar_ = Eigen::VectorXd::Zero(n_z_lidar_);
  // Predicted measurement covariance matrix for Lidar
  S_lidar_ = Eigen::MatrixXd::Zero(n_z_lidar_, n_z_lidar_);
}

UKF::~UKF() {}

void UKF::augmentState(Eigen::MatrixXd *X_aug_out)
{
  // Create augmented mean vector. This would automatically
  // incorporate that noise components have 0-mean
  Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(n_aug_);

  // Create augmented state covariance
  Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(n_aug_, n_aug_);

  // Create sigma point matrix
  Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);

  // Create augmented mean state
  x_aug.head(n_x_) = x_;

  // Create augmented covariance matrix
  Eigen::Matrix2d Q;
  // Noises are provided as standard deviation. We need to square them to
  // derive variance
  Q << std::pow(std_a_, 2.0), 0,
      0, std::pow(std_yawdd_, 2.0);
  P_aug.block(0, 0, n_x_, n_x_) = P_;
  P_aug.block(n_x_, n_x_, 2, 2) = Q;

  // Create square root matrix
  Eigen::MatrixXd A = P_aug.llt().matrixL();

  // Create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  Xsig_aug.block(0, 1, n_aug_, n_aug_) = (std::sqrt(lambda_ + n_aug_) * A).colwise() + x_aug;
  Xsig_aug.block(0, n_aug_ + 1, n_aug_, n_aug_) = (-std::sqrt(lambda_ + n_aug_) * A).colwise() + x_aug;
  *X_aug_out = Xsig_aug;
}

void UKF::translateStateToRadar()
{
  // Prepare sigma points matrix for measurement space
  Z_sig_radar_.setZero();
  // Prepare mean measurement vector
  z_pred_radar_.setZero();
  // Prepare measurement covariance matrix
  S_radar_.setZero();

  // Transform sigma points into measurement space
  for (size_t idx = 0; idx < 2 * n_aug_ + 1; idx++)
  {
    Eigen::VectorXd predicted_sigma = Xsig_pred_.col(idx);
    double pos_x = predicted_sigma[0];
    double pos_y = predicted_sigma[1];
    double velocity = predicted_sigma[2];
    double yaw_angle = predicted_sigma[3];
    double yaw_rate = predicted_sigma[4];
    // Translate CTRV state vector components to Radar measurement components
    double radial_distance = std::sqrt(std::pow(pos_x, 2.) + std::pow(pos_y, 2.));
    double radial_angle = std::atan(pos_y / pos_x);
    double radial_velocity = ((pos_x * std::cos(yaw_angle) * velocity) + (pos_y * std::sin(yaw_angle) * velocity)) / radial_distance;
    Eigen::VectorXd meas(n_z_radar_);
    meas << radial_distance, radial_angle, radial_velocity;
    Z_sig_radar_.col(idx) = meas;
  }

  // Compute translated measurement mean
  for (size_t idx = 0; idx < 2 * n_aug_ + 1; idx++)
  {
    z_pred_radar_ += weights_[idx] * Z_sig_radar_.col(idx);
  }

  // Compute translated measurement covariance matrix S
  Eigen::MatrixXd R = Eigen::MatrixXd(n_z_radar_, n_z_radar_);
  R.fill(0.);
  R(0, 0) = std::pow(std_radr_, 2.);
  R(1, 1) = std::pow(std_radphi_, 2.);
  R(2, 2) = std::pow(std_radrd_, 2.);
  for (size_t idx = 0; idx < 2 * n_aug_ + 1; idx++)
  {
    Eigen::VectorXd z_diff = Z_sig_radar_.col(idx) - z_pred_radar_;
    // Normalize radial angle to lie in [-180, 180]
    while (z_diff[1] > M_PI)
    {
      z_diff[1] -= 2 * M_PI;
    }
    while (z_diff[1] < -M_PI)
    {
      z_diff[1] += 2 * M_PI;
    }
    S_radar_ += (weights_[idx] * z_diff * z_diff.transpose());
  }
  S_radar_ += R;
}

void UKF::translateStateToLidar()
{
  // Prepare sigma points matrix for measurement space
  Z_sig_lidar_.setZero();
  // Prepare mean measurement vector
  z_pred_lidar_.setZero();
  // Prepare measurement covariance matrix
  S_lidar_.setZero();

  // Transform sigma points into measurement space
  for (size_t idx = 0; idx < 2 * n_aug_ + 1; idx++)
  {
    Z_sig_lidar_.col(idx) = Xsig_pred_.col(idx).head(n_z_lidar_);
  }

  // Compute translated measurement mean
  for (size_t idx = 0; idx < 2 * n_aug_ + 1; idx++)
  {
    z_pred_lidar_ += weights_[idx] * Z_sig_lidar_.col(idx);
  }

  // Compute translated measurement covariance matrix S
  Eigen::MatrixXd R = Eigen::MatrixXd(n_z_lidar_, n_z_lidar_);
  R.fill(0.);
  R(0, 0) = std::pow(std_laspx_, 2.);
  R(1, 1) = std::pow(std_laspy_, 2.);
  for (size_t idx = 0; idx < 2 * n_aug_ + 1; idx++)
  {
    Eigen::VectorXd z_diff = Z_sig_lidar_.col(idx) - z_pred_lidar_;
    S_lidar_ += (weights_[idx] * z_diff * z_diff.transpose());
  }
  S_lidar_ += R;
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  // If the internal state is uninitialized and measurement is the first Lidar
  // measurement we initialize x and y positions of the state vector. We are
  // specifically pairing the initialization with the Lidar measurement because
  // other components of the state vectors is hard to initialize either using
  // Lidar or Radar. That argument is valid with the velocity magnitude as well
  // because velocity magnitude in context of UKF state is not same as radial
  // velocity available from the Radar.
  if (!is_initialized_)
  {
    x_.fill(0.);
    P_ = Eigen::MatrixXd::Identity(n_x_, n_x_);
    if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_[0] = meas_package.raw_measurements_[0];
      x_[1] = meas_package.raw_measurements_[1];
      P_(0, 0) = std::pow(std_laspx_, 2.);
      P_(1, 1) = std::pow(std_laspy_, 2.);
    }
    else if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      double radial_distance_roh = meas_package.raw_measurements_[0];
      double angle_phi = meas_package.raw_measurements_[1];
      x_[0] = radial_distance_roh * std::cos(angle_phi);
      x_[1] = radial_distance_roh * std::sin(angle_phi);
      P_(0, 0) = std::pow(std_radr_, 2.);
      P_(1, 1) = std::pow(std_radr_, 2.);
      P_(2, 2) = std::pow(std_radrd_, 2.);
      P_(3, 3) = std::pow(std_radphi_, 2.);
      P_(4, 4) = std::pow(std_radrd_, 2.);
    }
    timestamp_mis_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }
  // With program control reaching here we can assume that the UKF state vector is
  // initialized with the earlier invocation and we can then process the measurements
  // as per the sensor type.

  // Compute elapsed time delta since last measurement and update current timestamp
  double delta_t = static_cast<double>((meas_package.timestamp_ - timestamp_mis_) / 1000000.0);
  timestamp_mis_ = meas_package.timestamp_;

  // Predict phase
  this->Prediction(delta_t);

  // Update phase
  if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    this->translateStateToLidar();
    this->UpdateLidar(meas_package);
  }
  else if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    this->translateStateToRadar();
    this->UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t)
{
  /**
   * TODO: Complete this function! Estimate the object's location.
   * Modify the state vector, x_. Predict sigma points, the state,
   * and the state covariance matrix.
   */
  // Augment state to incorporate the noise components
  Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);
  this->augmentState(&Xsig_aug);
  // Execute process model and derive `a priori` state distribution
  for (size_t col_idx = 0; col_idx < 2 * n_aug_ + 1; col_idx++)
  {
    // Extract sigma point as column vector
    Eigen::VectorXd sigma = Xsig_aug.col(col_idx);
    double pos_x = sigma[0];
    double pos_y = sigma[1];
    double velocity = sigma[2];
    double yaw = sigma[3];
    double yaw_rate = sigma[4];
    double long_acc_noise = sigma[5];
    double yaw_acc_noise = sigma[6];
    // Predict sigma points with process model
    double pos_offset_x = 0.5 * std::pow(delta_t, 2.) * std::cos(yaw) * long_acc_noise;
    double pos_offset_y = 0.5 * std::pow(delta_t, 2.) * std::sin(yaw) * long_acc_noise;
    double velocity_offset = delta_t * long_acc_noise;
    double yaw_offset = 0.5 * std::pow(delta_t, 2.) * yaw_acc_noise;
    double yaw_rate_offset = delta_t * yaw_acc_noise;
    double pred_pos_x, pred_pos_y;
    if (std::fabs(yaw_rate) > 0.001) // Prevent division by zero
    {
      pred_pos_x = ((velocity / yaw_rate) * (std::sin(yaw + yaw_rate * delta_t) - std::sin(yaw))) + pos_offset_x;
      pred_pos_y = (velocity / yaw_rate) * (-std::cos(yaw + yaw_rate * delta_t) + std::cos(yaw)) + pos_offset_y;
    }
    else
    {
      pred_pos_x = (velocity * std::cos(yaw) * delta_t) + pos_offset_x;
      pred_pos_y = (velocity * std::sin(yaw) * delta_t) + pos_offset_y;
    }
    double pred_velocity = 0. + velocity_offset;
    double pred_yaw = (yaw_rate * delta_t) + yaw_offset;
    double pred_yaw_rate = 0. + yaw_rate_offset;
    // Write predicted sigma points as column vector
    Eigen::VectorXd x = sigma.head(n_x_);
    Eigen::VectorXd state_offset = Eigen::VectorXd(n_x_);
    state_offset << pred_pos_x, pred_pos_y, pred_velocity, pred_yaw, pred_yaw_rate;
    Xsig_pred_.col(col_idx) = x + state_offset;
  }
  // Compute state vector mean
  x_.fill(0.);
  for (size_t idx = 0; idx < 2 * n_aug_ + 1; idx++)
  {
    x_ += weights_[idx] * Xsig_pred_.col(idx);
  }
  // Compute state covariance matrix
  P_.fill(0.);
  for (size_t idx = 0; idx < 2 * n_aug_ + 1; idx++)
  {
    Eigen::VectorXd x_diff = Xsig_pred_.col(idx) - x_;
    // Normalize the angle measurement for the yaw. Angle
    // measurements are problematic for Kalman filter. We
    // need to be careful when state vector has an angle
    // measurement. In this implementation we are normalizing
    // an angle to lie in the range of [-180, 180].
    // Index of the yaw angle in the state vector is 3
    while (x_diff[3] > M_PI)
    {
      x_diff[3] -= 2. * M_PI;
    }
    while (x_diff[3] < -M_PI)
    {
      x_diff[3] += 2. * M_PI;
    }
    P_ += weights_[idx] * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Use lidar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(n_x_, n_z_lidar_);
  for (size_t idx = 0; idx < 2 * n_aug_ + 1; idx++)
  {
    Tc += weights_[idx] * (Xsig_pred_.col(idx) - x_) * (Z_sig_lidar_.col(idx) - z_pred_lidar_).transpose();
  }
  // Compute Kalman gain
  Eigen::MatrixXd K = Tc * S_lidar_.inverse();
  // Update state mean and covariance matrix from apriori to posterior
  // for current timestamp
  Eigen::VectorXd z = meas_package.raw_measurements_.head(n_z_lidar_);
  x_ += K * (z - z_pred_lidar_);
  P_ -= K * S_lidar_ * K.transpose();
  /// TODO: Compute NIS for tracking consistency for Lidar
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
  Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(n_x_, n_z_radar_);
  for (size_t idx = 0; idx < 2 * n_aug_ + 1; idx++)
  {
    Tc += weights_[idx] * (Xsig_pred_.col(idx) - x_) * (Z_sig_radar_.col(idx) - z_pred_radar_).transpose();
  }
  // Compute Kalman gain
  Eigen::MatrixXd K = Tc * S_radar_.inverse();
  // Update state mean and covariance matrix from apriori to posterior
  // for current timestamp
  Eigen::VectorXd z = meas_package.raw_measurements_;
  x_ += K * (z - z_pred_radar_);
  P_ -= K * S_radar_ * K.transpose();
  /// TODO: Compute NIS for tracking consistency for Radar
}