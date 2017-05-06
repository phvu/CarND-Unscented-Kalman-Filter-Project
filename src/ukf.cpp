#include "ukf.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd::Zero(5);

  // initial covariance matrix
  P_ = MatrixXd::Identity(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;

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

  time_us_ = 5000;    // 5 ms

  n_x_ = (int) x_.size();
  n_aug_ = n_x_ + 2;
  lambda_ = 3 - n_aug_;

  weights_ = VectorXd::Zero(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  weights_.tail(2 * n_aug_).fill(0.5 / (n_aug_ + lambda_));

  tools_ = Tools();

  R_laser_ = MatrixXd::Zero(2, 2);
  R_laser_ << std_laspx_, 0,
      0, std_laspy_;

  H_laser_ = MatrixXd::Zero(2, 5);
  H_laser_ << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0;
}

UKF::~UKF() {}

/**
 * @param {@MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  if (!is_initialized_) {
    initialize(meas_package);
    is_initialized_ = true;
    return;
  }

  bool is_radar = meas_package.sensor_type_ == MeasurementPackage::RADAR;
  long long dt = meas_package.timestamp_ - previous_timestamp_;

  if (dt < time_us_ || (!use_laser_ && !is_radar) || (!use_radar_ && is_radar)) {
    return;
  }

  previous_timestamp_ = meas_package.timestamp_;
  Prediction(dt / 1000000.0);

  if (is_radar) {
    UpdateRadar(meas_package);
  } else {
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {@double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  VectorXd x_aug = VectorXd::Zero(n_aug_);

  // 1. generate sigma points
  x_aug.head(n_x_) = x_;

  //create augmented covariance matrix
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  // 2. predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    } else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    Xsig_aug(0, i) = px_p;
    Xsig_aug(1, i) = py_p;
    Xsig_aug(2, i) = v_p;
    Xsig_aug(3, i) = yaw_p;
    Xsig_aug(4, i) = yawd_p;
  }
  Xsig_pred_ = Xsig_aug.topLeftCorner(n_x_, Xsig_aug.cols());

  // 3. predict mean and covariance
  x_ = Xsig_pred_ * weights_;

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = tools_.normalizeAngle(x_diff(3));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {@MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  VectorXd z = VectorXd::Zero(2);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];

  VectorXd y = z - H_laser_ * x_;

  MatrixXd Ht = H_laser_.transpose();
  MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
  MatrixXd K = P_ * Ht * S.inverse();
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

  x_ = x_ + (K * y);
  P_ = (I - K * H_laser_) * P_;

  NIS_laser_ = (y.transpose() * S.inverse() * y)(0, 0);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {@MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  int n_z = 3;

  MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readability
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                        //r
    Zsig(1, i) = atan2(p_y, p_x);                                 //phi
    Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);
  z_pred = Zsig * weights_;

  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = tools_.normalizeAngle(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd::Zero(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;
  S = S + R;

  // Update x_ and P_
  // calculate cross correlation matrix
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = tools_.normalizeAngle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = tools_.normalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z = VectorXd::Zero(n_z);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];

  VectorXd z_diff = z - z_pred;
  z_diff(1) = tools_.normalizeAngle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  NIS_radar_ = (z_diff.transpose() * S.inverse() * z_diff)(0, 0);
}

void UKF::initialize(MeasurementPackage meas_package) {

  double px, py;
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

    // Convert radar from polar to cartesian coordinates
    double r = meas_package.raw_measurements_[0];
    double phi = meas_package.raw_measurements_[1];
    px = r * cos(phi);
    py = r * sin(phi);

  } else {
    px = meas_package.raw_measurements_[0];
    py = meas_package.raw_measurements_[1];
  }

  x_ << px, py, 0, 0, 0;

  long n = x_.size();
  P_ = MatrixXd::Identity(n, n);

  previous_timestamp_ = meas_package.timestamp_;
}
