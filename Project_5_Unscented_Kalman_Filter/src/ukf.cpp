#include <iostream>
#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd::Zero(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 4;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 3;
  
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
      
  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // initialize predictied sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Weights of sigma points
  weights_  = VectorXd(2 * n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  double weight = 0.5/(lambda_+n_aug_);
  
  weights_(0) = weight_0;
  for (int i=1; i < 2*n_aug_+1; ++i) {  
    weights_(i) = weight;
  }
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  if (!use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) return;
  else if (!use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) return;

  if (!is_initialized_)
  {
    // initialize state x
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_ << meas_package.raw_measurements_[0],
            meas_package.raw_measurements_[1],
            0,
            0,
            0;
    }
    else
    {
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];
      double px = rho * cos(phi);
      double py = rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      double v = sqrt(vx * vx + vy * vy);
      x_ << px,
            py,
            v,
            0,
            0;
    }
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  auto prev_time_us = time_us_;
  time_us_ = meas_package.timestamp_;
  auto delta_t = ((double)time_us_ - (double)prev_time_us) / 1000000.0;

  // state prediction
  Prediction(delta_t);

  // update (switch between lidar and radar measurements)
  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    UpdateLidar(meas_package);
  }
  else
  {
    UpdateRadar(meas_package);
  }

}

void UKF::Prediction(double delta_t) {
  /**
   * Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, and the state covariance matrix.
   */

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.fill(0.0);

  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);
  x_aug.fill(0.0);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.fill(0.0);

  // augmented mean state
  x_aug << x_(0),
           x_(1),
           x_(2),
           x_(3),
           x_(4),
           0,
           0;

  // create process noise covariance matrix
  MatrixXd Q = MatrixXd(2, 2);
  Q << std_a_ * std_a_, 0,
       0, std_yawdd_ * std_yawdd_;

  // create augmented covariance matrix
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug.bottomRightCorner(2, 2) = Q;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; ++i)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  // predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // noise part
    double mu_0 = 0.5 * delta_t * delta_t * cos(Xsig_aug(3, i)) * Xsig_aug(5, i);
    double mu_1 = 0.5 * delta_t * delta_t * sin(Xsig_aug(3, i)) * Xsig_aug(5, i);
    double mu_2 = delta_t * Xsig_aug(5, i);
    double mu_3 = 0.5 * delta_t * delta_t * Xsig_aug(6, i);
    double mu_4 = delta_t * Xsig_aug(6, i);

    if (fabs(Xsig_aug(4, i)) < 0.0001)
    {
      Xsig_pred_(0, i) = Xsig_aug(0, i) + Xsig_aug(2, i) * cos(Xsig_aug(4, i)) * delta_t + mu_0;
      Xsig_pred_(1, i) = Xsig_aug(1, i) + Xsig_aug(2, i) * sin(Xsig_aug(4, i)) * delta_t + mu_1;
      Xsig_pred_(2, i) = Xsig_aug(2, i) + mu_2;
      Xsig_pred_(3, i) = Xsig_aug(3, i) + Xsig_aug(4, i) * delta_t + mu_3;
      Xsig_pred_(4, i) = Xsig_aug(4, i) + mu_4;
    }
    else
    {
      double t0 = sin(Xsig_aug(3, i) + Xsig_aug(4, i) * delta_t) - sin(Xsig_aug(3, i));
      double t1 = -cos(Xsig_aug(3, i) + Xsig_aug(4, i) * delta_t) + cos(Xsig_aug(3, i));

      Xsig_pred_(0, i) = Xsig_aug(0, i) + (Xsig_aug(2, i) / Xsig_aug(4, i)) * t0 + mu_0;
      Xsig_pred_(1, i) = Xsig_aug(1, i) + (Xsig_aug(2, i) / Xsig_aug(4, i)) * t1 + mu_1;
      Xsig_pred_(2, i) = Xsig_aug(2, i) + mu_2;
      Xsig_pred_(3, i) = Xsig_aug(3, i) + Xsig_aug(4, i) * delta_t + mu_3;
      Xsig_pred_(4, i) = Xsig_aug(4, i) + mu_4;
    }
  }

  // predict state mean
  x_.fill(0.0);
  for (int i = 0; i < n_x_; i++)
  {
    double sum = 0.0;
    for (int j = 0; j < 2 * n_aug_ + 1; j++)
    {
      sum = sum + weights_(j) * Xsig_pred_(i, j);
    }
    x_(i) = sum;
  }

  // predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd v1 = VectorXd(n_x_);
    v1 = Xsig_pred_.col(i) - x_;

    while (v1(3) > M_PI)
      v1(3) -= 2. * M_PI;
    while (v1(3) < -M_PI)
      v1(3) += 2. * M_PI;

    P_ = P_ + weights_(i) * v1 * v1.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * Use lidar data to update the belief about the object's position. 
   * Modify the state vector, x_, and covariance, P_.
   */

  // set measurement dimension, lidar can measure positions x and y
  int n_z = 2;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  Zsig.fill(0.0);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // measurement model
    Zsig(0, i) = Xsig_pred_(0, i);
    Zsig(1, i) = Xsig_pred_(1, i);
  }

  // mean predicted measurement
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  MatrixXd R = MatrixXd(n_z, n_z);
  R.fill(0.0);
  R(0, 0) = std_laspx_ * std_laspx_;
  R(1, 1) = std_laspy_ * std_laspy_;
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd z_residual = VectorXd(n_z);
    z_residual = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_residual * z_residual.transpose();
  }
  S = S + R;

  // incoming lidar measurement
  VectorXd z = VectorXd(n_z);
  z.fill(0.0);
  z << meas_package.raw_measurements_[0], // position x
       meas_package.raw_measurements_[1]; // position y

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  // calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd v1 = VectorXd(n_x_);
    VectorXd v2 = VectorXd(n_z);
    v1 = Xsig_pred_.col(i) - x_;

    // angle normalization
    while (v1(3) > M_PI)
      v1(3) -= 2. * M_PI;
    while (v1(3) < -M_PI)
      v1(3) += 2. * M_PI;

    v2 = Zsig.col(i) - z_pred;
    Tc = Tc + weights_(i) * v1 * v2.transpose();
  }

  // calculate Kalman gain K;
  MatrixXd KGain = MatrixXd(n_x_, n_z);
  KGain.fill(0.0);
  KGain = Tc * S.inverse();

  // update state mean and covariance matrix
  x_ = x_ + KGain * (z - z_pred);
  P_ = P_ - KGain * S * KGain.transpose();
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * Use radar data to update the belief about the object's position. 
   * Modify the state vector, x_, and covariance, P_.
   */

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  Zsig.fill(0.0);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // measurement model
    Zsig(0, i) = sqrt(Xsig_pred_(0, i) * Xsig_pred_(0, i) + Xsig_pred_(1, i) * Xsig_pred_(1, i));
    Zsig(1, i) = atan2(Xsig_pred_(1, i), Xsig_pred_(0, i));
    double numerator_term = Xsig_pred_(0, i) * cos(Xsig_pred_(3, i)) * Xsig_pred_(2, i) + Xsig_pred_(1, i) * sin(Xsig_pred_(3, i)) * Xsig_pred_(2, i);
    Zsig(2, i) = numerator_term / Zsig(0, i);
  }

  // mean predicted measurement
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  MatrixXd R = MatrixXd(n_z, n_z);
  R.fill(0.0);
  R(0, 0) = std_radr_ * std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd z_residual = VectorXd(n_z);
    z_residual = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_residual(1) > M_PI)
      z_residual(1) -= 2. * M_PI;
    while (z_residual(1) < -M_PI)
      z_residual(1) += 2. * M_PI;

    S = S + weights_(i) * z_residual * z_residual.transpose();
  }
  S = S + R;

  // incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z.fill(0.0);
  z << meas_package.raw_measurements_[0], // rho in m
      meas_package.raw_measurements_[1],  // phi in rad
      meas_package.raw_measurements_[2];  // rho_dot in m/s

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  // calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd v1 = VectorXd(n_x_);
    VectorXd v2 = VectorXd(n_z);
    v1 = Xsig_pred_.col(i) - x_;

    // angle normalization
    while (v1(3) > M_PI)
      v1(3) -= 2. * M_PI;
    while (v1(3) < -M_PI)
      v1(3) += 2. * M_PI;

    v2 = Zsig.col(i) - z_pred;

    // angle normalization
    while (v2(1) > M_PI)
      v2(1) -= 2. * M_PI;
    while (v2(1) < -M_PI)
      v2(1) += 2. * M_PI;

    Tc = Tc + weights_(i) * v1 * v2.transpose();
  }

  // calculate Kalman gain K;
  MatrixXd KGain = MatrixXd(n_x_, n_z);
  KGain.fill(0.0);
  KGain = Tc * S.inverse();

  VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1) > M_PI)
    z_diff(1) -= 2. * M_PI;
  while (z_diff(1) < -M_PI)
    z_diff(1) += 2. * M_PI;

  // update state mean and covariance matrix
  x_ = x_ + KGain * (z_diff);
  P_ = P_ - KGain * S * KGain.transpose();
}