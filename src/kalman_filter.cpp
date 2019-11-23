#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

/**
* predict the x state and P state covariance
*/
void KalmanFilter::Predict()
{
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

/**
* Update the measurement of lidar x state 
* and P state covariance by using Kalman 
* Filter equations
*/
void KalmanFilter::Update(const VectorXd &z)
{
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  UpdateWithY(y);
}

/**
* Update the measurement of radar x state 
* and P state covariance by using Extended 
* Kalman Filter equations
*/
void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  // Recover state parameters
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho = sqrt(px * px + py * py);
  double phi = atan2(py, px);
  double rho_dot = (px * vx + py * vy) / (rho);

  VectorXd hx_func(3);
  hx_func << rho, phi, rho_dot;

  // compute state error between actual and predicted state 
  VectorXd y = z - hx_func;

  // Normalize angle in y vector to be between -PI and +PI
  while (y(1) > M_PI)
  {
    y(1) -= 2 * M_PI;
  }

  while (y(1) < -M_PI)
  {
    y(1) += 2 * M_PI;
  }

  UpdateWithY(y);
}

void KalmanFilter::UpdateWithY(const VectorXd &y)
{
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate x state vector and P covariance matrix
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}