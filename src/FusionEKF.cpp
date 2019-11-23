#include "FusionEKF.h"
#include "Eigen/Dense"
#include "tools.h"
#include <iostream>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // Laser measurement covariance matrix R_laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // Radar measurement covariance matrix R_radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Laser measurement matrix
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Radar jacobian measurement matrix Hj_
  Hj_ = MatrixXd(3, 4);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

/**
 * ProcessMeasurement() function
 * 
 * This function is the "Sensor Fusion General Flow":
 * 
 * This function is triggered each time it receives new measurements from a
 * lidar or radar sensor. At first iteration, the Kalman Filter position
 * vector x is initialized with the first sensor measurements. Subsequently,
 * the prediction and measurement update is called. Before prediction, the
 * elapsed time is computed between the current and previous observation.
 * Then based based on elapsed time, the new state transition and process
 * covariance matrices are calculated. The measurement update depends on
 * sensor type. If the current observation comes from a radar sensor, we
 * compute the new Jacobian Hj matrix, use the nonlinear measurement function
 * (ro, theta, ro_dot) to project the predicted state and call measurement
 * UpdateEKF(). Otherwise, if the current observation comes from a lidar sensor
 * , then we just set up the extended kalman filter with the laser H and R
 * matrices and call measurement Update(). Both update functions update
 * the state and covariance matrices.
 */
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  /**
   * Initialization
   */
  if (!is_initialized_)
  {
    // 4D state vector
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    // State covariance matrix
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    // Initial state transition matrix
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;

    // Process noise covariance matrix
    ekf_.Q_ = MatrixXd(4, 4);

    // Initialize Kalman Filter position vector with first sensor measurements

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      // Convert radar from polar to cartesian coordinate system
      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double px = rho * cos(phi);
      double py = rho * sin(phi);

      // Intialize state ekf_.x_ with first radar measurement (px, py)
      ekf_.x_ << px, // rho * cos(phi)
                 py,        // rho * sin(phi)
                 0,         // although radar gives velocity data in range rate rho dot,
                 0;         // it doesn't contain enough info to compute velocities vx,vy
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      // Intialize state ekf_.x_ with first laser measurement (px, py)
      ekf_.x_ << measurement_pack.raw_measurements_[0], // px
          measurement_pack.raw_measurements_[1],        // py
          0,                                            // there is no velocity data for lidar measurement
          0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    // Done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  // Before Prediction (delta t (dt) - expressed in seconds):
  // Compute the time elapsed between the current and previous measurements
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  // Update the state transition matrix F according to the new elapsed time
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Set process noise components ax, ay for process covariance matrix Q
  double noise_ax = 9;
  double noise_ay = 9;

  // Update the process noise covariance matrix Q based on elapsed time
  ekf_.Q_ << (dt_4 / 4) * noise_ax, 0, (dt_3 / 2) * noise_ax, 0,
             0, (dt_4 / 4) * noise_ay, 0, (dt_3 / 2) * noise_ay,
             (dt_3 / 2) * noise_ax, 0, dt_2 * noise_ax, 0,
             0, (dt_3 / 2) * noise_ay, 0, dt_2 * noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  // Use the sensor type (radar or lidar) to perform the update step
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    // Set up kalman filter with radar Hj and R matrices

    // Compute the new jacobian Hj to linearize measurement function h(x')
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;

    // Set up radar measurement covariance matrix
    ekf_.R_ = R_radar_;

    // Call measurement UpdateEKF with radar data to update state and covariance
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else
  {
    // Set up kalman filter with laser H and R matrices

    // Set up lidar measurement matrix
    ekf_.H_ = H_laser_;

    // Set up lidar measurement covariance matrix
    ekf_.R_ = R_laser_;

    // Call measurement Update with lidar data to update state and covariance
    ekf_.Update(measurement_pack.raw_measurements_);
  }
}
