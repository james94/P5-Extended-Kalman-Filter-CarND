#include "tools.h"
#include <iostream>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

/**
* Calculate the RMSE
* 
* Initializes variables for RMSE, checks the validity of the
* estimation and ground truth data, accumulates squared residuals,
* computes the mean and square root of mean to calculate the RMSE
*/
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
   // Initialize rmse
   VectorXd rmse(4);
   rmse << 0, 0, 0, 0;

   // Initialize number of elements in estimation vector
   int estimations_size = estimations.size();

   // check the validity for inputs:
   // the estimation vector size should not be zero
   // the estimation vector size should equal ground truth vector size
   if (estimations_size == 0 || (estimations_size != ground_truth.size()))
   {
      cout << "Invalid estimation or ground truth data" << endl;
   }
   else
   {
      // accumulate squared residuals
      for (unsigned int i = 0; i < estimations_size; ++i)
      {
         VectorXd residual = estimations[i] - ground_truth[i];
         residual = residual.array() * residual.array();
         rmse += residual; // compute residual sum
      }

      // calculate the residua mean
      rmse = rmse / estimations_size;
      // calculate the squared root of mean
      rmse = rmse.array().sqrt();
   }

   // return the result
   return rmse;
}

/**
* Calculate a Jacobian Matrix
* 
* Recovers state components px, py, vx, vy, pre-computes a set
* of terms to avoid repeated calculation, checks division by
* zero and computes jacobian matrix
*/
MatrixXd Tools::CalculateJacobian(const VectorXd &x_state)
{
   // Initialize Jacobian Matrix
   MatrixXd Hj(3, 4);

   // Recover state parameters
   double px = x_state(0);
   double py = x_state(1);
   double vx = x_state(2);
   double vy = x_state(3);

   // Pre-compute a set of terms to avoid repeated calculation
   double c1 = pow(px, 2) + pow(py, 2);
   double c2 = sqrt(c1);
   double c3 = (c1 * c2);

   // Check division by zero: x nor y should be 0
   if (px == 0 && py == 0)
   {
      cout << "CalculationJacobian () - Error - Division by Zero" << endl;
   }
   // Compute the Jacobian matrix
   else
   {
      Hj << (px / c2), (py / c2), 0, 0,
          -(py / c1), (px / c1), 0, 0,
          py * (vx * py - vy * px) / c3, px * (vy * px - vx * py) / c3, (px / c2), (py / c2);
   }
   return Hj;
}
