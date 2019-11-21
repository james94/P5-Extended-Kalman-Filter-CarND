#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

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
                              const vector<VectorXd> &ground_truth) {
  // Initialize rmse
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Initialize number of elements in estimation vector
  int estimations_size = estimations.size();

  // check the validity for inputs:
  // the estimation vector size should not be zero
  // the estimation vector size should equal ground truth vector size
  if(estimations_size == 0 || (estimations_size != ground_truth.size()))
  {
     cout << "Invalid estimation or ground truth data" << endl;
  }

  else
  {
     // accumulate squared residuals
     for(int i = 0; i < estimations_size; ++i)
     {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual; // compute residual sum
     }

     // calculate the residua mean
     rmse = rmse/estimations_size;
     // calculate the squared root of mean
     rmse = rmse.array().sqrt();
  }
  
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
