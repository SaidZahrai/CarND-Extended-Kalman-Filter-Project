#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::endl;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   //cout << "Tools::CalculateRMSE - 1" << endl;
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  VectorXd rr(4);
  
  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if ((estimations.size() == 0) || (estimations.size() != ground_truth.size())){
     // Give an error message if the lengths are not the same, otherwise, just return 0
      cout << "Incorrect input data" << endl;
  }
  else {
      // Accumulate squared residuals
      for (int i=0; i < int(estimations.size()); ++i) {
         rr = estimations[i] - ground_truth[i];
         rr = rr.array()*rr.array();
         rmse += rr;
      }

      // Calculate the mean
      rmse = rmse/estimations.size();

      // Calculate the squared root
      rmse = rmse.array().sqrt();
  }

  //cout << "Tools::CalculateRMSE" << endl;
  //cout << "--Error: " << rr.transpose() <<  " Estimate: " << estimations[0].transpose() << " truth: " << ground_truth[0].transpose() << endl;

  // return the result
  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
   /**
      * TODO:
      * Calculate a Jacobian here.
      */
   //cout << "Tools::CalculateJacobian - 1" << endl;
   MatrixXd Hj(3,4);
   // recover state parameters
   double px = x_state(0);
   double py = x_state(1);
   double vx = x_state(2);
   double vy = x_state(3);

   // Help variables

   double r2 = px*px+py*py;
   double r  = sqrt(r2);
   double r3 = r*r2;

   // Make sure that divisions do not cause large errors
   if (r  < 1.0e-4) r  = 1.0e-4;
   if (r2 < 1.0e-4) r2 = 1.0e-4;
   if (r3 < 1.0e-4) r3 = 1.0e-4;

   Hj <<                px/r,                py/r,    0,    0,
                      -py/r2,               px/r2,    0,    0,
         py*(vx*py-vy*px)/r3, px*(vy*px-vx*py)/r3, px/r, py/r;

   //cout << "Tools::CalculateJacobian - 2" << endl;
   return Hj;
}
