#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  //cout << "KalmanFilter::Predict - 1" << endl;
  // Prediction based on the latest known values - the same regardless the sensor type
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  //cout << "KalmanFilter::Predict - 2" << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  //cout << "KalmanFilter::Update - 1" << endl;
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  //calculate Kalman matrices
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  //cout << "KalmanFilter::Update - 2" << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  //cout << "KalmanFilter::UpdateEKF - 1" << endl;
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  VectorXd z_pred = VectorXd(3);
  // Transfer the prediction from cartesian coordinates to polar
  double rho = sqrt(px*px + py*py);
  if (rho < 1.0e-4) {
    rho = 1.0e-4;
  }
  double theta = atan2(py, px);
  double rho_dot = (px*vx + py*vy) / rho;
  z_pred << rho, theta, rho_dot;
  VectorXd y = z - z_pred;

  // Make sure the input to the Kalman gain is in the expected range [-pi,+pi]
  while (y(1) >  M_PI) y(1) -= 2.0*M_PI;
  while (y(1) < -M_PI) y(1) += 2.0*M_PI;

  //calculate Kalman matrices
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  //cout << "KalmanFilter::UpdateEKF - 2" << endl;
}
