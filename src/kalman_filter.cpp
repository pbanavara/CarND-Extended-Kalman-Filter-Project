#include <iostream>
using namespace std;
#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
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
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_[0];
  float py = x_[1];
  float px_1  = x_[2];
  float py_1  = x_[3];
  //Check for division by zero
  float eps = 0.000001;  // Make sure we don't divide by 0.
  if (fabs(px) < eps && fabs(py) < eps) {
    px = eps;
    py = eps;
  } else if (fabs(px) < eps) {
     px = eps;
  }
  float ro = sqrt(px * px + py * py);
  float phi = atan2(py, px);
  float ro_dot = (px * px_1 + py * py_1) / ro;
  VectorXd hx(3);
  hx << ro, phi, ro_dot;
  VectorXd y = z - hx;
  cout << "!z_ = " << z(1) << "!!!" << std::endl;
  cout << "!!hx_ = " << hx(1) << "!!!" << std::endl;
  cout << "!!!y_ = " << y(1) << "!!!" << std::endl;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //Angle normalization
  y[1] -= (2 * M_PI) * floor((y[1] + M_PI) / (2 * M_PI));
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  /*
  while (y[1]>M_PI) {
    y(1) -= 2 * M_PI;
  }
  while (y[1]<-M_PI) {
    y(1) += 2 * M_PI;
  }
  */
}
