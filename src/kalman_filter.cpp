#include "kalman_filter.h"

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
  MatrixXd Ft_ = F_.transpose();
  P_ = F_ * P_ * Ft_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  MatrixXd Ht_ = H_.transpose();
  MatrixXd S_ = H_ * P_ * Ht_ + R_;
  MatrixXd Si_ = S_.inverse();
  MatrixXd K_ = P_ * Ht_ * Si_;

  //Measurement update
  x_ = x_ + ( K_ * y);
  MatrixXd I_ = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I_ - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho;
  float c1 = px*px + py*py;

   // Avoid division by zero
  if (fabs(c1) < 0.00001) {
      c1 = 0.00001;
  }
  else {rho = sqrt(c1);}
  
  float phi = atan2(py, px);
  float rho_dot = (px * vx + py * vy) / rho; 

  VectorXd h(3);
  h << rho, phi, rho_dot;
  VectorXd y = z - h;

  while(y[1] > M_PI) {
    y[1] -= 2 * M_PI;
  }
  while(y[1] < -M_PI) {
    y[1] += 2 * M_PI;
  }

  MatrixXd Ht_ = H_.transpose();
  MatrixXd S_ = H_ * P_ * Ht_ + R_;
  MatrixXd Si_ = S_.inverse();
  MatrixXd K_ = P_ * Ht_ * Si_;

  //Measurement update
  x_ = x_ + ( K_ * y);
  MatrixXd I_ = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I_ - K_ * H_) * P_;
}
