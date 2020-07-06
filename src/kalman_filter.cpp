#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
    x_ = F_ * x_;
    MatrixXd F_t = F_.transpose();
    P_ = F_ * P_ * F_t + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
    VectorXd y = z - H_ * x_;
    MatrixXd H_t = H_.transpose();
    MatrixXd S = H_ * P_ * H_t + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * H_t * Si;

    //new state
    x_ = x_ + (K * y);
    P_ = P_ - K * H_ * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    VectorXd z_pre(3);
    z_pre(0) = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
    z_pre(1) = atan2(x_(1), x_(0)); //atan only return [-pi/2, pi/2], atan2 can return [-pi/2, pi/2]
    if (z_pre(0)<0.0001) {
        z_pre(2) =0;
    }else{
        z_pre[2] = (x_(0) * x_(2) + x_(1) * x_(3)) / z_pre(0);
    }

    VectorXd y = z - z_pre;
    float pi = 3.1415;
    if (y(1) > pi) {
        y(1) = y(1) - 2*pi;
    } else if (y(1) < -pi) {
        y(1) = y(1) + 2*pi;
    }
    MatrixXd H_t = H_.transpose();
    MatrixXd S = H_ * P_ * H_t + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * H_t * Si;

    //new state
    x_ = x_ + (K * y);
    P_ = P_ - K * H_ * P_;
    
}
