#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
#include <iostream>

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
    // state transfer
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    // P updatete
    P_ = F_ * P_ * Ft + Q_;
}

// KF measurement update
void KalmanFilter::Update(const VectorXd &z) {
    // predict measurement
    VectorXd z_pred = H_ * x_;
    // measurement error
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    // Kalman gain
    MatrixXd K = P_ * Ht * Si;
    
    // update state
    x_ = x_ + (K * y);
    int x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    // update process noise covariance
    P_ = (I - K * H_) * P_;
}

// EKF measurement update
void KalmanFilter::UpdateEKF(const VectorXd &z) {
    // predict measurement, directly use non-linear transform from measurement
    VectorXd z_pred = VectorXd(z.size());
    z_pred[0] = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
    z_pred[1] = atan2f(x_[1], x_[0]);
    z_pred[2] = (x_[0] * x_[2] + x_[1] * x_[3]) / sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
    // measurement error
    VectorXd y = z - z_pred;
    // check angle range -pi~pi
    if (y[1] < -M_PI) {
        y[1] += 2 * M_PI;
    }
    else if (y[1] > M_PI) {
        y[1] -= 2 * M_PI;
    }
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;
    
    // update state
    x_ = x_ + (K * y);
    int x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    // update process noise covariance
    P_ = (I - K * H_) * P_;
}
