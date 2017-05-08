#include "kalman_filter.h"
#include <iostream>
using namespace std;
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
    UpdateHelperFunction(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    // Convert the current state vector into radial form
    double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));


    double theta = 0.0;
    if (fabs(x_(0)) > 0.0001) {
        theta = atan2(x_(1), x_(0));
    }
    //cout << x_(0) << "\t" << x_(1) << "\t" << theta << endl;
    double rho_dot;
    if (fabs(rho) < 0.0001) {
        rho_dot = 0;
    } else {
        rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
    }
 
    // Vector h'
    VectorXd z_pred = VectorXd(3);
    z_pred << rho, theta, rho_dot;

    
    //VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    while (y(1) > M_PI) {
        y(1) -= M_PI;
    }

    while (y(1) < -M_PI) {
        y(1) += M_PI;
    }
    cout << y(1) << M_PI<< endl;
    UpdateHelperFunction(y);
}

void KalmanFilter::UpdateHelperFunction(const VectorXd &y) {
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;   
}
