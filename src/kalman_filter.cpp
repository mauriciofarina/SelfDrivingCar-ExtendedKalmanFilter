#include "kalman_filter.h"
#include <iostream>

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


//Default Kalman Filter Prediction
void KalmanFilter::Predict() {

    x_ = F_*x_;
    P_ = (F_ * P_ * F_.transpose()) + Q_;
    
}

//Default Kalman Filter Update
void KalmanFilter::Update(const VectorXd &z) {
    
    VectorXd y_ = z - (H_ * x_);
    
    CommonUpdate(y_);
    
}

//Extended Kalman Filter Update
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
    //Prevent Division by zero
    if(x_[0] == 0.0 && x_[1] == 0.0)
        return;
    
    //Calculate Rho
    float rho = sqrt(pow(x_[0], 2) + pow(x_[1], 2));
    
    //Calculate Theta
    float theta = atan2(x_[1], x_[0]);
    
    //Prevent Division by zero
    if (rho < 0.0001) {
        rho = 0.0001;
    }
    
    //Calculate Rho Dot
    float dRho = (x_[0] * x_[2] + x_[1] * x_[3]) / rho;
    
    
    VectorXd h_(3);
    h_ << rho, theta, dRho;
    
    //Calculate error
    VectorXd y_ = z - h_;
    
    //Normalize Theta (-pi < theta < pi)
    while (y_[1] < -M_PI){
        y_[1] += 2 * M_PI;
    }
    
    while (y_[1] > M_PI){
        y_[1] -= 2 * M_PI;
    }
    
    CommonUpdate(y_);
    
    
}

//Common Equations Implementation
void KalmanFilter::CommonUpdate(const VectorXd &y) {
    
    
    MatrixXd S_ = (H_ * P_ * H_.transpose()) + R_;
    MatrixXd K_ = P_ * H_.transpose() * S_.inverse();
    
    x_ = x_ + (K_ * y);
    MatrixXd I_ = MatrixXd::Identity(4,4);
    P_ = (I_ - (K_ * H_)) * P_;
    
}
