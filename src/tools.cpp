#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &groundTruth) {
    
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != groundTruth.size() || estimations.size() == 0) {
        std::cout << "Invalid estimation or ground truth data" << std::endl;
        return rmse;
    }
    
    // accumulate squared residuals
    for (unsigned int i=0; i < estimations.size(); ++i) {
        
        VectorXd residual = estimations[i] - groundTruth[i];
        
        // coefficient-wise multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }
    
    // calculate the mean
    rmse = rmse/estimations.size();
    
    // calculate the squared root
    rmse = rmse.array().sqrt();
    
    // return the result
    return rmse;
    
    
}

MatrixXd Tools::CalculateJacobian(const VectorXd& xState) {
  
    MatrixXd Hj(3,4);
    
    std::cout << Hj << std::endl;
    // recover state parameters
    float px = xState(0);
    float py = xState(1);
    float vx = xState(2);
    float vy = xState(3);
    
    // pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);
    
    // check division by zero
    if (fabs(c1) < 0.0001) {
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj;
    }
    
    // compute the Jacobian matrix
    Hj << (px/c2), (py/c2), 0, 0,
    -(py/c1), (px/c1), 0, 0,
    py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
    
    return Hj;
}
