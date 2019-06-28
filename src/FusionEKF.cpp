#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    
    previous_timestamp_ = 0;
    
    // initializing matrices
    
    
    // state vector
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    // state covariance matrix
    ekf_.P_ = MatrixXd(4,4);
    ekf_.P_ <<
    1 , 0 ,  0   ,  0   ,
    0 , 1 ,  0   ,  0   ,
    0 , 0 , 1000 ,  0   ,
    0 , 0 ,  0   , 1000 ;
    
    // state transition matrix F
    ekf_.F_ = MatrixXd(4,4);
    ekf_.F_ <<
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;
    
    // process covariance matrix Q
    float noise_ax = 9;
    float noise_ay = 9;
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<
    (noise_ax/4) ,      0       , (noise_ax/2) ,      0       ,
          0      , (noise_ay/4) ,      0       , (noise_ay/2) ,
    (noise_ax/2) ,      0       ,  (noise_ax)  ,      0       ,
          0      , (noise_ay/2) ,      0       ,  (noise_ay)  ;
    
    
    
    // measurement matrix H
    
    // Laser
    H_laser_ = MatrixXd(2, 4);
    H_laser_ <<
    1, 0, 0, 0,
    0, 1, 0, 0;
    
    // Radar
    H_radar_ = MatrixXd(3, 4);
    
    
    // measurement covariance matrix R
    
    // Laser
    R_laser_ = MatrixXd(2, 2);
    R_laser_ <<
    0.0225 ,   0    ,
       0   , 0.0225 ;
    
    // Radar
    R_radar_ = MatrixXd(3, 3);
    R_radar_ <<
    0.09 ,   0    ,  0   ,
     0   , 0.0009 ,  0   ,
     0   ,   0    , 0.09 ;
    
    
    
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    
    static long long lastTimestamp = 0;
    
    long long deltaTime = (measurement_pack.timestamp_ - lastTimestamp)/pow(10,6);
    
    lastTimestamp = measurement_pack.timestamp_;
    
    
    /**
     * Initialization
     */
    if (!is_initialized_) {
        
        
        // first measurement
        
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            
            float rho = measurement_pack.raw_measurements_[0];
            float theta = measurement_pack.raw_measurements_[1];
            float dRho = measurement_pack.raw_measurements_[2];
            
            ekf_.x_ <<
            (cos(theta) * rho)  ,
            (sin(theta) * rho)  ,
            (cos(theta) * dRho) ,
            (sin(theta) * dRho) ;
            
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            
            float x = measurement_pack.raw_measurements_[0];
            float y = measurement_pack.raw_measurements_[1];
            
            ekf_.x_ << x , y , 0 , 0 ;
            
        }
        
        
        ekf_.P_ =  ekf_.F_ * ekf_.P_ * ekf_.F_.transpose();
        
        
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }
    
    /**
     * Prediction
     */
    
    /**
     * TODO: Update the state transition matrix F according to the new elapsed time.
     * Time is measured in seconds.
     * TODO: Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    
    ekf_.Predict();
    
    /**
     * Update
     */
    
    /**
     * TODO:
     * - Use the sensor type to perform the update step.
     * - Update the state and covariance matrices.
     */
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // TODO: Radar updates
        
    } else {
        // TODO: Laser updates
        
    }
    
    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
