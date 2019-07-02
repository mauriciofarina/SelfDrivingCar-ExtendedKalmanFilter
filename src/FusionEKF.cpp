#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

FusionEKF::FusionEKF()
{

    //Set as Not Initialized
    is_initialized_ = false;

    //Clear Previous Timestamp
    previous_timestamp_ = 0;

    // initializing matrices

    // state vector x
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 0, 0;

    // state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

    // state transition matrix F
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // Process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);

    // measurement matrix H

    // Laser
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
        0, 1, 0, 0;

    // Radar
    H_radar_ = MatrixXd(3, 4);
    H_radar_ << 1, 1, 0, 0,
        1, 1, 0, 0,
        1, 1, 1, 1;

    // measurement covariance matrix R

    // Laser
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << 0.0225, 0,
        0, 0.0225;

    // Radar
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

    noise_ax = 9;
    noise_ay = 9;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{

    float deltaTime = (measurement_pack.timestamp_ - previous_timestamp_) / pow(10, 6);
    previous_timestamp_ = measurement_pack.timestamp_;

    //Check if filter is initialized
    if (!is_initialized_)
    {

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {

            //Convert Polar to Cartesian Coordinates
            float ro = measurement_pack.raw_measurements_(0);
            float phi = measurement_pack.raw_measurements_(1);
            float ro_dot = measurement_pack.raw_measurements_(2);
            ekf_.x_(0) = ro * cos(phi);
            ekf_.x_(1) = ro * sin(phi);
            ekf_.x_(2) = ro_dot * cos(phi);
            ekf_.x_(3) = ro_dot * sin(phi);
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {

            //Get Laser Cartesian Coordinates
            ekf_.x_(0) = measurement_pack.raw_measurements_(0);
            ekf_.x_(1) = measurement_pack.raw_measurements_(1);
            ekf_.x_(2) = 0.0;
            ekf_.x_(3) = 0.0;
        }

        //Set filter as initialized
        is_initialized_ = true;
        return;
    }

    //Predict Step
    if (deltaTime != 0.0)
    {

        //Update Transition Matrix F with new Delta Time
        ekf_.F_(0, 2) = deltaTime;
        ekf_.F_(1, 3) = deltaTime;

        //Update Process Matrix Q with new Delta Time
        ekf_.Q_ << pow(deltaTime, 4) * (noise_ax / 4), 0, pow(deltaTime, 3) * (noise_ax / 2), 0,
            0, pow(deltaTime, 4) * (noise_ay / 4), 0, pow(deltaTime, 3) * (noise_ay / 2),
            pow(deltaTime, 3) * (noise_ax / 2), 0, pow(deltaTime, 2) * (noise_ax), 0,
            0, pow(deltaTime, 3) * (noise_ay / 2), 0, pow(deltaTime, 2) * (noise_ay);

        ekf_.Predict();
    }

    //Update Step
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {

        Tools tools;

        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;

        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    else
    {

        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;

        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
