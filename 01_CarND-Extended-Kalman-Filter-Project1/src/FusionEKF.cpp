#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    
    previous_timestamp_ = 0;
    
    // initializing matrices
    //measurement covariance matrix - laser
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    // H_laser is constant
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    // Hj will be updated each time in update()
    Hj_ = MatrixXd(3, 4);       
    
    
    // x will be initialized with the first measurement
    VectorXd xInit = VectorXd(4);
    MatrixXd PInit = MatrixXd(4, 4);
    PInit << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

    // F and Q will be initialized and updated with dt(after we get the first measurement data)
    MatrixXd FInit = MatrixXd(4, 4);
    MatrixXd QInit = MatrixXd(4, 4);

    // H and R will be initialized when we know the sensor type in update()
    MatrixXd HInit = MatrixXd(2, 4);
    MatrixXd RInit = MatrixXd(3, 3);

    ekf_.Init(xInit, PInit, FInit, HInit, RInit, QInit);    
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    
    
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        // first measurement        
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            float rho = measurement_pack.raw_measurements_[0];
            float phi = measurement_pack.raw_measurements_[1];
            float x_ = cos(phi) * rho;
            float y_ = sin(rho) * rho;
            ekf_.x_ << x_, y_, 0, 0;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
        }
        
        // done initializing, no need to predict or update
        is_initialized_ = true;
        previous_timestamp_ = measurement_pack.timestamp_;
        cout << "EKF initialization finished!" << endl;

        return;
    }
    
    /*****************************************************************************
     *  Prediction
     ****************************************************************************/       
//    update dt and precious_timestamp
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    // update F
    ekf_.F_ << 1, 0, dt, 0,
               0, 1, 0, dt,
               0, 0, 1, 0,
               0, 0, 0, 1;
    // update Q
    float dt_4 = pow(dt, 4);
    float dt_3 = pow(dt, 3);
    float dt_2 = pow(dt, 2);
    float ax = 9.0f;
    float ay = 9.0f;
    ekf_.Q_ << dt_4 / 4 * ax, 0, dt_3 / 2 * ax, 0,
               0, dt_4 / 4 * ay, 0, dt_3 / 2 * ay,
               dt_3 / 2 * ax, 0, dt_2 * ax, 0,
               0, dt_3 / 2 * ay, 0, dt_2 * ay;
    
    ekf_.Predict();
    
    /*****************************************************************************
     *  Update
     ****************************************************************************/
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // update radar R
        ekf_.R_ = R_radar_;
        // update radar H(Jocobian)
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        // radar measurement update
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        // update lidar R
        ekf_.R_ = R_laser_;
        // update lidar H(constant)
        ekf_.H_ = H_laser_;
        // Laser measurement updates
        ekf_.Update(measurement_pack.raw_measurements_);
    }
    
    // print the output
//    cout << "x_ = " << ekf_.x_ << endl;
//    cout << "P_ = " << ekf_.P_ << endl;
}
