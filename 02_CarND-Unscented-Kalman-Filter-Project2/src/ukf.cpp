#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
    is_initialized_ = false;

    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);
    x_.fill(0.0);

    // initial covariance matrix
    P_ = MatrixXd::Identity(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 10;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;

    //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
    //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

    // state dimension
    n_x_ = 5;

    // augmented state dimension
    n_aug_ = 7;

    // sigma point spreading point
    lambda_ = 3 - n_x_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    // check sensor usage, ignore if specified not to use
    if ((!use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) ||
            (!use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)) {
        return;
    }

    /********* Initialization *********/
    if (!is_initialized_) {
        if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            float px = meas_package.raw_measurements_[0];
            float py = meas_package.raw_measurements_[1];
            x_[0] = px;
            x_[1] = py;
            cout << "Initialization finished with Laser data!" << endl;
        }
        else {
            float rho = meas_package.raw_measurements_[0];
            float phi = meas_package.raw_measurements_[1];
            float px = cos(phi) * rho;
            float py = sin(phi) * rho;
            x_[0] = px;
            x_[1] = py;
            cout << "Initialization finished with Radar data!" << endl;
        }

        // Finish initialization
        is_initialized_ = true;
        time_us_ = meas_package.timestamp_;

        return;
    }

    /********* Prediction ANd UPdate *********/
    float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    MatrixXd xSigAug = GenerateSigmaAug();
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}

/**
* Generate sigma points using current state vector and state covariance
* @return MatrixXd where each column represents a sigma point in the state space
*/
MatrixXd UKF::GenerateSigmaAug() {
    // Create augmented mean vector
    VectorXd xAug = VectorXd(n_aug_);
    xAug.head(n_x_) = x_;
    xAug[5] = 0;
    xAug[6] = 0;

    // Create angmented state covariance
    MatrixXd PAug = MatrixXd::Zero(n_aug_, n_aug_);
    PAug.topLeftCorner(n_x_, n_x_) = P_;
    PAug(5, 5) = std_a_ * std_a_;
    PAug(6, 6) = std_yawdd_ * std_yawdd_;

    // Create sigma points matrix
    MatrixXd xSigAUg = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    // Calculate the square root of PAug
    MatrixXd sqrtPAug = PAug.llt().matrixL();

    // Update sigma points matrix
    xSigAUg.col(0) = xAug;
    int ind = 1;
    for (int i = 0; i < n_aug_; ++i) {
        xSigAUg.col(ind) = xAug + sqrt(lambda_ + n_x_) * sqrtPAug.col(i);
        xSigAUg.col(ind + n_aug_) = xAug + sqrt(lambda_ + n_aug_) * sqrtPAug.col(i);
    }

    return xSigAUg;
}
