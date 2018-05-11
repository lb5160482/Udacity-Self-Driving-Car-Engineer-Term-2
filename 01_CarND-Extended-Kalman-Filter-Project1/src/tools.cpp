#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    for(int i=0; i < estimations.size(); ++i){
        VectorXd diff = estimations[i] - ground_truth[i];
        // element wise product
        VectorXd diffElementProd = diff.array() * diff.array();
        rmse += diffElementProd;
    }
    rmse = rmse/estimations.size();
    // element wise sqrt
    rmse = rmse.array().sqrt();
    
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    float px_2 = px * px;
    float py_2 = py * py;
    
    //check division by zero
    if (px == 0 && py == 0) {
        cout << "Jacobian sigularuty!" <<endl;
        px = 0.0001;
    }
    //compute the Jacobian matrix
    Hj(0, 0) = px / sqrt(px_2 + py_2);
    Hj(0, 1) = py / sqrt(px_2 + py_2);
    Hj(0, 2) = 0;
    Hj(0, 3) = 0;
    Hj(1, 0) = - py / (px_2 + py_2);
    Hj(1, 1) = px / (px_2 + py_2);
    Hj(1, 2) = 0;
    Hj(1, 3) = 0;
    Hj(2, 0) = py * (vx * py - vy * px) / pow(px_2 + py_2, 1.5);
    Hj(2, 1) = px * (vy * px - vx * py) / pow(px_2 + py_2, 1.5);
    Hj(2, 2) = px / sqrt(px_2 + py_2);
    Hj(2, 3) = py / sqrt(px_2 + py_2);
    
    return Hj;
}
