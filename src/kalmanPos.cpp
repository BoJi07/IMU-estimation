#include "kalmanPos.h"
#include <iostream>

KalmanFilterPos::KalmanFilterPos(){
    is_initialized = false;
    n_x = 6;
    n_z = 2;
    x = VectorXd::Zero(n_x);
    P = MatrixXd::Zero(n_x,n_x);
    F = MatrixXd::Zero(n_x, n_x);
    Q = MatrixXd::Zero(n_x,n_x);
    H = MatrixXd(n_z,n_x);
    H<<0.0,0.0,0.0,0.0,1.0,0.0,
       0.0,0.0,0.0,0.0,0.0,1.0;
    R = MatrixXd(n_z,n_z);
    R<<0.00004,0.0,
       0.0,0.00004;
}

VectorXd KalmanFilterPos::processMeasurement(vector<double> input){
    if(!is_initialized){
        previous_time = input[0];
        x<<0.0,0.0,0.0,0.0,input[1],input[2];
        is_initialized = true;
    }
    else{
        double dt = input[0]-previous_time;
        predict(dt);
        update(input);
        previous_time = input[0];
    }

    return x;
}

void KalmanFilterPos::predict(double dt){
    F<<1.0,0,dt,0,0.5*dt*dt,0,
    0,1.0,0,dt,0,0.5*dt*dt,
    0,0,1.0,0,dt,0,
    0,0,0,1.0,0,dt,
    0,0,0,0,1.0,0,
    0,0,0,0,0,1.0;
    x = F*x;
    P = F*P*F.transpose()+Q;
}

void KalmanFilterPos::update(const vector<double> &measurement){
    VectorXd z = VectorXd(n_z);
    z<<measurement[1],measurement[2];
    VectorXd z_pred = H*x;
    VectorXd y = z-z_pred;
    MatrixXd S = H*P*H.transpose()+R;
    MatrixXd K = P*H.transpose()*S.inverse();
    x = x + (K*y);
    P = P - (K*H*P);
}
