#ifndef KALMANPOS_H
#define KALMANPOS_H

#include <eigen3/Eigen/Dense>
#include <vector>

using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class KalmanFilterPos{
public:
    KalmanFilterPos();
    VectorXd processMeasurement(const vector<double> input);
private:
    bool is_initialized;
    int n_x;
    int n_z;
    double previous_time;
    VectorXd x;
    MatrixXd P;
    MatrixXd F;
    MatrixXd Q;
    MatrixXd H;
    MatrixXd R;
    void predict(double dt);
    void update(const vector<double> &measurement); 
};

#endif
