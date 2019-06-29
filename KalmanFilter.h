#ifndef KalmanFilter_H_
#define KalmanFilter_H_

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;


class KalmanFilter{
    //contructor
    KalmanFilter();
    // destructor
    virtual ~KalmanFilter();
    // predict step: predict the state and covariance using the
    // the process model
    void Predict();

    // update step: combined the value of predict step with the measurement
    void Update(const VectorXd &z);

    //state vector (position x, postition y, velocity x, velocity y)
    VectorXd x_;
    // state covariance Matrix
    MatrixXd P_;
    // state transistion matrix
    MatrixXd F_;
    // process covariance MatrixXd
    MatrixXd Q_,
    // measurement MatrixXd
    MatrixXd H_;
    // measurement covariance MatrixXd
    MatrixXd R_;

};
#endif // Kalman filter h
