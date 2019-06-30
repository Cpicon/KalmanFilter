#ifndef KalmanFilter_H_
#define KalmanFilter_H_

#include "Eigen/dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;


class KalmanFilter {
public:
//contructor
KalmanFilter();
// destructor
virtual ~KalmanFilter();
// predict step: predict the state and covariance using the
// the process model
void Predict();

// update step: combined the value of predict step with the measurement
void Update(const Eigen::VectorXd &z);

//state vector (position x, postition y, velocity x, velocity y)
Eigen::VectorXd x_;
// state covariance Matrix
Eigen::MatrixXd P_;
// state transistion matrix
Eigen::MatrixXd F_;
// process covariance MatrixXd
Eigen::MatrixXd Q_;
// measurement MatrixXd
Eigen::MatrixXd H_;
// measurement covariance MatrixXd
Eigen::MatrixXd R_;

};
#endif // Kalman filter h
