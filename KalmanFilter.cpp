#include "KalmanFilter_H_"
#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(){

};

KalmanFilter::~KalmanFilter(){

};

void KalmanFilter::Predict(){
    x_ = F_*x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z){
    VectorXd z_predict = H_ * x_;
    VectorXd Error = z - z_predict;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Sinverse = S.inverse();
    MatrixXd K = P_* Ht * Sinverse;
    // new estimate
    x_ = x_ + K * Error;
    MatrixXd I = MatrixXd::Identity(x_size(),x_size());
    p_ = (I - K * H_) * P_
}
