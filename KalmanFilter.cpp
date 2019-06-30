#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(){

};

KalmanFilter::~KalmanFilter(){

};

void KalmanFilter::Predict(){
        x_ = F_*x_;
        Eigen::MatrixXd Ft = F_.transpose();
        P_ = F_ * P_ * Ft + Q_;
};

void KalmanFilter::Update(const VectorXd &z){
        Eigen::VectorXd z_predict = H_ * x_;
        Eigen::VectorXd Error = z - z_predict;
        Eigen::MatrixXd Ht = H_.transpose();
        Eigen::MatrixXd S = H_ * P_ * Ht + R_;
        Eigen::MatrixXd Sinverse = S.inverse();
        Eigen::MatrixXd K = P_* Ht * Sinverse;
        // new estimate
        x_ = x_ + K * Error;
        Eigen::MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
        P_ = (I - K * H_) * P_;
};
