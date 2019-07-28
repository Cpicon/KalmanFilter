#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &Hj_in, MatrixXd &R_laser_in, MatrixXd &R_radar_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Hj_ = Hj_in;
  R_laser_= R_laser_in;
  R_radar_= R_radar_in;
  Q_ = Q_in;
  I_ = MatrixXd::Identity(4, 4);
  x_polar = VectorXd(3); 
}

void KalmanFilter::Predict() {
  //predict the state
	x_ = F_*x_;
  	MatrixXd Ft_ = F_.transpose();
  	P_ = F_ * P_ * Ft_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  //update the state by using Kalman Filter equations
	VectorXd Y_;
  	Y_ = z - H_*x_;
  	MatrixXd Ht = H_.transpose();
  	MatrixXd S = H_ * P_ * Ht + R_laser_;
  	MatrixXd Si = S.inverse();
  	MatrixXd K =  P_ * Ht * Si;

  // new state
  	x_ = x_ + (K * Y_);
  	P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //  update the state by using Extended Kalman Filter equations
  // calculate Jacobian matrix and convert to polar form cartesians coordinates 
  	JcobianMatrix(x_);
	VectorXd Y_;
  	Y_ = z - x_polar;
  	while(Y_(1) > M_PI){
      Y_(1) -= 2 * M_PI;
    }

    while(Y_(1) < -M_PI){
      Y_(1) += 2 * M_PI;
    }
  	MatrixXd Ht = Hj_.transpose();
  	MatrixXd S = Hj_ * P_ * Ht + R_radar_;
  	MatrixXd Si = S.inverse();
  	MatrixXd K =  P_ * Ht * Si;

  // new state
  	x_ = x_ + (K * Y_);
  	P_ = (I_ - K * Hj_) * P_;
  
 
}

void KalmanFilter::JcobianMatrix(const VectorXd & x_in){
	//recover state parameter
  float  px = x_in(0);
  float  py = x_in(1);
  const float vx = x_in(2);
  const float vy = x_in(3);
  // check division by zero
  if (px==0 && py == 0){
    std::cout<<"Err0r: division by zero is not permitted"<<std::endl;
    std::cout<<"Adjusting to values px 0.01 and py 0.01"<<std::endl;
  	px = 0.01;
    py = 0.01;
  }
  float pxpluspy = std::pow(px,2) + std::pow(py,2);
  // derivates to range
  float dp_Px = px / std::sqrt(pxpluspy);
  float dp_Py = py / std::sqrt(pxpluspy);// dp_Vx and dp_Vy are equals to zero
  //derivates to bearing 
  float dphi_Px = -1*py/pxpluspy;
  float dphi_Py = px/pxpluspy; // dphi_Vx and dphi_Vy are equals to zero
  //derivates to radial velocity
  float d2p_Px = py * (vx*py-vy*px) / std::pow(pxpluspy,1.5);
  float d2p_Py = px * (vx*py-vy*px) / std::pow(pxpluspy,1.5);
  //put in together
  Hj_<< dp_Px, dp_Py, 0, 0,
  		dphi_Px, dphi_Py, 0, 0,
  		d2p_Px, d2p_Py, dp_Px, dp_Py;
  //convert to polar coordinates h(x_)
  double rho, phi, rho_dot;
  rho = sqrt(px*px + py*py);
  phi = atan2(py, px);

  // protection from division by zero
  if (rho < 0.000001) {
    rho = 0.000001;
  }
  	
  rho_dot = (px * vx + py * vy) / rho;
  x_polar << rho, phi, rho_dot;
}