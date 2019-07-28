#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_<< 1,0,0,0,
  			 0,1,0,0;
  
  //initialize noise components
  noiseAx = 9;
  noiseAy = 9;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    VectorXd x_ = VectorXd(4);
    x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      //assing the first measurements from radar data
      x_<<measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]),//px = rho (tangente) * conseno(anglulo)
      		measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]),//py = rho (tangente) * seno(anglulo)
      		0,
      		0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
		//assign the first measurements from laser data
      x_<< measurement_pack.raw_measurements_[0],
      		measurement_pack.raw_measurements_[1],
      		0,
      		0;  
    }

    // done initializing, no need to predict or update
    // states transition matrix
    MatrixXd F_ = MatrixXd(4,4);
    F_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;
    // states covariance matrix
    MatrixXd P_ = MatrixXd(4, 4);
  	P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,// high uncertainty in velocity because we don't know starting velocity value
          0, 0, 0, 1000;
    // states procces covariance matrix
  	MatrixXd Q_ = MatrixXd(4, 4);
    // initialize kalman filters variable
    ekf_.Init(x_, P_, F_,H_laser_, Hj_, R_laser_,R_radar_, Q_);
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */
  
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
	//integrate dt in F transition Matrix
  ekf_.F_ << 1, 0, dt, 0,
        	0, 1, 0, dt,
        	0, 0, 1, 0,
        	0, 0, 0, 1;
	// set the process covariance matrix Q
	float dt_2 = std::pow(dt, 2);
	float dt_3 = std::pow(dt, 3);
	float dt_4 = std::pow(dt, 4);
  	
	ekf_.Q_<< dt_4/4*noiseAx, 0, dt_3/2*noiseAx, 0,
        	0, dt_4/4*noiseAy, 0, dt_3/2*noiseAy,
        	dt_3/2*noiseAx, 0, dt_2*noiseAx, 0,
        	0, dt_3/2*noiseAy, 0, dt_2*noiseAy;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout<< "------"<<endl;
  cout << "P_ = " << ekf_.P_ << endl;
  cout<< "==========="<<endl;
}
