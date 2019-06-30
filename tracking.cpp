#include "tracking.h"


//initialize constructor
tracking::tracking(){
        is_initialized = false;
        //initialize the time in zero (It's the beginning)
        previous_Timestamp_  = 0;
        //initialize empty state vector (position X, position Y, velocity X, velocity Y).
        //At the beginning, we don't have measurement to predict the next values.
        Kf_.x_ = Eigen::VectorXd(4);
        // initialize covariance matrix
        Kf_.P_ = Eigen::MatrixXd(4,4);
        Kf_.P_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
        //initialize R covariance (measurement uncertainy)
        Kf_.R_ = Eigen::MatrixXd(2, 2);
        Kf_.R_ << 0.0225, 0,
                0, 0.0225;

        // initialize measurement matrix. Keep in mind that we just get the position
        // of the tracking object. The velocity we can't measurement.
        Kf_.H_ = Eigen::MatrixXd(2, 4);
        Kf_.H_ << 1, 0, 0, 0,
                0, 1, 0, 0;

        // initialize the initial transition matrix. We assume delta_t is 1 second.
        Kf_.F_ = Eigen::MatrixXd(4, 4);
        Kf_.F_ << 1, 0, 1, 0,
                0, 1, 0, 1;
        0, 0, 1, 0,
        0, 0, 0, 1;

        //initialize noise components
        noiseAx = 5;
        noiseAy = 5;
};

tracking::~tracking(){
};

void tracking::processMeasurement(const sensorMeasurement &measurementData){
        if(!is_initialized) {
                std::cout<<"Kalman Filters initialization..."<<std::endl;
                //initialize postion at the beginning. velocity is zero (The car was stopped)
                Kf_.x_= measurementData.raw_measurements_[0],
                measurementData.raw_measurements_[1],
                0,
                0;
                is_initialized = true;
                //save the specific time whem the measurement was obtained
                previous_Timestamp_= measurementData.Timestamp_;
        };
};

//compute the time elapsed between the current and previous measurement
// dt in seconds
float dt = (measurementData.Timestamp_ - previous_Timestamp_) / 1000000.0;
//update the previous Timestamp_
previous_Timestamp_ = measurementData.Timestamp_;
//integrate dt in F transition Matrix
Kf_.F_ << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
// set the process covariance matrix Q
float dt_2 = std::pow(dt, 2);
float dt_3 = std::pow(dt, 3);
float dt_4 = std::pow(dt, 4);

Kf_.Q_<< dt_4/4*noiseAx, 0, df_3/2*noiseAx,
        0, dt_4/4*noiseAy, 0, dt_3/2*noiseAy,
        dt_3/2*noiseAx, 0, dt_2*noiseAx, 0,
        0, dt_3/2*noiseAy, 0, dt_2*noiseAy;

//predict step
Kf_.Predict();

//update step
Kf_.Update(sensorMeasurement.raw_measurements_);

std::cout<<"X (state vector)"<<Kf_.x_<<std::endl;
std::cout<<"P (uncertainy matrix)"<<Kf_.P_<<std::endl;
