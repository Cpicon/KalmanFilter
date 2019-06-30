#ifndef TRACKING_H_
#define TRACKING_H_

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include "Eigen/dense"

#include "KalmanFilter.h"
#include "sensorMeasurement.h"

class tracking {
//check if the variable was initialized
bool is_initialized;
//save the exactly time measurement
int64_t previous_Timestamp_;
//aceleration components
float noiseAx;
float noiseAy;

public:
//declare constructor
tracking();
//declare destructor
virtual ~tracking();
//declare function to get data from sensors
void processMeasurement(const sensorMeasurement &measurementData);
//declare Kalman Filter object
KalmanFilter Kf_;
};

#endif //TRACKING_H_
