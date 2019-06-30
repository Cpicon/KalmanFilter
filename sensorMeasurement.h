#ifndef sensorMeasurement_H_
#define sensorMeasurement_H_

#include "Eigen/dense"

class sensorMeasurement {
public:
enum SensorType {
        LASER, RADAR
} sensor_type_;

Eigen::VectorXd raw_measurements_;

int64 Timestamp_;
};

#endif //sensorMeasurement
