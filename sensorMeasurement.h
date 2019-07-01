#ifndef sensorMeasurement_H_
#define sensorMeasurement_H_

#include "Eigen/dense"

class sensorMeasurement {
public:
enum SensorType {
        LIDAR, RADAR
} sensor_type_;

Eigen::VectorXd raw_measurements_;

int64_t Timestamp_;
};

#endif //sensorMeasurement
