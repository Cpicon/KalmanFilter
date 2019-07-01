#include <sstream>
#include "sensorMeasurement.h"
#include "tracking.h"

int main() {
        /* code */
        // set measurement
        std::vector<sensorMeasurement> measurements;
        std::string name_file = "input_data.txt";
        // .c_str save the string as constan char
        std::ifstream myFile(name_file.c_str(), std::ifstream::in);
        if(!myFile.is_open()) {
                std::cout<<"The file "<<name_file<<" cannot be open"<<std::endl;
        }
        else{
                std::string line;
                //read the file line by line
                while(std::getline(myFile, line)) {
                        sensorMeasurement data;
                        // save the items separate by whitespace
                        std::istringstream iss(line);
                        // save the sensor type given by the input data (first element) current line.
                        // (R) radar and (L) lidar.
                        std::string sensorType;
                        iss>>sensorType;
                        // save the time whan the measurement was obtained
                        int64_t time_measurement;
                        // save the X and Y coordinates
                        float x;
                        float y;
                        // verify if the sensor correspond to (L) lidar
                        /* if compare retuns 0, means that the two string are equal.
                           A positive value means that the compared string is longer, or the first non-matching character is greater.
                           A negative value means that the compared string is shorter, or the first non-matching character is lower.*/
                        if(sensorType.compare("L")==0) {
                                // Asign to sensor type as LASER
                                data.sensor_type_ = sensorMeasurement::LIDAR;
                                //save the coordinates values
                                data.raw_measurements_ = Eigen::VectorXd(2);
                                iss>>x;
                                iss>>y;
                                data.raw_measurements_<<x,y;
                                // save the time whan the measurement was obtained
                                iss>>time_measurement;
                                data.Timestamp_ = time_measurement;
                                //save the data measurements to the current line
                                measurements.push_back(data);
                        }
                        // else verify if the sensor correspond to (R) Radar
                        else if(sensorType.compare("R")==0) {
                                continue;
                        }
                }
        }
        return 0;
}
