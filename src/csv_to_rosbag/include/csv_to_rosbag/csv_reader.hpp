#include <csv_to_rosbag/vector3.hpp>
#include <vector>
#include <string>

#ifndef _CSV_READER_H_
#define _CSV_READER_H_

std::pair<std::vector<double>, std::vector<Vector3>> readAccelerationFromScv(const std::string &file_name);

#endif