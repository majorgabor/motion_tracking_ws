#include <csv_to_rosbag/csv_reader.hpp>
#include <fstream>
#include <sstream>

std::pair<std::vector<double>, std::vector<Vector3>> readAccelerationFromScv(const std::string &file_name)
{
    std::vector<double> time_stamps;
    std::vector<Vector3> acc_values;

    std::ifstream file(file_name);
    if (!file.is_open())
    {
        printf("Unable to open file: %s", file_name.c_str());
    }
    else
    {
        std::string line, cell;
        double t, x, y, z;
        std::getline(file, line); // skip header
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::getline(ss, cell, ',');
            t = stod(cell);
            std::getline(ss, cell, ',');
            x = stod(cell);
            std::getline(ss, cell, ',');
            y = stod(cell);
            std::getline(ss, cell, ',');
            z = stod(cell);

            time_stamps.push_back(t);
            acc_values.push_back(Vector3(x, y, z));
        }
    }
    return {time_stamps, acc_values};
}