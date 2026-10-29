#ifndef CSV_READER_HPP
#define CSV_READER_HPP
#include <string>
#include <vector>

struct Frame { double x, y, z, yaw; };
std::vector<Frame> load(const std::string& path);

#endif