#include "csv_reader.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>



std::vector<Frame> load(const std::string& path) {
    std::vector<Frame> out;

    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("无法打开文件");
    }
    
    std::string line;
    std::getline(f, line);  

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;
        double v[4]; int i = 0;

        while (std::getline(ss, cell, ',') && i < 4) v[i++] = std::stod(cell);
        if (i == 4) out.push_back({v[0], v[1], v[2], v[3]});
    }

    return out;

}

