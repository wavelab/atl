#ifndef ATL_UTILS_DATA_HPP
#define ATL_UTILS_DATA_HPP

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "atl/utils/math.hpp"

namespace atl {

// CSV ERROR MESSAGES
#define E_CSV_DATA_LOAD "Error! failed to load test data [%s]!!\n"
#define E_CSV_DATA_OPEN "Error! failed to open file for output [%s]!!\n"

int csvrows(const std::string &file_path);
int csvcols(const std::string &file_path);
int csv2mat(const std::string &file_path, const bool header, MatX &data);
int mat2csv(const std::string &file_path, MatX data);

} // namespace atl
#endif
