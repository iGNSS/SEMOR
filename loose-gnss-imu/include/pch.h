#ifndef PCH_H
#define PCH_H

// TODO: add headers that you want to pre-compile here
#include <iostream>
#include <iomanip>
#include <string>
#include <ctime>
#include <fstream>
#include <sstream> 
#include <iterator>
#include <vector>
#include <map>
#include <algorithm>
#include <Eigen/Dense>

const std::vector<double> init_att_uno = {0.3, 0.3, 0.5};
const std::vector<int> init_vel_uno = {10, 10, 10};
const std::vector<int> init_pos_uno = {30, 30, 30};
const double init_bg_uno = 4.8481367284e-05;
const double init_ba_uno = 9.80665E-3;
#endif //PCH_H
