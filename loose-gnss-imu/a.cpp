#include "pch.h"


const double PI = 3.1415926535898;
//Variance
Eigen::VectorXd x;
Eigen::Matrix<double, 11, 11> P;
Eigen::MatrixXd v;
Eigen::MatrixXd R;
Eigen::MatrixXd H;
Eigen::MatrixXd T;
Eigen::Matrix<double, 1, 15> xa;
Eigen::Matrix<double, 15, 15> Pa;
const std::vector<double> init_att_unc = {0.3, 0.3, 0.5};
const std::vector<int> init_vel_unc = {10, 10, 10};
const std::vector<int> init_pos_unc = {30, 30, 30};
const double init_bg_unc = 4.8481367284e-05;
const double init_ba_unc = 0.048901633857000000;
const double psd_gyro  = 3.38802348178723e-09;
const double psd_acce      =     2.60420170553977e-06;     // acce noise PSD (m^2/s^3)  
const double psd_bg        =     2.61160339323310e-14;     // gyro bias random walk PSD (rad^2/s^3)
const double psd_ba        =     1.66067346797506e-09;
const int sample_rate = 104;
int nt=1/sample_rate;
double D2R = PI/180.0;
double RE_WGS84 = 6378137.0;

//new GNSS + imu solution calculated as pos
void init(){
pos = IMUsol.posXYZ;
vel = IMUsol.velXYZ;
pos = xyz2lbh(pos);

init_att_unc.at(0) = init_att_unc.at(0)* D2R;
init_pos_unc * RE_WGS84;

if (vel.at(0) < 1e-4){
	vel.at(0) = 1e-4;
}
double yaw = -atan2(vel(0), vel(1));
std::vector att = eigVector2std(double2eigVector(0, 0, yaw));

eigen::Matrix<9, 1> avp0;
avp0 << att.at(0), att.at(1), att.at(2), vel.at(0), vel.at(1), vel.at(2), pos.at(0), pos.at(1), pos.at(2);
eigen::Matrix<9, 1> avp0;
}
