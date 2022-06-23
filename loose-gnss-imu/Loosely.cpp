/*
* Loosely.cpp
* Loosely Coupled Integration of GPS & IMU
*  Created on: Sept 10, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "Loosely.h"
#include "semor.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/time.h>

using namespace std;
using namespace Eigen;

ofstream out;

int first = 1;

// PI
const double PI = 3.1415926535898;

//Variance
VectorXd x(15);
MatrixXd P(15, 15);
MatrixXd Q(15, 15);
MatrixXd v(15,15);
MatrixXd R;
MatrixXd H(3, 15);
Matrix3d Cne;
//Dblh2Dxyz
Matrix3d T;
MatrixXd Dxyz;
//blh2xyz
Matrix3d Cen;
//Vector3d xyz;

Matrix3d Cnb;

Matrix<double, 15, 1> xa;
Matrix<double, 15, 15> Pa;
Vector3d init_att_unc;
Vector3d init_vel_unc;
Vector3d init_pos_unc;
const double nt=1.0/sample_rate;
const double D2R = PI/180.0;
const double RE_WGS84 = 6378137.0;
const double FE_WGS84    =(1.0/298.257223563); //earth flattening (WGS84)
const double ECC_WGS84   =sqrt(2*FE_WGS84-pow(FE_WGS84, 2)); 
int fst_pos = 1;

class Earth{
	public:
		double Re;
		double f;
		double Rp;
		double e1;
		double e2;
		double wie;
		double g0;
		double RN;
		double RM;
		double Mpv2;
		double Mpv4;
		double RNh;
		double RMh;
		double tanL;
		double secL;
		double sinL;
		double cosL;
		double sin2L;
		double cos2L;
		Vector3d wnie;
		Vector3d wnen;
		Vector3d wnin;
		double g;
		Vector3d gn;
		Vector3d wnien;
		Vector3d gcc;

};
Vector3d lever;
Vector3d pos;
Vector3d vel;
double yaw;
Vector3d att;
MatrixXd avp0(9, 1);
Vector3d acc;
Vector3d bg;
Vector3d ba;
Matrix3d Kg;
Matrix3d Ka;
Vector3d tauG;
Vector3d tauA;
Earth eth;
Matrix3d Mpv;
Vector3d wib;
Vector3d fb;
MatrixXd fn; 					
Vector3d web;
Vector3d mx_init_bg_unc;
Vector3d mx_init_ba_unc;
Vector3d mx_pad_gyro;
Vector3d mx_pad_acce;
Vector3d mx_pad_bg;
Vector3d mx_pad_ba;
VectorXd initP(15);
VectorXd initQ(15);
Matrix3d posvar;
Matrix3d posvar1;
Vector3d initPosvar;
VectorXd va(6);
Vector3d dv0;
Vector3d dw0;
Vector3d dv;
Vector3d dw;
Vector3d pos_mid;
Vector3d vel_mid;
MatrixXd Q0(15, 15);
MatrixXd P0(15, 15);
Vector3d pos_new;
Vector3d vel_new;
Vector3d old_dv;
Vector3d old_dw;
Vector3d dv_rot;
Vector3d dv_scul;
Vector3d dv_sf;
Vector3d dv_cor;
Vector3d dw_cone;
Vector3d phi_b_ib;
Vector3d phi_n_in;
Matrix3d Cbb;
Matrix3d Cnn;
Matrix3d Cnb_new;
Vector3d att_new;

Vector3d VAR1;
Vector3d VAR2;

Vector3d ve;

Matrix3d Pp;



IMUmechECEF MechECEF; 

ifstream fimu;

InitializeIMU iniIMU;
double IMU_INI_TIME_END;

ReaderGNSS OBSgnss;
ReaderIMU OBSimu;

/*double avg_acc_x;
double avg_acc_y;
double avg_acc_z;

double tot_acc_x;
double tot_acc_y;
double tot_acc_z;

int nsamples;*/

Matrix<double, 1, 3> GNSS_pos; 
Matrix<double, 1, 3> GNSS_vel; 
Matrix<double, 1, 6> gnss_posP;
Matrix<double, 1, 6> gnss_velP;

const int VAR_POS=10^2; /*const double VAR_VEL=0.0225*/; const int MAX_DPOS=10; /*const int MAX_DVEL=5;*/

Vector3d rr; 
Vector3d pos_GNSS; 

int last_imu_epoch = 0;

Loosely::Loosely(){


	init_att_unc << 0.3, 0.3, 0.5;
	init_vel_unc << 10, 10, 10;
	init_pos_unc << 30, 30, 30;
	lever << 0, 0, 0;
}

void Loosely::close_out_file(){
	out.close();
}

void print_time(){
	struct timeval tv;
	gettimeofday(&tv, NULL);
	int week = ((tv.tv_sec+LEAP_SECONDS-GPS_EPOCH))/(7*24*3600);
	double sec = (double)(((tv.tv_sec+LEAP_SECONDS-GPS_EPOCH))%(7*24*3600))+(tv.tv_usec / 1000000.0);
	printf("%lf\n", sec);
}

void read_imu(){
	string line;
	if(debug){
		getline(fimu, line);
		if (line.find("GPS") != std::string::npos) {
			getline(fimu, line);
		}
		OBSimu.clearObs();
		OBSimu.obsEpoch(line);
	}
	else{
		char buf[IMU_LENGTH];
		do{
		strcpy(buf, imu[imu_count++]);
		imu_count = imu_count % IMUBUF_CAPACITY;

		line = string(buf);
		OBSimu.clearObs();
		OBSimu.obsEpoch(line);
		}while(OBSimu._IMUdata.imuTime)
	}

	if(logs){
		out << line << endl;
		out.flush();
	}

	//cout << line << endl;
	//cout.flush();
	
}

void closeF(){
	out.close();
}

int continuePrint = 0;

void print(MatrixXd m){
	if(!continuePrint)
		return;
	stringstream ss;
	for(int i=0; i<m.rows(); i++){
		for(int j=0; j<m.cols(); j++){
			ss << m(i, j) << ",";
		}
		cout << ss.str() << endl;
		ss.str("");
	}
	cout.flush();
}

void print(Matrix3d m){
	if(!continuePrint)
		return;
	stringstream ss;
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			ss << m(i, j) << ",";
		}
		cout << ss.str() << endl;
		ss.str("");
	}
	cout.flush();
}

void print(Vector3d v){
	if(!continuePrint)
		return;
	stringstream ss;
	for(int i=0; i<3; i++){
		for(int j=0; j<1; j++){
			ss << v(i, j) << ",";
		}
		cout << ss.str() << endl;
		ss.str("");
	}
	cout.flush();
}

void print(VectorXd v){
	if(!continuePrint)
		return;
	stringstream ss;
	for(int i=0; i<v.rows(); i++){
		for(int j=0; j<1; j++){
			ss << v(i, j) << ",";
		}
		cout << ss.str() << endl;
		ss.str("");
	}
	cout.flush();
}

void print(double value){
	if(!continuePrint)
		return;
	cout << value << endl;
	cout.flush();
}

// A routine to facilitate IMU mechanization in ECEF
void Loosely::SolutionIMU(ReaderIMU IMU, IMUmechECEF& MechECEF) {
	// Compute time interval
	_dTimu = IMU._IMUdata.imuTime - _epochIMU; //Difference between current epoch and previous
	// IMU Mechanization
	MechECEF.MechanizerECEF(_dTimu, IMU._IMUdata.Acc, IMU._IMUdata.Gyr, _LLH_o); //Update ECEF position adding to it accelerometer and gyroscope data (previous data is accumulated)
	// Update solution
	_epochIMU = IMU._IMUdata.imuTime; //Update epocIMU with current epoch
	IMUsol.posXYZ = MechECEF._pos;		//Update IMU solution
	IMUsol.velXYZ = MechECEF._vel;		//Update IMU solution
	IMUsol.attXYZ = MechECEF._att;		//Update IMU solution
	_Heading_imu = normalise(IMUsol.attXYZ.at(2), 0, 2 * PI);
}

VectorXd double2eigVector(double a, double b, double c){
	VectorXd v = VectorXd::Zero(3);
	v(0) = a;
	v(1) = b;
	v(2) = c;
	return v;
}

void att2Cnb(Vector3d att){
	Vector3d sina; Vector3d cosa;
	sina << sin(att(0)), sin(att(1)), sin(att(2));
	cosa << cos(att(0)), cos(att(1)), cos(att(2));
	double sinp=sina(0);  double sinr=sina(1); double siny=sina(2);
	double cosp=cosa(0);  double cosr=cosa(1); double cosy=cosa(2);
				
	Cnb << cosr*cosy-sinp*sinr*siny, -cosp*siny,  sinr*cosy+sinp*cosr*siny, cosr*siny+sinp*sinr*cosy, cosp*cosy, sinr*siny-sinp*cosr*cosy, -cosp*sinr, sinp, cosp*cosr;;
}

Vector3d Cnb2att(Matrix3d Cnb){
	Vector3d att;
	att << asin(Cnb(2,1)), atan2(-Cnb(2,0),Cnb(2,2)), atan2(-Cnb(0,1),Cnb(1,1));
	return att;
}

Matrix3d askew(Matrix<double, 3, 1> vector){
	Matrix3d matrix;
	matrix << 0, -vector(2, 0), vector(1, 0), vector(2, 0), 0, -vector(0, 0), -vector(1, 0), vector(0, 0), 0;
	return matrix;
}

Matrix3d rvec2mat(Matrix<double, 3, 1> vec){
	double theta=sqrt(vec.transpose()*vec);
	return Matrix3d::Identity()+(sin(theta)/theta*askew(vec))+((1-cos(theta))/pow(theta, 2)*(askew(vec).array().square().matrix()));
}

MatrixXd update_trans_mat(Earth eth, Vector3d vel, MatrixXd fn, Vector3d tauA, Vector3d tauG, double nt, Matrix3d Cnb, Matrix3d Mpv){
	
	Matrix3d zero33 = Matrix3d::Zero();
	double tanL = eth.tanL;   double secL = eth.secL;
	double sin2L= eth.sin2L;  double cos2L= eth.cos2L;
	double RNh  = eth.RNh;    double RMh  = eth.RMh;
	double vE   = vel(0);     double vN   = vel(1);  double vU   = vel(2);

	Matrix3d F1=zero33; F1(0, 1)=-eth.wnie(2); F1(0, 2)=eth.wnie(1);

	Matrix3d F2=zero33; F2(0, 1)=1/RNh; F2(0, 2)=tanL/RNh; F2(1, 0)=-1/RMh;

	Matrix3d F3=zero33; F3(0, 2)=vE*pow(secL, 2)/RNh; F3(2, 0)=vN/pow(RMh, 2); F3(2, 1)=-vE/pow(RNh, 2); F3(2, 2)=-vE*tanL/pow(RNh, 2);

	double x = -eth.g0*sin2L*(5.27094e-3-4*2.32718e-5*cos2L); 
	Matrix3d F4=zero33; F4(0, 2)=x; F4(2, 2)=3.086e-6;

	// transition matrix for phi
	Matrix3d Faa = -askew(eth.wnin);     
	Matrix3d Fav = F2;
	Matrix3d Fap = F1+F3;
		
	// transition matrix for delta-vel
	Matrix3d Fva = askew(fn);
	Matrix3d Fvv = askew(vel)*F2 - askew(eth.wnien);
	Matrix3d Fvp = askew(vel)*(2*F1+F3+F4);

	// transition matrix for delta-pos
	Matrix3d Fpp = zero33; Fpp(0, 1)=vE*secL*tanL/RNh; Fpp(2, 1)=-vN/pow(RMh, 2); Fpp(2, 2)= -vE*secL/pow(RNh, 2);
	Matrix3d Fpv = Mpv;    
		
	// time continuous state transition matrix
	Vector3d tauG2;
	tauG2(0) = -1/tauG(0);
	tauG2(1) = -1/tauG(1);
	tauG2(2) = -1/tauG(2);
	Vector3d tauA2;
	tauA2(0) = -1/tauA(0);
	tauA2(1) = -1/tauA(1);
	tauA2(2) = -1/tauA(2);
	MatrixXd Ft(15, 15);
	/*Ft << Faa,       Fav,       Fap,      -Cnb,            zero33,
		Fva,       Fvv,       Fvp,      zero33,              Cnb,
		zero33,    Fpv,       Fpp,      zero33,              zero33,
		zero33,    zero33,    zero33,   tauG2.asDiagonal(),  zero33,
		zero33,    zero33,    zero33,   zero33,              tauA2.asDiagonal();*/

	Ft << 	Faa(0,0),Faa(0,1),Faa(0,2),Fav(0,0),Fav(0,1),Fav(0,2),Fap(0,0),Fap(0,1),Fap(0,2),-Cnb(0,0),-Cnb(0,1),-Cnb(0,2), 0, 0, 0,
			Faa(1,0),Faa(1,1),Faa(1,2),Fav(1,0),Fav(1,1),Fav(1,2),Fap(1,0),Fap(1,1),Fap(1,2),-Cnb(1,0),-Cnb(1,1),-Cnb(1,2), 0, 0, 0,
			Faa(2,0),Faa(2,1),Faa(2,2),Fav(2,0),Fav(2,1),Fav(2,2),Fap(2,0),Fap(2,1),Fap(2,2),-Cnb(2,0),-Cnb(2,1),-Cnb(2,2), 0, 0, 0,

			Fva(0,0),Fva(0,1),Fva(0,2),Fvv(0,0),Fvv(0,1),Fvv(0,2),Fvp(0,0),Fvp(0,1),Fvp(0,2),0,0,0,Cnb(0,0),Cnb(0,1),Cnb(0,2),
			Fva(1,0),Fva(1,1),Fva(1,2),Fvv(1,0),Fvv(1,1),Fvv(1,2),Fvp(1,0),Fvp(1,1),Fvp(1,2),0,0,0,Cnb(1,0),Cnb(1,1),Cnb(1,2),
			Fva(2,0),Fva(2,1),Fva(2,2),Fvv(2,0),Fvv(2,1),Fvv(2,2),Fvp(2,0),Fvp(2,1),Fvp(2,2),0,0,0,Cnb(2,0),Cnb(2,1),Cnb(2,2),

			0,0,0,Fpv(0,0),Fpv(0,1),Fpv(0,2),Fpp(0,0),Fpp(0,1),Fpp(0,2),0,0,0,0,0,0,
			0,0,0,Fpv(1,0),Fpv(1,1),Fpv(1,2),Fpp(1,0),Fpp(1,1),Fpp(1,2),0,0,0,0,0,0,
			0,0,0,Fpv(2,0),Fpv(2,1),Fpv(2,2),Fpp(2,0),Fpp(2,1),Fpp(2,2),0,0,0,0,0,0,

			0,0,0,0,0,0,0,0,0,   tauG2(0),0,0,   0,0,0,
			0,0,0,0,0,0,0,0,0,   0,tauG2(1),0,   0,0,0,
			0,0,0,0,0,0,0,0,0,   0,0,tauG2(2),   0,0,0,

			0,0,0,0,0,0,0,0,0,0,0,0,   tauA2(0),0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,   0,tauA2(1),0,
			0,0,0,0,0,0,0,0,0,0,0,0,   0,0,tauA2(2);

	// discretization
	MatrixXd ret(15, 15);
	ret = MatrixXd::Identity(15, 15);
	ret = ret+Ft*nt;

	MatrixXd a(15, 15);
	a = Ft*nt;
	a = a.array().exp().matrix();
	if(nt>0.1)
		return a;
	else
		return ret;
}

void earth_update(Vector3d pos, Vector3d vel){
	eth.Re = 6378137;                    
	eth.f  = 1/298.257223563;            
	eth.Rp = (1-eth.f)*eth.Re;           
	eth.e1 = sqrt(pow(eth.Re, 2)-pow(eth.Rp, 2))/eth.Re;
	eth.e2 = sqrt(pow(eth.Re, 2)-pow(eth.Rp, 2))/eth.Rp; 
	eth.wie = 7.2921151467e-5;           
	eth.g0 = 9.7803267714;

	// update earth related parameters
	double B, L, h;
	double ve, vn, vu;
	B = pos(0); L = pos(1); h = pos(2); //ok
	ve = vel(0); vn = vel(1); vu = vel(2); //ok
	eth.RN = eth.Re/sqrt(1-pow(eth.e1, 2)*pow(sin(B), 2));           
	eth.RM = eth.RN*(1-pow(eth.e1, 2))/(1-pow(eth.e1, 2)*pow(sin(B), 2)); 
	eth.Mpv2 = sec(B)/(eth.RN+h);
	eth.Mpv4 = 1/(eth.RM+h);
	eth.RNh  = eth.RN+h;
	eth.RMh  = eth.RM+h;
	eth.tanL = tan(B);
	eth.secL = sec(B);
	eth.sinL = sin(B);
	eth.cosL = cos(B);
	eth.sin2L= sin(2*B);
	eth.cos2L= cos(2*B);

	//earth rotation rate projected in the n-frame
	eth.wnie = Vector3d::Zero();
	eth.wnie << 0, eth.wie*cos(B), eth.wie*sin(B);

	// the rate of n-frame respect to e-fame projected in the n-frame
	eth.wnen = Vector3d::Zero();
	eth.wnen << -vn/(eth.RM+h), ve/(eth.RN+h), ve/(eth.RN+h)*tan(B);

	// the rate of n-frame respect to i-fame projected in the n-frame
	eth.wnin = eth.wnie + eth.wnen;

	// earth gravity vector projected in the n-frame
	eth.g  = eth.g0*(1+5.27094e-3*pow(sin(B), 2)+2.32718e-5*pow(sin(B), 4))-3.086e-6*h; 
	eth.gn << 0, 0, -eth.g;

	// gcc:gravitational acceleration/coriolis acceleration/centripetal acceleration
	eth.wnien = 2*eth.wnie + eth.wnen;
	eth.gcc = -eth.wnien.cross(vel)+eth.gn;
}

void Dblh2Dxyz(Vector3d blh){
	// Convert perturbation error from n-frame to e-frame


	double B = blh(0); double L = blh(1); double H = blh(2);
	double sinB = sin(B); double cosB = cos(B); double sinL = sin(L); double cosL = cos(L);
	double N = RE_WGS84/sqrt(1-pow(ECC_WGS84, 2)*pow(sinB, 2)); double NH = N+H;
	double e2=pow(ECC_WGS84, 2);
	
	T << -NH*sinB*cosL, -NH*cosB*sinL, cosB*cosL, -NH*sinB*sinL, NH*cosB*cosL, cosB*sinL, (NH-e2*N)*cosB, 0, sinB;

	//Dxyz = T*Dblh;
}


Vector3d blh2xyz(Vector3d blh){
	Vector3d xyz;
	double B = blh(0); double L = blh(1); double H = blh(2);
	double sinB = sin(B); double cosB = cos(B); double sinL = sin(L); double cosL = cos(L);
	double N = RE_WGS84/sqrt(1-pow(ECC_WGS84, 2)*pow(sinB, 2));
	double X = (N+H)*cosB*cosL;
	double Y = (N+H)*cosB*sinL;
	double Z = (N*(1-pow(ECC_WGS84, 2))+H)*sinB;
	xyz << X, Y, Z;
	// transformation matrix from n-frame to e-frame
	Cen << -sinL, cosL, 0, -sinB*cosL, -sinB*sinL, cosB, cosB*cosL, cosB*sinL, sinB;
	Cen.transposeInPlace();
	return xyz;
}

Vector3d xyz2blh(Vector3d xyz){

	Vector3d blh = ecef2geo(xyz);
	double B=blh(0); double L=blh(1);
	double sinB = sin(B); double cosB = cos(B);
	double sinL = sin(L); double cosL = cos(L);

	Cne <<   -sinL,      cosL,        0,
        -sinB*cosL, -sinB*sinL, cosB,
        cosB*cosL,  cosB*sinL, sinB;

	return blh;
}

void calculateSTD(gnss_sol_t* int_sol){
	//STANDARD DEVIATION START
	if(fst_pos){ //Variance Initialization
		Vector3d geostdpos;
		Vector3d tmp;

		/*
		decomment for production

		pos << IMUsol.posXYZ.at(0), IMUsol.posXYZ.at(1), IMUsol.posXYZ.at(2);
		vel << IMUsol.velXYZ.at(0), IMUsol.velXYZ.at(1), IMUsol.velXYZ.at(2);

		geostdpos = ecef2geo(eigVector2std(pos));
		tmp(0) = geostdpos.at(0);
		tmp(1) = geostdpos.at(1);
		tmp(2) = geostdpos.at(2);
		pos = tmp;
		*/

		//debug start, to be removed when everything works
		//pos << -2.408691046019300e+06 , 4.698107906468073e+06, 3.566697863037622e+06;
		pos << (*int_sol).a , (*int_sol).b, (*int_sol).c;

		pos = ecef2geo(pos);
		/*tmp(0) = geostdpos(0);
		tmp(1) = geostdpos(1);
		tmp(2) = geostdpos(2);
		pos = tmp;*/
		//debug end, to be removed

		//vel << 0, 0, 0;
		init_att_unc(0) = init_att_unc(0) * D2R;
		init_att_unc(1) = init_att_unc(1) * D2R;
		init_att_unc(2) = init_att_unc(2) * D2R;

		init_pos_unc(0) = init_pos_unc(0) / RE_WGS84;
		init_pos_unc(1) = init_pos_unc(1) / RE_WGS84;
		init_pos_unc(2) = init_pos_unc(2);



		/*if (fabs(vel(0)) < 1e-4){
			vel(0) = 1e-4;
		}
		yaw = -atan2(vel(0), vel(1));

		att << 0, 0, yaw;

		avp0 << att, vel, pos;
		acc = Vector3d::Zero();
		att2Cnb(att);*/
		//Initialize IMU error
		bg = Vector3d::Zero();
		ba = Vector3d::Zero();
		Kg = Matrix3d::Identity();
		Ka = Matrix3d::Identity();
		tauG << std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity();
		tauA << std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity();

		//Initialize earth related parameters
		earth_update(pos, vel); 
		Mpv << 0, eth.Mpv4, 0, eth.Mpv2, 0, 0, 0, 0, 1;
		wib = Vector3d::Zero(3);
		fb = Vector3d::Zero(3);
		fn = -eth.gn; 					
		web = Vector3d::Zero(3);

		mx_init_bg_unc << init_bg_unc, init_bg_unc, init_bg_unc;

		mx_init_ba_unc << init_ba_unc, init_ba_unc, init_ba_unc;

		mx_pad_gyro << psd_gyro, psd_gyro, psd_gyro;

		mx_pad_acce << psd_acce, psd_acce, psd_acce;

		mx_pad_bg << psd_bg, psd_bg, psd_bg;

		mx_pad_ba << psd_ba, psd_ba, psd_ba;

		x << att, vel, pos, bg, ba;

		initP << init_att_unc, init_vel_unc, init_pos_unc, mx_init_bg_unc, mx_init_ba_unc;
		P = initP.array().square().matrix().asDiagonal();
		
		initQ << mx_pad_gyro, mx_pad_acce, Vector3d::Zero(3), mx_pad_bg, mx_pad_ba;
		Q = initQ.array().matrix().asDiagonal() * nt;

		blh2xyz(pos);

		Dblh2Dxyz(pos);
		
		initPosvar << P(6, 6), P(7, 7), P(8, 8);
		posvar = initPosvar.array().matrix().asDiagonal(); //blh variance

		posvar = T * posvar * T.transpose(); //xyz variance

		//variance
		va << posvar(0, 0), posvar(1, 1), posvar(2, 2), posvar(0,1), posvar(1,2), posvar(0,2);
		//printf("BB: %lf\n", posvar(0, 0));

		for(int i = 0; i < 6; i++){
			if(va(i) < 0)
				va(i) = -sqrt(-va(i));
			else
				va(i) = sqrt(va(i));
		}

		fst_pos = 0;

		/*dv0 << -0.791625976562500, 0.177917480468750, 9.638977050781250;
		dw0 << -0.004286024305556, -0.024490017361111, 0.126399739583333;*/

		//Debug print
		//printf("AA: %lf %lf %lf\n", va(0), va(1), va(2));
		(*int_sol).sda = va(0);
		(*int_sol).sdb = va(1);
		(*int_sol).sdc = va(2);
		(*int_sol).sdab = va(3);
		(*int_sol).sdbc = va(4);
		(*int_sol).sdca = va(5);
	}else{
		/*GNSS_pos << (*int_sol).a , (*int_sol).b, (*int_sol).c;
		GNSS_vel << 0, 0, 0;
		gnss_posP << (*int_sol).sda, (*int_sol).sdb, (*int_sol).sdc, (*int_sol).sdab, (*int_sol).sdbc, (*int_sol).sdca;
		gnss_velP << 0, 0, 0, 0, 0, 0;
		rr = GNSS_pos.transpose(); 
		pos_GNSS=xyz2blh(rr); */
		//Vector3d vel_GNSS=Cne*GNSS_vel.transpose();

		Pp << gnss_posP(0), gnss_posP(3), gnss_posP(5), 
		gnss_posP(3), gnss_posP(1), gnss_posP(4),
		gnss_posP(5), gnss_posP(4), gnss_posP(2);

		print(Pp);

		if(Pp.diagonal().maxCoeff()>VAR_POS)
			rr=Vector3d::Zero();
		else{
			Pp=T.inverse()*Pp*T.transpose().inverse();
			VAR1 << Pp(0,0), Pp(1,1), Pp(2,2);
		}

		print(VAR1);

		/*Matrix<double, 1, 6> velP=gnss_velP; 
		Pp << velP(0), velP(3), velP(5),
			velP(3), velP(1), velP(4),
			velP(5), velP(4), velP(2);
		

		if(Pp.diagonal().maxCoeff()>VAR_VEL)
			ve=Vector3d::Zero();
		else{
			Pp=Cne*Pp*Cne.transpose();
			VAR2 << Pp(0,0), Pp(1,1), Pp(2,2);
			if(VAR2.norm()==0)
				VAR2 << VAR_VEL, VAR_VEL, VAR_VEL;
		}*/

		Vector3d pos_INS=pos+Mpv*Cnb*lever;

		print(pos_INS);
		//Vector3d vel_INS=vel+Cnb*askew(web)*lever;

		v=pos_INS-pos_GNSS; 
		R=VAR1.asDiagonal();
		H=MatrixXd::Zero(3,15);
		H(0,6)=1;H(1,7)=1;H(2,8)=1;

		print(v);
		print(R);
		print(H);

		VectorXd x_pre = x;
		MatrixXd P_pre = P;
		print(x_pre);
		print(P_pre);

		////////////////////// measurement update
		int nx= 15 /*size(x_pre,1)*/; int stat=1;

		Q=H*P_pre*H.transpose()+R;
		if(Q.determinant()==0)
			stat=0;

		print(Q);

		MatrixXd K(15, 3); K =P_pre*H.transpose()*Q.inverse();
		VectorXd xx(15); xx = K*v;
		P=(MatrixXd::Identity(nx, nx)-K*H)*P_pre;

		print(K);

		print(xx);

		print(P);

		//if(~isreal(xx)||~isreal(P))
		//	stat=0;

		Vector3d h; h << xx(0), xx(1), xx(2);
		Cnb = (Matrix3d::Identity()+askew(h))*Cnb;

		print(Cnb);
		//if ~isreal(Cnb)
		//	stat=0;

		//Cnb = Cnb;
		att = Cnb2att(Cnb);

		print(att);
		Vector3d xx_i;
		//xx_i << xx(3), xx(4), xx(5);
		//vel = vel-xx_i;
		xx_i << xx(6), xx(7), xx(8);
		pos = pos-xx_i;
		xx_i << xx(9), xx(10), xx(11);
		bg  = bg+xx_i;
		xx_i << xx(12), xx(13), xx(14);
		ba  = ba+xx_i;
		x  << att, vel, pos, bg, ba;

		print(pos);
		print(bg);
		print(ba);
		print(x);
		//P = P;

		Vector3d helper; helper << x(6), x(7), x(8);
		rr=blh2xyz(helper); Dblh2Dxyz(helper);
		pos=rr.transpose();

		print(rr);

		print(pos);

		helper << x(3), x(4), x(5);
		//vel=(Cen*helper).transpose();
		helper << x(0), x(1), x(2);
		att=helper.transpose()/0.0175;

		print(att);

		if(att(2)>=0)
			att(2)=360-att(2);
		else
			att(2)=-att(2);

		initPosvar << P(6, 6), P(7, 7), P(8, 8);
		posvar = initPosvar.array().matrix().asDiagonal(); //blh variance
        posvar1 << P(6,6), P(6,7), P(6,8), P(7,6), P(7,7), P(7, 8), P(8, 6), P(8, 7), P(8, 8);
		posvar=T*posvar1*T.transpose();        //blh_var to xyz_var

		print(posvar1);

		//velvar=P(4:6,4:6); velvar=Cen*velvar*Cen';    %enu_var to xyz_var


		va << posvar(0, 0), posvar(1, 1), posvar(2, 2), posvar(0,1), posvar(1,2), posvar(0,2);

		print(va);

		for(int i = 0; i < 6; i++){
			if(va(i) < 0)
				va(i) = -sqrt(-va(i));
			else
				va(i) = sqrt(va(i));
		}

		//Debug print
		//printf("AA: %lf %lf %lf\n", va(0), va(1), va(2));

		(*int_sol).sda = va(0);
		(*int_sol).sdb = va(1);
		(*int_sol).sdc = va(2);
		(*int_sol).sdab = va(3);
		(*int_sol).sdbc = va(4);
		(*int_sol).sdca = va(5);

	}
	//STANDARD DEVIATION END
}
int deb_enabled = 1;
void deb(char message[300]){
    if(deb_enabled){
        printf("%s\n", message);
    }
}

void Loosely::get_imu_sol(gnss_sol_t* int_sol){
	printf("get_imu_sol()\n");
	 /* struct timeval tv;
            gettimeofday(&tv, NULL);
            int week = ((tv.tv_sec+LEAP_SECONDS-GPS_EPOCH))/(7*24*3600);
            double sec = (double)(((tv.tv_sec+LEAP_SECONDS-GPS_EPOCH))%(7*24*3600))+(tv.tv_usec / 1000000.0);
	printf("to process: %d | now: %lf\n", (*int_sol).time.sec, sec); */
	//Check if imu is initializing
	if(imu_ready == 0){
		int n = 0;
		while(/* n < 104 */ OBSimu._IMUdata.imuTime < (*int_sol).time.sec){ //read whole imu date of a particular gnss epoch
			if(imu_ready == 0 && iniIMU.stepInitializeIMU(OBSimu, IMU_INI_TIME_END, _LLH_o) == 1){ //it calculates _ACCbias and _GYRbias, _RPY
				_dT = 1.0;
				// Initialize IMU Mechanization
				MechECEF.InitializeMechECEF(_ECEF_imu, _LLH_o, GNSSsol.velXYZ, iniIMU._RPY, iniIMU._ACCbias, iniIMU._GYRbias); //it initializes MechECEF with _ACCbias, _GYRbias and _RPY
				imu_ready = 1; //Tells client.c that it can read imu data
				printf("SEMOR: End initialization\n");
			}
			read_imu();
			if(imu_ready == 1){
				imu_ready = 2;
				(*int_sol).time.week = OBSimu._IMUdata.week;
				/*avg_acc_x = tot_acc_x/((double)nsamples);
				avg_acc_y = tot_acc_y/((double)nsamples);
				avg_acc_z = tot_acc_z/((double)nsamples);

				printf("%lf - %lf - %lf\n", avg_acc_x, avg_acc_y, avg_acc_z);*/
			}
			_epochIMU = OBSimu._IMUdata.imuTime;
			n++;
		}
		//cout << n << endl;
		//cout.flush();
		return;
	}
	//IMU is ready:

	//Initialize IMU mechanization with initial position, initial velocity and biases
	_ECEF_o = eigVector2std(double2eigVector((*int_sol).a, (*int_sol).b, (*int_sol).c));
	_LLH_o = eigVector2std(ecef2geo(double2eigVector((*int_sol).a, (*int_sol).b, (*int_sol).c)));	
	_ECEF_imu = _ECEF_o;
	GNSSsol.velXYZ = eigVector2std(double2eigVector((*int_sol).va, (*int_sol).vb, (*int_sol).vc));
	/*if(1){
		MechECEF.InitializeMechECEF(_ECEF_imu, _LLH_o, GNSSsol.velXYZ, iniIMU._RPY, iniIMU._ACCbias, iniIMU._GYRbias);
		first = 0;
	}*/
	//here we have previous gnss position



	//print_time();
	
	
	int n = 0;
	do {
		//Here I already have the first sample of this epoch, so I read new imu data only at end of iteration
		// Process IMU
		SolutionIMU(OBSimu, MechECEF);	//this is the gnss+first imu data		                                                                      	//Get position from current MechECEF state and IMU data just read - This has effects on: MechECEF e IMUsol
		//Here we have gnss+acc+gyr

		 (*int_sol).a = IMUsol.posXYZ.at(0);
		(*int_sol).b = IMUsol.posXYZ.at(1);
		(*int_sol).c = IMUsol.posXYZ.at(2);

		(*int_sol).va = IMUsol.velXYZ.at(0);
		(*int_sol).vb = IMUsol.velXYZ.at(1);
		(*int_sol).vc = IMUsol.velXYZ.at(2); 

		//calculateSTD(int_sol);
		//printf("%f, %f, %f\n", (*int_sol).sda, (*int_sol).sdb, (*int_sol).sdc);
		//Update Time
		_epochIMU = OBSimu._IMUdata.imuTime;

		read_imu();

		n++;
	} while (/* n <= 104 */ _epochIMU <= (*int_sol).time.sec);
	if(logs){
		out << "---------------------------------------------------" << endl;
		out.flush();
	}
	//cout << n << endl;
	//cout.flush();

	//

	//here we have next gnss+imu position

	//We have position and STD of this epoch


	//Mark imu solution as usable for the comparison (in order to get the best solution for SEMOR)
	(*int_sol).time.week = OBSimu._IMUdata.week;
}

void loop_imu_thread(){
	char buf[IMU_LENGTH];
	while(1){
		get_imu_data(buf);

		//modificare matrce "imu"
		
	}
}

void Loosely::init_imu(gnss_sol_t fst_pos){
	FileIO FIO;

	time_t rawtime;
    struct tm info;
    time( &rawtime );
    info = *localtime( &rawtime );
    char str_time[13];

    sprintf(str_time, "%d_%02d_%02d_%02d_%02d", (info.tm_year+1900), (info.tm_mon+1), info.tm_mday, info.tm_hour, info.tm_min);

	stringstream ss, ss1;
	if(relative){
		ss << "test/imu.csv";
		ss1 << log_dir << "/imu_raw" << ".log";
	}
	else{
		ss << root_path << "test/imu.csv";
		ss1 << log_dir << "/imu_raw" << ".log";
	}
	FIO.fileSafeIn(ss.str(), fimu);

	if(logs){
		out.open(ss1.str());
	}

	OBSgnss.readEpoch(fst_pos);
	read_imu(); //fill OBSimu

	_epochIMU = OBSimu._IMUdata.imuTime;
	_epochGNSS = fst_pos.time.sec;

	// Initial ECEF position for vehicle from GNSS     It puts the first position of the gnss file (first line) in _ECEF_o (and in _ECEF_imu e in GNSSsol.posXYZ)
	_ECEF_o = eigVector2std(double2eigVector(fst_pos.a, fst_pos.b, fst_pos.c));
	_ECEF_imu = _ECEF_o; GNSSsol.posXYZ = _ECEF_o;
	GNSSsol.velXYZ = eigVector2std(double2eigVector(fst_pos.va, fst_pos.vb, fst_pos.vc));

	_epochTime = _epochGNSS;

	// Initial Position in Geodetic and ENU
	Vector3d h;
	h << _ECEF_o.at(0), _ECEF_o.at(1), _ECEF_o.at(2);
	_LLH_o = eigVector2std(ecef2geo(h));	

	IMU_INI_TIME_END = _epochIMU+imu_init_epochs; // Time taken to initialize the imu  (first imu epoch + 300) (for example)

	int err = pthread_create(&imu_thread_id, NULL, loop_imu_thread(), NULL);

}

// Facilitates the GNSS-IMU Loose integration process
/*Loosely::Loosely(gnss_sol_t fst_pos){
	//Inizializzazione IMU
	ReaderGNSS OBSgnss;
	ReaderIMU OBSimu;

	_epochIMU = OBSimu._IMUdata.imuTime; //TODO: read imu time
	_epochGNSS = fst_pos.time.sec;

	// Initial ECEF position for vehicle from GNSS     It puts the first position of the gnss file (first line) in _ECEF_o (and in _ECEF_imu e in GNSSsol.posXYZ)
	_ECEF_o = eigVector2std(double2vector(fst_pos.a, fst_pos.b, fst_pos.c));
	_ECEF_imu = _ECEF_o; GNSSsol.posXYZ = _ECEF_o;
	GNSSsol.velXYZ = eigVector2std(fst_pos.va, fst_pos.vb, fst_pos.vc);

	_epochTime = _epochGNSS;

	// Initial Position in Geodetic and ENU
	_LLH_o = ecef2geo(_ECEF_o);	

	double IMU_INI_TIME_END = 72600.0; // Time taken to initialize the imu  (first imu epoch + 300) (for example)
	InitializeIMU iniIMU(IMU_INI_TIME_END, _LLH_o); //It initializes IMU with initial position + accelerometer and gyroscope data from IMU file

	_dT = 1.0;						//Strange: this value is not used (it is always reset to another value but it is not read), so i think this line is useless

	// Initialize IMU Mechanization
	IMUmechECEF MechECEF; 
	MechECEF.InitializeMechECEF(_ECEF_imu, _LLH_o, GNSSsol.velXYZ, iniIMU._RPY, iniIMU._ACCbias, iniIMU._GYRbias);
	OBSimu.obsEpoch(fin_imu); _epochIMU = OBSimu._IMUdata.imuTime;
	IMUsol.posXYZ = _ECEF_imu;

	double Tdiff = 0;
		// Loop through IMU and GPS observation file
		while (1) { //finché semor non viene terminato
			// At this point, IMU is 1s behind GPS
			 
			// *** Read and Solve IMU 100Hz
			//Tdiff = fabs(_epochIMU - _epochGNSS);						//Useless line
			do {
				OBSimu.clearObs();							//Clear OBSimu (i remind you that this is the current line of the imu file in a structured version)
				OBSimu.obsEpoch(fin_imu);					//Read the next line
				// Process IMU
				SolutionIMU(OBSimu, MechECEF);				//This has effects on: MechECEF e IMUsol
				// Output to file
				epochOutputIMU(fout_out);
				// Update Time
				_epochIMU = OBSimu._IMUdata.imuTime;
				Tdiff = fabs(_epochIMU - _epochGNSS);
			} while (Tdiff > 0.0001);

			// *** Inegrated Solution (Loosely Coupled)
			LooseCoupling(OBSgnss, MechECEF);				//This has effects on: MechECEF, on OBSgnss e on INTsol, it utilize (reads data from) GNSSsol

			// *** Read GPS 1Hz
			OBSgnss.clearObs();
			OBSgnss.readEpoch(fin_gnss);
			// Compute timing information
			_epochGNSS = OBSgnss._GNSSdata.gpsTime;
			// Process Epoch
			SolutionGNSS(OBSgnss);							//This has effects on: _epochGNSS, GNSSsol
		}	
			
}*/


/*Loosely::Loosely(std::string filePathGNSS, std::string filePathIMU, std::string filePathOUT) {
	if (isValid(filePathGNSS) && isValid(filePathIMU)) {
		
		// Input File Streams
		ifstream fin_gnss;
		ifstream fin_imu;
		// Output File Streams
		ofstream fout_out;

		// File IO Handler
		FileIO FIO;			//Utilized for opening files

		// Setting up output file					//File opening
		FIO.fileSafeOut(filePathOUT, fout_out);
		// Reading input files: Setting up the pointers after checking for errors
		FIO.fileSafeIn(filePathGNSS, fin_gnss);
		FIO.fileSafeIn(filePathIMU, fin_imu);

		// Create Obs Data Object
		ReaderGNSS OBSgnss;		//It represents the current line of the gnss input file (structured)
		ReaderIMU OBSimu;		//Same but for the imu input file

		// Read Obs Header from input file
		OBSgnss.readHeader(fin_gnss);

		// Reading first line of sensor data to determine time of obs
		string line;
		getline(fin_imu, line); OBSimu.obsEpoch(fin_imu); //getline deletes the first line from the imu file (trash) OBSimu.obsEpoch reads the next line (the first) from the imu file
		getline(fin_gnss, line); OBSgnss.readEpoch(fin_gnss); //getline deletes the first line from the gnss file (trash) OBSgnss.readEpoch reads the next line (the first) from the gnss file
		_epochIMU = OBSimu._IMUdata.imuTime;
		_epochGNSS = OBSgnss._GNSSdata.gpsTime;

		// Initial ECEF position for vehicle from GNSS     It puts the first position of the gnss file (first line) in _ECEF_o (and in _ECEF_imu e in GNSSsol.posXYZ)
		_ECEF_o = eigVector2std(OBSgnss._GNSSdata.Pxyz);
		_ECEF_imu = _ECEF_o; GNSSsol.posXYZ = _ECEF_o;
		GNSSsol.velXYZ = eigVector2std(OBSgnss._GNSSdata.Vxyz);

		// Since GNSS data starts before IMU
		_epochTime = _epochGNSS;				//GNSS starts before IMU because IMU needs to be initialized based on the GNSS data

		// Initial Position in Geodetic and ENU
		_LLH_o = ecef2geo(_ECEF_o);				//Initial position (_ECEF_o) turned from x y z to lat long and height coordinates

		// Preparing Output File
		initialOutput(fout_out, _ECEF_o); //Output file's header, it writes the ecef coordinates of the initial position

		// Initializing IMU attitude using stationary data
		double IMU_INI_TIME_END = 72600.0; // Time taken to initialize the imu  (first imu epoch + 300) (for example)
		InitializeIMU iniIMU(fin_imu, IMU_INI_TIME_END, _LLH_o); //It initializes IMU with initial position + accelerometer and gyroscope data from IMU file

		// Setting default values
		_dT = 1.0;						//Strange: this value is not used (it is always reset to another value but it is not read), so i think this line is useless

		// Initialize IMU Mechanization
		IMUmechECEF MechECEF; 
		MechECEF.InitializeMechECEF(_ECEF_imu, _LLH_o, GNSSsol.velXYZ, iniIMU._RPY, iniIMU._ACCbias, iniIMU._GYRbias);
		OBSimu.obsEpoch(fin_imu); _epochIMU = OBSimu._IMUdata.imuTime;
		IMUsol.posXYZ = _ECEF_imu;

		// *** Integrated Solution
		double Tdiff = 0;
		// Loop through IMU and GPS observation file
		while ((!fin_imu.eof()) && (!fin_gnss.eof())) {
			// At this point, IMU is 1s behind GPS
			 
			// *** Read and Solve IMU 100Hz
			//Tdiff = fabs(_epochIMU - _epochGNSS);						//Useless line
			do {
				OBSimu.clearObs();							//Clear OBSimu (i remind you that this is the current line of the imu file in a structured version)
				OBSimu.obsEpoch(fin_imu);					//Read the next line
				// Process IMU
				SolutionIMU(OBSimu, MechECEF);				//This has effects on: MechECEF e IMUsol
				// Output to file
				epochOutputIMU(fout_out);
				// Update Time
				_epochIMU = OBSimu._IMUdata.imuTime;
				Tdiff = fabs(_epochIMU - _epochGNSS);
			} while (Tdiff > 0.0001);

			// *** Inegrated Solution (Loosely Coupled)
			LooseCoupling(OBSgnss, MechECEF);				//This has effects on: MechECEF, on OBSgnss e on INTsol, it utilize (reads data from) GNSSsol

			// *** Read GPS 1Hz
			OBSgnss.clearObs();
			OBSgnss.readEpoch(fin_gnss);
			// Compute timing information
			_epochGNSS = OBSgnss._GNSSdata.gpsTime;
			// Process Epoch
			SolutionGNSS(OBSgnss);							//This has effects on: _epochGNSS, GNSSsol
		}		
		// Close Files
		fin_imu.close(); fin_gnss.close();
	}
}
*/