/*
* Loosely.cpp
* Loosely Coupled Integration of GPS & IMU
*  Created on: Sept 10, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "Loosely.h"
#include "ReadIMU.h"
using namespace std;
using namespace Eigen;


// PI
const double PI = 3.1415926535898;

//Variance
MatrixXd x(1, 15);
MatrixXd P(15, 15);
MatrixXd Q(15, 15);
MatrixXd v;
MatrixXd R;
MatrixXd H;
//Dblh2Dxyz
MatrixXd T;
MatrixXd Dxyz;
//blh2xyz
MatrixXd Cen(3, 3);
MatrixXd xyz(1, 3);

MatrixXd Cnb(3, 3);

Matrix<double, 1, 15> xa;
Matrix<double, 15, 15> Pa;
MatrixXd init_att_unc(1, 3);
MatrixXd init_vel_unc(1, 3);
MatrixXd init_pos_unc(1, 3);
double init_bg_unc = 4.8481367284e-05;
double init_ba_unc = 0.048901633857000000;
double psd_gyro  = 3.38802348178723e-09;
double psd_acce      =     2.60420170553977e-06;     // acce noise PSD (m^2/s^3)  
double psd_bg        =     2.61160339323310e-14;     // gyro bias random walk PSD (rad^2/s^3)
double psd_ba        =     1.66067346797506e-09;
const int sample_rate = 104;
const int nt=1/sample_rate;
const double D2R = PI/180.0;
const double RE_WGS84 = 6378137.0;
const double FE_WGS84    =(1.0/298.257223563); //earth flattening (WGS84)
const double ECC_WGS84   =sqrt(2*FE_WGS84-pow(FE_WGS84, 2)); 
int fst_pos = 1;


IMUmechECEF MechECEF; 

ifstream fimu;

InitializeIMU iniIMU;
double IMU_INI_TIME_END;

ReaderGNSS OBSgnss;
ReaderIMU OBSimu;

Loosely::Loosely(){
	init_att_unc << 0.3, 0.3, 0.5;
	init_vel_unc << 10, 10, 10;
	init_pos_unc << 30, 30, 30;
}


void read_imu(){
	string line;
	//Read IMU i2c
	//char buf[IMU_LENGTH];
	//read_raw_imu(buf);
	//TODO: char[] to line

	//Read IMU file
	getline(fimu, line);
	if (line.find("GPS") != std::string::npos) {
		getline(fimu, line);
	}
	OBSimu.clearObs();
	OBSimu.obsEpoch(line);
}

// Function
/*bool Loosely::isValid(string observation_filepath) {
	ifstream fin(observation_filepath);
	if (!fin.good()) {
		return false;
		cout << "File does not exist ->> No File for reading";
		exit(1);
	}
	else {
		return true;
	}
}

// A function to initialize output text file
void initialOutput(ofstream& fout, vector<double> iniPOS) {
	fout << "\n\tEPOCHWISE SOLUTIONS FOR THE TRADITIONAL INTEGRATION STRATEGY.\n"
		<< "\tInitial Estimate of Receiver Position : \n" << std::left << std::fixed << std::setprecision(3)
		<< std::setw(15) << "\tX (m): " << std::setw(15) << iniPOS.at(0) << "\n"
		<< std::setw(15) << "\tY (m): " << std::setw(15) << iniPOS.at(1) << "\n"
		<< std::setw(15) << "\tZ (m): " << std::setw(15) << iniPOS.at(2) << "\n"
		<< "---------------------------------------------------------------------------------------------------------------------"
		<< "---------------------------------------------------------------------------------------------------------------------" << "\n"
		<< "\t|| Loose GNSS-IMU Integration ||" << "\n"
		<< "---------------------------------------------------------------------------------------------------------------------"
		<< "---------------------------------------------------------------------------------------------------------------------" << "\n"
		<< std::setw(12) << "EPOCH"
		<< std::setw(15) << "X(m)" << std::setw(15) << "Y(m)" << std::setw(15) << "Z(m)"
		<< std::setw(15) << "Vx(m/s)" << std::setw(15) << "Vy(m/s)" << std::setw(15) << "Vz(m/s)"
		<< std::setw(15) << "Tx(deg)" << std::setw(15) << "Ty(deg)" << std::setw(15) << "Tz(deg)"
		<< "\n"
		<< "---------------------------------------------------------------------------------------------------------------------"
		<< "---------------------------------------------------------------------------------------------------------------------" << "\n";
}

// A function to output epochwise solution into an organized text file
void Loosely::epochOutput(ofstream& fout) {
	// Normalize attitude
	double r = rad2deg(IMUsol.attXYZ.at(0));
	double p = rad2deg(IMUsol.attXYZ.at(1));
	double h = rad2deg(IMUsol.attXYZ.at(2));
	// Posting Results to Output File
	fout << std::left << std::fixed << std::setprecision(3)
		<< std::setw(12) << _epochTime
		<< std::setw(15) << INTsol.posXYZ.at(0) << std::setw(15) << INTsol.posXYZ.at(1) << std::setw(15) << INTsol.posXYZ.at(2)
		<< std::setw(15) << INTsol.velXYZ.at(0) << std::setw(15) << INTsol.velXYZ.at(1) << std::setw(15) << INTsol.velXYZ.at(2)
		<< std::setw(15) << r << std::setw(15) << p << std::setw(15) << h
		<< "\n";
}

// A function to output epochwise solution into an organized text file
void Loosely::epochOutputIMU(ofstream& fout) {
	// Normalize attitude
	double r = rad2deg(IMUsol.attXYZ.at(0));
	double p = rad2deg(IMUsol.attXYZ.at(1));
	double h = rad2deg(IMUsol.attXYZ.at(2));
	// Posting Results to Output File
	fout << std::left << std::fixed << std::setprecision(3)
		<< std::setw(12) << _epochIMU
		<< std::setw(15) << IMUsol.posXYZ.at(0) << std::setw(15) << IMUsol.posXYZ.at(1) << std::setw(15) << IMUsol.posXYZ.at(2)
		<< std::setw(15) << IMUsol.velXYZ.at(0) << std::setw(15) << IMUsol.velXYZ.at(1) << std::setw(15) << IMUsol.velXYZ.at(2)
		<< std::setw(15) << r << std::setw(15) << p << std::setw(15) << h
		<< "\n";
}

// A function to output epochwise solution into an organized text file
void Loosely::epochOutputGPS(ofstream& fout) {
	// Posting Results to Output File
	fout << std::left << std::fixed << std::setprecision(3)
		<< std::setw(12) << _epochGNSS
		<< std::setw(15) << GNSSsol.posXYZ.at(0) << std::setw(15) << GNSSsol.posXYZ.at(1) << std::setw(15) << GNSSsol.posXYZ.at(2)
		<< std::setw(15) << GNSSsol.velXYZ.at(0) << std::setw(15) << GNSSsol.velXYZ.at(1) << std::setw(15) << GNSSsol.velXYZ.at(2)
		<< "\n";
}*/

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

// A routine to organize GNSS solution from file
/*void Loosely::SolutionGNSS(ReaderGNSS GNSS) {
	// Compute time interval
	_dT = GNSS._GNSSdata.gpsTime - _epochGNSS;
	// Update solution
	_epochGNSS = GNSS._GNSSdata.gpsTime;
	GNSSsol.posXYZ = eigVector2std(GNSS._GNSSdata.Pxyz);
	GNSSsol.velXYZ = eigVector2std(GNSS._GNSSdata.Vxyz);
}

// A routine to process observations for integrated position solution
void Loosely::LooseCoupling(ReaderGNSS &GNSS, IMUmechECEF& Mech) {
	// Time interval
	_dT = 0.001;
	// Loose Coupling Kalman Filter
	LooseKF KF(Mech, _dT);
	KF.SetObs(GNSS, Mech);
	KF.Filter(Mech);
	// Update solution
	_epochTime = GNSSsol.epoch;
	INTsol.posXYZ = KF.sol.posXYZ;
	INTsol.velXYZ = KF.sol.velXYZ;
	INTsol.attXYZ = KF.sol.attXYZ;
	INTsol.df = KF.sol.df;
	INTsol.dw = KF.sol.dw;
	// Update IMU Position
	Mech._pos = INTsol.posXYZ;
	Mech._vel = INTsol.velXYZ;
	Mech._att = INTsol.attXYZ;
	Mech._fbias = INTsol.df;
	Mech._gbias = INTsol.dw;
	// Add heading corrections below for improved results...

}*/

VectorXd double2eigVector(double a, double b, double c){
	VectorXd v = VectorXd::Zero(3);
	v(0) = a;
	v(1) = b;
	v(2) = c;
	return v;
}

class Earth{
	public:
		int Re;
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
		MatrixXd wnie;
		MatrixXd wnen;
		MatrixXd wnin;
		double g;
		MatrixXd gn;
		MatrixXd wnien;
		MatrixXd gcc;

};

void att2Cnb(MatrixXd att){
	MatrixXd sina(1, 3); MatrixXd cosa(1, 3);
	sina << sin(att(0)), sin(att(1)), sin(att(2));
	cosa << cos(att(0)), cos(att(1)), cos(att(2));
	double sinp=sina(1);  double sinr=sina(2); double siny=sina(3);
	double cosp=cosa(1);  double cosr=cosa(2); double cosy=cosa(3);
				
	Cnb << cosr*cosy-sinp*sinr*siny, -cosp*siny,  sinr*cosy+sinp*cosr*siny, cosr*siny+sinp*sinr*cosy, cosp*cosy, sinr*siny-sinp*cosr*cosy, -cosp*sinr, sinp, cosp*cosr;;
}

void earth_update(MatrixXd pos, MatrixXd vel, Earth& eth){//todo use global variable
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
	B = pos(1); L = pos(2); h = pos(3); //ok
	ve = vel(1); vn = vel(2); vu = vel(3); //ok
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
	eth.wnie = MatrixXd::Zero(1, 3);
	eth.wnie << 0, eth.wie*cos(B), eth.wie*sin(B);

	// the rate of n-frame respect to e-fame projected in the n-frame
	eth.wnen = MatrixXd::Zero(1, 3);
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

void Dblh2Dxyz(MatrixXd blh){
	// Convert perturbation error from n-frame to e-frame


	double B = blh(1); double L = blh(2); double H = blh(3);
	double sinB = sin(B); double cosB = cos(B); double sinL = sin(L); double cosL = cos(L);
	double N = RE_WGS84/sqrt(1-pow(ECC_WGS84, 2)*pow(sinB, 2)); double NH = N+H;
	double e2=pow(ECC_WGS84, 2);
	
	T << -NH*sinB*cosL, -NH*cosB*sinL, cosB*cosL, -NH*sinB*sinL, NH*cosB*cosL, cosB*sinL, (NH-e2*N)*cosB, 0, sinB;

	//Dxyz = T*Dblh;
}


void blh2xyz(MatrixXd blh){
	double B = blh(1); double L = blh(2); double H = blh(3);
	double sinB = sin(B); double cosB = cos(B); double sinL = sin(L); double cosL = cos(L);
	double N = RE_WGS84/sqrt(1-pow(ECC_WGS84, 2)*pow(sinB, 2));
	double X = (N+H)*cosB*cosL;
	double Y = (N+H)*cosB*sinL;
	double Z = (N*(1-pow(ECC_WGS84, 2))+H)*sinB;
	xyz << X, Y, Z;

	// transformation matrix from n-frame to e-frame
	Cen << -sinL, cosL, 0, -sinB*cosL, -sinB*sinL, cosB, cosB*cosL, cosB*sinL, sinB;
	Cen = Cen.transpose();
}

void Loosely::get_imu_sol(gnss_sol_t* int_sol){
	//Check if imu is initializing
	if(imu_ready == 0){
		while(OBSimu._IMUdata.imuTime < (*int_sol).time.sec){ //read whole imu date of a particular gnss epoch
			if(imu_ready == 0 && iniIMU.stepInitializeIMU(OBSimu, IMU_INI_TIME_END, _LLH_o) == 1){
				_dT = 1.0;
				// Initialize IMU Mechanization
				MechECEF.InitializeMechECEF(_ECEF_imu, _LLH_o, GNSSsol.velXYZ, iniIMU._RPY, iniIMU._ACCbias, iniIMU._GYRbias);

				imu_ready = 1; //Tells client.c that it can read imu data
				printf("end initialization\n");
			}
			read_imu();
			if(imu_ready){
				_epochIMU = OBSimu._IMUdata.imuTime;
			}
		}
		return;
	}
	//If imu is ready:

	//Initialize IMU mechanization with initial position, initial velocity and biases
	_ECEF_o = eigVector2std(double2eigVector((*int_sol).a, (*int_sol).b, (*int_sol).c));
	_LLH_o = ecef2geo(_ECEF_o);	
	_ECEF_imu = _ECEF_o;
	GNSSsol.velXYZ = eigVector2std(double2eigVector((*int_sol).va, (*int_sol).vb, (*int_sol).vc));
	MechECEF.InitializeMechECEF(_ECEF_imu, _LLH_o, GNSSsol.velXYZ, iniIMU._RPY, iniIMU._ACCbias, iniIMU._GYRbias);
	do {
		read_imu();

		// Process IMU
		SolutionIMU(OBSimu, MechECEF);				//Get position from current MechECEF state and IMU data just read - This has effects on: MechECEF e IMUsol

		//Update Time
		_epochIMU = OBSimu._IMUdata.imuTime;
	} while (_epochIMU <= (*int_sol).time.sec);

	//Return imu solution
	(*int_sol).a = IMUsol.posXYZ.at(0);
	(*int_sol).b = IMUsol.posXYZ.at(1);
	(*int_sol).c = IMUsol.posXYZ.at(2);

	//With standard deviation
	if(fst_pos){ //Variance Initialization
		MatrixXd pos(1, 3);
		MatrixXd vel(1, 3);

		pos << IMUsol.posXYZ.at(0), IMUsol.posXYZ.at(1), IMUsol.posXYZ.at(2);
		vel << IMUsol.velXYZ.at(0), IMUsol.velXYZ.at(1), IMUsol.velXYZ.at(2);

		MatrixXd tmp(1, 3);
		tmp(0, 0) = ecef2geo({pos(0, 0), pos(0, 1), pos(0, 2)}).at(0);
		tmp(0, 1) = ecef2geo({pos(0, 0), pos(0, 1), pos(0, 2)}).at(1);
		tmp(0, 2) = ecef2geo({pos(0, 0), pos(0, 1), pos(0, 2)}).at(2);
		pos = tmp;

		init_att_unc(0) = init_att_unc(0) * D2R;
		init_att_unc(1) = init_att_unc(1) * D2R;
		init_att_unc(2) = init_att_unc(2) * D2R;

		init_pos_unc(0) = init_pos_unc(0) * RE_WGS84;
		init_pos_unc(1) = init_pos_unc(1) * RE_WGS84;
		init_pos_unc(2) = init_pos_unc(2) * RE_WGS84;

		if (vel(0, 0) < 1e-4){
			vel(0, 0) = 1e-4;
		}
		double yaw = -atan2(vel(0, 0), vel(0, 1));
		//std::vector<double> att = {0, 0, yaw};
		MatrixXd att(1, 3);
		att << 0, 0, yaw;

		MatrixXd avp0(9, 1);
		avp0 << att(0, 0), att(0, 1), att(0, 2), vel(0, 0), vel(0, 1), vel(0, 2), pos(0, 0), pos(0, 1), pos(0, 2);
		MatrixXd acc = MatrixXd::Zero(3, 1);
		att2Cnb(att);
		//Initialize IMU error
		MatrixXd bg = MatrixXd::Zero(3, 1);
		MatrixXd ba = MatrixXd::Zero(3, 1);
		MatrixXd Kg = MatrixXd::Identity(3, 3);
		MatrixXd Ka = MatrixXd::Identity(3, 3);
		MatrixXd tauG(3, 1);
		tauG << std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity();
		MatrixXd tauA(3, 1);
		tauA << std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity();

		//Initialize earth related parameters
		Earth eth;
		earth_update(pos,vel, eth); 
		MatrixXd Mpv(3, 3);
		Mpv << 0, eth.Mpv4, 0, eth.Mpv2, 0, 0, 0, 0, 1;
		MatrixXd wib = MatrixXd::Zero(3, 1);
		MatrixXd fb = MatrixXd::Zero(3, 1);
		MatrixXd fn = -eth.gn; 					
		MatrixXd web = MatrixXd::Zero(3, 1);

		MatrixXd mx_init_bg_unc(1, 3);
		mx_init_bg_unc << init_bg_unc, init_bg_unc, init_bg_unc;

		MatrixXd mx_init_ba_unc(1, 3);
		mx_init_ba_unc << init_ba_unc, init_ba_unc, init_ba_unc;

		MatrixXd mx_pad_gyro(1, 3);
		mx_pad_gyro << psd_gyro, psd_gyro, psd_gyro;

		MatrixXd mx_pad_acce(1, 3);
		mx_pad_acce << psd_acce, psd_acce, psd_acce;

		MatrixXd mx_pad_bg(1, 3);
		mx_pad_bg << psd_bg, psd_bg, psd_bg;

		MatrixXd mx_pad_ba(1, 3);
		mx_pad_ba << psd_ba, psd_ba, psd_ba;

		x << att(0, 0), att(0, 1), att(0, 2), vel(0, 0), vel(0, 1), vel(0, 2), pos(0, 0), pos(0, 1), pos(0, 2), bg, ba;

		MatrixXd initP(1, 15);
		initP << init_att_unc, init_vel_unc, init_pos_unc, mx_init_bg_unc, mx_init_ba_unc;
		P = initP.array().square().matrix().asDiagonal();
		
		MatrixXd initQ(1, 15);
		initQ << mx_pad_gyro, mx_pad_acce, MatrixXd::Zero(1, 3), mx_pad_bg, mx_pad_ba;
		Q = initQ.array().matrix().asDiagonal() * nt;

		blh2xyz(pos);

		Dblh2Dxyz(pos);
		
		MatrixXd posvar(3, 3);
		MatrixXd initPosvar(1, 3);
		initPosvar << x(0, 6), x(0, 7), x(0, 8);
		posvar = initPosvar.array().matrix().asDiagonal(); //blh variance

		posvar = T * posvar * T.transpose(); //xyz variance

		//variance
		MatrixXd va(1, 3);
		va << posvar(0, 0), posvar(1, 1), posvar(2, 2);

		for(int i = 0; i < 6; i++){
			if(va(0, i) < 0)
				va(0, i) = -sqrt(fabs(va(0, i)));
			else
				va(0, i) = sqrt(va(0, i));
		}

		fst_pos = 0;
		//Debug print
		printf("%lf %lf %lf", va(0, 0), va(0, 1), va(0, 2));
	}else{

	}

	//Mark imu solution as usable for the comparison (in order to get the best solution for SEMOR)
	(*int_sol).time.week = 0;
}

void Loosely::init_imu(gnss_sol_t fst_pos){
	FileIO FIO;
	FIO.fileSafeIn("tokyo_imu.csv", fimu);

	OBSgnss.readEpoch(fst_pos);
	read_imu(); //fill OBSimu

	_epochIMU = OBSimu._IMUdata.imuTime; //TODO: read imu time
	_epochGNSS = fst_pos.time.sec;

	// Initial ECEF position for vehicle from GNSS     It puts the first position of the gnss file (first line) in _ECEF_o (and in _ECEF_imu e in GNSSsol.posXYZ)
	_ECEF_o = eigVector2std(double2eigVector(fst_pos.a, fst_pos.b, fst_pos.c));
	_ECEF_imu = _ECEF_o; GNSSsol.posXYZ = _ECEF_o;
	GNSSsol.velXYZ = eigVector2std(double2eigVector(fst_pos.va, fst_pos.vb, fst_pos.vc));

	_epochTime = _epochGNSS;

	// Initial Position in Geodetic and ENU
	_LLH_o = ecef2geo(_ECEF_o);	

	IMU_INI_TIME_END = _epochIMU+10; // Time taken to initialize the imu  (first imu epoch + 300) (for example)
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