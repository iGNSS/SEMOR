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
VectorXd x(15);
MatrixXd P(15, 15);
MatrixXd Q(15, 15);
MatrixXd v;
MatrixXd R;
MatrixXd H;
//Dblh2Dxyz
Matrix3d T;
MatrixXd Dxyz;
//blh2xyz
Matrix3d Cen;
Vector3d xyz;

Matrix3d Cnb;

Matrix<double, 15, 1> xa;
Matrix<double, 15, 15> Pa;
Vector3d init_att_unc;
Vector3d init_vel_unc;
Vector3d init_pos_unc;
const double nt=1/sample_rate;
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
Vector3d initPosvar;
Vector3d va;
MatrixXd dv0(3, 1);
MatrixXd dw0(3, 1);
MatrixXd dv(3, 1);
MatrixXd dw(3, 1);
Vector3d pos_mid;
Vector3d vel_mid;
MatrixXd Q0(15, 15);
MatrixXd P0(15, 15);
Vector3d pos_new;
Vector3d vel_new;
MatrixXd old_dv(3, 1);
MatrixXd old_dw(3, 1);
MatrixXd dv_rot(3, 1);
MatrixXd dv_scul(3, 1);
Matrix3d dv_sf;
Vector3d dv_cor;
MatrixXd dw_cone(3, 1);
MatrixXd phi_b_ib(3, 1);
MatrixXd phi_n_in(3, 1);
Matrix3d Cbb;
Matrix3d Cnn;
Matrix3d Cnb_new;
Vector3d att_new;



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

void att2Cnb(Vector3d att){
	Vector3d sina; Vector3d cosa;
	sina << sin(att(0)), sin(att(1)), sin(att(2));
	cosa << cos(att(0)), cos(att(1)), cos(att(2));
	double sinp=sina(0);  double sinr=sina(1); double siny=sina(2);
	double cosp=cosa(0);  double cosr=cosa(1); double cosy=cosa(2);
				
	Cnb << cosr*cosy-sinp*sinr*siny, -cosp*siny,  sinr*cosy+sinp*cosr*siny, cosr*siny+sinp*sinr*cosy, cosp*cosy, sinr*siny-sinp*cosr*cosy, -cosp*sinr, sinp, cosp*cosr;;
}

/*Vector3d Cnb2att(Matrix3d Cnb){
	Vector3d att;
	att << asin(Cnb(2,1)), atan2(-Cnb(2,0),Cnb(2,2)), atan2(-Cnb(0,1),Cnb(1,1));
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
	Matrix<double, 15, 15> Ft;
	Ft << Faa       Fav       Fap      -Cnb            zero33
		Fva       Fvv       Fvp      zero33              Cnb
		zero33    Fpv       Fpp      zero33              zero33
		zero33    zero33    zero33   diag(-1./tauG)  zero33
		zero33    zero33    zero33   zero33              diag(-1./tauA);      

	// discretization
	if(nt>0.1)
		return (Ft*nt).exp(); 
	else
		return MatrixXd::Identity(size(Ft))+Ft*nt;
}*/

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


void blh2xyz(Vector3d blh){
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

		pos << IMUsol.posXYZ.at(0), IMUsol.posXYZ.at(1), IMUsol.posXYZ.at(2);
		vel << IMUsol.velXYZ.at(0), IMUsol.velXYZ.at(1), IMUsol.velXYZ.at(2);

		Vector3d tmp;
		std::vector<double> geostdpos = ecef2geo(eigVector2std(pos));
		tmp(0) = geostdpos.at(0);
		tmp(1) = geostdpos.at(1);
		tmp(2) = geostdpos.at(2);
		pos = tmp;

		//togliere start
		pos << -2408691.0460, 4698107.9065, 3566697.8630;
		//pos << 34.04173, 117.14394, 6365076123.9;

		geostdpos = ecef2geo(eigVector2std(pos));
		tmp(0) = geostdpos.at(0);
		tmp(1) = geostdpos.at(1);
		tmp(2) = geostdpos.at(2);
		pos = tmp;
		//togliere end

		//pos << 0.597256716722096, 2.044547372093737, 35.064853186719120;
		//vel << -1.723511926180124, -0.216003377465377, 0.024235396676032;
		vel << 1.46913, 0.91224, -0.16498;
		init_att_unc(0) = init_att_unc(0) * D2R;
		init_att_unc(1) = init_att_unc(1) * D2R;
		init_att_unc(2) = init_att_unc(2) * D2R;

		init_pos_unc(0) = init_pos_unc(0) / RE_WGS84;
		init_pos_unc(1) = init_pos_unc(1) / RE_WGS84;
		init_pos_unc(2) = init_pos_unc(2);



		if (vel(0) < 1e-4){
			vel(0) = 1e-4;
		}
		yaw = -atan2(vel(0), vel(1));

		att << 0, 0, yaw;

		avp0 << att, vel, pos;
		acc = Vector3d::Zero();
		att2Cnb(att);
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
		va << posvar(0, 0), posvar(1, 1), posvar(2, 2);
		//printf("BB: %lf\n", posvar(0, 0));

		for(int i = 0; i < 3; i++){
			if(va(i) < 0)
				va(i) = -sqrt(-va(i));
			else
				va(i) = sqrt(va(i));
		}

		fst_pos = 0;
		//Debug print
		//printf("AA: %lf %lf %lf\n", va(0), va(1), va(2));
		(*int_sol).sda = va(0);
		(*int_sol).sdb = va(1);
		(*int_sol).sdc = va(2);
	}else{
		/*Vector3d oldpos = pos;
		dv0 << -0.791625976562500, 0.177917480468750, 9.638977050781250;
		dw0 << -0.004286024305556, -0.024490017361111, 0.126399739583333;
		dw = Kg*dw0-bg*nt;
		dv = Ka*dv0-ba*nt;

		// extrapolate velocity and position
		vel_mid = vel+ acc*(nt/2);                    
		pos_mid = pos+ Mpv*(vel+vel_mid)/2*nt; 

		// update the earth related parameters
		earth_update(pos_mid,vel_mid);
		wib = dw/nt;
		fb  = dv/nt;
		fn  = Cnb*fb;
		web = wib-Cnb.transpose()*eth.wnie;

		// update velocity 
		dv_rot  = 0.5*dw0.cross(dv0);
		dv_scul = 1/12*(old_dw.cross(dv0)+old_dv.cross(dw0));
		dv_sf   = (Matrix3d::Identity()-0.5*nt*askew(eth.wnin))*Cnb*dv + Cnb*(dv_rot+dv_scul);
		dv_cor  = eth.gcc*nt;
		vel_new = vel+dv_sf+dv_cor;

		// update position 
		Mpv(0, 1) = eth.Mpv2;
		Mpv(1, 0) = eth.Mpv4;
		pos_new = pos + Mpv*(vel+vel_new)/2*nt;

		// update attitude 
		dw_cone  = 1/12*old_dw.cross(dw0);
		phi_b_ib = dw+dw_cone;
		phi_n_in = eth.wnin*nt;
		Matrix3d result;
		Cbb = rvec2mat(phi_b_ib).transpose();
		Cnn = rvec2mat(phi_n_in).transpose();
		Cnb_new = Cnn*Cnb*Cbb;
		att_new = Cnb2att(Cnb_new);

		// update INS result
		Cnb = Cnb_new;
		att = att_new;
		vel = vel_new;
		pos = pos_new;
		x << att, vel, pos, bg, ba;

		old_dw=dw0;
		old_dv=dv0;

		MatrixXd Phi = update_trans_mat(eth,vel,fn,tauA,tauG,nt,Cnb,Mpv);

		MatrixXd G = MatrixXd::Zero(15,15);
		//G(1:3,1:3)=-Cnb;
		for(int i=0; i < 3; i++){
			for(int j=0; j < 3; j++){
				G(i, j) = -Cnb(i, j);
			}
		}
		//G(4:6,4:6)= Cnb;
		for(int i=0; i < 3; i++){
			for(int j=0; j < 3; j++){
				G(i+3, j+3) = Cnb(i, j);
			}
		}
		Matrix3d id = Matrix3d::Identity();
		//G(10:12,10:12)=Matrix3d::Identity();
		for(int i=0; i < 3; i++){
			for(int j=0; j < 3; j++){
				G(i+9, j+9) = id(i, j);
			}
		}
		//G(13:15,13:15)=Matrix3d::Identity();
		for(int i=0; i < 3; i++){
			for(int j=0; j < 3; j++){
				G(i+12, j+12) = id(i, j);
			}
		}

		Q0=G*Q*G.transpose();
		P0=P+0.5*Q0;
		P=Phi*P0*Phi.transpose()+0.5*Q0;

		*/
	}

	//Mark imu solution as usable for the comparison (in order to get the best solution for SEMOR)
	(*int_sol).time.week = 0;
}

void Loosely::init_imu(gnss_sol_t fst_pos){
	FileIO FIO;
	FIO.fileSafeIn("test/tokyo_imu.csv", fimu);

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