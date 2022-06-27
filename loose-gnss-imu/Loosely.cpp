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

IMUmechECEF MechECEF; 

ifstream fimu;

InitializeIMU iniIMU;
double IMU_INI_TIME_END;

ReaderGNSS OBSgnss;
ReaderIMU OBSimu;



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
	}
	else{
		char buf[IMU_LENGTH];

		get_imu_data(buf);

		line = string(buf);
	}

	if(logs){
		out << line << endl;
		out.flush();
	}

	//cout << line << endl;
	//cout.flush();
	OBSimu.clearObs();
	OBSimu.obsEpoch(line);
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

int deb_enabled = 1;
void deb(char message[300]){
    if(deb_enabled){
        printf("%s\n", message);
    }
}

Loosely::Loosely(){}

void Loosely::get_imu_sol(gnss_sol_t* int_sol){
	//cout << (*int_sol).time.sec << endl;

	//Check if imu is initializing
	if(imu_ready == 0){
		int n = 0;
		while(/* n < 104 */ OBSimu._IMUdata.imuTime < (*int_sol).time.sec){ //read whole imu date of a particular gnss epoch
			if(imu_ready == 0 && iniIMU.stepInitializeIMU(OBSimu, IMU_INI_TIME_END, _LLH_o) == 1){ //it calculates _ACCbias and _GYRbias, _RPY
				_dT = 1.0;
				// Initialize IMU Mechanization
        
        /* HUGO 
        cout << iniIMU._RPY.at(0) << endl<< iniIMU._RPY.at(1) << endl<< iniIMU._RPY.at(2) << endl << endl;
        /* HUGO END */
        
				MechECEF.InitializeMechECEF(_ECEF_imu, _LLH_o, GNSSsol.velXYZ, iniIMU._RPY, iniIMU._ACCbias, iniIMU._GYRbias); //it initializes MechECEF with _ACCbias, _GYRbias and _RPY
        
        /* HUGO
        cout << iniIMU._ACCbias.at(0) << endl<< iniIMU._ACCbias.at(1) << endl<< iniIMU._ACCbias.at(2) << endl << endl;
        cout << iniIMU._GYRbias.at(0) << endl<< iniIMU._GYRbias.at(1) << endl<< iniIMU._GYRbias.at(2) << endl << endl;
        /* HUGO END */
        
				imu_ready = 1; //Tells client.c that it can read imu data
				printf("SEMOR: End initialization\n");
			}
			read_imu();
			if(imu_ready == 1){
				imu_ready = 2;
				(*int_sol).time.week = OBSimu._IMUdata.week;
			}
			_epochIMU = OBSimu._IMUdata.imuTime;
			n++;
		}
		imu_sec = (*int_sol).time.sec;
		/* cout << n << endl;
		cout.flush(); */
		return;
	}
	//IMU is ready:

	//Initialize IMU mechanization with initial position, initial velocity and biases
	/* _ECEF_o = eigVector2std(double2eigVector((*int_sol).a, (*int_sol).b, (*int_sol).c));
	_LLH_o = eigVector2std(ecef2geo(double2eigVector((*int_sol).a, (*int_sol).b, (*int_sol).c)));	
	_ECEF_imu = _ECEF_o;
	GNSSsol.velXYZ = eigVector2std(double2eigVector((*int_sol).va, (*int_sol).vb, (*int_sol).vc)); */


 
	MechECEF._pos.at(0) = (*int_sol).a;
	MechECEF._pos.at(1) = (*int_sol).b;
	MechECEF._pos.at(2) = (*int_sol).c;

	MechECEF._vel.at(0) = (*int_sol).va;
	MechECEF._vel.at(1) = (*int_sol).vb;
	MechECEF._vel.at(2) = (*int_sol).vc;
 
 
 
	/*
	MechECEF.InitializeMechECEF(_ECEF_imu, _LLH_o, GNSSsol.velXYZ, iniIMU._RPY, iniIMU._ACCbias, iniIMU._GYRbias);
	first = 0;
	*/
	//here we have previous gnss position



	//print_time();
	
	
	double next_time = _epochIMU;
	double group_time = 0.2;

	double avg_ax, avg_ay, avg_az;
	double avg_gx, avg_gy, avg_gz;
	int n = 0, nlocal = 0;

	//pre: ho già il primo dato imu, quindi non leggo subito
	do {
		avg_ax +=OBSimu._IMUdata.Ax;
		avg_ay +=OBSimu._IMUdata.Ay;
		avg_az +=OBSimu._IMUdata.Az;

		avg_gx +=OBSimu._IMUdata.Gx;
		avg_gy +=OBSimu._IMUdata.Gy;
		avg_gz +=OBSimu._IMUdata.Gz;

		n++;
		nlocal++;

		if(_epochIMU >= next_time){

			avg_ax /= nlocal;
			avg_ay /= nlocal;
			avg_az /= nlocal;

			avg_gx /= nlocal;
			avg_gy /= nlocal;
			avg_gz /= nlocal;

			OBSimu._IMUdata.Ax = avg_ax;
			OBSimu._IMUdata.Ay = avg_ay;
			OBSimu._IMUdata.Az = avg_az;

			OBSimu._IMUdata.Gx = avg_gx;
			OBSimu._IMUdata.Gy = avg_gy;
			OBSimu._IMUdata.Gz = avg_gz;

			SolutionIMU(OBSimu, MechECEF); 

			//Reset variables for next iterations
			next_time += group_time;
			avg_ax = avg_ay = avg_az = 0;
			avg_gx = avg_gy = avg_gz = 0;
			nlocal = 0;
		}                                                	
		
		read_imu();

		//Update Time
		_epochIMU = OBSimu._IMUdata.imuTime;
	} while (/* n <= 104 */ _epochIMU <= (*int_sol).time.sec);
	 /* if(logs){
		out << "---------------------------------------------------" << endl;
		out.flush();
	}  */
	cout << n <<  endl;
	cout.flush();

	(*int_sol).a = IMUsol.posXYZ.at(0);
	(*int_sol).b = IMUsol.posXYZ.at(1);
	(*int_sol).c = IMUsol.posXYZ.at(2);

	(*int_sol).va = IMUsol.velXYZ.at(0);
	(*int_sol).vb = IMUsol.velXYZ.at(1);
	(*int_sol).vc = IMUsol.velXYZ.at(2); 

	//

	//here we have next gnss+imu position

	//Mark imu solution as usable for the comparison (in order to get the best solution for SEMOR)
	(*int_sol).time.week = OBSimu._IMUdata.week;
	imu_sec = (*int_sol).time.sec;
	cout << "-----------------------" << endl;
	cout.flush();
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
}