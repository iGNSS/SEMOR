/*
* Loosely.cpp
* Loosely Coupled Integration of GPS & IMU
*  Created on: Sept 10, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "Loosely.h"
using namespace std;
using namespace Eigen;

// PI
const double PI = 3.1415926535898;

char imu_data[IMU_DATA_LAST_N_SEC * IMU_HZ][75]; //for semor (constants from semor.h) - Last N seconds of imu data (imu send data at IMU_HZ Hz and it's a string of about 75 chars)

IMUmechECEF MechECEF; 

ifstream fimu;

InitializeIMU iniIMU;
double IMU_INI_TIME_END;

ReaderGNSS OBSgnss;
ReaderIMU OBSimu;

void read_imu(){
	string line;
	getline(fimu, line);
	if (line.find("GPS") != std::string::npos) {
		getline(fimu, line);
	}
	OBSimu.clearObs();
	OBSimu.obsEpoch(line);
}

// Function
bool Loosely::isValid(string observation_filepath) {
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

// A routine to organize GNSS solution from file
void Loosely::SolutionGNSS(ReaderGNSS GNSS) {
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

}

VectorXd double2eigVector(double a, double b, double c){
	VectorXd v = VectorXd::Zero(3);
	v(0) = a;
	v(1) = b;
	v(2) = c;
	return v;
}

void Loosely::get_imu_sol(gnss_sol_t* int_sol){
	//Check if imu is initializing
	if(imu_ready == 0){
		while(OBSimu._IMUdata.imuTime < (*int_sol).time.sec){ //read whole imu date of a particular gnss epoch
			if(imu_ready == 0 && iniIMU.stepInitializeIMU(OBSimu, IMU_INI_TIME_END, _LLH_o) == 1){
				// Initialize IMU Mechanization
				MechECEF.InitializeMechECEF(_ECEF_imu, _LLH_o, GNSSsol.velXYZ, iniIMU._RPY, iniIMU._ACCbias, iniIMU._GYRbias);
				read_imu();
				_epochIMU = OBSimu._IMUdata.imuTime;
				IMUsol.posXYZ = _ECEF_imu;

				imu_ready = 1; //Tells client.c that it can read imu data
			}
			read_imu();
		}
		return;
	}
	//If imu is ready:
	do {
		// Process IMU
		SolutionIMU(OBSimu, MechECEF);				//This has effects on: MechECEF e IMUsol

		read_imu();

		//Update Time
		_epochIMU = OBSimu._IMUdata.imuTime;
	} while (_epochIMU <= (*int_sol).time.sec); //_epochIMU <= _epochGNSS

	// *** Inegrated Solution (Loosely Coupled)
	//LooseCoupling(OBSgnss, MechECEF);				//This has effects on: MechECEF, on OBSgnss e on INTsol, it utilize (reads data from) GNSSsol

	// *** Read GPS 1Hz
	//OBSgnss.clearObs();
	//OBSgnss.readEpoch(*int_sol); //Set gnss position to generate next position with imu (in the next function call)

	//Return imu solution
	(*int_sol).a = IMUsol.posXYZ.at(0);
	(*int_sol).b = IMUsol.posXYZ.at(1);
	(*int_sol).c = IMUsol.posXYZ.at(2);
	// Compute timing information
	//_epochGNSS = OBSgnss._GNSSdata.gpsTime;
	// Process Epoch
	//SolutionGNSS(OBSgnss);							//This has effects on: _epochGNSS, GNSSsol
}

void Loosely::init_imu(gnss_sol_t fst_pos){
	fimu.open("tokyo_imu.csv");

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

	IMU_INI_TIME_END = _epochIMU+30; // Time taken to initialize the imu  (first imu epoch + 300) (for example)
}

Loosely::Loosely(){

}

// Facilitates the GNSS-IMU Loose integration process
Loosely::Loosely(gnss_sol_t fst_pos){
	/*//Inizializzazione IMU
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
		*/	
}


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