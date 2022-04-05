#ifndef SEMOR_H
#define SEMOR_H

#include <unistd.h>
#include <stdint.h>
#include <limits.h>

/*
#ifdef __cplusplus
#include "Loosely.h"
#endif
*/

#define MAXSTR 256
#define IMU_PORT "8100"
#define PORT1 "8090" /* rtkrcv1 */
#define PORT2 "8091" /* rtkrcv2 */
#define IMU_DATA_LAST_N_SEC 2
#define IMU_HZ 100
#define LEAP_SECONDS 18

extern char root_path[PATH_MAX-200];

extern int logs;
extern int debug;
extern double init_bg_unc; //START
extern double init_ba_unc;
extern double psd_gyro;
extern double psd_acce;     // acce noise PSD (m^2/s^3)  
extern double psd_bg;     // gyro bias random walk PSD (rad^2/s^3)
extern double psd_ba;
extern int sample_rate; //END
extern int imu_init_epochs;//in seconds

extern pid_t str2str_pid, rtkrcv1_pid, rtkrcv2_pid;

//extern char imu_data[IMU_DATA_LAST_N_SEC * IMU_HZ][75]; /* Last 2 seconds of imu data (imu send data at 100Hz and it's a string of about 75 chars)*/

typedef struct {
    double dLon;
    double dLat;
    double dAcc;
    double dGyro;
    double dHeigth;
    double dSdn;
    double dSde;
    double dSdu;
    double dRobInd;
    uint8_t ui8FixQual;
    uint8_t ui8TS[50+1]; //+1 \0
}LocData_t;

extern LocData_t get_data();

typedef struct tow{ /* Time Of Week */
    int week;
    int sec;
}tow_t;

typedef struct gnss_solution{ /* a = x | n ,    b = y | e ,     c = z | u*/
    tow_t time;
    double a;
    double b;
    double c;
    short Q;
    short ns;
    double sda;
    double sdb;
    double sdc;
    double sdab;
    double sdbc;
    double sdca;
    float age;
    float ratio;
    double va;
    double vb;
    double vc;

    //int compared; //If it has no matches for its epoch, it will be printed alone with no comparisons
} gnss_sol_t;

//IMU
extern int isset_first_pos;
extern gnss_sol_t first_pos;
extern int imu_ready;
extern gnss_sol_t best;


//Functions
extern void start_processing(void);
extern void close_semor(int status);

extern int get_imu_data(char line[100]);


#ifdef __cplusplus
//extern Loosely *imu; //##CHANGED##
extern "C"{
#endif
void init_imu(gnss_sol_t fst_pos);
void imu_sol(gnss_sol_t* cur_gnss); /* Called for each GNSS epoch*/
void close_ctocpp(void); //Delete loosely
#ifdef __cplusplus
}
#endif



#endif /* SEMOR_H */
