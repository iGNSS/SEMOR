#ifndef SEMOR_H
#define SEMOR_H

#include <unistd.h>

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

extern pid_t str2str_pid, rtkrcv1_pid, rtkrcv2_pid;

//extern char imu_data[IMU_DATA_LAST_N_SEC * IMU_HZ][75]; /* Last 2 seconds of imu data (imu send data at 100Hz and it's a string of about 75 chars)*/

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


//Functions
extern void start_processing(void);
extern void close_semor(int);


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
