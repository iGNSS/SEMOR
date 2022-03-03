#ifndef SEMOR_H
#define SEMOR_H

#include <unistd.h>

#ifdef __cplusplus
#include "Loosely.h"
#endif

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

typedef struct gnss_solution{
    tow_t time;
    double x;
    double y;
    double z;
    short Q;
    short ns;
    double sdn;
    double sde;
    double sdu;
    double sdne;
    double sdeu;
    double sdun;
    float age;
    float ratio;

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
extern Loosely *looseGps;
extern Loosely *looseGalileo;
extern "C"{
#endif
void init_imu(gnss_sol_t fst_pos, int type);
void int_sol(gnss_sol_t *cur_gnss, int type); /* Called for each GNSS epoch*/
void close_ctocpp(void); //Delete loosely
#ifdef __cplusplus
}
#endif



#endif /* SEMOR_H */
