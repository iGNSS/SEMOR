#include "semor.h"
#include "Loosely.h"
//##CHANGED##
Loosely imu;

void imu_sol(gnss_sol_t* cur_gnss){
    imu.get_imu_sol(cur_gnss);
}

void init_imu(gnss_sol_t fst_pos){
    //imu = Loosely();
    imu.init_imu(fst_pos);
}

void close_ctocpp(void){
    //delete imu;
    imu.close_out_file();
}
//##CHANGED##