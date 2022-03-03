#include "semor.h"

Loosely *looseGps;
Loosely *looseGalileo;

void int_sol(gnss_sol_t *cur_gnss, int type){
    if(type == 0){
        looseGps->get_int_sol(cur_gnss);
    }else{
        looseGalileo->get_int_sol(cur_gnss);
    }
}

void init_imu(gnss_sol_t fst_pos, int type){
    if(type == 0){
        looseGps = new Loosely();
        looseGps->init_imu(fst_pos);
    }else{
        looseGalileo = new Loosely();
        looseGalileo->init_imu(fst_pos);
    }
}

void close_ctocpp(void){
    delete looseGps;
    delete looseGalileo;
}