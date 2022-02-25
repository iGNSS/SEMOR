#ifndef SEMOR_H
#define SEMOR_H

#include <unistd.h>

#define MAXSTR 1024

extern pid_t str2str_pid, rtkrcv1_pid, rtkrcv2_pid;

typedef struct tow{ /* Time Of Week */
    int week;
    int sec;
}tow_t;

typedef struct gnss_solution{
    tow_t time;
    double lat;
    double lng;
    double height;
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


/*typedef struct ins_solution{

}ins_sol_t;*/


//Functions
extern void start_processing(void);
extern void close_semor(int);



#endif /* SEMOR_H */
