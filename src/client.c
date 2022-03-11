#include "semor.h"
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <sys/poll.h>

#define DEBUG_FILE 1 //If semor takes data from file

#define FILE_PATH "output.txt"

#define GPS_FILE "test/gps.pos"
#define GALILEO_FILE "test/galileo.pos"

#define GPS 0
#define GALILEO 1
#define IMU 2

//Solutions
/*gnss_sol_t gps;
gnss_sol_t galileo;*/

gnss_sol_t sol[3]; /* GPS=0, GALILEO=1, IMU=2 */


//Integrated solutions
/*gnss_sol_t gps_imu;
gnss_sol_t galileo_imu;*/

FILE *file;
//int instance_no[2] = {0, 1};
//pthread_t id[3];
pid_t str2str_pid, rtkrcv1_pid, rtkrcv2_pid;

int isset_first_pos;
int imu_ready;
gnss_sol_t first_pos;

int wait_read[2] = {0, 0};
int last_week[2];
int seconds;

gnss_sol_t best_sol;
int best_idx;


pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;


//Close SEMOR and all processes started by it (rtkrcv and str2str)
void close_semor(int status){
    if(str2str_pid != -1 && kill(str2str_pid, SIGKILL) == -1){
        perror("SEMOR: Error killing str2str process");
    }
    if(rtkrcv1_pid != -1 && kill(rtkrcv1_pid, SIGKILL) == -1){
        perror("SEMOR: Error killing rtkrcv(1) process");
    }
    if(rtkrcv2_pid != -1 && kill(rtkrcv2_pid, SIGKILL) == -1){
        perror("SEMOR: Error killing rtkrcv(2) process");
    }
    close_ctocpp();
    fclose(file);
    printf("\nSEMOR terminated.\n");
    exit(status);
}

gnss_sol_t str2gnss(char str[MAXSTR]){
    gnss_sol_t gnss;
    char *eptr;
    char copy[MAXSTR];

    strcpy(copy, str);

    gnss.time.week = atoi(strtok(copy, " "));
    gnss.time.sec = atoi(strtok(NULL, " "));
    gnss.a = strtod(strtok(NULL, " "), &eptr);
    gnss.b = strtod(strtok(NULL, " "), &eptr);
    gnss.c = strtod(strtok(NULL, " "), &eptr);
    gnss.Q = atoi(strtok(NULL, " "));
    gnss.ns = atoi(strtok(NULL, " "));
    gnss.sda = strtod(strtok(NULL, " "), &eptr);
    gnss.sdb = strtod(strtok(NULL, " "), &eptr);
    gnss.sdc = strtod(strtok(NULL, " "), &eptr);
    gnss.sdab = strtod(strtok(NULL, " "), &eptr);
    gnss.sdbc = strtod(strtok(NULL, " "), &eptr);
    gnss.sdca = strtod(strtok(NULL, " "), &eptr);
    gnss.age = strtof(strtok(NULL, " "), &eptr);
    gnss.ratio = strtof(strtok(NULL, " "), &eptr);
    gnss.va = strtod(strtok(NULL, " "), &eptr);
    gnss.vb = strtod(strtok(NULL, " "), &eptr);
    gnss.vc = strtod(strtok(NULL, " "), &eptr);

    return gnss;
}

void gnss2str(char* str, gnss_sol_t gnss){
    /*sprintf(str, "%d %d %lf %lf %lf %d %d %lf %lf %lf %lf %lf %lf %f %f %lf %lf %lf", gnss.time.week,
    gnss.time.sec, gnss.a, gnss.b, gnss.c, gnss.Q, gnss.ns, 
    gnss.sda, gnss.sdb, gnss.sdc, gnss.sdab, gnss.sdbc, gnss.sdca,
    gnss.age, gnss.ratio, gnss.va, gnss.vb, gnss.vc);*/
    sprintf(str, "%d %lf %lf %lf", gnss.time.sec, gnss.a, gnss.b, gnss.c);
}

void gnsscopy(gnss_sol_t *dest, gnss_sol_t src){
    (*dest).time.week = src.time.week;
    (*dest).time.sec = src.time.sec;
    (*dest).a = src.a;
    (*dest).b = src.b;
    (*dest).c = src.c;
    (*dest).Q = src.Q;
    (*dest).ns = src.ns;
    (*dest).sda = src.sda;
    (*dest).sdb = src.sdb;
    (*dest).sdc = src.sdc;
    (*dest).sdab = src.sdab;
    (*dest).sdbc = src.sdbc;
    (*dest).sdca = src.sdca;
    (*dest).age = src.age;
    (*dest).ratio = src.ratio;
    (*dest).va = src.va;
    (*dest).vb = src.vb;
    (*dest).vc = src.vc;
}

void print_solutions(){
    char sol1[MAXSTR], sol2[MAXSTR], sol3[MAXSTR];
    gnss2str(sol1, sol[GPS]);
    gnss2str(sol2, sol[GALILEO]);
    gnss2str(sol3, sol[IMU]);
    fprintf(file, "%s ||| %s ||| %s\n", (sol[GPS].time.week == 0) ? "no GPS data" : sol1, (sol[GALILEO].time.week == 0) ? "no GALILEO data" : sol2, (sol[IMU].time.week == -1) ? "no IMU data" : sol3);
    fflush(file);
}

int read_gnss(int fd, int* offset, char buf[MAXSTR]){
    int nbytes = 0;

    //TODO: quando smetto di usare file e riprendo i socket, devo decommentare questa parte e commentare la lettura da file
    /*if ((nbytes = recv(fd, buf + *offset, (sizeof buf)-*offset, 0)) < 0) {
        if(errno != EAGAIN){
            perror("read");
            close_semor(1);
        }
    }
    *offset = *offset+nbytes;
    */

    do{
        nbytes = read(fd, buf+*offset, 1);
        if(nbytes == -1){
            perror("SEMOR read socket");
            close_semor(1);
        }else if(nbytes == 0){ // Rtkrcv1 is down
            printf("rtkrcv down\n"); //debug
            continue;
        }
        (*offset)++;
    }while(buf[(*offset)-1] != '\r' && buf[(*offset)-1] != '\n');

    return nbytes;
}

void process_solutions(int* check_sols){
    int chk_sols = *check_sols;
    int i;

    if(DEBUG_FILE){
        for(i = 0; i < 2; i++){
            if(sol[i].time.sec > seconds){
                if(!wait_read[i])
                    last_week[i] = sol[i].time.week;
                wait_read[i] = 1;
                sol[i].time.week = 0;
            }
            else{
                wait_read[i] = 0;
                if(last_week[i] != 0){
                    sol[i].time.week = last_week[i];
                    last_week[i] = 0;
                }
            }
        }
    }
    
    switch(chk_sols){ //Get best solution between GPS, GALILEO, IMU
        case 0: //only IMU
            if(sol[IMU].time.week == -1) 
                break;
            break;
        case 1: //only GPS
            break;
        case 2: // only GALILEO
            break;
        case 3:
            
            break;
    }
    if(!wait_read[GPS]) //DEBUG ________ RIMUOVERE!!!!!!!
        best_idx =  GPS;
    else
        best_idx = -1;

    //Here we have the best position
    print_solutions();
    //So let's generate the next imu position
    sol[IMU].time.week = -1;
    if(best_idx != -1){ //se c'è una soluzione migliore:
        gnsscopy(&sol[IMU], sol[best_idx]); //TODO: get best solution
        sol[IMU].time.sec += 1; //Get imu position of the next second
        imu_sol(&sol[IMU]); //this takes 1 second
    }

}
void check_termination(){
    char cmd;
    cmd = getchar();
    if(cmd == 'q' || cmd == 'Q'){
        close_semor(0);
    }
}

void handle_connection(){
    struct addrinfo hints, *res;
    int status;
    char buf[2][MAXSTR];
    int offset[2] = {0, 0};
    int nbytes;
    int socketfd[2];
    char port[5];
    int ret;
    int i;
    struct pollfd fds[3];
    int timeout_msecs = 500;
    int check_sols = 0; //3 if both rtk and ppp are read, 1 if only rtk read, 2 if only ppp and 0 if none
    int galileo_ready = 0;
    int new_gnss_data = 0;

    //Inizializzo soluzione imu
    sol[IMU].time.week = -1;
    //Inizializzazione socket

    socketfd[GPS] = open(GPS_FILE, O_RDONLY);
    socketfd[GALILEO] = open(GALILEO_FILE, O_RDONLY);

    fds[0].fd = socketfd[GPS];
    fds[1].fd = socketfd[GALILEO];
    fds[3].fd = STDIN_FILENO;
    fds[0].events = fds[1].events = fds[3].events = POLLIN;

    fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);

    while(1){
        check_sols = 0;
        ret = poll(fds, 2, timeout_msecs);
        if (ret == -1){
            perror("SEMOR: poll");
            close_semor(1);
        }
        //if(fds[3].revents & POLLIN){ //Get input from standard input
        check_termination();
       // }
       usleep(300000); //gives time to the solutions to be read
        for(i = 0; i < 2; i++){ //Get GPS and GALILEO solutions
            if(fds[i].revents & POLLIN){
                if(wait_read[i]){
                    continue;
                }
                read_gnss(socketfd[i], &offset[i], buf[i]);
                if(offset[i] != 0 && buf[i][offset[i]-1] == '\r' || buf[i][offset[i]-1] == '\n'){
                    buf[i][offset[i]-1] = '\0';
                    offset[i] = 0;
                    if(!(strstr(buf[i], "lat") || strstr(buf[i], "latitude") || strlen(buf[i]) < 5)){ //Continue if the buffer contains the header or empty line
                        check_sols |= i+1;
                    }
                    sol[i] = str2gnss(buf[i]);
                    if(i == GPS && isset_first_pos == 0){ //Initialize imu with GPS solution
                        isset_first_pos = 1;
                        init_imu(sol[i]);
                    }
                }
            }
        }

        if(DEBUG_FILE){
            if(seconds == 0){
                seconds = (sol[GPS].time.sec <= sol[GALILEO].time.sec) ? sol[GPS].time.sec : sol[GALILEO].time.sec;
            }
        }

        process_solutions(&check_sols);

        if(DEBUG_FILE)
            seconds++;
    }


}

void start_processing(void){
    int ret;

    file = fopen(FILE_PATH, "w");
    if(file == NULL){
        perror("SEMOR fopen()");
    }
    fflush(file);

    handle_connection();
}

