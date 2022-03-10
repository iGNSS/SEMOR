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
FILE *p;
//int instance_no[2] = {0, 1};
//pthread_t id[3];
pid_t str2str_pid, rtkrcv1_pid, rtkrcv2_pid;

int isset_first_pos;
int imu_ready;
gnss_sol_t first_pos;


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
    if(p != NULL)
        fclose(p);
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

/*void process_gnss_data(char buf[MAXSTR], int ins){
    char sol1[MAXSTR], sol2[MAXSTR], sol3[MAXSTR];
    gnss_sol_t intsol;

    fprintf(p, "(%d) %s\n", ins, buf); //debug
    fflush(p);
    pthread_mutex_lock(&mutex);
    //##CHANGED##
    if(ins == 0){ //Se questo handler gestisce la prima istanza di rtkrcv
        if(gps.time.week != 0){ //La soluzione gps precedente non è stata consumata, la invio da sola prima di quella corrente
            gnss2str(sol1, gps);
            gnss2str(sol3, gps_imu);
            fprintf(file, "ALONE %s ||| %s\n", sol1, (gps_imu.time.week == -1) ? "IMU initializing" : sol3);
            gps.time.week = 0;
        }
        gps = str2gnss(buf);
        if(isset_first_pos == 0){
            isset_first_pos = 1;
            first_pos = gps;
            init_imu(first_pos);
        }
        if(galileo_imu.time.sec != gps.time.sec){ //prendo soluzione imu se non è già stata presa da galileo
            gnsscopy(&gps_imu, gps);
            gps_imu.time.week = -1;
            imu_sol(&gps_imu);
        }
        if(galileo.time.week != 0 && gps.time.sec == galileo.time.sec){ //E' arrivata prima la soluzione galileo, quindi posso già farei controlli
            //Add line with both solution side by side to the file
            gnss2str(sol1, gps);
            gnss2str(sol2, galileo);
            gnss2str(sol3, gps_imu.time.week == 0 ? gps_imu : galileo_imu);
            fprintf(file, "%s ||| %s ||| %s\n", sol1, sol2, (gps_imu.time.week & galileo_imu.time.week != 0) ? "IMU initializing" : sol3);
            //Dico alle prossime iterazioni che i precedenti valori sono già stati consumati
            gps.time.week = 0;
            galileo.time.week = 0;
        }
    }else{ //Se questo handler gestisce la seconda istanza di rtkrcv
        if(galileo.time.week != 0){ //La soluzione galileo precedente non è stata consumata, la invio da sola prima di quella corrente
            gnss2str(sol2, galileo);
            gnss2str(sol3, galileo_imu);
            fprintf(file, "ALONE %s ||| %s\n", sol2, (galileo_imu.time.week == -1) ? "IMU initializing" : sol3);
            galileo.time.week = 0;
        }
        galileo = str2gnss(buf);
        if(isset_first_pos == 0){ //forse meglio di no
            isset_first_pos = 1;
            first_pos = galileo;
            init_imu(first_pos);
        }
        if(gps_imu.time.sec != galileo.time.sec){ //prendo soluzione imu se non è già stata presa da gps
            gnsscopy(&galileo_imu, galileo);
            galileo_imu.time.week = -1;
            imu_sol(&galileo_imu);
        }
        if(gps.time.week != 0 && gps.time.sec == galileo.time.sec){ //E' arrivata prima la soluzione gps, quindi posso già farei controlli
            //Add line with both solution side by side to the file
            gnss2str(sol1, gps);
            gnss2str(sol2, galileo);
            gnss2str(sol3, gps_imu.time.week == 0 ? gps_imu : galileo_imu);
            fprintf(file, "%s ||| %s ||| %s\n", sol1, sol2, (gps_imu.time.week & galileo_imu.time.week != 0) ? "IMU initializing" : sol3);
            gps.time.week = 0;
            galileo.time.week = 0;
        }
    }
    //##CHANGED##
    fflush(file);
    pthread_mutex_unlock(&mutex);
}*/

void print_solutions(){
    char sol1[MAXSTR], sol2[MAXSTR], sol3[MAXSTR];
    gnss2str(sol1, sol[GPS]);
    gnss2str(sol2, sol[GALILEO]);
    gnss2str(sol3, sol[IMU]);
    fprintf(file, "%s ||| %s ||| %s\n", sol1, (sol[GALILEO].time.sec == 0) ? "GALILEO initializing" : sol2, (sol[IMU].time.week == -1) ? "IMU initializing" : sol3);
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

int get_best_sol(){ //Checks
    return GPS;
}

void handle_connection(){
    struct addrinfo hints, *res;
    int status;
    char buf[2][MAXSTR];
    int offset[2] = {0, 0};
    int nbytes;
    int socketfd[2]; //not socket but file for testing ##CHANGED##
    char port[5];
    int ret;
    int i;
    struct pollfd fds[3];
    int timeout_msecs = 500;
    char cmd;
    gnss_sol_t best_sol;
    int best_idx;
    int no_go = 0; //If got header or GNSS data incomplete
    int galileo_ready = 0;
    //##CHANGED##

    //Inizializzo soluzione imu
    sol[IMU].time.week = -1;
    //Inizializzazione socket

    socketfd[GPS] = open(GPS_FILE, O_RDONLY);
    socketfd[GALILEO] = open(GALILEO_FILE, O_RDONLY);

    fds[0].fd = socketfd[GPS];
    fds[1].fd = socketfd[GALILEO];
    fds[3].fd = STDIN_FILENO;
    fds[0].events = fds[1].events = fds[3].events = POLLIN;
    //Inizializzazione socket
    //##CHANGED##

    fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);

    while(1){
        ret = poll(fds, 2, timeout_msecs);
        if (ret == -1){
            perror("poll");
            close_semor(1);
        }
        if(fds[3].revents & POLLIN){ //Get input from standard input
            cmd = getchar();
            if(cmd == 'q' || cmd == 'Q'){
                close_semor(0);
            }
        }
        for(i = 0; i < 2; i++){ //Get GPS and GALILEO solutions
            if(fds[i].revents & POLLIN){
                read_gnss(socketfd[i], &offset[i], buf[i]);
                if(offset[i] != 0 && buf[i][offset[i]-1] == '\r' || buf[i][offset[i]-1] == '\n'){
                    buf[i][offset[i]-1] = '\0';
                    offset[i] = 0;
                    if(strstr(buf[i], "lat") || strstr(buf[i], "latitude") || strlen(buf[i]) < 5){ //Continue if the buffer contains the header
                        no_go |= i+1;
                        continue;
                    }
                    sol[i] = str2gnss(buf[i]);
                    if(i == GPS && isset_first_pos == 0){ //Initialize imu with GPS solution
                        isset_first_pos = 1;
                        init_imu(sol[i]);
                    }
                    if(i == GALILEO){
                        galileo_ready = 1;
                    }
                }
                else{
                    no_go |= i+1;
                }
            }
        }
        if(no_go == 3){
            no_go = 0;
            continue;
        }
        if(no_go != 0 && galileo_ready == 1){ //One of the two gnss solutions isn't ready
            continue;
        }
        if(sol[IMU].time.sec == 0){ //IMU is initializing
            print_solutions();
            //TODO: Return GPS data

            gnsscopy(&sol[IMU], sol[GPS]); //Irrelevant to use GPS or GALILEO, we only need a copy of the time
            sol[IMU].time.week = -1;
            sol[IMU].time.sec += 1; //Get imu position of the next second
            imu_sol(&sol[IMU]);
        }
        else{//Get best solution between GPS, GALILEO, IMU
            if(sol[GPS].time.sec == sol[GALILEO].time.sec){ //Se le epoche GPS e GALILEO coincidono
                best_idx = get_best_sol(&best_sol);
                print_solutions();
                //TODO: Return best sol

                //Process imu for next epoch
                gnsscopy(&sol[IMU], sol[best_idx]); //Irrelevant to use GPS or GALILEO, we only need a copy of the time
                sol[IMU].time.week = -1;
                sol[IMU].time.sec += 1; //Get imu position of the next second
                imu_sol(&sol[IMU]); //this takes 1 second
            }
            else{ //Altrimenti prendo quella più recente e decido se dare in output quella posizione o quella dell'imu
                
            }
        }

        //Simulo 1Hz
        sleep(1);
    }


}

void start_processing(void){
    int ret;
    //int instance_no[2];
    p = fopen("prova.txt", "w");

    file = fopen(FILE_PATH, "w");
    if(file == NULL){
        perror("SEMOR fopen()");
    }
    fflush(file);

    handle_connection();
}

