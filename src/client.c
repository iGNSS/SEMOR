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

#define THRESHOLD 0.2

//Solutions

gnss_sol_t sol[3]; /* GPS=0, GALILEO=1, IMU=2 */
gnss_sol_t best;

FILE* sol_file[3];

FILE *file;

pid_t str2str_pid, rtkrcv1_pid, rtkrcv2_pid;

int isset_first_pos;
int imu_ready;
gnss_sol_t first_pos;

int wait_read[2] = {0, 0};
int last_week[2];
int seconds;


pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

LocData_t get_data(){ //SiConsulting
    LocData_t data;

    data.dLat = best.a;
    data.dLon = best.b;
    data.dHeigth = best.c;
    
    return data;
}



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
    if(log){
        fclose(sol_file[GPS]);
        fclose(sol_file[GALILEO]);
        fclose(sol_file[IMU]);
    }
    fclose(file);
    printf("\nSEMOR terminated.\n");
    exit(status);
}

/*LocData_t get_data(){

}*/

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
    fprintf(file, "%s ||| %s ||| %s\n", (sol[GPS].time.week == 0) ? "no GPS data" : sol1, (sol[GALILEO].time.week == 0) ? "no GALILEO data" : sol2, (sol[IMU].time.week == 0) ? "no IMU data" : sol3);
    fflush(file);
}

void print_solution(int sol_index){
    char sol1[MAXSTR];
    gnss2str(sol1, sol[sol_index]);
    fprintf(sol_file[sol_index], "%s\n", (sol[sol_index].time.week == 0) ? "no data" : sol1);
    fflush(sol_file[sol_index]);
}

int read_gnss(int fd, int* offset, char buf[MAXSTR]){
    int nbytes = 0;

    if(debug){
        do{
            nbytes = read(fd, buf+*offset, 1);
            if(nbytes == -1){
                perror("SEMOR read socket");
                close_semor(1);
            }else if(nbytes == 0){ // Rtkrcv1 is down
                printf("nothing to read\n"); //debug
                continue;
            }
            (*offset)++;
        }while(buf[(*offset)-1] != '\r' && buf[(*offset)-1] != '\n');
    }else{
        if ((nbytes = recv(fd, buf + *offset, (sizeof buf)-*offset, 0)) < 0) {
            if(errno != EAGAIN){
                perror("read");
                close_semor(1);
            }
        }
        *offset = *offset+nbytes;
    }

    return nbytes;
}

int similar_pos(gnss_sol_t p1, gnss_sol_t p2){
    int threshold;
    //Check a
    if(p1.a < p2.a){
        if(!(p1.a+3*p1.sda > p2.a-3*p2.sda && fabs(p1.a+3*p1.sda-p2.a-3*p2.sda) < THRESHOLD)){
            return 0;
        }
    }
    else{
        if(!(p2.a+3*p2.sda > p1.a-3*p1.sda && fabs(p2.a+3*p2.sda-p1.a-3*p1.sda) < THRESHOLD)){
            return 0;
        }
    }
    /*if(!(((p1.a+3*p1.sda > p2.a-3*p2.sda && p1.a-3*p1.sda < p2.a-3*p2.sda) || (p1.a-3*p1.sda < p2.a+3*p2.sda && p1.a-3*p1.sda > p2.a-3*p2.sda)) && fabs(p1.a+3*p1.sda - p2.a-3*p2.sda) < THRESHOLD || fabs(p2.a+3*p2.sda - p1.a-3*p1.sda))){
        return 0;
    }*/

    //Check b
    if(p1.b < p2.b){
        if(!(p1.b+3*p1.sdb > p2.b-3*p2.sdb && fabs(p1.b+3*p1.sdb-p2.b-3*p2.sdb) < THRESHOLD)){
            return 0;
        }
    }
    else{
        if(!(p2.b+3*p2.sdb > p1.b-3*p1.sdb && fabs(p2.b+3*p2.sdb-p1.b-3*p1.sdb) < THRESHOLD)){
            return 0;
        }
    }
    
    /*if(!(((3*p1.sdb > -3*p2.sdb && -3*p1.sdb < -3*p2.sdb) || (-3*p1.sdb < 3*p2.sdb && -3*p1.sdb > -3*p2.sdb)) && (fabs(3*p1.sdb - 3*p2.sdb) < THRESHOLD))){
        return 0;
    }*/
    //Check c
    if(p1.c < p2.c){
        if(!(p1.c+3*p1.sdc > p2.c-3*p2.sdc && fabs(p1.c+3*p1.sdc-p2.c-3*p2.sdc) < THRESHOLD)){
            return 0;
        }
    }
    else{
        if(!(p2.c+3*p2.sdc > p1.c-3*p1.sdc && fabs(p2.c+3*p2.sdc-p1.c-3*p1.sdc) < THRESHOLD)){
            return 0;
        }
    }
    /*if(!(((3*p1.sdc > -3*p2.sdc && -3*p1.sdc < -3*p2.sdc) || (-3*p1.sdc < 3*p2.sdc && -3*p1.sdc > -3*p2.sdc)) && fabs(3*p1.sdc - 3*p2.sdc) < THRESHOLD)){
        return 0;
    }*/

    return 1;
}

void gnss_avg_2(gnss_sol_t sol1, gnss_sol_t sol2){
    best.a = (sol1.a + sol2.a)/2;
    best.b = (sol1.b + sol2.b)/2;
    best.c = (sol1.c + sol2.c)/2;
}

void gnss_avg_3(gnss_sol_t sol1, gnss_sol_t sol2, gnss_sol_t sol3){
    best.a = (sol1.a + sol2.a + sol3.a)/3;
    best.b = (sol1.b + sol2.b + sol3.b)/3;
    best.c = (sol1.c + sol2.c + sol3.c)/3;
}

int get_best_sol2(int sol1_idx, int sol2_idx){ //if 2 gnss solutions available - 0: no best found, 1: best found
    if(similar_pos(sol[sol1_idx], sol[sol2_idx])){
        gnss_avg_2(sol[sol1_idx], sol[sol2_idx]);
        return 1;
    }
    return 0;
}

int get_best_sol_3(){ //if 3 solutions available - 0: no best found, 1: best found
    if(similar_pos(sol[GPS], sol[GALILEO])){
        if(similar_pos(sol[GPS], sol[IMU])){
            gnss_avg_3(sol[GPS], sol[GALILEO], sol[IMU]);
            return 1;
        }
        //Solo le due soluzioni GNSS sono simili
        gnss_avg_2(sol[GPS], sol[GALILEO]);
        return 1;
    }

    if(similar_pos(sol[GPS], sol[IMU])){
        gnss_avg_2(sol[GPS], sol[IMU]);
        return 1;
    }

    if(similar_pos(sol[GALILEO], sol[IMU])){
        gnss_avg_2(sol[GALILEO], sol[IMU]);
        return 1;
    }
    return 0;
}

void process_solutions(int* check_sols){
    int chk_sols = *check_sols;
    int i;
    int best_found = 1;
    int best_idx = -1; //0: gps, 1:galileo, 2:imu, 3:best

    if(debug){
        for(i = 0; i < 2; i++){ //for each gnss solution check if its epoch is higher than the "seconds" variable, if so wait next iterations 
        //before displaying the solution till epochs are equal
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
    if(sol[IMU].time.week != -1) 
        chk_sols |= 4;
    switch(chk_sols){ //Get best solution between GPS, GALILEO, IMU
        case 0: //no solutions, return
            return;
        case 1: //only GPS
            best_idx = GPS;
            break;
        case 2: // only GALILEO
            best_idx = GALILEO;
            break;
        case 3: //GPS and GALILEO
            best_idx = 3;
            best_found = get_best_sol2(GPS, GALILEO);
            break;
        case 4: //only IMU
            best_idx = IMU;
            break;
        case 5: //GPS and IMU
            best_idx = 3;
            best_found = get_best_sol2(GPS, IMU);
            break;
        case 6: //GALILEO and IMU
            best_idx = 3;
            best_found = get_best_sol2(GALILEO, IMU);
            break;
        case 7: //all solutions available
            best_idx = 3;
            best_found = get_best_sol_3();
            break;
    }
    if(!wait_read[GPS]) //DEBUG ________ RIMUOVERE!!!!!!!
        best_idx =  GPS;
    else
        best_idx = -1;

    //Here we have the best position
    if(logs){
        print_solution(GPS);
        print_solution(GALILEO);
        print_solution(IMU);
    }

    print_solutions();
    //So let's generate the next imu position
    if(best_idx != -1){ //se c'è una soluzione migliore:
        gnsscopy(&sol[IMU], sol[best_idx]); //TODO: get best solution
        sol[IMU].time.week = 0;
        sol[IMU].time.sec += 1; //Get imu position of the next second
        imu_sol(&sol[IMU]); //this takes 1 second
    }
    else{
        sol[IMU].time.week = 0; //if no best solutions, mark the imu solution as already used
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
    int timeout_msecs = 1000;
    int check_sols = 0; //3 if both rtk and ppp are read, 1 if only rtk read, 2 if only ppp and 0 if none
    int galileo_ready = 0;
    int new_gnss_data = 0;

    //Inizializzo soluzione imu
    sol[IMU].time.week = 0;
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
        ret = poll(fds, 2, timeout_msecs); //wait for events on the 3 fds
        if (ret == -1){
            perror("SEMOR: poll");
            close_semor(1);
        }
        //if(fds[3].revents & POLLIN){
        check_termination(); //Check if user requested the termination of the process
       // }
        for(i = 0; i < 2; i++){ //Get GPS and GALILEO solutions
        usleep(150000); //gives time to the solutions to be read (if one of them is late)
            if(fds[i].revents & POLLIN){
                if(debug && wait_read[i]){ //Don't read solution i (0:GPS, 1:GALILEO) if the solution in the previous iterations has a higher epoch than "seconds" variable
                    continue;
                }
                read_gnss(socketfd[i], &offset[i], buf[i]);
                if(offset[i] != 0 && buf[i][offset[i]-1] == '\r' || buf[i][offset[i]-1] == '\n'){ //If incoming data is a full gnss measurement string
                    buf[i][offset[i]-1] = '\0'; //Replace new line with end of string
                    offset[i] = 0; //Reset offset for next reads
                    if(!(strstr(buf[i], "lat") || strstr(buf[i], "latitude") || strlen(buf[i]) < 5)){ //Check if the input string contains gnss data or if it is an empty line or header
                        check_sols |= i+1;
                    }
                    sol[i] = str2gnss(buf[i]);//Parse string to gnss_sol_t structure
                    if(i == GPS && !isset_first_pos){ //Initialize imu with rtk solution (since it is more accurate)
                        isset_first_pos = 1;
                        init_imu(sol[i]);
                    }
                }
            }
        }

        if(debug){
            if(seconds == 0){
                seconds = (sol[GPS].time.sec <= sol[GALILEO].time.sec) ? sol[GPS].time.sec : sol[GALILEO].time.sec; //Set current second to the minimum of the epochs of the 2 gnss solution
            }
        }

        process_solutions(&check_sols); //Get best solution, output it and use it to calculate next imu position

        if(debug)
            seconds++; //Update current second
    }


}

void start_processing(void){
    int ret;
    int i;
    char path[PATH_MAX];

    if(logs){
        sprintf(path, "%sgps.log", root_path);
        sol_file[GPS] = fopen(path, "w");

        sprintf(path, "%sgalileo.log", root_path);
        sol_file[GALILEO] = fopen(path, "w");

        sprintf(path, "%simu.log", root_path);
        sol_file[IMU] = fopen(path, "w");
    }

    file = fopen(FILE_PATH, "w");
    if(file == NULL){
        perror("SEMOR fopen()");
    }
    fflush(file);

    handle_connection();
}

