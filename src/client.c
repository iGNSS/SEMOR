#include "semor.h"
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

#define THRESHOLD 20000000

//Solutions

//char rtk_port_rtkrcv[6];
//char ppp_port_rtkrcv[6];

gnss_sol_t sol[3]; /* GPS=0, GALILEO=1, IMU=2 */
gnss_sol_t best;

gnss_sol_t initial_pos; //initial position for imu initialization

FILE* sol_file[3];

FILE *file;

pid_t str2str_pid, rtkrcv1_pid, rtkrcv2_pid;

int isset_first_pos;
int imu_ready;
gnss_sol_t first_pos;

int wait_read[2];
int last_week[2];
int seconds;
int n_imu; //how many times the imu solution has been used consecutively

int first_time = 1;

LocData_t get_data(){ //SiConsulting
    LocData_t data;

    data.dLat = best.a;
    data.dLon = best.b;
    data.dHeigth = best.c;
    
    return data;
}



//Close SEMOR and all processes started by it (rtkrcv and str2str)
void close_semor(int status){
    printf("\nSEMOR terminated.\n");
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
    if(logs){
        fclose(sol_file[GPS]);
        fclose(sol_file[GALILEO]);
        fclose(sol_file[IMU]);
    }
    fclose(file);
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
    sprintf(str, "sec: %d, pos: %lf %lf %lf, std: %lf %lf %lf", gnss.time.sec, gnss.a, gnss.b, gnss.c, gnss.sda, gnss.sdb, gnss.sdc);
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

void output(gnss_sol_t sol){
    char sol1[MAXSTR];
    gnss2str(sol1, sol);
    fprintf(file, "%s\n", (sol.time.week == 0) ? "no data" : sol1);
    fflush(file);
}

void print_solution(int sol_index){
    char sol1[MAXSTR];
    gnss2str(sol1, sol[sol_index]);
    fprintf(sol_file[sol_index], "%s\n", (sol[sol_index].time.week == 0) ? "no data" : sol1);
    fflush(sol_file[sol_index]);
}

int similar_pos(gnss_sol_t p1, gnss_sol_t p2){

    //Check a
    if(p1.a < p2.a){
        if(p1.a+3*p1.sda < p2.a-3*p2.sda){
            return 0;
        }
    }
    else{
        if(p2.a+3*p2.sda < p1.a-3*p1.sda){
            return 0;
        }
    }

    //Check b
    if(p1.b < p2.b){
        if(p1.b+3*p1.sdb < p2.b-3*p2.sdb){
            return 0;
        }
    }
    else{
        if(p2.b+3*p2.sdb < p1.b-3*p1.sdb){
            return 0;
        }
    }

    //Check c
    if(p1.c < p2.c){
        if(p1.c+3*p1.sdc < p2.c-3*p2.sdc){
            return 0;
        }
    }
    else{
        if(p2.c+3*p2.sdc < p1.c-3*p1.sdc){
            return 0;
        }
    }

    return 1;
}

void gnss_avg_2(gnss_sol_t sol1, gnss_sol_t sol2){
    //Position
    best.a = (sol1.a + sol2.a)/2;
    best.b = (sol1.b + sol2.b)/2;
    best.c = (sol1.c + sol2.c)/2;

    //Velocity
    best.va = (sol1.va + sol2.va)/2;
    best.vb = (sol1.vb + sol2.vb)/2;
    best.vc = (sol1.vc + sol2.vc)/2;

    //Standard deviation
    best.sda = sqrt(pow(sol1.sda/2, 2) + pow(sol2.sda/2, 2));
    best.sdb = sqrt(pow(sol1.sdb/2, 2) + pow(sol2.sdb/2, 2));
    best.sdc = sqrt(pow(sol1.sdc/2, 2) + pow(sol2.sdc/2, 2));

    best.sdab = sqrt(pow(sol1.sdab/2, 2) + pow(sol2.sdab/2, 2));
    best.sdbc = sqrt(pow(sol1.sdbc/2, 2) + pow(sol2.sdbc/2, 2));
    best.sdca = sqrt(pow(sol1.sdca/2, 2) + pow(sol2.sdca/2, 2));
}

void gnss_avg_3(gnss_sol_t sol1, gnss_sol_t sol2, gnss_sol_t sol3){
    //Position
    best.a = (sol1.a + sol2.a + sol3.a)/3;
    best.b = (sol1.b + sol2.b + sol3.b)/3;
    best.c = (sol1.c + sol2.c + sol3.c)/3;

    //Velocity
    best.va = (sol1.va + sol2.va + sol3.va)/3;
    best.vb = (sol1.vb + sol2.vb + sol3.vb)/3;
    best.vc = (sol1.vc + sol2.vc + sol3.vc)/3;

    //Standard deviation
    best.sda = sqrt(pow(sol1.sda/3, 2) + pow(sol2.sda/3, 2) + pow(sol3.sda/3, 2));
    best.sdb = sqrt(pow(sol1.sdb/3, 2) + pow(sol2.sdb/3, 2) + pow(sol3.sdb/3, 2));
    best.sdc = sqrt(pow(sol1.sdc/3, 2) + pow(sol2.sdc/3, 2) + pow(sol3.sdc/3, 2));

    best.sdab = sqrt(pow(sol1.sdab/3, 2) + pow(sol2.sdab/3, 2) + pow(sol3.sdab/3, 2));
    best.sdbc = sqrt(pow(sol1.sdbc/3, 2) + pow(sol2.sdbc/3, 2) + pow(sol3.sdbc/3, 2));
    best.sdca = sqrt(pow(sol1.sdca/3, 2) + pow(sol2.sdca/3, 2) + pow(sol3.sdca/3, 2));
}

int get_best_sol2(int sol1_idx, int sol2_idx){ //if 2 gnss solutions available - 0: no best found, 1: best found
    if(similar_pos(sol[sol1_idx], sol[sol2_idx])){
        gnss_avg_2(sol[sol1_idx], sol[sol2_idx]);
        return 1;
    }
    return 0;
}

int get_best_sol_3(){ //if 3 solutions available - 0: no best found, 1: best found (if no best found -> use IMU)
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

void process_solutions(int chk_sols){
    int i;
    int is_best_found = 1;

    if(debug){
        for(i = 0; i < 2; i++){ //for each gnss solution check if its epoch is higher than the "seconds" variable, if so wait next iterations 
        //before displaying the solution till epochs are equal
            if(sol[i].time.sec > seconds){
                if(!wait_read[i])
                    last_week[i] = sol[i].time.week;
                wait_read[i] = 1;
                sol[i].time.week = 0;

                chk_sols -= pow(2, i); //this position belongs to a next epoch, so it won't be utilized
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
    if(!imu_ready){
        gnsscopy(&sol[IMU], sol[GPS]);
        imu_sol(&sol[IMU]); //this takes 1 second
        sol[IMU].time.sec++;
        return;
    }

    if(sol[IMU].time.week != 0) 
        chk_sols |= 4;
    switch(chk_sols){ //Get best solution between GPS, GALILEO, IMU
        case 0: //no solutions, return
            return;
        case 1: //only GPS
            gnsscopy(&best, sol[GPS]);
            is_best_found = 1;
            break;
        case 2: // only GALILEO
            gnsscopy(&best, sol[GALILEO]);
            is_best_found = 1;
            break;
        case 3: //GPS and GALILEO
            is_best_found = get_best_sol2(GPS, GALILEO);
            best.time.week = sol[GPS].time.week;
            best.time.sec = sol[GPS].time.sec;
            break;
        case 4: //only IMU
            gnsscopy(&best, sol[IMU]);
            is_best_found = 1;
            break;
        case 5: //GPS and IMU
            is_best_found = get_best_sol2(GPS, IMU);
            best.time.week = sol[GPS].time.week;
            best.time.sec = sol[GPS].time.sec;
            break;
        case 6: //GALILEO and IMU
            is_best_found = get_best_sol2(GALILEO, IMU);
            best.time.week = sol[GALILEO].time.week;
            best.time.sec = sol[GALILEO].time.sec;
            break;
        case 7: //all solutions available
            is_best_found = get_best_sol_3();
            best.time.week = sol[GPS].time.week;
            best.time.sec = sol[GPS].time.sec;
            break;
    }

    

    if(logs && imu_ready){
        print_solution(GPS);
        print_solution(GALILEO);
        print_solution(IMU);
    }

    if(is_best_found == 0 || chk_sols == 4){ //if is_best_found == 0 (IMU solution is used)
        if(n_imu == imu_drift){
            //REINITIALIZE SEMOR
            printf("Re-initialize SEMOR\n");
            close_semor(1);
        }
        else
            n_imu++;
    }
    else{
        n_imu = 0;
    }

    //Output
    if(!is_best_found){
        gnsscopy(&best, sol[IMU]);
    }

    //Here we have the solution
    output(best);

    if(!is_best_found && chk_sols < 4){ //no best solution and no imu solution
        //TODO
    }

    //Flag solutions as already used
    sol[GPS].time.week = 0;
    sol[GALILEO].time.week = 0;

    //Post comparison and output
    //So let's generate the next imu position
    gnsscopy(&sol[IMU], best);
    sol[IMU].time.week = 0;
    sol[IMU].time.sec += 1; //Get imu position of the next second
    imu_sol(&sol[IMU]); //this runs until next second

}
void check_termination(){
    char cmd;
    cmd = getchar();
    if(cmd == 'q' || cmd == 'Q'){
        close_semor(0);
    }
}

void setup_tcp_socket(int* fd, char port[6]){
    int status;
    struct addrinfo hints, *res;

    memset(&hints, 0, sizeof hints);
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    hints.ai_family = AF_INET; 

    
    if ((status = getaddrinfo(NULL, port, &hints, &res)) != 0) {
        fprintf(stderr, "SEMOR: getaddrinfo: %s\n", gai_strerror(status));
        close_semor(1);
    }
    *fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    
    if(*fd == -1){
        perror("SEMOR socket()");
        close_semor(1);
    }

    int flags = fcntl(*fd, F_GETFL, 0);

    if(fcntl(*fd, F_SETFL, flags | O_NONBLOCK) == -1){
        perror("SEMOR: can't set non blocking socket()");
        close_semor(1);
    }

    do{
        connect(*fd, res->ai_addr, res->ai_addrlen);
        if(errno != EISCONN){ //(errno == ECONNREFUSED || errno == EAGAIN || errno == EINPROGRESS || errno == EALREADY)
            continue; //Wait for rtkrcv to start and to send data
        }
        break; //Stop while
    }while(1);

    free(res);
}

int read_gnss(int fd, int* offset, char* buf, int buf_length){
    int nbytes = 0;

    if(debug){
        do{
            nbytes = read(fd, buf+*offset, 1);
            if(nbytes == -1){
                perror("SEMOR read file");
                close_semor(1);
            }else if(nbytes == 0){ // Nothing to read
                continue;
            }
            (*offset)++;
        }while(buf[(*offset)-1] != '\r' && buf[(*offset)-1] != '\n');
    }else{
        if ((nbytes = recv(fd, buf + *offset, buf_length - *offset, 0)) < 0) { //(sizeof buf) /*strlen(buf)*/-*offset
            if(errno != EAGAIN && errno != EWOULDBLOCK && errno != ECONNREFUSED){
                perror("SEMOR: read");
                close_semor(1);
            }
        }
    *offset = *offset+nbytes;
    }

    return nbytes;
}

void handle_connection(){
    struct addrinfo hints, *res;
    int status;
    char buf[2][MAXSTR];
    int offset[2] = {0, 0};
    int nbytes;
    int socketfd[2];
    char port[6];
    int ret;
    int i;
    struct pollfd fds[3];
    int timeout_msecs = 1000;
    int check_sols = 0; //3 if both rtk and ppp are read, 1 if only rtk read, 2 if only ppp and 0 if none
    int galileo_ready = 0;
    int new_gnss_data = 0;
    int first_input = 0;

    printf("SEMOR: Start initialization. Please wait...\n");
    
    //Initialize input file descriptors
    if(debug){
        socketfd[GPS] = open(GPS_FILE, O_RDONLY);
        socketfd[GALILEO] = open(GALILEO_FILE, O_RDONLY);
    }
    else{
        setup_tcp_socket(&socketfd[GPS], rtk_port_rtkrcv);
        setup_tcp_socket(&socketfd[GALILEO], ppp_port_rtkrcv);
    }

    fds[0].fd = socketfd[GPS];
    fds[1].fd = socketfd[GALILEO];
    fds[2].fd = STDIN_FILENO;
    fds[0].events = fds[1].events = fds[2].events = POLLIN;

    //fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);

    wait_read[0] = wait_read[1] = 0;
    n_imu = 0;

    sol[IMU].time.week = 0;


    while(1){
        check_sols = 0;
        //usleep(300000);
        ret = poll(fds, 3, timeout_msecs); //wait for events on the 3 fds
        if (ret == -1){
            perror("SEMOR: poll");
            close_semor(1);
        }
        if(fds[2].revents & POLLIN){
            check_termination(); //Check if user requested the termination of the process
        }
        for(i = 0; i < 2; i++){ //Get GPS and GALILEO solutions
            offset[i] = 0;
                //strcpy(buf[i], "");
                //memset(buf[i], 0, sizeof buf);
                //strncpy(dest_string,"",strlen(dest_string));
            usleep(150000); //gives time to the solutions to be read (if one of them is late)
            //ret = poll(fds, 3, timeout_msecs);
            if(1/*fds[i].revents & POLLIN*/){
                if(debug && wait_read[i]){ //Don't read solution i (0:GPS, 1:GALILEO) if the solution in the previous iterations has a higher epoch than "seconds" variable
                    continue;
                }
                nbytes = read_gnss(socketfd[i], &offset[i], buf[i], sizeof buf[i]);

                if(strstr(buf[i], "lat") || strstr(buf[i], "latitude") || strstr(buf[i], "ecef") || strlen(buf[i]) < 5){ //Check if the input string contains gnss data or if it is an empty line or header
                    offset[i] = 0;
                    //strcpy(buf[i], "");
                    //memset(buf[tmp], 0, sizeof buf);
                    for(int j=0; j<MAXSTR;j++){
                        buf[i][j] = 0;
                    }
                    continue;
                }

                if(offset[i] != 0 && (buf[i][offset[i]-1] == '\r' || buf[i][offset[i]-1] == '\n')){ //If incoming data is a full gnss measurement string
                    buf[i][offset[i]-1] = '\0'; //Replace new line with end of string
                    //offset[i] = 0; //Reset offset for next reads
                    check_sols |= i+1;
                    //printf("%s\n", buf[i]);
                    first_input |= i+1;
                    sol[i] = str2gnss(buf[i]);//Parse string to gnss_sol_t structure
                    //strcpy(buf[i], "");
                    //memset(buf[tmp], 0, sizeof buf);
                    for(int j=0; j<MAXSTR;j++){
                        buf[i][j] = 0;
                    }
                }
            }
        }

        if(first_input < 3){
            continue;
        }

        if(debug){
            if(seconds == 0){
                seconds = (sol[GPS].time.sec <= sol[GALILEO].time.sec) ? sol[GPS].time.sec : sol[GALILEO].time.sec; //Set current second to the minimum of the epochs of the 2 gnss solution
            }
        }

        if(first_time){
            //Initialize imu epoch

            struct timeval tv;
            gettimeofday(&tv, NULL);
            int week = ((tv.tv_sec+LEAP_SECONDS-GPS_EPOCH))/(7*24*3600);
            int sec = (double)(((tv.tv_sec+LEAP_SECONDS-GPS_EPOCH))%(7*24*3600))+(tv.tv_usec / 1000000.0);

            sol[IMU].time.week = week;
            sol[IMU].time.sec = sec;

            if(debug){
                sol[IMU].time.sec = seconds;
            }
            init_imu(sol[IMU]);
            //initial_pos.time.sec++;
            first_time = 0;
        }

        process_solutions(check_sols); //Get best solution, output it and use it to calculate next imu position

        if(debug)
            seconds++; //Update current second
    }


}

void start_processing(void){
    int ret;
    int i;
    char path[3][PATH_MAX];

    //Initialize structure of initial_pos
    sol[IMU].a = init_x;
    sol[IMU].b = init_y;
    sol[IMU].c = init_z;

    sol[IMU].va = 0;
    sol[IMU].vb = 0;
    sol[IMU].vc = 0;

    //DEBUG
    //initial_pos.time.week = 2032;
    //initial_pos.time.sec = 273375;


    if(logs){
        if(relative){
            sprintf(path[0], "logs/gps.log");
            sprintf(path[1], "logs/galileo.log");
            sprintf(path[2], "logs/imu.log");
        }
        else{
            sprintf(path[0], "%slogs/gps.log", root_path);
            sprintf(path[1], "%slogs/galileo.log", root_path);
            sprintf(path[2], "%slogs/imu.log", root_path);
        }
        sol_file[GPS] = fopen(path[0], "w");

        sol_file[GALILEO] = fopen(path[1], "w");

        sol_file[IMU] = fopen(path[2], "w");
    }

    file = fopen(FILE_PATH, "w");
    if(file == NULL){
        perror("SEMOR fopen()");
    }
    fflush(file);

    handle_connection();
}

