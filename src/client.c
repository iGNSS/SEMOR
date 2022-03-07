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

#define FILE_PATH "/home/semor/SEMOR/output.txt"

//Solutions
gnss_sol_t gps;
gnss_sol_t galileo;

//Integrated solutions
gnss_sol_t gps_imu;
gnss_sol_t galileo_imu;

FILE *file;
FILE *p;
int instance_no[2] = {0, 1};
pthread_t id[3];
pid_t str2str_pid, rtkrcv1_pid, rtkrcv2_pid;

char imu_data[200][75]; //Updated by Loosely.cpp

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
    gnss.x = strtod(strtok(NULL, " "), &eptr);
    gnss.y = strtod(strtok(NULL, " "), &eptr);
    gnss.z = strtod(strtok(NULL, " "), &eptr);
    gnss.Q = atoi(strtok(NULL, " "));
    gnss.ns = atoi(strtok(NULL, " "));
    gnss.sdn = strtod(strtok(NULL, " "), &eptr);
    gnss.sde = strtod(strtok(NULL, " "), &eptr);
    gnss.sdu = strtod(strtok(NULL, " "), &eptr);
    gnss.sdne = strtod(strtok(NULL, " "), &eptr);
    gnss.sdeu = strtod(strtok(NULL, " "), &eptr);
    gnss.sdun = strtod(strtok(NULL, " "), &eptr);
    gnss.age = strtof(strtok(NULL, " "), &eptr);
    gnss.ratio = strtof(strtok(NULL, " "), &eptr);

    return gnss;
}

void gnss2str(char* str, gnss_sol_t gnss){
    sprintf(str, "%d %d %lf %lf %lf %d %d %lf %lf %lf %lf %lf %lf %f %f", gnss.time.week,
    gnss.time.sec, gnss.x, gnss.y, gnss.z, gnss.Q, gnss.ns, 
    gnss.sdn, gnss.sde, gnss.sdu, gnss.sdne, gnss.sdeu, gnss.sdun,
    gnss.age, gnss.ratio);
}

void gnsscopy(gnss_sol_t *dest, gnss_sol_t src){
    (*dest).time.week = src.time.week;
    (*dest).time.sec = src.time.sec;
    (*dest).x = src.x;
    (*dest).y = src.y;
    (*dest).z = src.z;
    (*dest).Q = src.Q;
    (*dest).ns = src.ns;
    (*dest).sdn = src.sdn;
    (*dest).sde = src.sde;
    (*dest).sdu = src.sdu;
    (*dest).sdne = src.sdne;
    (*dest).sdeu = src.sdeu;
    (*dest).sdun = src.sdun;
    (*dest).age = src.age;
    (*dest).ratio = src.ratio;
}

void process_gnss_data(char buf[MAXSTR], int ins){
    char sol1[MAXSTR], sol2[MAXSTR];
    gnss_sol_t intsol;

    fprintf(p, "(%d) %s\n", ins, buf); //debug
    fflush(p);
    pthread_mutex_lock(&mutex);
    if(ins == 0){ //Se questo handler gestisce la prima istanza di rtkrcv
        if(gps.time.week != 0){ //La soluzione gps precedente non è stata consumata, la invio da sola prima di quella corrente
            gnss2str(sol1, gps);
            fprintf(file, "ALONE %s\n", sol1);
            gps.time.week = 0;
        }
        gps = str2gnss(buf);
        if(isset_first_pos == 0){
            isset_first_pos == 1;
            first_pos = gps;
            init_imu(first_pos, ins);
        }
        if(imu_ready == 1){
            gnsscopy(&gps_imu, gps);
            int_sol(&gps_imu, ins);
        }
        if(galileo.time.week != 0 && gps.time.sec == galileo.time.sec){ //E' arrivata prima la soluzione galileo, quindi posso già farei controlli
            //Add line with both solution side by side to the file
            gnss2str(sol1, gps);
            gnss2str(sol2, galileo);
            fprintf(file, "%s ||||||||| %s\n", sol1, sol2);
            //Dico alle prossime iterazioni che i precedenti valori sono già stati consumati
            gps.time.week = 0;
            galileo.time.week = 0;
        }
    }else{ //Se questo handler gestisce la seconda istanza di rtkrcv
        if(galileo.time.week != 0){ //La soluzione galileo precedente non è stata consumata, la invio da sola prima di quella corrente
            gnss2str(sol2, galileo);
            fprintf(file, "ALONE %s\n", sol2);
            galileo.time.week = 0;
        }
        galileo = str2gnss(buf);
        gnsscopy(&galileo_imu, galileo);
        int_sol(&galileo_imu, ins);
        if(isset_first_pos == 0){
            isset_first_pos == 1;
            first_pos = galileo;
            init_imu(first_pos, ins);
        }
        if(imu_ready == 1){
            gnsscopy(&galileo_imu, galileo);
            int_sol(&galileo_imu, ins);
        }
        if(gps.time.week != 0 && gps.time.sec == galileo.time.sec){ //E' arrivata prima la soluzione gps, quindi posso già farei controlli
            //Add line with both solution side by side to the file
            gnss2str(sol1, gps);
            gnss2str(sol2, galileo);
            fprintf(file, "%s ||||||||| %s\n", sol1, sol2);
            gps.time.week = 0;
            galileo.time.week = 0;
        }
    }
    fflush(file);
    pthread_mutex_unlock(&mutex);
}

void* handle_connection(void* inst_no){
    int ins = *((int *) inst_no);
    struct addrinfo hints, *res;
    int status;
    char buf[MAXSTR];
    int offset = 0;
    int nbytes;
    int socketfd;
    char port[5];

    memset(&hints, 0, sizeof hints);
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    hints.ai_family = AF_INET; 

    if(ins == 0){
        strcpy(port, PORT1);
    }
    else{
        strcpy(port, PORT2);
    }
    if ((status = getaddrinfo(NULL,port, &hints, &res)) != 0) {
        fprintf(stderr, "SEMOR: getaddrinfo: %s\n", gai_strerror(status));
        close_semor(1);
    }

    //Socket creation and connection
    socketfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if(socketfd == -1){
        perror("SEMOR socket1()");
        close_semor(1);
    }
    do{
        if(connect(socketfd, res->ai_addr, res->ai_addrlen) == -1){
            if(errno == ECONNREFUSED) continue; //Wait for rtkrcv to start and to send data
        }
        break; //Stop while
    }while(1);

    free(res);

    while(1){
        offset = 0;
        do{
            nbytes = read(socketfd, buf+offset, 1);
            if(nbytes == -1){
                perror("SEMOR read socket1");
                close_semor(1);
            }else if(nbytes == 0){ // Rtkrcv1 is down
                printf("rtkrcv down\n"); //debug
                continue;
            }
            offset++;
        }while(buf[offset-1] != '\r' && buf[offset-1] != '\n');
        buf[offset-1] = '\0';
        
        if(strstr(buf, "lat") || strstr(buf, "latitude") || strlen(buf) < 5){ //Se la linea letta contiene fa parte dell'header
            continue;
        }

        process_gnss_data(buf, ins);
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

    //Ho bisogno di avviare gli handler in thread separati in caso una delle due istanze di rtkrcv non sia up in un determinato momento
    //Avvio handler per rtkrcv1
    ret = pthread_create(&id[0], NULL, handle_connection, (void *)&instance_no[0]);
    if(ret != 0){
        errno = ret;
        perror("SEMOR: thread create 0");
        close_semor(1);
    }
    //Avvio handler per rtkrcv2
    ret = pthread_create(&id[1], NULL, handle_connection, (void *)&instance_no[1]);
    if(ret != 0){
        errno = ret;
        perror("SEMOR: thread create 1");
        close_semor(1);
    }
}

