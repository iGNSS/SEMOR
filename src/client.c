#include "semor.h"
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <string.h>

#define PORT1 "8090"
#define PORT2 "8091"
#define FILE_PATH "/home/pi/REPOSITORY/SEMOR/output.txt"

static gnss_sol_t gps;
static gnss_sol_t galileo;
FILE *file;
int socketfd1, socketfd2;

//Close SEMOR and all processes started by it (rtkrcv and str2str)
void close_semor(int status){
    if(kill(str2str_pid, SIGKILL) == -1){
        perror("Error killing str2str process");
    }
    if(kill(rtkrcv1_pid, SIGKILL) == -1){
        perror("Error killing rtkrcv1 process");
    }
    if(kill(rtkrcv2_pid, SIGKILL) == -1){
        perror("Error killing rtkrcv2 process");
    }
    fclose(file);
    close(socketfd1);
    close(socketfd2);
    exit(status);
}

gnss_sol_t str2gnss(char str[MAXSTR]){
    gnss_sol_t gnss;
    char *eptr;
    char copy[MAXSTR];

    strcpy(copy, str);

    gnss.time.week = atoi(strtok(copy, " "));
    gnss.time.sec = (int)strtod(strtok(NULL, " "), &eptr);
    gnss.lat = strtod(strtok(NULL, " "), &eptr);
    gnss.lng = strtod(strtok(NULL, " "), &eptr);
    gnss.height = strtod(strtok(NULL, " "), &eptr);
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

void* sock_open(void* p){
    struct addrinfo hints, *res;
    int status;
    char buf1[MAXSTR], buf2[MAXSTR];
    int offset = 0;

    memset(&hints, 0, sizeof hints);
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    hints.ai_family = AF_INET; 

    if ((status = getaddrinfo(NULL,PORT1, &hints, &res)) != 0) {
        fprintf(stderr, "SEMOR: getaddrinfo: %s\n", gai_strerror(status));
        close_semor(1);
    }

    //Socket creation and connection
    socketfd1 = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if(socketfd1 == -1){
        perror("SEMOR socket1()");
        close_semor(1);
    }
    do{
        if(connect(socketfd1, res->ai_addr, res->ai_addrlen) == -1){
            if(errno == ECONNREFUSED) continue; //Wait for rtkrcv to start and to send data
            perror("SEMOR connect1()");
            close_semor(1);
        }
        break; //Stop while
    }while(1);

    if ((status = getaddrinfo(NULL,PORT2, &hints, &res)) != 0) {
        fprintf(stderr, "SEMOR: getaddrinfo: %s\n", gai_strerror(status));
        close_semor(1);
    }

    socketfd2 = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if(socketfd2 == -1){
        perror("SEMOR socket2()");
        close_semor(1);
    }
    do{
        if(connect(socketfd2, res->ai_addr, res->ai_addrlen) == -1){
            if(errno == ECONNREFUSED) continue; //Wait for rtkrcv to start and to send data
            perror("SEMOR connect2()");
            close_semor(1);
        }
        break; //Stop while
    }while(1);
    free(res);

    //Read loop
    while(1){ //Stops when SEMOR ends
        offset = 0;
        do{
            if(read(socketfd1, buf1+offset, 1) == -1){
                perror("SEMOR read socket1");
                close_semor(1);
            }
            offset++;
        }while(buf1[offset-1] != '\r' && buf1[offset-1] != '\n');
        buf1[offset-1] = '\0';
        offset = 0;
        do{
            if(read(socketfd2, buf2+offset, 1) == -1){
                perror("SEMOR read socket2");
                close_semor(1);
            }
            offset++;
        }while(buf2[offset-1] != '\r' && buf2[offset-1] != '\n');
        buf2[offset-1] = '\0';

        if(strlen(buf1) == 0){ //Rtkrcv sends one empty line between messages
            continue;
        }
        gps = str2gnss(buf1);
        galileo = str2gnss(buf2);
        if(gps.time.sec == galileo.time.sec){
            //Add line with both solution side by side to the file
            fprintf(file, "%s ## %s\n", buf1, buf2);
            fflush(file);
        }
    }
    free(res);
}


void start_processing(void){
    pthread_t id;
    int res;

    file = fopen(FILE_PATH, "w");
    if(file == NULL){
        perror("SEMOR fopen()");
    }
    fflush(file);
    //Start connection 1
    res = pthread_create(&id, NULL, sock_open, NULL);
    if(res != 0){
        errno = res;
        perror("SEMOR: thread create 0");
        close_semor(1);
    }
}

