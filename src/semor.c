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
#define FILE_PATH "/home/pi/REPOSITORY/SEMOR/confronto.txt"

/*static sol_t sol;*/

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
    char *token;
    gnss_sol_t gnss;
    char *eptr;

    token = strtok(str, " \n");
    gnss.time.week = strtod(token, &eptr);
    gnss.time.sec = strtod(strtok(NULL, " \n"), &eptr);
    gnss.lat = strtod(strtok(NULL, " \n"), &eptr);
    gnss.lng = strtod(strtok(NULL, " \n"), &eptr);
    gnss.height = strtod(strtok(NULL, " \n"), &eptr);
    gnss.Q = atoi(strtok(NULL, " \n"));
    gnss.ns = atoi(strtok(NULL, " \n"));
    gnss.sdn = strtod(strtok(NULL, " \n"), &eptr);
    gnss.sde = strtod(strtok(NULL, " \n"), &eptr);
    gnss.sdu = strtod(strtok(NULL, " \n"), &eptr);
    gnss.sdne = strtod(strtok(NULL, " \n"), &eptr);
    gnss.sdeu = strtod(strtok(NULL, " \n"), &eptr);
    gnss.sdun = strtod(strtok(NULL, " \n"), &eptr);
    gnss.age = strtof(strtok(NULL, " \n"), &eptr);
    gnss.ratio = strtof(strtok(NULL, " \n"), &eptr);

    return gnss;
}

void* sock_open(void* p){
    struct addrinfo hints, *res;
    int status;
    char ipstr[INET6_ADDRSTRLEN];
    //int rcv_no = *(int *)p;
    int max_fd;
    int i, j; //For counters
    int nbytes;
    char buf1[MAXSTR], buf2[MAXSTR], buf[MAXSTR];
    struct timeval timeout;
    int select_ret;
    int flag1 = 0, flag2 = 0;
    int offset = 0, offset1 = 0, offset2 = 0;
    int vfcntl;
    int curr_socket;

    fd_set master;    // master file descriptor list
    fd_set read_fds;  // temp file descriptor list for select()

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC; // AF_INET or AF_INET6 to force version
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;

    if ((status = getaddrinfo(NULL,PORT1, &hints, &res)) != 0) {
        fprintf(stderr, "SEMOR: getaddrinfo: %s\n", gai_strerror(status));
        close_semor(1);
    }

    socketfd1 = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if(socketfd1 == -1){
        perror("SEMOR socket1()");
        close_semor(1);
    }
    vfcntl = fcntl(socketfd1 ,F_GETFL, 0);
    if(vfcntl == -1){
        perror("SEMOR: vfcntl1()");
        close_semor(1);
    }
    if(fcntl(socketfd1, F_SETFL, vfcntl | O_NONBLOCK) == -1){
        perror("SEMOR: fcntl1()");
        close_semor(1);
    }

    if(connect(socketfd1, res->ai_addr, res->ai_addrlen) == -1){
        if(errno != EINPROGRESS){
            perror("SEMOR connect1()");
            close_semor(1);
        }
    }

    if ((status = getaddrinfo(NULL,PORT2, &hints, &res)) != 0) {
        fprintf(stderr, "SEMOR: getaddrinfo: %s\n", gai_strerror(status));
        close_semor(1);
    }

    socketfd2 = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if(socketfd2 == -1){
        perror("SEMOR socket2()");
        close_semor(1);
    }

    vfcntl = fcntl(socketfd2 ,F_GETFL, 0);
    if(vfcntl == -1){
        perror("SEMOR: vfcntl1()");
        close_semor(1);
    }

    if(fcntl(socketfd2, F_SETFL, vfcntl | O_NONBLOCK) == -1){
        perror("SEMOR: fcntl2()");
        close_semor(1);
    }

    if(connect(socketfd2, res->ai_addr, res->ai_addrlen) == -1){
        if(errno != EINPROGRESS){
            perror("SEMOR connect2()");
            close_semor(1);
        }
    }

    free(res);
    

    FD_ZERO(&master);    // clear the master and temp sets
    FD_ZERO(&read_fds);

    //Add sockets fd to fd sets
    FD_SET(socketfd1, &master);
    FD_SET(socketfd2, &master);

    max_fd = socketfd1 > socketfd2 ? socketfd1 : socketfd2;
    timeout.tv_sec = 0;
    timeout.tv_usec = 200000;

    fprintf(file, "prova\n");

    while(1){ //Stops when SEMOR ends
        read_fds = master; // copy it
        //fprintf(file, "data");
        select_ret = select(max_fd+1, &read_fds, NULL, NULL, &timeout);
        if (select_ret == -1) {
            perror("SEMOR: select");
            close_semor(1);
        }
        else if(select_ret == 0)
            continue; //Timeout

        //Leggo
        for(i = 0; i < 2; i++){
            if(i == 0){
                curr_socket = socketfd1;
                strcpy(buf, buf1);
                offset = offset1;
            }else{
                curr_socket = socketfd2;
                strcpy(buf, buf2);
                offset = offset2;
            }

            if ((nbytes = recv(curr_socket, buf + offset, (sizeof buf)-offset, 0)) <= 0) {
                // got error or connection closed by client
                if (nbytes == 0) {
                    //if(errno == EAGAIN)
                        continue; //E' stato aggiornato l'altro fd, non questo
                    // connection closed
                    //perror("SEMOR: rtkrcv server shut down");
                } else {
                    //perror("SEMOR: recv");
                    //Non si può uscire qui, è possibile che  il codice di ritorno sia -1 perché rtkrcv non ha ancora generato dati
                    continue;
                }
                //close_semor(1);
            } else {
                //Read data
                fprintf(file, "nuova misurazione catturata\n");
                //Aggiorno offset
                offset = offset + nbytes;
                if(curr_socket == socketfd1)
                    offset1 = offset1 + nbytes;
                else
                    offset1 = offset1 + nbytes;
                //Se buf contiene new line creo struttura
                if(buf[offset-1] == '\n'){
                    if(strstr(buf, "latitude"))
                        continue; //Header, ignoro

                    if(curr_socket == socketfd1){
                        gps = str2gnss(buf);
                        offset1 = 0;
                    }
                    else{
                        galileo = str2gnss(buf);
                        offset2 = 0;
                    }
                    if(gps.time.sec == galileo.time.sec){
                        //Add line with both solution side by side to the file
                        fprintf(file, "%s ### %s\n", buf1, buf2);
                    }
                }
            }
        }
    }
    free(res);
}


void start_processing(void){
    pthread_t id;
    /*int rcv_no = 0; 0: rtkrcv1, 1: rtkrcv2*/
    int res;

    file = fopen(FILE_PATH, "w");

    //Start connection 1
    res = pthread_create(&id, NULL, sock_open, NULL);
    if(res != 0){
        errno = res;
        perror("SEMOR: thread create 0");
        close_semor(1);
    }
    //Start connection 2
    /*rcv_no = 1;
    res = pthread_create(&id[1], NULL, sock_open, &rcv_no);
    if(res != 0){
        errno = res;
        perror("SEMOR: thread create 1");
    }*/
}

