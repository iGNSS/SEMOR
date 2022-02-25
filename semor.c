#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include "src/semor.h"

void close_io(void){
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);
}

int main(){
    char cmd;
    str2str_pid = rtkrcv1_pid = rtkrcv2_pid = -1;

    char *const str2str_args[] = {"/home/semor/SEMOR/RTKLIB-b34e/app/consapp/str2str/gcc/str2str", "-in", "tcpcli://192.168.2.91:8081", "-out", "tcpsvr://:8085", "-out", "tcpsvr://:8086", NULL};
    char *const rtkrcv1_args[] = {"/home/semor/SEMOR/RTKLIB-b34e/app/consapp/rtkrcv/gcc/rtkrcv", "-s", "-o", "/home/pi/REPOSITORY/SEMOR/conf/rtk4pid.conf", NULL}; //rtk4pid.conf
    char *const rtkrcv2_args[] = {"/home/semor/SEMOR/RTKLIB-b34e/app/consapp/rtkrcv/gcc/rtkrcv", "-s", "-o", "/home/pi/REPOSITORY/SEMOR/conf/ppp4pid_navcast.conf", NULL}; //ppp4pid_navcast.conf

    //Execute str2str
    
    if ((str2str_pid = fork()) == -1){
        perror("SEMOR: fork error: str2str");
        close_semor(1);
    }
    else if (str2str_pid == 0) {
        close_io();
        execv(str2str_args[0], str2str_args);
        printf("\nSEMOR: execv error (str2str)");
        close_semor(1);
    }
    else{
        printf("\nstr2str pid: %d", (int)str2str_pid); //debug
    }

    //Execute first rtkrcv instance
    if ((rtkrcv1_pid = fork()) == -1){
        perror("SEMOR: fork error: rtkrcv(1)");
        close_semor(1);
    }
    else if (rtkrcv1_pid == 0) {
        close_io();
        execv(rtkrcv1_args[0], rtkrcv1_args);
        printf("\nSEMOR: execv error (rtkrcv(1))");
        close_semor(1);
    }else{
        //printf("\nrtkrcv1 pid: %d", (int)rtkrcv1_pid); //debug
    }

    //Execute second rtkrcv instance

    if ((rtkrcv2_pid = fork()) == -1){
        
        perror("SEMOR: fork error: rtkrcv(2)");
        close_semor(1);
    }
    else if (rtkrcv2_pid == 0) {
        close_io();
        execv(rtkrcv2_args[0], rtkrcv2_args); //da cambiare con bin/rtkrcv
        printf("\nSEMOR: execv error (rtkrcv(2))");
        close_semor(1);
    }
    else{
        //printf("\nrtkrcv2 pid: %d", (int)rtkrcv2_pid); //debug
        
    }

    //Create pids.txt file to manually kill previous executed processes if needed
    FILE *pids = fopen("/home/semor/SEMOR/pids.txt", "w");
    fprintf(pids, "str2str: %d\nrtkrcv(1): %d\nrtkrcv(2): %d", str2str_pid, rtkrcv1_pid, rtkrcv2_pid);
    fclose(pids);
    


    /*
    Semor starts here
    */
    start_processing();


    //Stop semor
    printf("\n");
    printf("SEMOR: press q to stop\n");
    do{
        cmd = getchar();
    }while(cmd != 'q' && cmd != 'Q');

    close_semor(0);

    printf("\nSEMOR stopped.\n");

    return 0;
}