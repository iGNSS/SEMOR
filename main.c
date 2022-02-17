#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <wait.h>
#include <string.h>
#include <fcntl.h>

void close_io(void){
    close(STDIN_FILENO);
    //close(STDOUT_FILENO);
    close(STDERR_FILENO);
}

int main(){
    pid_t str2str_pid, rtkrcv1_pid, rtkrcv2_pid;
    char cmd;

    char *const str2str_args[] = {"/home/pi/REPOSITORY/SEMOR/RTKLIB-b34e/app/consapp/str2str/gcc/str2str", "-in", "tcpcli://192.168.2.91:8081", "-out", "tcpsvr://:8085", "-out", "tcpsvr://:8086", NULL};
    char *const rtkrcv1_args[] = {"/home/pi/REPOSITORY/SEMOR/RTKLIB-b34e/app/consapp/rtkrcv/gcc/rtkrcv", "-s", "-o", "/home/pi/REPOSITORY/SEMOR/conf/test1.conf", NULL};
    char *const rtkrcv2_args[] = {"/home/pi/REPOSITORY/SEMOR/RTKLIB-b34e/app/consapp/rtkrcv/gcc/rtkrcv", "-s", "-o", "/home/pi/REPOSITORY/SEMOR/conf/test2.conf", NULL};

    //Avvio str2str
    if ((str2str_pid = fork()) == -1){
        perror("fork error: str2str");
    }
    else if (str2str_pid == 0) {
        close_io();
        execv(str2str_args[0], str2str_args); //da cambiare con bin/str2str
        printf("\nexecv error (str2str)");
    }
    else{
        printf("\nstr2str pid: %d", (int)str2str_pid);
    }

    //Avvio prima istanza di rtkrcv
    if ((rtkrcv1_pid = fork()) == -1){
        perror("fork error: rtkrcv1");
    }
    else if (rtkrcv1_pid == 0) {
        close_io();
        execv(rtkrcv1_args[0], rtkrcv1_args); //da cambiare con bin/rtkrcv
        printf("\nexecv error (rtkrcv1)");
    }else{
        printf("\nrtkrcv1 pid: %d", (int)rtkrcv1_pid);
    }

    //Avvio seconda istanza di rtkrcv

    if ((rtkrcv2_pid = fork()) == -1){
        
        perror("fork error: rtkrcv2");
    }
    else if (rtkrcv2_pid == 0) {
        close_io();
        execv(rtkrcv2_args[0], rtkrcv2_args); //da cambiare con bin/rtkrcv
        printf("\nexecv error (rtkrcv2)");
    }
    else{
        printf("\nrtkrcv2 pid: %d", (int)rtkrcv2_pid);
    }

    printf("\n");
    do{
        printf("SEMOR> ");
        scanf("%c", &cmd);
    }while(cmd != 'q' && cmd != 'Q');

    if(kill(str2str_pid, SIGKILL) == -1){
        perror("Error killing str2str process");
    }
    if(kill(rtkrcv1_pid, SIGKILL) == -1){
        perror("Error killing rtkrcv1 process");
    }
    if(kill(rtkrcv2_pid, SIGKILL) == -1){
        perror("Error killing rtkrcv2 process");
    }


    printf("\nSEMOR stopped.\n");

    return 0;
}