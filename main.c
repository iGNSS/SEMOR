#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <wait.h>
#include <string.h>

int main(){
    pid_t str2str_pid, rtkrcv1_pid, rtkrcv2_pid;
    char cmd[20];

    int status1, status2, status3;

    char *const str2str_args[] = {"/home/pi/REPOSITORY/SEMOR/RTKLIB-b34e/app/consapp/str2str/gcc/str2str", "-in", "tcpcli://192.168.2.91:8081", "-out", "tcpsvr://:8085", "-out", "tcpsvr://:8086", NULL};
    char *const rtkrcv1_args[] = {"/home/pi/REPOSITORY/SEMOR/RTKLIB-b34e/app/consapp/rtkrcv/gcc/rtkrcv", "-s", "-o", "/home/pi/REPOSITORY/SEMOR/conf/test1.conf", NULL};
    char *const rtkrcv2_args[] = {"/home/pi/REPOSITORY/SEMOR/RTKLIB-b34e/app/consapp/rtkrcv/gcc/rtkrcv", "-s", "-o", "/home/pi/REPOSITORY/SEMOR/conf/test2.conf", NULL};
//home/pi/REPOSITORY/raw_data_from_ublox/output_ubx/raw_obs_LIGE_20220251500.ubx
    //Avvio str2str
    if ((str2str_pid = fork()) == -1){
        perror("fork error: str2str");
    }
    else if (str2str_pid == 0) {
        execv(str2str_args[0], str2str_args); //da cambiare con bin/str2str
        printf("execv error (str2str)");
    }
    else{
        printf("str2str pid: %d", (int)str2str_pid);
    }

    //Avvio prima istanza di rtkrcv

    if ((rtkrcv1_pid = fork()) == -1){
        
        perror("fork error: rtkrcv1");
    }
    else if (rtkrcv1_pid == 0) {
        printf("rtkrcv1 pid: %d", (int)rtkrcv1_pid);
        execv(rtkrcv1_args[0], rtkrcv1_args); //da cambiare con bin/rtkrcv
        printf("execv error (rtkrcv1)");
    }else{
        printf("rtkrcv1 pid: %d", (int)rtkrcv1_pid);
    }

    //Avvio seconda istanza di rtkrcv

    if ((rtkrcv2_pid = fork()) == -1){
        
        perror("fork error: rtkrcv2");
    }
    else if (rtkrcv2_pid == 0) {
        printf("rtkrcv2 pid: %d", (int)rtkrcv2_pid);
        execv(rtkrcv2_args[0], rtkrcv2_args); //da cambiare con bin/rtkrcv
        printf("execv error (rtkrcv2)");
    }
    else{
        printf("rtkrcv2 pid: %d", (int)rtkrcv2_pid);
    }

    //Controllo
    waitpid(str2str_pid, &status1, 0);
    if (WIFSIGNALED(status1) || !WEXITSTATUS(status1)){
        printf("Error with str2str process\n");
    }

    waitpid(rtkrcv1_pid, &status2, 0);
    if (WIFSIGNALED(status2) || !WEXITSTATUS(status2)){
        printf("Error with rtkrcv process (first instance)\n");
    }

    waitpid(rtkrcv2_pid, &status3, 0);
    if (WIFSIGNALED(status3) || !WEXITSTATUS(status3)){
        printf("Error with rtkrcv process (second instance)\n");
    }
    do{
        printf("SEMOR> ");
        scanf("%s", cmd);
    }while(strcmp(cmd, "stop") != 0 && strcmp(cmd, "STOP") != 0);

    kill(str2str_pid, SIGTERM);
    kill(rtkrcv1_pid, SIGTERM);
    kill(rtkrcv2_pid, SIGTERM);


    printf("Program stopped.");

    return 0;
}