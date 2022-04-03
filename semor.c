#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include "src/semor.h"

#define MAX_LINE 256

//Set default configuration
int logs=0;
int debug=1;
double init_bg_unc = 4.8481367284e-05;
double init_ba_unc = 0.048901633857000000;
double psd_gyro  = 3.38802348178723e-09;
double psd_acce      =     2.60420170553977e-06;     // acce noise PSD (m^2/s^3)  
double psd_bg        =     2.61160339323310e-14;     // gyro bias random walk PSD (rad^2/s^3)
double psd_ba        =     1.66067346797506e-09;
int sample_rate = 104; //in hz
int imu_init_epochs =  300;//in seconds

char str2str_path[PATH_MAX];
char rtkrcv_path[PATH_MAX];
char rtkconf[PATH_MAX];
char pppconf[PATH_MAX];

char root_path[PATH_MAX-200];

char str2str_in[30] = "tcpcli://192.168.2.91:8081";
char str2str_out1_port[16] = "tcpsvr://:8085";
char str2str_out2_port[16] = "tcpsvr://:8086";


void close_io(void){
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);
}

void read_conf_line(char line[MAX_LINE]){
    int i;
    char *token;
    char *eptr;
    token = strtok(line, "#= \t");
    
    if(strstr(token, "debug")){
        debug = atoi(strtok(NULL, "#= \t"));
        return;
    }
    if(strstr(token, "logs")){
        logs = atoi(strtok(NULL, "#= \t"));
        return;
    }
    if(strstr(token, "str2str-in")){
        strcpy(str2str_in, strtok(NULL, "#= \t"));
        return;
    }
    if(strstr(token, "str2str-out1-port")){
        strcpy(str2str_out1_port, strtok(NULL, "#= \t"));
        return;
    }
    if(strstr(token, "str2str-out2-port")){
        strcpy(str2str_out2_port, strtok(NULL, "#= \t"));
        return;
    }
    if(strstr(token, "str2str-path")){
        strcpy(str2str_path, strtok(NULL, "#= \t"));
        return;
    }
    if(strstr(token, "rtkrcv-path")){
        strcpy(rtkrcv_path, strtok(NULL, "#= \t"));
        return;
    }
    if(strstr(token, "rtkconf")){
        strcpy(rtkconf, strtok(NULL, "#= \t"));
        return;
    }
    if(strstr(token, "pppconf")){
        strcpy(pppconf, strtok(NULL, "#= \t"));
        return;
    }
    if(strstr(token, "sample-rate")){
        sample_rate = atoi(strtok(NULL, "#= \t"));
        return;
    }
    if(strstr(token, "imu-init-epochs")){
        imu_init_epochs = atoi(strtok(NULL, "#= \t"));
        return;
    }
    if(strstr(token, "init-bg-unc")){
        init_bg_unc = strtod(strtok(NULL, "#= \t"), &eptr);
        return;
    }
    if(strstr(token, "init-ba-unc")){
        init_ba_unc = strtod(strtok(NULL, "#= \t"), &eptr);
        return;
    }
    if(strstr(token, "psd-gyro")){
        psd_gyro = strtod(strtok(NULL, "#= \t"), &eptr);
        return;
    }
    if(strstr(token, "psd-acce")){
        psd_acce = strtod(strtok(NULL, "#= \t"), &eptr);
        return;
    }
    if(strstr(token, "psd-bg")){
        psd_bg = strtod(strtok(NULL, "#= \t"), &eptr);
        return;
    }
    if(strstr(token, "psd-ba")){
        psd_ba = strtod(strtok(NULL, "#= \t"), &eptr);
        return;
    }
}

int main(int argc, char *argv[]){
    char cmd;
    pid_t str2str_pid, rtkrcv1_pid, rtkrcv2_pid;
    int isset_first_pos;
    int imu_ready;

    //Setup from configuration
    int i;
    char line[MAX_LINE];
	char cwd[PATH_MAX-400];
    char semor_conf_path[PATH_MAX];

	if (getcwd(cwd, sizeof(cwd)) != NULL) {
		for(i=0;;i++){
			if(cwd[i] == '\0'){
				cwd[i] = '/';
				cwd[i+1] = '\0';
				break;
			}
		}
		//printf("Current working dir: %s\n", cwd);
	} else {
		perror("getcwd() error");
		return 1;
	}

	if(argv[0][0] == '.')
		memmove(argv[0], argv[0]+2, strlen(argv[0]));

    for(i=strlen(argv[0])-1; i >= 0; i--){
        if(argv[0][i] != '/'){
            argv[0][i] = '\0';
        }
        else
            break;
    }

	sprintf(root_path, "%s%s", cwd, argv[0]);
    //Set default paths
    sprintf(str2str_path, "%sRTKLIB-b34e/app/consapp/str2str/gcc/str2str", root_path);
    sprintf(rtkrcv_path, "%sRTKLIB-b34e/app/consapp/rtkrcv/gcc/rtkrcv", root_path);
    sprintf(rtkconf, "%sconf/rtk4pid.conf", root_path);
    sprintf(pppconf, "%sconf/ppp4pid_navcast.conf", root_path);
    

    sprintf(semor_conf_path, "%ssemor.conf", root_path);
    FILE* f;
    if( access( semor_conf_path, F_OK ) == 0 ) {
        // read file
        f = fopen(semor_conf_path, "r");
        while(fgets(line, sizeof(line), f)){
            read_conf_line(line);
        }
    } else {
        // create file and use default parameters
        f = fopen(semor_conf_path, "w");

        fprintf(f, "#General\n");
        fprintf(f, "debug=%d   #0:disabled (get realtime data from rtkrcv and imu), 1:enabled (get data from files (in test folder))\n", debug);
        fprintf(f, "logs=%d   #0:disabled, 1:enabled\n", logs);
        fprintf(f, "str2str-path=%sRTKLIB-b34e/app/consapp/str2str/gcc/str2str\n", root_path);
        fprintf(f, "str2str-in=%s\n", str2str_in);
        fprintf(f, "str2str-out1-port=%s\n", str2str_out1_port);
        fprintf(f, "str2str-out2-port=%s\n", str2str_out2_port);
        fprintf(f, "rtkrcv-path=%sRTKLIB-b34e/app/consapp/rtkrcv/gcc/rtkrcv\n", root_path);
        fprintf(f, "rtkconf=%sconf/rtk4pid.conf\n", root_path);
        fprintf(f, "pppconf=%sconf/ppp4pid_navcast.conf\n", root_path);
        fprintf(f, "\n#IMU parameters\n");
        fprintf(f, "imu-init-epochs(sec)=%d\n", imu_init_epochs);
        fprintf(f, "sample-rate(hz)=%d\n", sample_rate);
        fprintf(f, "init-bg-unc=%.15g\n", init_bg_unc);
        fprintf(f, "init-ba-unc=%.15g\n", init_ba_unc);
        fprintf(f, "psd-gyro=%.15g\n", psd_gyro);
        fprintf(f, "psd-acce=%.15g\n", psd_acce);
        fprintf(f, "psd-bg=%.15g\n", psd_bg);
        fprintf(f, "psd-ba=%.15g\n", psd_ba);
    }
    fclose(f);

    //Initialize shared variables
    str2str_pid = rtkrcv1_pid = rtkrcv2_pid = -1;
    isset_first_pos = 0;
    imu_ready = 0;
    if(!debug){
        char *const str2str_args[] = {str2str_path, "-in", str2str_in, "-out", str2str_out1_port, "-out", str2str_out2_port, NULL};
        char *const rtkrcv1_args[] = {rtkrcv_path, "-s", "-o", rtkconf, NULL}; //rtk4pid.conf
        char *const rtkrcv2_args[] = {rtkrcv_path, "-s", "-o", pppconf, NULL}; //ppp4pid_navcast.conf

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

        //Create pids.txt file to manually kill previous executed processes if needed
        char pids_file[PATH_MAX];
        sprintf(pids_file, "%spids.txt", root_path);
        FILE *pids = fopen(pids_file, "w");
        fprintf(pids, "str2str: %d\nrtkrcv(1): %d\nrtkrcv(2): %d", str2str_pid, rtkrcv1_pid, rtkrcv2_pid);
        fclose(pids);

    }
    /*
    Semor starts here
    */

    printf("\n");
    printf("SEMOR: press q to stop\n");
    start_processing();



    return 0;
}