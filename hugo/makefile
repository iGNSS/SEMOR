default: semor

imu_read.o:
	gcc -c imu_read.c -g -o imu_read.o

semor: imu_read.o
	gcc imu_read.o -o imu_read

clean:
	-rm -f imu_read
	-rm -f imu_read.o