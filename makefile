default: semor

semor.o:
	gcc -c semor.c -g -o bin/semor.o

client.o:
	gcc -c src/client.c -g -o bin/client.o

semor: semor.o client.o
	gcc bin/semor.o bin/client.o -o bin/semor -lm

clean:
	-rm -f bin/semor.o
	-rm -f bin/semor
	-rm -f bin/client.o