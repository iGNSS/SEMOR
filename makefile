default: main

main.o:
	gcc -c main.c -g -o bin/main.o

semor.o:
	gcc -c src/semor.c -g -o bin/semor.o

main: main.o semor.o
	gcc bin/main.o bin/semor.o -lpthread -o bin/main

clean:
	-rm -f bin/main.o
	-rm -f bin/main
	-rm -f bin/semor.o