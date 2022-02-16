default: main

main.o:
	gcc -c main.c -o bin/main.o

main: main.o
	gcc bin/main.o -o bin/main

clean:
	-rm -f bin/main.o
	-rm -f bin/main