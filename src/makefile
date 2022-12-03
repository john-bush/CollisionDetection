CC = gcc

all: gjk_serial gjk

gjk_serial: gjk_serial.cpp
	$(CC) -lrt -fopenmp -g -Wall -pg -o gjk_serial gjk_serial.cpp

gjk: gjk.cpp
	$(CC) -lrt -fopenmp -g -Wall -pg -o gjk gjk.cpp