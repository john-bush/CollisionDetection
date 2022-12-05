CC = g++

CFLAGS = -lrt -fopenmp -g -pg -gdwarf-4 -o

all: gjk_serial gjk_for gjk_section

gjk_serial: gjk_serial.cpp
	$(CC) $(CFLAGS) gjk_serial gjk_serial.cpp

gjk_for: gjk_for.cpp
	$(CC) $(CFLAGS) gjk_for gjk_for.cpp

gjk_section: gjk_section.cpp
	$(CC) $(CFLAGS) gjk_section gjk_section.cpp

clean:
	rm gjk_for gjk_serial gjk_section