CC = g++

CFLAGS = 

all: gjk_serial gjk_for gjk_section gjk_pthread

gjk_serial: gjk_serial.cpp
	$(CC) --std=c++11 -lrt -g -pg -gdwarf-4 -fopenmp -o gjk_serial gjk_serial.cpp

gjk_for: gjk_for.cpp
	$(CC) --std=c++11 -lrt -g -O1 -pg -gdwarf-4 -fopenmp -o gjk_for gjk_for.cpp

gjk_section: gjk_section.cpp
	$(CC) --std=c++11 -lrt -O2 -g -pg -gdwarf-4 -fopenmp -o gjk_section gjk_section.cpp

gjk_pthread: gjk_pthread.cpp
	$(CC) --std=c++11 -lrt -O2 -g -pg -gdwarf-4 -lpthread -o gjk_pthread gjk_pthread.cpp

gjk_thread: gjk_thread.cpp
	$(CC) --std=c++11 -lrt -O2 -g -pg -gdwarf-4 -lpthread -o gjk_thread gjk_thread.cpp

clean:
	rm gjk_for gjk_serial gjk_section gjk_pthread
