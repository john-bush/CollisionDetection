CC = g++
CFLAGS = -Wall -g


PWD = ${CURDIR}
SRC=/src
INC=/include
BUILDDIR=/build
OUTDIR=/build/bin
TEMPDIR=/build/tmp
MKDIR_P = mkdir


# ****************************************************
.PHONY: all directories clean run

all: $(OUTDIR)/output $(TEMPDIR)/main.o $(TEMPDIR)/gjk.o $(TEMPDIR)/generateRandomPolygon.o $(TEMPDIR)/convexHull.o

$(OUTDIR)/output: main.o gjk.o generateRandomPolygon.o convexHull.o
	$(CC) $(CFLAGS) -o output main.o gjk.o generateRandomPolygon.o

$(TEMPDIR)/main.o: $(SRC)/main.cpp $(INC)/gjk.h $(INC)/primitives.h
	$(CC) $(CFLAGS) -c main.cpp

$(TEMPDIR)/gjk.o: $(SRC)/gjk.cpp $(INC)/gjk.h $(INC)/primitives.h
	$(CC) $(CFLAGS) -c $(SRC)/gjk.cpp

$(TEMPDIR)/generateRandomPolygon.o: $(SRC)/generateRandomPolygon.cpp $(INC)/generateRandomPolygon.h $(INC)/convexHull.h $(INC)/primitives.h
	$(CC) $(CFLAGS) -c $(SRC)/generateRandomPolygon.cpp

$(TEMPDIR)/convexHull.o: $(SRC)/convexHull.cpp $(INC)/convexHull.h $(INC)/primitives.h
	$(CC) $(CFLAGS) -c $(SRC)/convexHull.cpp

directories: ${OUTDIR} $(TEMPDIR)

$(OUTDIR): 
	${MKDIR_P} $(OUTDIR)

$(TEMPDIR):
	${MKDIR_P} $(TEMPDIR)

clean:
	rm -rf $(BUILDDIR)/*.o

run:
	$(OUTDIR)/output