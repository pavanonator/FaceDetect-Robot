CC = g++
CFLAGS = -pedantic -Wall -fPIC --std=c++11 -O3
LIBS = -larmadillo
SERIAL = $(shell pwd)/../serial
BASEROBOT = $(shell pwd)/..
INCLUDE = -I$(SERIAL) -I$(BASEROBOT)
SUBDIRS = $(SERIAL)
SUBOBJS = $(SERIAL)/serial.o
OBJECTS = tachikoma.o

all: subdirs $(OBJECTS)

subdirs:
	make -C $(BASEROBOT) baserobot.o
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

tachikoma.o: tachikoma.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

test.o: test.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

xboxctrl.o: xboxctrl.c
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

test: $(BASEROBOT)/baserobot.o $(OBJECTS) $(SUBOBJS) xboxctrl.o test.o
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) $(SUBOBJS) xboxctrl.o test.o test
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
