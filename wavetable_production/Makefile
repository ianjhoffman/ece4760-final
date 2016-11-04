CC = g++
CFLAGS = -c -Wall
LFLAGS = -Wall
STD = -std=c++11

all: convert

convert: convert.o
	$(CC) $(LFLAGS) convert.o -o convert $(STD)

convert.o: convert.cpp
	$(CC) $(CFLAGS) convert.cpp $(STD)