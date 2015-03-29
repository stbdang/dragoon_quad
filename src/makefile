CC=g++

all: ik_test

%.o: %.c
	$(CC) -c -o $@ $<

ik_test: main.o ik.o
	$(CC) -o $@ $^ -lm
