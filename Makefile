COMPILE = gcc -O3 -pipe
LIBS = -L./lib/ -lserial_232_485

all:
	$(COMPILE) main.c $(LIBS) -o serialecho