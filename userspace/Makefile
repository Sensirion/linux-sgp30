CC=gcc
CFLAGS=-I. -O -Wall -Wconversion -Wsign-compare
OBJS=sgp30_userspace.o

sgp30_test:${OBJS}
	${CC} ${CFLAGS} -o $@ ${OBJS} sgp30_test.c

clean:
	rm -f sgp30_test *.o

.c.o:
	${CC} ${CFLAGS} -c $<
