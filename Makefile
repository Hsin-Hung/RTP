CC          = gcc
LIBDIR      = /arch/gnu/lib
LIBFLAGS    = -L${LIBDIR}
CFLAGS      = $(LIBFLAGS) -g 
GCCLIBS     = -lstdc++

OBJS = sr.o 
EXE = sr

all: ${EXE}

%.o: %.cpp
	${CC} ${CFLAGS} -c $*.cpp

${EXE}: ${OBJS}
	${CC} ${CFLAGS} -o ${EXE} ${OBJS} ${GCCLIBS}

sr.o: sr.cpp

clean:
	rm -f ${EXE} ${OBJS} *~
     