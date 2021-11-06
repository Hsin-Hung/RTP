
all: sr gbn

sr: sr.o
	g++ sr.cpp -o sr

gbn: gbn.o
	g++ gbn.cpp -o gbn

clean:
	rm -f sr gbn *.o OutputFile
     