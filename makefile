MAKEFLAGS += --no-print-directory
CC = g++
CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
OBJECT = thetas_viewer.o
TARGET = thetas_viewer

.cpp.o:
	$(CC) $(CFLAGS) -c $<

all:
	$(MAKE) $(TARGET)
	$(MAKE) clean

$(TARGET):$(OBJECT)
	$(CC) -o $@ $(OBJECT) $(LIBS)

clean:
	rm -f *.o


