CC = gcc
CFLAGS = -Wall -Wextra -O2 -pthread

TARGET = peer

all: $(TARGET)

$(TARGET): peer.o
	$(CC) $(CFLAGS) -o $@ $^

ipc_peer.o: peer.c
	$(CC) $(CFLAGS) -c peer.c

clean:
	rm -f *.o $(TARGET)

.PHONY: all clean
