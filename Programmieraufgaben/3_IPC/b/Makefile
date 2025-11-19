CC = gcc
CFLAGS = -Wall -Wextra -O2 -pthread

TARGET = client

all: $(TARGET)

$(TARGET): client.c
	$(CC) $(CFLAGS) -o $@ client.c

clean:
	rm -f $(TARGET)

.PHONY: all clean