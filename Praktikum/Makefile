# Compiler und Flags
CC = gcc
CFLAGS = -Wall -Wextra -D_POSIX_C_SOURCE=200809L

# Zielprogramm
TARGET = anfrage_client

# Standardziel
all: $(TARGET)

# Kompilierung
$(TARGET): anfrage_client.c
	$(CC) $(CFLAGS) -o $(TARGET) anfrage_client.c

# Starten
run: $(TARGET)
	./$(TARGET)

# Aufräumen
clean:
	rm -f $(TARGET)
