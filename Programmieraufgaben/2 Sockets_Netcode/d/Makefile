# Compiler und Compiler-Optionen
CC = gcc
CFLAGS = -Wall -Wextra -D_POSIX_C_SOURCE=200809L -pthread

# Programmname
TARGET = anfrage_peer

# Standardziel: Kompilieren
all: $(TARGET)

# Regel zum Bauen des Servers
$(TARGET): anfrage_peer.c
	$(CC) $(CFLAGS) -o $(TARGET) anfrage_peer.c

# Server starten (optional)
run: $(TARGET)
	./$(TARGET)

# Aufräumen: erzeugte Dateien löschen
clean:
	rm -f $(TARGET)
