#define _POSIX_C_SOURCE 200809L //aktiviert POSIX-Funktionen (siehe man feature_test_macros)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define SERVER_IP "127.0.0.1"
#define SERVER_PORT 6666
#define BUF_SIZE 1024

int main() {
    int sockfd; //Dateideskriptor des Sockets (ein Integer)
    struct sockaddr_in server_addr;//IPv4-Adresse + Port des Servers
    char buf[BUF_SIZE];//Platz für eingehende Daten
    ssize_t bytes_sent, bytes_received;//für Rückgaben von send/recv (kann -1 bei Fehlern sein)

    //Socket erstellen (vgl mit server nichtnochmal detailiert hier)
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    // Server-Adresse konfigurieren
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);

    /*inet_pton: wandelt die IP-Adresse aus Text ("127.0.0.1") in die binäre Form,
    die sin_addr erwartet. Rückgabe: 1 = Erfolg, 0 = ungültige Darstellung,
    -1 = Fehler (errno gesetzt).
    Typische Fehler:
    inet_pton liefert 0 → IP-String falsch formatiert.
    htons wird oft vergessen — führt zu Verbindungsproblemen (anderer Port).

    Quelle : man 3 inet_pton*/
    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
        perror("inet_pton");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    // Verbindung zum Server herstellen (man 2 connect)
    if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        perror("connect");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
    /*baut eine TCP-Verbindung zum einem socket (hier den Server) auf*/


    printf("Mit Server %s:%d verbunden.\n", SERVER_IP, SERVER_PORT);

    // Initiale Nachricht senden (man 2 send)
    const char *message = "Hallo vom Client!";
    bytes_sent = send(sockfd, message, strlen(message), 0);
    /*Erklärung

send() schreibt Bytes in den TCP-Ausgangspuffer.

Rückgabe: Anzahl gesendeter Bytes (kann < strlen(message) sein!).

Flags=0 bedeutet Standardverhalten.

Wichtig: Partial sends

TCP ist streambasiert; send() kann weniger Bytes übertragen als verlangt. Robuster Code überprüft bytes_sent und sendet in einer Schleife weiter, bis alles gesendet ist.

Für kurze Nachrichten < 1 KB ist send() oft vollständig, aber man darf das nicht voraussetzen.*/
    if (bytes_sent == -1) {
        perror("send");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("Nachricht gesendet: %s\n", message);

    // Antwort/reaktion vom Server empfangen
    bytes_received = recv(sockfd, buf, BUF_SIZE - 1, 0);
    /*Erklärung

recv() liest maximal BUF_SIZE-1 Bytes und liefert die tatsächliche Anzahl bytes_received.

Wenn bytes_received == 0: der Peer hat die Verbindung ordentlich geschlossen (EOF).

Wenn bytes_received == -1: Fehler (z.B. EINTR, ECONNRESET).

Auch hier gilt: recv() kann weniger Daten liefern als gesendet — bis zum gewünschten Terminator (z.B. \n) ggf. in einer Schleife lesen.

Man

man 2 recv*/
    if (bytes_received == -1) {
        perror("recv");
    } else {
        buf[bytes_received] = '\0';
        printf("Antwort vom Server: %s\n", buf);
    }

    char input[BUF_SIZE];
    while (1) {
        printf("Nachricht an Server (oder 'quit' zum Beenden): ");
        if (!fgets(input, sizeof(input), stdin))
            break;

        // Zeilenumbruch entfernen
        input[strcspn(input, "\n")] = '\0';

        if (strcmp(input, "quit") == 0)
            break;

        // Nachricht senden
        if (send(sockfd, input, strlen(input), 0) == -1) {
            perror("send");
            break;
        }

        // Antwort empfangen
        ssize_t n = recv(sockfd, buf, BUF_SIZE - 1, 0);
        if (n <= 0)
            break;
        buf[n] = '\0';
        printf("Antwort: %s\n", buf);
    }

    // Verbindung schließen
    close(sockfd);
    return 0;
}
