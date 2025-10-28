/*
 anfrage_server.c
 Lauscht auf TCP-Port, der eine Nachrichtenliste
 im Arbeitsspeicher verwaltet. 
 Clients können per Textbefehlen die Liste lesen und manipulieren.
 Funktionen die vom client genutzt werden:
    ADD <Text>       - fügt <Text> als neue Nachricht hinzu
    LIST             - gibt alle Nachrichten (index: text) zurück
    DELETE <id>      - löscht Nachricht mit Nummer <id> (1-basiert)
    MOVE <from> <to> - verschiebt Nachricht von Index from nach to (1-basiert)
    QUIT             - beendet die Verbindung

 * Hinweise:
  - Diese Implementierung hält Nachrichten im RAM (keine Persistenz).
  - Zur Einfachheit verarbeitet der Server jeweils einen Client
    komplett, bevor er den nächsten annimmt (sequenziell).
  - Für echte Parallelität müsste msg_list durch ein Mutex geschützt und
    jeder Client in einem Thread oder Prozess bearbeitet werden.
*/

#include <stdio.h> //Für Ein-/Ausgabe-Funktionen (z.B. printf, perror)
#include <stdlib.h> //Für allgemeine Hilfsfunktionen (z.B. exit, malloc)
#include <string.h> //Für Stringoperationen (memset, strlen)
#include <unistd.h> //Für POSIX-Funktionen wie close()
#include <sys/types.h> //Für socketbezogene Systemaufrufe (socket(), bind(), listen(), accept(), send(), recv())
#include <sys/socket.h> //Für socketbezogene Systemaufrufe (socket(), bind(), listen(), accept(), send(), recv())
#include <netinet/in.h> //Für Internet-Adressenstrukturen (sockaddr_in) und Funktionen wie htons().
#include <arpa/inet.h> // Für Funktionen zur Umwandlung von IP-Adressen (z.B. inet_ntoa() zum Drucken der Client-IP).
#include <limits.h>  // Enthält INT_MIN, INT_MAX

#define PORT 6666 //Port für den Server (port numbers below1024 are called privileged ports, deswegen hier höhre nummer)
#define BACKLOG 5 // Max anzahl an Verbindungsanfragen die Gleichzeitig in Warteschlange sein können
#define BUF_SIZE 1024 //Puffergröße zum Lesen der daten vom client
#define MAX_MSGS 200 //Maximale anzahl an Nachichten die Gespeichert werden


static char messages[MAX_MSGS][BUF_SIZE]; // Array alsGlobale Nachrichtenliste
static int msg_count = 0;//Anzahl der Nachichten

//_____________________
//Funktionen Listen Operationen:

int add_message(const char *msg) {
    /* add_message: hängt msg an die Liste an, wenn nicht Max Anzahl an Nachichten
    bereits erreicht, und gibt zurück die wievielte Nachicht hinzugefügt wurde*/
    if (msg_count >= MAX_MSGS) return -1;
    strncpy(messages[msg_count], msg, BUF_SIZE - 1);
    messages[msg_count][BUF_SIZE - 1] = '\0';
    msg_count++;
    return msg_count; 
}

void list_messages(char *out, size_t out_size) {
/* list_messages: Gibt Nachichten Verlauf der Liste aus */
    if (msg_count == 0) {
        snprintf(out, out_size, "Keine Nachrichten gespeichert.\n");
        return;
    }
    out[0] = '\0';
    for (int i = 0; i < msg_count; ++i) {
        char line[BUF_SIZE + 32];
        snprintf(line, sizeof(line), "%d: %s\n", i + 1, messages[i]);
        strncat(out, line, out_size - strlen(out) - 1);
    }
}


int delete_message(int id) {
/* delete_message: löscht Nachricht mit id; gibt 0 bei Erfolg, -1 bei Fehler */
    if (id < 1 || id > msg_count) return -1;
    int idx = id - 1;
    // Elemente nach idx nach links schieben
    for (int i = idx; i < msg_count - 1; ++i) {
        strncpy(messages[i], messages[i + 1], BUF_SIZE);
    }
    msg_count--;
    return 0;
}


int move_message(int from, int to) {
/* move_message: verschiebt Nachricht von Position 'from' nach 'to' in der Liste */
    if (from < 1 || from > msg_count) return -1;
    if (to < 1) to = 1;
    if (to > msg_count) to = msg_count;
    int f = from - 1;
    int t = to - 1;
    if (f == t) return 0; // nichts zu tun

    char temp[BUF_SIZE];
    strncpy(temp, messages[f], BUF_SIZE);

    if (f < t) {
        // links nach rechts verschieben
        for (int i = f; i < t; ++i) {
            strncpy(messages[i], messages[i + 1], BUF_SIZE);
        }
        strncpy(messages[t], temp, BUF_SIZE);
    } else { // f > t
        // rechts nach links verschieben
        for (int i = f; i > t; --i) {
            strncpy(messages[i], messages[i - 1], BUF_SIZE);
        }
        strncpy(messages[t], temp, BUF_SIZE);
    }
    return 0;
}

int parse_int_arg(const char *s, int *out) {
/* parse_int_arg: wandelt zahlen im msg str aus Befehl in int Zahl um 
Konvertierung von Zeichenketten in Ganzzahlen wird strtol() verwendet.
Um sicherzustellen, dass der konvertierte Wert in den Bereich eines int passt, werden die Konstanten INT_MIN und INT_MAX aus der Header-Datei <limits.h> verwendet.
Dies verhindert Über- oder Unterläufe beim Typumwandeln.*/
    char *end;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0') return -1;
    if (v < INT_MIN || v > INT_MAX) return -1;
    *out = (int)v;
    return 0;
}
//_____________________

int main(){
    //var definitionen
    int listen_fd = -1;   // FD (file descriptor) des Servers, der auf Verbindungen hört
    int client_fd = -1;   // FD für die Verbindung zu einem bestimmten Client
    //=-1 damit fehler wenn was schiefläuft
    struct sockaddr_in server_addr, client_addr;//IPv4-Adresse + Port des Servers
    socklen_t sin_size;
    char buf[BUF_SIZE];//Platz für eingehende Daten
    int num_bytes;

    //Socket erstellen (man socket) (erzeugt den Kommunikationsendpunkt)
    listen_fd = socket(AF_INET, SOCK_STREAM, 0);/*  socket() erzeugt neuen socket
    AF_INET     =   selectet comunication domain belongs to the IPv4 Intenet protocols comunication domain
    SOCK_STREAM =   selectet type which specifies comunication semantics is Sockstream which Provides sequenced, 
                    reliable, two-way, connection-based byte streams. An out-of-band data transmission  
                    mechanism may be supported.
    0           =   Der dritte parameter ist das verwendete protocol des sockets (standartprotockol TCP/IP)

    RETURN VALUE
       On  success,  a file descriptor for the new socket is returned.  On er‐
       ror, -1 is returned, and errno is set to indicate the error.

    Quelle und mehr infos sihe: man socket
    */
    if (listen_fd==-1){//fehler beim erstellen
        perror("socket creation failed look at manpages(man socket)");
        exit(EXIT_FAILURE);
    }

    //Serveradresse vorbereiten(sihe man 7 ip)
    memset(&server_addr, 0, sizeof(server_addr)); //leert den speicher der serveradresse (setzt auf 0)
    server_addr.sin_family = AF_INET; //always set to AF_INET(IP4 adresse)
    server_addr.sin_port = htons(PORT); //contains the port in network byte order
    server_addr.sin_addr.s_addr = INADDR_ANY;/*is  the IP host address.  The s_addr member of struct in_addr
       contains the host interface address in  network  byte  order. 
       INADDR_ANY (0.0.0.0) means any address for socket binding;
       indet den Server an alle verfügbaren Netzwerkschnittstellen (z.B. alle IPs, die die Maschine hat).*/

    //binded (man bind)
   if (bind(listen_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        perror("bind");
        close(listen_fd);
        exit(EXIT_FAILURE);
    }//dindet Socket an IP adresse(liefert fehler wenn Port schon belegt)
    
    // listen for conection (man listen)
    if (listen(listen_fd, BACKLOG) == -1) {
        perror("listen");
        close(listen_fd);
        exit(EXIT_FAILURE);
    }//Socket hört auf eingehende Verbindungen. BACKLOG ist die maximale Warteschlangenlänge

    printf("Server wartet auf Verbindungen auf Port %d...\n", PORT);

    while (1) {
        // auf verbindung warten
        sin_size = sizeof(client_addr);
        // accept a conection(man accept)
        client_fd = accept(listen_fd, (struct sockaddr*)&client_addr, &sin_size);
        /*accept wartet auf eingehende verbindung
        Gibt einen neuen Socket new_fd zurück, der für die Kommunikation mit dem Client verwendet wird.
        Die Adresse des Clients wird in client_addr gespeichert.
        */
        if (client_fd == -1) {
            perror("accept");
            continue;
        }
        
        printf("Verbindung von %s\n", inet_ntoa(client_addr.sin_addr));

        //Schleife für wiederholte Kommunikation mit einem Client
        while (1) {
            //Daten empfangen (man recv)
            num_bytes = recv(client_fd, buf, BUF_SIZE - 1, 0);//empfängt die nachicht vom socket(client)
            //empfangene Datenlänge wird in num_bytes gespeichert
                          
            /*
            recv() gibt 0 zurück, wenn der Client die Verbindung geschlossen hat.
            recv() gibt -1 zurück, wenn ein Fehler auftritt.
            In beiden Fällen verlassen wir die Schleife.
            */
            if (num_bytes == 0){
                printf("Client %s hat die Verbindung beendet.\n", inet_ntoa(client_addr.sin_addr));
                break;
            }
                else if (num_bytes == -1){
                perror("recv");
                break;
            }

            buf[num_bytes] = '\0';  // String beenden
            /* evtle zeilen ümbrüche werden gelöscht */
            while (num_bytes > 0 && (buf[num_bytes - 1] == '\n' || buf[num_bytes - 1] == '\r')) {
                buf[num_bytes - 1] = '\0'; 
                num_bytes--;
            }

            printf("Nachricht vom Client: %s\n", buf);

            // Befehle auswerten 
            //Nachicht Hinzufügen
            if (strncmp(buf, "ADD ", 4) == 0) {
                const char *msg = buf + 4;
                if (strlen(msg) == 0) {
                    const char *resp = "Fehler: leere Nachricht nicht erlaubt.\n";
                    send(client_fd, resp, strlen(resp), 0);
                    continue;
                }
                if (add_message(msg) == -1) {
                    const char *resp = "Fehler: Nachrichtenliste voll.\n";
                    send(client_fd, resp, strlen(resp), 0);
                } else {
                    const char *resp = "OK: Nachricht hinzugefügt.\n";
                    send(client_fd, resp, strlen(resp), 0);
                }
            }
            //Nachichten Liste an client senden
            else if (strncmp(buf, "LIST", 4) == 0) {
                char out[BUF_SIZE * 4];
                list_messages(out, sizeof(out));
                send(client_fd, out, strlen(out), 0);
            }
            //Nachicht löschen
            else if (strncmp(buf, "DELETE ", 7) == 0) {
                char *arg = buf + 7;
                int id;
                if (parse_int_arg(arg, &id) == -1) {
                    const char *resp = "Fehler: DELETE erwartet eine Nummer (z.B. DELETE 2).\n";
                    send(client_fd, resp, strlen(resp), 0);
                } else {
                    if (delete_message(id) == 0) {
                        const char *resp = "OK: Nachricht gelöscht.\n";
                        send(client_fd, resp, strlen(resp), 0);
                    } else {
                        const char *resp = "Fehler: ungültige ID.\n";
                        send(client_fd, resp, strlen(resp), 0);
                    }
                }
            }
            //Nachicht in Liste Verschieben
            else if (strncmp(buf, "MOVE ", 5) == 0) {
                /* Format: MOVE <from> <to> */
                char *p = buf + 5;
                char *tok1 = strtok(p, " ");
                char *tok2 = strtok(NULL, " ");
                if (!tok1 || !tok2) {
                    const char *resp = "Fehler: MOVE erwartet zwei Zahlen (z.B. MOVE 3 1).\n";
                    send(client_fd, resp, strlen(resp), 0);
                } else {
                    int from, to;
                    if (parse_int_arg(tok1, &from) == -1 || parse_int_arg(tok2, &to) == -1) {
                        const char *resp = "Fehler: MOVE-Argumente müssen Zahlen sein.\n";
                        send(client_fd, resp, strlen(resp), 0);
                    } else {
                        if (move_message(from, to) == 0) {
                            const char *resp = "OK: Nachricht verschoben.\n";
                            send(client_fd, resp, strlen(resp), 0);
                        } else {
                            const char *resp = "Fehler: ungültige Indizes für MOVE.\n";
                            send(client_fd, resp, strlen(resp), 0);
                        }
                    }
                }
            }
            //Verbindung zu Client Beenden
            else if (strncmp(buf, "QUIT", 4) == 0) {
                const char *resp = "OK: Verbindung wird beendet.\n";
                send(client_fd, resp, strlen(resp), 0);
                break;
            }
            //Fehler
            else {
                const char *resp = "Unbekannter Befehl. Verfügbar: ADD, LIST, DELETE, MOVE, QUIT\n";
                send(client_fd, resp, strlen(resp), 0);
            }
        } 
        // Verbindung vom client schließen
        close(client_fd);
        printf("Verbindung geschlossen.\n");
    }

    close(listen_fd);
    return 0;
}
