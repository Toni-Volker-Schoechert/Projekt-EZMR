/*
anfrage_peer.c:
 Dieses Programm vereint die Funktionalität eines Clients und Servers in einer einzigen Anwendung
 und stellt damit eine einfache Peer-to-Peer (P2P) Kommunikationsstruktur bereit.
 Jede Instanz kann:
   - eingehende TCP-Verbindungen annehmen (Server-Funktion)
   - aktiv eine Verbindung zu einem anderen Peer herstellen (Client-Funktion)
   - Nachrichten über bestehende Verbindungen senden und empfangen und speichern
 
 Damit wird die Trennung zwischen Server- und Client-Anwendung aufgehoben
 Kommunikation erfolgt über TCP-Sockets.
  
 Zusätzlich verwaltet jeder Peer eine lokale Nachrichtenliste, die mit anderen Peers geteilt,
 abgefragt oder manipuliert werden kann.
 Clients/andere Peers können per Textbefehlen die Liste lesen und manipulieren.
 Funktionen die vom client genutzt werden:
    ADD <Text>       - fügt <Text> als neue Nachricht hinzu
    LIST             - gibt alle Nachrichten (index: text) zurück
    DELETE <id>      - löscht Nachricht mit Nummer <id> (1-basiert)
    MOVE <from> <to> - verschiebt Nachricht von Index from nach to (1-basiert)
    QUIT             - beendet die Verbindung

 Bedienung der lokalen CLI (Main Thread):
  connect <ip> <port>    - baut Verbindung zu Peer auf (erzeugt neuen connection-thread)
  peers                  - listet aktuelle Verbindungen (eingehend + ausgehend)
  send <idx> <COMMAND>   - sendet COMMAND an Peer Nummer idx (1-basiert)
  broadcast <COMMAND>    - sendet COMMAND an alle Peers
  local_list             - zeigt lokale Nachrichtenliste
  help                   - Hilfetext
  exit                   - beendet diesen Peer (schließt alle Verbindungen)
 
 * Wichtige Hinweise:
 * - Diese Implementierung ist demonstrativ: für einfache P2P-Effekte.
 * - Die Nachrichtenliste ist mit einem mutex geschützt (pthread_mutex).
 * - Jeder Peer-Connection hat einen eigenen Handler-Thread.
 *
 * Relevante Man-Pages:
 *   man 2 socket, bind, listen, accept, connect, recv, send, close
 *   man 3 inet_pton, inet_ntop, htons
 *   man 3 pthread_create, pthread_mutex_init, pthread_mutex_lock, pthread_mutex_unlock
 *   Beej's Guide: https://beej.us/guide/bgnet/ 
*/

#include <stdio.h> //Für Ein-/Ausgabe-Funktionen (z.B. printf, perror)
#include <stdlib.h> //Für allgemeine Hilfsfunktionen (z.B. exit, malloc)
#include <string.h> //Für Stringoperationen (memset, strlen)
#include <unistd.h> //Für POSIX-Funktionen wie close()
#include <sys/types.h>
#include <sys/socket.h> //Für socketbezogene Systemaufrufe (socket(), bind(), listen(), accept(), send(), recv())
#include <netinet/in.h> //Für Internet-Adressenstrukturen (sockaddr_in) und Funktionen wie htons().
#include <arpa/inet.h> // Für Funktionen zur Umwandlung von IP-Adressen (z.B. inet_ntoa() zum Drucken der Client-IP).
#include <limits.h>  // Enthält INT_MIN, INT_MAX
#include <pthread.h> //für Multithreading benötigt
#include <ctype.h>


#define DEFAULT_PORT 6666 //Default Port für den Peer
#define BACKLOG 5 // Max anzahl an Verbindungsanfragen die Gleichzeitig in Warteschlange sein können
#define BUF_SIZE 1024 //Puffergröße zum Lesen der daten vom client
#define MAX_MSGS 200 //Maximale anzahl an Nachichten die Gespeichert werden
#define MAX_PEERS 10


static char messages[MAX_MSGS][BUF_SIZE]; // Array alsGlobale Nachrichtenliste
static int msg_count = 0;//Anzahl der Nachichten
static pthread_mutex_t msg_mutex = PTHREAD_MUTEX_INITIALIZER; // schützt alle Operationen, die messages/msg_count lesen oder schreiben
//man pthread_mutex_lock, man pthread_mutex_unlock

/*Peer-Verbindungs-Liste:
 speichert für jede aktive Verbindung FD, Remote-Addr, Port und Thread
  So kann später gesehen werden welche Verbindungen aktiv sind, und gezielt an einen Peer senden*/
typedef struct {
    int fd;
    char addr[INET_ADDRSTRLEN];
    int port;
    pthread_t tid;
    int active;
} peer_conn_t;

static peer_conn_t peers[MAX_PEERS];
static pthread_mutex_t peers_mutex = PTHREAD_MUTEX_INITIALIZER;

int peer_sock = -1; // Aktive Verbindung (falls verbunden)
int listen_port;    // Port, auf dem dieser Peer lauscht

//_____________________
/*Funktionen Listen Operationen:
Core-Funktionen für die Liste
add_message, list_messages_out, delete_message, move_message — alle sperren den mutex, arbeiten und geben ihn wieder frei.
list_messages_out baut eine große Ausgabe, die dem anfragenden Peer zurückgesendet werden kann.
Man-Pages / Konzepte: String-Funktionen: man 3 strncpy, man 3 snprintf, man 3 strncat.
*/
int add_message(const char *msg) {
    /* add_message: hängt msg an die Liste an, wenn nicht Max Anzahl an Nachichten
    bereits erreicht, und gibt zurück die wievielte Nachicht hinzugefügt wurde*/
    pthread_mutex_lock(&msg_mutex);
    if (msg_count >= MAX_MSGS) {
        pthread_mutex_unlock(&msg_mutex);
        return -1;
    }
    strncpy(messages[msg_count], msg, BUF_SIZE - 1);
    messages[msg_count][BUF_SIZE - 1] = '\0';
    msg_count++;
    pthread_mutex_unlock(&msg_mutex);
    return msg_count;
}

void list_messages(char *out, size_t out_size) {
/* list_messages: Gibt Nachichten Verlauf der Liste aus */
    pthread_mutex_lock(&msg_mutex);
    if (msg_count == 0) {
        snprintf(out, out_size, "Keine Nachrichten gespeichert.\n");
        pthread_mutex_unlock(&msg_mutex);
        return;
    }
    out[0] = '\0';
    for (int i = 0; i < msg_count; ++i) {
        char line[BUF_SIZE + 32];
        snprintf(line, sizeof(line), "%d: %s\n", i + 1, messages[i]);
        strncat(out, line, out_size - strlen(out) - 1);
    }
    pthread_mutex_unlock(&msg_mutex);
}


int delete_message(int id) {
/* delete_message: löscht Nachricht mit id; gibt 0 bei Erfolg, -1 bei Fehler */
    pthread_mutex_lock(&msg_mutex);
    if (id < 1 || id > msg_count) {
        pthread_mutex_unlock(&msg_mutex);
        return -1;
    }
    int idx = id - 1;
    for (int i = idx; i < msg_count - 1; ++i) {
        strncpy(messages[i], messages[i + 1], BUF_SIZE);
    }
    msg_count--;
    pthread_mutex_unlock(&msg_mutex);
    return 0;
}

int move_message(int from, int to) {
/* move_message: verschiebt Nachricht von Position 'from' nach 'to' in der Liste */
    pthread_mutex_lock(&msg_mutex);
    if (from < 1 || from > msg_count) { pthread_mutex_unlock(&msg_mutex); return -1; }
    if (to < 1) to = 1;
    if (to > msg_count) to = msg_count;
    int f = from - 1;
    int t = to - 1;
    if (f == t) { pthread_mutex_unlock(&msg_mutex); return 0; }

    char temp[BUF_SIZE];
    strncpy(temp, messages[f], BUF_SIZE);

    if (f < t) {
        for (int i = f; i < t; ++i) {
            strncpy(messages[i], messages[i + 1], BUF_SIZE);
        }
        strncpy(messages[t], temp, BUF_SIZE);
    } else {
        for (int i = f; i > t; --i) {
            strncpy(messages[i], messages[i - 1], BUF_SIZE);
        }
        strncpy(messages[t], temp, BUF_SIZE);
    }
    pthread_mutex_unlock(&msg_mutex);
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

// PEER Registrierung

int register_peer(int fd, const char *ip, int port, pthread_t tid) {
    /*register_peer: Regestriert den Peer und gibt nächste verfügbare ID, solange nicht 
    mehr als MAY_PEERS bereits aktiv sind*/
    pthread_mutex_lock(&peers_mutex);
    for (int i = 0; i < MAX_PEERS; ++i) {
        if (!peers[i].active) {
            peers[i].fd = fd;
            strncpy(peers[i].addr, ip ? ip : "unknown", sizeof(peers[i].addr)-1);
            peers[i].addr[sizeof(peers[i].addr)-1] = '\0';
            peers[i].port = port;
            peers[i].tid = tid;
            peers[i].active = 1;
            pthread_mutex_unlock(&peers_mutex);
            return i; // index
        }
    }
    pthread_mutex_unlock(&peers_mutex);
    return -1;
}

void unregister_peer_by_fd(int fd) {
    /*entfernt PEER wieder*/
    pthread_mutex_lock(&peers_mutex);
    for (int i = 0; i < MAX_PEERS; ++i) {
        if (peers[i].active && peers[i].fd == fd) {
            peers[i].active = 0;
            close(peers[i].fd);
            peers[i].fd = -1;
            break;
        }
    }
    pthread_mutex_unlock(&peers_mutex);
}

void print_peers(void) {
/*Gibt Alle aktiven Verbindungen aus als übersicht,
 damit man z.b einen bestimmten peer finden kann um ihn später geziehlt anzusprechen*/
    pthread_mutex_lock(&peers_mutex);
    printf("Aktive Peers:\n");
    for (int i = 0; i < MAX_PEERS; ++i) {
        if (peers[i].active) {
            printf("  %d) %s:%d (fd=%d)\n", i+1, peers[i].addr, peers[i].port, peers[i].fd);
        }
    }
    pthread_mutex_unlock(&peers_mutex);
}

int send_to_peer_fd(int fd, const char *msg) {
/* send_to_peer_fd: sendet msg an fd, sorgt für vollständigen send */
    size_t left = strlen(msg);
    const char *p = msg;
    while (left > 0) {
        ssize_t s = send(fd, p, left, 0);
        if (s == -1) return -1;
        p += s;
        left -= s;
    }
    return 0;
}


void *connection_handler(void *arg) {
/**connection_handler: Connection Handler (für eingehende & ausgehende FDs)
 arg = pointer auf int der fd enthält; thread freed das arg  Für jede Verbindung empfäng in schleife  Nachichten und
 verarbeitet die Befehle (ADD LIST MOVE DELETE QUIT)
 ebenfalls sendet sie Antworten (mit send_to_peer_fd()) wiederholt bis alle Bytes Übertragen
 Wenn die Verbindung beendet wird (recv() liefert 0) oder ein Fehler auftritt,
 registriert die Funktion das Entfernen des Peers und endet**/
    int fd = *(int*)arg;
    free(arg);

    char buf[BUF_SIZE];
    struct sockaddr_in peer_sa;
    socklen_t sa_len = sizeof(peer_sa);
    if (getpeername(fd, (struct sockaddr*)&peer_sa, &sa_len) == 0) {
        char remote[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &peer_sa.sin_addr, remote, sizeof(remote));
        // update peer record addr if exists
        pthread_mutex_lock(&peers_mutex);
        for (int i = 0; i < MAX_PEERS; ++i) {
            if (peers[i].active && peers[i].fd == fd) {
                strncpy(peers[i].addr, remote, sizeof(peers[i].addr)-1);
                peers[i].port = ntohs(peer_sa.sin_port);
                break;
            }
        }
        pthread_mutex_unlock(&peers_mutex);
    }

    while (1) {
        ssize_t num_bytes = recv(fd, buf, sizeof(buf) - 1, 0);
        if (num_bytes == 0) {
                // Peer hat ordentlich geschlossen
                fprintf(stderr, "Peer fd=%d hat Verbindung beendet\n", fd);
                break;
        } else if (num_bytes < 0){
            perror("recv");
            break;
        }
        
        buf[num_bytes] = '\0';  // String beenden
        /* evtle zeilen ümbrüche werden gelöscht */
        while (num_bytes > 0 && (buf[num_bytes - 1] == '\n' || buf[num_bytes - 1] == '\r')) {
            buf[num_bytes - 1] = '\0'; 
            num_bytes--;
        }
        printf("[recv fd=%d] %s\n", fd, buf);

        // Befehle auswerten 
        //Nachicht Hinzufügen
            if (strncmp(buf, "ADD ", 4) == 0) {
                const char *msg = buf + 4;
                if (strlen(msg) == 0) {
                    const char *resp = "Fehler: leere Nachricht nicht erlaubt.\n";
                    send_to_peer_fd(fd, resp);
                    continue;
                }
                if (add_message(msg) == -1) {
                    const char *resp = "Fehler: Nachrichtenliste voll.\n";
                    send_to_peer_fd(fd, resp);
                } else {
                    const char *resp = "OK: Nachricht hinzugefügt.\n";
                    send_to_peer_fd(fd, resp);
                }
            }

            //Nachichten Liste an client senden
            else if (strncmp(buf, "LIST", 4) == 0) {
                char out[BUF_SIZE * 4];
                list_messages(out, sizeof(out));
                send_to_peer_fd(fd, out);
            }

            //Nachicht löschen
            else if (strncmp(buf, "DELETE ", 7) == 0) {
                char *arg = buf + 7;
                int id;
                if (parse_int_arg(arg, &id) == -1) {
                    const char *resp = "Fehler: DELETE erwartet eine Nummer (z.B. DELETE 2).\n";
                    send_to_peer_fd(fd, resp);
                } else {
                    if (delete_message(id) == 0) {
                        const char *resp = "OK: Nachricht gelöscht.\n";
                        send_to_peer_fd(fd, resp);
                    } else {
                        const char *resp = "Fehler: ungültige ID.\n";
                        send_to_peer_fd(fd, resp);
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
                    send_to_peer_fd(fd, resp);
                } else {
                    int from, to;
                    if (parse_int_arg(tok1, &from) == -1 || parse_int_arg(tok2, &to) == -1) {
                        const char *resp = "Fehler: MOVE-Argumente müssen Zahlen sein.\n";
                        send_to_peer_fd(fd, resp);
                    } else {
                        if (move_message(from, to) == 0) {
                            const char *resp = "OK: Nachricht verschoben.\n";
                            send_to_peer_fd(fd, resp);
                        } else {
                            const char *resp = "Fehler: ungültige Indizes für MOVE.\n";
                            send_to_peer_fd(fd, resp);
                        }
                    }
                }
            }

            //Verbindung zu Client Beenden
            else if (strncmp(buf, "QUIT", 4) == 0) {
                const char *resp = "OK: Verbindung wird beendet.\n";
                send_to_peer_fd(fd, resp);
                break;
            }

            //Fehler
            else {
                const char *resp = "Unbekannter Befehl. Verfügbar: ADD, LIST, DELETE, MOVE, QUIT\n";
                send_to_peer_fd(fd, resp);
            }
        } 

    // cleanup: unregister and close
    unregister_peer_by_fd(fd);
    return NULL;
}


void *listener_thread(void *arg) {
/* *listener_thread: arg = pointer auf int Port oder NULL für DEFAULT_PORT 
  Erstellt einen listen_fd mit socket(), bind() und listen() 
  In einer accept()-Schleife nimmt er eingehende Verbindungen an
  Für jede accept()-Verbindung erzeugt er einen handler-thread via 
  pthread_create(connection_handler, ...) und registriert die Peer-Verbindung.
  sihe man 2 socket, man 2 bind, man 2 listen, man 2 accept
  */
    int port = (arg ? *(int*)arg : DEFAULT_PORT);
    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd==-1){//fehler beim erstellen
        perror("listener: socket creation failed look at manpages(man socket)");
        return NULL;
    }

    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = htons(port);
    sa.sin_addr.s_addr = INADDR_ANY;

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(listen_fd, (struct sockaddr*)&sa, sizeof(sa)) == -1) {
        perror("listener: bind");
        close(listen_fd);
        return NULL;
    }

    if (listen(listen_fd, BACKLOG) == -1) {
        perror("listener: listen");
        close(listen_fd);
        return NULL;
    }

    printf("listener: warte auf Verbindungen auf Port %d...\n", port);

    while (1) {
        struct sockaddr_in cli;
        socklen_t cli_len = sizeof(cli);
        int client_fd = accept(listen_fd, (struct sockaddr*)&cli, &cli_len);
        if (client_fd == -1) {
            perror("listener: accept");
            continue;
        }

        char remote[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &cli.sin_addr, remote, sizeof(remote));
        int remote_port = ntohs(cli.sin_port);

        // handler thread erzeugen
        int *argfd = malloc(sizeof(int));
        if (!argfd) { close(client_fd); continue; }
        *argfd = client_fd;
        pthread_t tid;
        if (pthread_create(&tid, NULL, connection_handler, argfd) != 0) {
            perror("pthread_create");
            free(argfd);
            close(client_fd);
            continue;
        }
        // register peer
        register_peer(client_fd, remote, remote_port, tid);
    }

    close(listen_fd);
    return NULL;
}

int main(int argc, char *argv[]) {
    int port = DEFAULT_PORT;
    if (argc >= 2) {
        port = atoi(argv[1]);
    }

     // Start listener thread
    pthread_t ltid;
    int *parg = malloc(sizeof(int));
    *parg = port;
    if (pthread_create(&ltid, NULL, listener_thread, parg) != 0) {
        perror("pthread_create listener");
        free(parg);
        exit(EXIT_FAILURE);
    }

    printf("Peer gestartet. Lokaler Listener auf Port %d\n", port);
    printf("Befehle: connect <ip> <port> | peers | send <idx> <COMMAND> | broadcast <COMMAND> | local_list | help | exit\n");

    // Main thread: CLI für connect/send
    char line[BUF_SIZE];
    while (1) {
        printf("> ");
        if (!fgets(line, sizeof(line), stdin)) break;
        line[strcspn(line, "\n")] = '\0';
        if (strlen(line) == 0) continue;

        // tokenize command
        char *cmd = strtok(line, " ");
        if (!cmd) continue;

        if (strcmp(cmd, "help") == 0) { //Hilfe für Eingabe
            printf("connect <ip> <port>    - Verbindung zu Peer herstellen\n");
            printf("peers                  - Liste aktiver Verbindungen\n");
            printf("send <idx> <COMMAND>   - COMMAND (z.B. ADD Hi) an Peer idx senden\n");
            printf("broadcast <COMMAND>    - COMMAND an alle Peers senden\n");
            printf("<COMMAND>: \n");
            printf("\t ADD <Text>       - fügt <Text> als neue Nachricht hinzu\n");
            printf("\t LIST             - gibt alle Nachrichten (index: text) zurück\n");
            printf("\t DELETE <id>      - löscht Nachricht mit Nummer <id> (1-basiert)\n");
            printf("\t MOVE <from> <to> - verschiebt Nachricht von Index from nach to (1-basiert)\n");
            printf("\t QUIT             - beendet die Verbindung\n");
            printf("local_list             - zeige lokale Nachrichtenliste\n");
            printf("exit                   - beende Peer (schließt alle Verbindungen)\n");
        }

        else if (strcmp(cmd, "connect") == 0) { /*baut eine ausgehende Verbindung auf (socket + connect),
            erstellt ebenfalls einen handler-thread (so werden ausgehende Verbindungen gleich 
            behandelt wie eingehende).*/
            char *ip = strtok(NULL, " ");
            char *port_s = strtok(NULL, " ");
            if (!ip || !port_s) { printf("Usage: connect <ip> <port>\n"); continue; }
            int portn;
            if (parse_int_arg(port_s, &portn) == -1) { printf("Ungültiger Port\n"); continue; }

            int fd = socket(AF_INET, SOCK_STREAM, 0);
            if (fd == -1) { perror("socket"); continue; }

            struct sockaddr_in sa;
            memset(&sa, 0, sizeof(sa));
            sa.sin_family = AF_INET;
            sa.sin_port = htons(portn);
            if (inet_pton(AF_INET, ip, &sa.sin_addr) <= 0) {
                perror("inet_pton");
                close(fd);
                continue;
            }

            if (connect(fd, (struct sockaddr*)&sa, sizeof(sa)) == -1) {
                perror("connect");
                close(fd);
                continue;
            }

            // spawn handler thread for outgoing connection
            int *argfd = malloc(sizeof(int));
            if (!argfd) { close(fd); continue; }
            *argfd = fd;
            pthread_t tid;
            if (pthread_create(&tid, NULL, connection_handler, argfd) != 0) {
                perror("pthread_create");
                free(argfd);
                close(fd);
                continue;
            }
            register_peer(fd, ip, portn, tid);
            printf("Verbunden zu %s:%d (fd=%d)\n", ip, portn, fd);
        }

        else if (strcmp(cmd, "peers") == 0) {//Aktiv verbundene PEERS Anzeigen
            print_peers();
        }

        else if (strcmp(cmd, "send") == 0) { //Nachicht/Befehl an einen PEER Senden
            char *idxs = strtok(NULL, " ");
            char *rest = strtok(NULL, ""); // rest der Zeile als COMMAND
            if (!idxs || !rest) { printf("Usage: send <idx> <COMMAND>\n"); continue; }
            int idx;
            if (parse_int_arg(idxs, &idx) == -1) { printf("Ungültiger Index\n"); continue; }
            int fd = -1;
            pthread_mutex_lock(&peers_mutex);
            if (idx >= 1 && idx <= MAX_PEERS && peers[idx-1].active) fd = peers[idx-1].fd;
            pthread_mutex_unlock(&peers_mutex);
            if (fd == -1) { printf("Peer nicht gefunden\n"); continue; }
            // append newline for remote readability
            char msg[BUF_SIZE];
            snprintf(msg, sizeof(msg), "%s\n", rest);
            if (send_to_peer_fd(fd, msg) == -1) {
                perror("send");
            } else {
                // read response - blocking recv
                char resp[BUF_SIZE * 3];
                ssize_t rn = recv(fd, resp, sizeof(resp)-1, 0);
                if (rn <= 0) {
                    if (rn == 0) printf("Peer hat Verbindung geschlossen\n");
                    else perror("recv");
                    unregister_peer_by_fd(fd);
                } else {
                    resp[rn] = '\0';
                    printf("%s", resp);
                }
            }
        }

        else if (strcmp(cmd, "broadcast") == 0) { //Nachicht/Befehl an alle PEER Senden
            char *rest = strtok(NULL, "");
            if (!rest) { printf("Usage: broadcast <COMMAND>\n"); continue; }
            char msg[BUF_SIZE];
            snprintf(msg, sizeof(msg), "%s\n", rest);
            pthread_mutex_lock(&peers_mutex);
            for (int i = 0; i < MAX_PEERS; ++i) {
                if (peers[i].active) {
                    if (send_to_peer_fd(peers[i].fd, msg) == -1) {
                        perror("send broadcast");
                    } else {
                        // try to read immediate response (best-effort)
                        char resp[BUF_SIZE * 2];
                        ssize_t rn = recv(peers[i].fd, resp, sizeof(resp)-1, MSG_DONTWAIT);
                        if (rn > 0) {
                            resp[rn] = '\0';
                            printf("Antwort von %s:%d -> %s", peers[i].addr, peers[i].port, resp);
                        }
                    }
                }
            }
            pthread_mutex_unlock(&peers_mutex);
        }
        else if (strcmp(cmd, "local_list") == 0) { //Nachichten Liste ausgeben
            char out[BUF_SIZE * 4];
            list_messages(out, sizeof(out));
            printf("%s", out);
        }

        else if (strcmp(cmd, "exit") == 0) { //Peer beenden
            printf("Beende Peer: schliesse Verbindungen...\n");
            // close all peer fds
            pthread_mutex_lock(&peers_mutex);
            for (int i = 0; i < MAX_PEERS; ++i) {
                if (peers[i].active) {
                    close(peers[i].fd);
                    peers[i].active = 0;
                }
            }
            pthread_mutex_unlock(&peers_mutex);
            break;
        }
        else {
            printf("Unbekannter Befehl. Tippe 'help' für Hilfe.\n");
        }
    }
    return 0;
}