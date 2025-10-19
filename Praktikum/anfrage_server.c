/*
 anfrage_server.c
 Lauscht auf TCP-Port, nimmt Anfragen entgegen und antwortet darauf.
 */

#include <stdio.h> //Für Ein-/Ausgabe-Funktionen (z.B. printf, perror)
#include <stdlib.h> //Für allgemeine Hilfsfunktionen (z.B. exit, malloc)
#include <string.h> //Für Stringoperationen (memset, strlen)
#include <unistd.h> //Für POSIX-Funktionen wie close()
#include <sys/types.h> //Für socketbezogene Systemaufrufe (socket(), bind(), listen(), accept(), send(), recv())
#include <sys/socket.h> //Für socketbezogene Systemaufrufe (socket(), bind(), listen(), accept(), send(), recv())
#include <netinet/in.h> //Für Internet-Adressenstrukturen (sockaddr_in) und Funktionen wie htons().
#include <arpa/inet.h> // Für Funktionen zur Umwandlung von IP-Adressen (z.B. inet_ntoa() zum Drucken der Client-IP).

#define PORT 6666 //Port für den Server (port numbers below1024 are called privileged ports, deswegen hier höhre nummer)
#define BACKLOG 5 // Max anzahl an Verbindungsanfragen die Gleichzeitig in Warteschlange sein können
#define BUF_SIZE 1024 //Puffergröße zum Lesen der daten vom client


int main(){
    //var definitionen
    int sockfd, new_fd;//Dateideskriptor des Sockets (ein Integer)
    struct sockaddr_in server_addr, client_addr;//IPv4-Adresse + Port des Servers
    socklen_t sin_size;
    char buf[BUF_SIZE];//Platz für eingehende Daten
    int num_bytes;

    //Socket erstellen (man socket) (erzeugt den Kommunikationsendpunkt)
    sockfd = socket(AF_INET, SOCK_STREAM, 0);/*  socket() erzeugt neuen socket
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
    if (sockfd==-1){//fehler beim erstellen
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
   if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        perror("bind");
        close(sockfd);
        exit(EXIT_FAILURE);
    }//dindet Socket an IP adresse(liefert fehler wenn Port schon belegt)
    
    // listen for conection (man listen)
    if (listen(sockfd, BACKLOG) == -1) {
        perror("listen");
        close(sockfd);
        exit(EXIT_FAILURE);
    }//Socket hört auf eingehende Verbindungen. BACKLOG ist die maximale Warteschlangenlänge

    printf("Server wartet auf Verbindungen auf Port %d...\n", PORT);

    while (1) {
        // auf verbindung warten
        sin_size = sizeof(client_addr);
        // accept a conection(man accept)
        new_fd = accept(sockfd, (struct sockaddr*)&client_addr, &sin_size);
        /*accept wartet auf eingehende verbindung
        Gibt einen neuen Socket new_fd zurück, der für die Kommunikation mit dem Client verwendet wird.
        Die Adresse des Clients wird in client_addr gespeichert.
        */
        if (new_fd == -1) {
            perror("accept");
            continue;
        }
        
        printf("Verbindung von %s\n", inet_ntoa(client_addr.sin_addr));
        //Schleife für wiederholte Kommunikation mit dem Client
        while (1) {
            //Daten empfangen (man recv)
            num_bytes = recv(new_fd, buf, BUF_SIZE - 1, 0);//empfängt die nachicht vom socket(client)
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

            buf[num_bytes] = '\0';  // String terminieren
            printf("Nachricht vom Client: %s\n", buf);

            // Wenn der Client "quit" sendet → Verbindung beenden
            if (strcmp(buf, "quit") == 0) {
                printf("Client %s hat 'quit' gesendet. Verbindung wird geschlossen.\n", inet_ntoa(client_addr.sin_addr));
                break;
            }

            //Antwort an cliet senden (man send)
            const char *response = "Nachricht erhalten\n";
                if (send(new_fd, response, strlen(response), 0) == -1) {
                    perror("send");
            }
        }
        // Verbindung vom client schließen
        close(new_fd);
    }

    close(sockfd);
    return 0;
}