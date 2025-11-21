// client.c
#define _GNU_SOURCE
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <errno.h>

const char *SOCK_PATH = "/tmp/ipc_example.sock";

int main(void) {
    int sfd;
    struct sockaddr_un addr;

    // Socket erstellen
    if ((sfd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
        perror("socket");
        return 1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCK_PATH, sizeof(addr.sun_path)-1);

    // Verbindung zum Server
    if (connect(sfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("connect");
        close(sfd);
        return 1;
    }

    printf("Connected to server at %s\n", SOCK_PATH);
    printf("Type your messages. Empty line or 'quit' to exit.\n");

    char buf[1024];
    while (1) {
        printf("> ");
        if (!fgets(buf, sizeof(buf), stdin)) {
            break; // EOF
        }

        size_t len = strlen(buf);
        if (len > 0 && buf[len-1] == '\n') buf[len-1] = '\0'; // newline entfernen

        if (len == 0 || strcmp(buf, "quit") == 0) {
            // Zero-length PDU -> Server wird geschlossen
            uint32_t zero = 0;
            send(sfd, &zero, sizeof(zero), 0);
            break;
        }

        uint32_t netlen = htonl((uint32_t)strlen(buf));
        if (send(sfd, &netlen, sizeof(netlen), 0) != sizeof(netlen)) {
            perror("send length");
            break;
        }
        if (send(sfd, buf, strlen(buf), 0) != (ssize_t)strlen(buf)) {
            perror("send payload");
            break;
        }

        // Antwort empfangen
        uint32_t resp_len_net;
        if (recv(sfd, &resp_len_net, sizeof(resp_len_net), MSG_WAITALL) != sizeof(resp_len_net)) {
            perror("recv length");
            break;
        }

        uint32_t resp_len = ntohl(resp_len_net);
        char *resp = malloc(resp_len + 1);
        if (!resp) {
            perror("malloc");
            break;
        }

        if (recv(sfd, resp, resp_len, MSG_WAITALL) != (ssize_t)resp_len) {
            perror("recv payload");
            free(resp);
            break;
        }

        resp[resp_len] = '\0';
        printf("Server response: %s\n", resp);
        free(resp);
    }

    close(sfd);
    printf("Disconnected from server.\n");
    return 0;
}
