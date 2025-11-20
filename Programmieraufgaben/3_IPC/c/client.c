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

static void send_cmd(int sfd, const char *cmd)
{
    uint32_t len = strlen(cmd);
    uint32_t net = htonl(len);

    send(sfd, &net, sizeof(net), 0);
    send(sfd, cmd, len, 0);

    uint32_t rlen_net;
    recv(sfd, &rlen_net, sizeof(rlen_net), MSG_WAITALL);
    uint32_t rlen = ntohl(rlen_net);

    char *resp = malloc(rlen+1);
    recv(sfd, resp, rlen, MSG_WAITALL);
    resp[rlen] = '\0';

    printf("%s\n", resp);
    free(resp);
}

int main(void) {
    int sfd = socket(AF_UNIX, SOCK_STREAM, 0);

    struct sockaddr_un addr = {0};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCK_PATH, sizeof(addr.sun_path)-1);

    connect(sfd, (struct sockaddr*)&addr, sizeof(addr));

    printf("Connected. Commands:\n");
    printf("  ADD:<text>\n");
    printf("  LIST\n");
    printf("  CLEAR\n");
    printf("  quit\n\n");

    char buf[512];
    while (1) {
        printf("> ");
        if (!fgets(buf, sizeof(buf), stdin)) break;
        buf[strcspn(buf, "\n")] = 0;

        if (strcmp(buf, "quit") == 0) {
            uint32_t z = 0;
            send(sfd, &z, sizeof(z), 0);
            break;
        }

        send_cmd(sfd, buf);
    }

    close(sfd);
    return 0;
}