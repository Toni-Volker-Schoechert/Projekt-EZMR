// client.c
#define _GNU_SOURCE
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <arpa/inet.h>

const char *SOCK_PATH = "/tmp/ipc_example.sock";

static int send_pdu_and_get_response(int sfd, const char *payload, uint32_t payload_len) {
    uint32_t n = htonl(payload_len);
    if (send(sfd, &n, sizeof(n), 0) != sizeof(n)) { perror("send len"); return -1; }
    if (payload_len > 0) {
        if (send(sfd, payload, payload_len, 0) != (ssize_t)payload_len) { perror("send payload"); return -1; }
    }

    // receive response header
    uint32_t netlen;
    if (recv(sfd, &netlen, sizeof(netlen), MSG_WAITALL) != sizeof(netlen)) { perror("recv resp len"); return -1; }
    uint32_t rlen = ntohl(netlen);
    char *buf = malloc(rlen + 1);
    if (!buf) { perror("malloc"); return -1; }
    if (recv(sfd, buf, rlen, MSG_WAITALL) != (ssize_t)rlen) { perror("recv resp payload"); free(buf); return -1; }
    buf[rlen] = '\0';
    printf("Response (%u): %s\n", rlen, buf);
    free(buf);
    return 0;
}

int main(int argc, char **argv) {
    int sfd;
    struct sockaddr_un addr;

    if ((sfd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) { perror("socket"); exit(1); }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCK_PATH, sizeof(addr.sun_path)-1);

    if (connect(sfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) { perror("connect"); close(sfd); exit(1); }

    const char *msg = (argc > 1) ? argv[1] : "Hello from client";
    send_pdu_and_get_response(sfd, msg, (uint32_t)strlen(msg));

    // Send zero-length PDU to indicate close (optional)
    uint32_t z = 0;
    send(sfd, &z, sizeof(z), 0);

    close(sfd);
    return 0;
}