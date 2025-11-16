// server.c
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
#include <signal.h>

const char *SOCK_PATH = "/tmp/ipc_example.sock";

static void handle_client(int cfd) {
    while (1) {
        uint32_t netlen;
        ssize_t r = recv(cfd, &netlen, sizeof(netlen), MSG_WAITALL);
        if (r == 0) { /* EOF */ break; }
        if (r < 0) { perror("recv"); break; }
        if (r != sizeof(netlen)) { fprintf(stderr,"short read\n"); break; }

        uint32_t n = ntohl(netlen);
        if (n == 0) {
            // Zero-length PDU -> close
            break;
        }
        char *buf = malloc(n + 1);
        if (!buf) { perror("malloc"); break; }
        ssize_t got = recv(cfd, buf, n, MSG_WAITALL);
        if (got != (ssize_t)n) { perror("recv payload"); free(buf); break; }
        buf[n] = '\0';

        // Simple processing: echo with prefix and status
        char resp_prefix[] = "STATUS:OK:Echo:";
        size_t resp_len = strlen(resp_prefix) + n;
        char *resp = malloc(resp_len + 1);
        if (!resp) { perror("malloc resp"); free(buf); break; }
        memcpy(resp, resp_prefix, strlen(resp_prefix));
        memcpy(resp + strlen(resp_prefix), buf, n);
        resp[resp_len] = '\0';

        uint32_t resp_n = htonl((uint32_t)resp_len);
        if (send(cfd, &resp_n, sizeof(resp_n), 0) != sizeof(resp_n)) { perror("send len"); free(buf); free(resp); break; }
        if (send(cfd, resp, resp_len, 0) != (ssize_t)resp_len) { perror("send payload"); free(buf); free(resp); break; }

        free(buf);
        free(resp);
    }
    close(cfd);
}

static void sigchld_handler(int sig) {
    (void)sig;
    // Reap children
    while (waitpid(-1, NULL, WNOHANG) > 0) {}
}

int main(void) {
    int sfd, cfd;
    struct sockaddr_un addr;

    signal(SIGCHLD, sigchld_handler);
    unlink(SOCK_PATH);

    if ((sfd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
        perror("socket");
        exit(1);
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCK_PATH, sizeof(addr.sun_path)-1);

    if (bind(sfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sfd);
        exit(1);
    }

    if (listen(sfd, 16) < 0) {
        perror("listen");
        close(sfd);
        unlink(SOCK_PATH);
        exit(1);
    }

    printf("Server listening on %s\n", SOCK_PATH);

    for (;;) {
        if ((cfd = accept(sfd, NULL, NULL)) < 0) {
            if (errno == EINTR) continue;
            perror("accept");
            break;
        }

        pid_t pid = fork();
        if (pid < 0) {
            perror("fork");
            close(cfd);
        } else if (pid == 0) {
            // child
            close(sfd);
            handle_client(cfd);
            _exit(0);
        } else {
            // parent
            close(cfd);
        }
    }

    close(sfd);
    unlink(SOCK_PATH);
    return 0;
}