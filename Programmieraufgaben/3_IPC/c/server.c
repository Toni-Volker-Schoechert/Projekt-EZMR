// server.c — erweitert für Aufgabe C
#define _GNU_SOURCE
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <arpa/inet.h>
#include <signal.h>
#include <sys/wait.h>

#define MAX_MSGS 100
#define MAX_MSG_LEN 256

const char *SOCK_PATH = "/tmp/ipc_example.sock";

/* ---------------- SHM + SEMAPHORES ---------------- */

struct message_list {
    int count;
    char msgs[MAX_MSGS][MAX_MSG_LEN];
};

static int shm_id, sem_id;
static struct message_list *shm;

/* Semaphore helpers */
static void sem_lock() {
    struct sembuf op = {0, -1, 0};
    semop(sem_id, &op, 1);
}
static void sem_unlock() {
    struct sembuf op = {0, +1, 0};
    semop(sem_id, &op, 1);
}

/* ---------------------------------------------------- */

static void process_command(const char *cmd, char **response, size_t *resp_len)
{
    if (strncmp(cmd, "ADD:", 4) == 0) {
        const char *msg = cmd + 4;

        sem_lock();
        if (shm->count < MAX_MSGS) {
            strncpy(shm->msgs[shm->count], msg, MAX_MSG_LEN-1);
            shm->msgs[shm->count][MAX_MSG_LEN-1] = '\0';
            shm->count++;
            sem_unlock();

            *response = strdup("STATUS:OK:Message added");
        } else {
            sem_unlock();
            *response = strdup("STATUS:ERR:List full");
        }

    } else if (strcmp(cmd, "LIST") == 0) {

        sem_lock();
        size_t total = 20; // header
        for (int i = 0; i < shm->count; i++)
            total += strlen(shm->msgs[i]) + 2;

        char *out = malloc(total);
        strcpy(out, "STATUS:OK:\n");

        for (int i = 0; i < shm->count; i++) {
            strcat(out, shm->msgs[i]);
            strcat(out, "\n");
        }
        sem_unlock();

        *response = out;

    } else if (strcmp(cmd, "CLEAR") == 0) {

        sem_lock();
        shm->count = 0;
        sem_unlock();
        *response = strdup("STATUS:OK:List cleared");

    } else {
        *response = strdup("STATUS:ERR:Unknown command");
    }

    *resp_len = strlen(*response);
}


static void handle_client(int cfd) {
    while (1) {
        uint32_t netlen;
        ssize_t r = recv(cfd, &netlen, sizeof(netlen), MSG_WAITALL);
        if (r == 0) break;
        if (r < 0) { perror("recv"); break; }

        uint32_t n = ntohl(netlen);
        if (n == 0) break;

        char *buf = malloc(n+1);
        recv(cfd, buf, n, MSG_WAITALL);
        buf[n] = '\0';

        char *response;
        size_t resp_len;

        process_command(buf, &response, &resp_len);

        uint32_t resp_n = htonl(resp_len);
        send(cfd, &resp_n, sizeof(resp_n), 0);
        send(cfd, response, resp_len, 0);

        free(buf);
        free(response);
    }
    close(cfd);
}


static void sigchld_handler(int sig) {
    (void)sig;
    while (waitpid(-1, NULL, WNOHANG) > 0) {}
}


int main(void) {
    signal(SIGCHLD, sigchld_handler);

    /* ----- Shared Memory initialisieren ----- */
    shm_id = shmget(0x12345, sizeof(struct message_list), IPC_CREAT | 0600);
    shm = shmat(shm_id, NULL, 0);
    shm->count = 0;

    /* ----- Semaphore initialisieren ----- */
    sem_id = semget(0x54321, 1, IPC_CREAT | 0600);
    semctl(sem_id, 0, SETVAL, 1);

    /* ----- Socket einrichten ----- */
    int sfd, cfd;
    struct sockaddr_un addr;

    unlink(SOCK_PATH);
    sfd = socket(AF_UNIX, SOCK_STREAM, 0);

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCK_PATH, sizeof(addr.sun_path)-1);

    bind(sfd, (struct sockaddr*)&addr, sizeof(addr));
    listen(sfd, 5);

    printf("Server ready at %s\n", SOCK_PATH);

    for (;;) {
        cfd = accept(sfd, NULL, NULL);

        pid_t pid = fork();
        if (pid == 0) {
            close(sfd);
            handle_client(cfd);
            exit(0);
        }
        close(cfd);
    }

    return 0;
}