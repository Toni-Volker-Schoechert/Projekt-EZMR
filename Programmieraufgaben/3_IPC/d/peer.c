// peer.c 
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
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <arpa/inet.h>
#include <signal.h>

#define MAX_MSGS 100
#define MAX_MSG_LEN 256
#define SHM_KEY 0x12345
#define SEM_KEY 0x54321
#define SOCK_PATH "/tmp/ipc_example.sock"

/* ---------------- SHARED MEMORY ---------------- */

struct message_list {
    int count;
    char msgs[MAX_MSGS][MAX_MSG_LEN];
};

static int shm_id, sem_id;
static struct message_list *shm;

/* ---------------- SEMAPHORE HELPERS ---------------- */

static void sem_lock() {
    struct sembuf op = {0, -1, 0};
    semop(sem_id, &op, 1);
}

static void sem_unlock() {
    struct sembuf op = {0, +1, 0};
    semop(sem_id, &op, 1);
}

/* ---------------- COMMAND PROCESSOR ---------------- */

static void process_command(const char *cmd, char **response, size_t *resp_len)
{
    if (strncmp(cmd, "ADD:", 4) == 0) {

        const char *msg = cmd + 4;

        sem_lock();
        if (shm->count < MAX_MSGS) {
            strncpy(shm->msgs[shm->count], msg, MAX_MSG_LEN - 1);
            shm->msgs[shm->count][MAX_MSG_LEN - 1] = '\0';
            shm->count++;
            sem_unlock();
            *response = strdup("STATUS:OK:Message added");
        } else {
            sem_unlock();
            *response = strdup("STATUS:ERR:List full");
        }

    } else if (!strcmp(cmd, "LIST")) {

        sem_lock();
        size_t total = 20;
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

    } else if (!strcmp(cmd, "CLEAR")) {

        sem_lock();
        shm->count = 0;
        sem_unlock();

        *response = strdup("STATUS:OK:List cleared");

    } else {
        *response = strdup("STATUS:ERR:Unknown command");
    }

    *resp_len = strlen(*response);
}

/* ---------------- SERVER PART ---------------- */

static void *server_thread(void *arg)
{
    int sfd, cfd;
    struct sockaddr_un addr;

    unlink(SOCK_PATH);
    sfd = socket(AF_UNIX, SOCK_STREAM, 0);

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCK_PATH, sizeof(addr.sun_path) - 1);

    bind(sfd, (struct sockaddr *)&addr, sizeof(addr));
    listen(sfd, 5);

    printf("[Peer] Server thread ready at %s\n", SOCK_PATH);

    while (1) {
        cfd = accept(sfd, NULL, NULL);

        uint32_t netlen;
        if (recv(cfd, &netlen, sizeof(netlen), MSG_WAITALL) <= 0) {
            close(cfd);
            continue;
        }

        uint32_t len = ntohl(netlen);
        if (len == 0) {
            close(cfd);
            continue;
        }

        char *buf = malloc(len + 1);
        recv(cfd, buf, len, MSG_WAITALL);
        buf[len] = '\0';

        char *resp;
        size_t resp_len;
        process_command(buf, &resp, &resp_len);

        uint32_t rn = htonl(resp_len);
        send(cfd, &rn, sizeof(rn), 0);
        send(cfd, resp, resp_len, 0);

        free(buf);
        free(resp);
        close(cfd);
    }
}

/* ---------------- CLIENT HELPER ---------------- */

static void send_cmd(const char *cmd)
{
    int sfd = socket(AF_UNIX, SOCK_STREAM, 0);
    struct sockaddr_un addr = {0};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCK_PATH, sizeof(addr.sun_path) - 1);

    if (connect(sfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("connect");
        close(sfd);
        return;
    }

    uint32_t len = strlen(cmd);
    uint32_t net = htonl(len);

    send(sfd, &net, sizeof(net), 0);
    send(sfd, cmd, len, 0);

    uint32_t rlen_net;
    recv(sfd, &rlen_net, sizeof(rlen_net), MSG_WAITALL);
    uint32_t rlen = ntohl(rlen_net);

    char *resp = malloc(rlen + 1);
    recv(sfd, resp, rlen, MSG_WAITALL);
    resp[rlen] = '\0';

    printf("%s\n", resp);
    free(resp);
    close(sfd);
}

/* ---------------- MAIN: PEER PROGRAM ---------------- */

int main(void)
{
    /* Shared Memory */
    shm_id = shmget(SHM_KEY, sizeof(struct message_list), IPC_CREAT | 0600);
    shm = shmat(shm_id, NULL, 0);

    /* Semaphore */
    sem_id = semget(SEM_KEY, 1, IPC_CREAT | 0600);
    semctl(sem_id, 0, SETVAL, 1);

    /* Start server thread */
    pthread_t tid;
    pthread_create(&tid, NULL, server_thread, NULL);

    printf("Peer ready. Commands:\n");
    printf("  ADD:<text>\n");
    printf("  LIST\n");
    printf("  CLEAR\n");
    printf("  quit\n\n");

    /* Client loop */
    char buf[512];
    while (1) {
        printf("> ");
        if (!fgets(buf, sizeof(buf), stdin)) break;
        buf[strcspn(buf, "\n")] = 0;

        if (!strcmp(buf, "quit"))
            break;

        send_cmd(buf);
    }

    printf("Peer shutting down.\n");
    return 0;
}
