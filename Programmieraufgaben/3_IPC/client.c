/* client.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <unistd.h>

#define MSG_KEY_PATH "/tmp"
#define PROJ_ID 'S'

typedef struct {
    long mtype;
    pid_t client_pid;
    char cmd;
    int index;
    char data[256];
} ipc_request_t;

typedef struct {
    long mtype;
    int status;
    char payload[512];
} ipc_response_t;

int main(int argc,char **argv) {
    key_t key = ftok(MSG_KEY_PATH, PROJ_ID);
    int msqid = msgget(key, 0666);
    if (msqid < 0) { perror("msgget"); return 1; }

    ipc_request_t req;
    req.mtype = 1; /* server listens on mtype 1 */
    req.client_pid = getpid();

    /* Example: PUT "hello" */
    req.cmd = 'P';
    strcpy(req.data, "Hello from client");
    msgsnd(msqid, &req, sizeof(req) - sizeof(long), 0);

    /* wait for response (mtype = our pid) */
    ipc_response_t resp;
    msgrcv(msqid, &resp, sizeof(resp) - sizeof(long), getpid(), 0);
    printf("RESP status=%d payload=%s\n", resp.status, resp.payload);
    return 0;
}
