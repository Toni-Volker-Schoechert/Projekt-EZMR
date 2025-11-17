
/* server.c
 * System V message queue server for Aufgabe a)
 * Compile: gcc -o server server.c
 *
 * Listens for requests on mtype=1; expects message text "<pid>|<COMMAND...>"
 * Replies to mtype = <pid> with "OK: ..." or "ERR: ..."
 *
 * Uses ftok("/tmp", 'S') to create a stable key.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <signal.h>
#include <unistd.h>

#define FTOK_PATH "/tmp"
#define FTOK_PROJ_ID 'S'
#define MAXTEXT 512

static int msgid = -1;

struct msgbuf {
    long mtype;
    char mtext[MAXTEXT];
};

void cleanup_and_exit(int sig) {
    if (msgid != -1) {
        if (msgctl(msgid, IPC_RMID, NULL) == 0) {
            printf("\nServer: message queue (id=%d) removed.\n", msgid);
        } else {
            perror("Server: msgctl(IPC_RMID)");
        }
    }
    _exit(0);
}

int main(void) {
    key_t key;
    struct msgbuf req, resp;
    ssize_t rcv_len;

    signal(SIGINT, cleanup_and_exit);
    signal(SIGTERM, cleanup_and_exit);

    key = ftok(FTOK_PATH, FTOK_PROJ_ID);
    if (key == -1) {
        perror("ftok");
        return 1;
    }

    /* Create (or get) message queue with read/write for owner */
    msgid = msgget(key, IPC_CREAT | 0600);
    if (msgid == -1) {
        perror("msgget");
        return 1;
    }

    printf("Server: message queue created/obtained. msgid=%d, key=0x%x\n", msgid, (unsigned)key);
    printf("Server: waiting for requests (mtype=1). Stop with Ctrl+C\n");

    while (1) {
        memset(&req, 0, sizeof(req));
        /* Receive requests with mtype=1 */
        rcv_len = msgrcv(msgid, &req, sizeof(req.mtext), 1, 0);
        if (rcv_len == -1) {
            if (errno == EINTR) continue; /* signal interrupted */
            perror("msgrcv");
            break;
        }

        /* req.mtext expected "<pid>|<COMMAND...>" */
        pid_t client_pid = 0;
        char command[MAXTEXT];
        command[0] = '\0';

        if (sscanf(req.mtext, "%d|%511[^\n]", &client_pid, command) < 1) {
            /* malformed */
            fprintf(stderr, "Server: malformed request: '%s'\n", req.mtext);
            continue;
        }

        printf("Server: request from pid=%d, command='%s'\n", client_pid, command);

        /* Simple command handling: support "ECHO <text>" and "TIME" and "PING" */
        resp.mtype = (long) client_pid;
        if (strncmp(command, "ECHO ", 5) == 0) {
            snprintf(resp.mtext, sizeof(resp.mtext), "OK: %s", command + 5);
        } else if (strcmp(command, "TIME") == 0) {
            char timestr[128];
            time_t t = time(NULL);
            struct tm *tm = localtime(&t);
            if (tm) {
                strftime(timestr, sizeof(timestr), "%Y-%m-%d %H:%M:%S", tm);
                snprintf(resp.mtext, sizeof(resp.mtext), "OK: %s", timestr);
            } else {
                snprintf(resp.mtext, sizeof(resp.mtext), "ERR: cannot get time");
            }
        } else if (strcmp(command, "PING") == 0) {
            snprintf(resp.mtext, sizeof(resp.mtext), "OK: PONG");
        } else if (strcmp(command, "") == 0) {
            snprintf(resp.mtext, sizeof(resp.mtext), "ERR: empty command");
        } else {
            snprintf(resp.mtext, sizeof(resp.mtext), "ERR: unknown command '%s'", command);
        }

        if (msgsnd(msgid, &resp, strlen(resp.mtext)+1, 0) == -1) {
            perror("msgsnd (response)");
        } else {
            printf("Server: replied to pid=%d: %s\n", client_pid, resp.mtext);
        }
    }

    cleanup_and_exit(0);
    return 0;
}