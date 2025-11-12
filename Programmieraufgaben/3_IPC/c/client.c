/* client.c
 * Simple client for the System V message queue server
 * Usage: ./client ECHO "Hello"   -> will send "ECHO Hello"
 *        ./client TIME
 *        ./client PING
 *
 * The client sends a request with mtype=1 and waits for a reply with mtype=getpid().
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <time.h>

#define FTOK_PATH "/tmp"
#define FTOK_PROJ_ID 'S'
#define MAXTEXT 512

struct msgbuf {
    long mtype;
    char mtext[MAXTEXT];
};

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s COMMAND [ARGS...]\nCommands: ECHO <text> | TIME | PING\n", argv[0]);
        return 1;
    }

    key_t key = ftok(FTOK_PATH, FTOK_PROJ_ID);
    if (key == -1) {
        perror("ftok");
        return 1;
    }

    int msgid = msgget(key, 0600);
    if (msgid == -1) {
        perror("msgget (is the server running?)");
        return 1;
    }

    struct msgbuf req, resp;
    memset(&req, 0, sizeof(req));
    pid_t pid = getpid();

    /* Build command string */
    char cmd[MAXTEXT];
    if (strcmp(argv[1], "ECHO") == 0) {
        if (argc < 3) {
            fprintf(stderr, "ECHO requires text\n");
            return 1;
        }
        size_t off = 0;
        off += snprintf(cmd + off, sizeof(cmd) - off, "ECHO ");
        for (int i = 2; i < argc && off < sizeof(cmd)-1; ++i) {
            off += snprintf(cmd + off, sizeof(cmd) - off, "%s%s", (i==2) ? "" : " ", argv[i]);
        }
    } else {
        /* TIME, PING or custom single-token command */
        snprintf(cmd, sizeof(cmd), "%s", argv[1]);
    }

    /* Format mtext: "<pid>|<COMMAND>" */
    snprintf(req.mtext, sizeof(req.mtext), "%d|%s", (int)pid, cmd);
    req.mtype = 1; /* requests go to server with type 1 */

    if (msgsnd(msgid, &req, strlen(req.mtext)+1, 0) == -1) {
        perror("msgsnd (request)");
        return 1;
    }

    /* Wait for reply with mtype == pid */
    memset(&resp, 0, sizeof(resp));
    if (msgrcv(msgid, &resp, sizeof(resp.mtext), (long)pid, 0) == -1) {
        perror("msgrcv (response)");
        return 1;
    }

    printf("Client(%d): got response: %s\n", pid, resp.mtext);

    return 0;
}
