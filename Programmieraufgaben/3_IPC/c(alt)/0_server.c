/* server.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <unistd.h>

#define SHM_KEY_PATH "/tmp"
#define MSG_KEY_PATH "/tmp"
#define PROJ_ID  'S'
#define MAX_DATA 256
#define MAX_ENTRIES 256

/* Request */
typedef struct {
    long mtype;
    pid_t client_pid;
    char cmd;
    int index;
    char data[MAX_DATA];
} ipc_request_t;

/* Response */
typedef struct {
    long mtype;
    int status;
    char payload[512];
} ipc_response_t;

/* Shared list */
typedef struct {
    int count;
    int next_id;
    char entries[MAX_ENTRIES][MAX_DATA];
} shared_list_t;

/* semaphore helpers */
void sem_op(int semid, int op) {
    struct sembuf s = {0, op, 0};
    semop(semid, &s, 1);
}

int main() {
    key_t msgkey = ftok(MSG_KEY_PATH, PROJ_ID);
    key_t shmkey = ftok(SHM_KEY_PATH, PROJ_ID+1);
    key_t semkey = ftok(SHM_KEY_PATH, PROJ_ID+2);

    int msqid = msgget(msgkey, IPC_CREAT | 0666);
    int shmid = shmget(shmkey, sizeof(shared_list_t), IPC_CREAT | 0666);
    int semid = semget(semkey, 1, IPC_CREAT | 0666);

    /* init semaphore to 1 if newly created */
    union semun { int val; } su;
    su.val = 1;
    semctl(semid, 0, SETVAL, su);

    shared_list_t *list = (shared_list_t*) shmat(shmid, NULL, 0);
    /* init shared list if needed */
    if (list->next_id == 0 && list->count == 0) {
        list->count = 0;
        list->next_id = 1;
    }

    printf("Server ready. MSQID=%d SHMID=%d SEMID=%d\n", msqid, shmid, semid);

    ipc_request_t req;
    while (1) {
        /* receive request (any mtype) */
        if (msgrcv(msqid, &req, sizeof(req) - sizeof(long), 0, 0) < 0) {
            perror("msgrcv");
            break;
        }

        ipc_response_t resp;
        resp.mtype = req.client_pid;
        resp.status = 0;
        memset(resp.payload, 0, sizeof(resp.payload));

        if (req.cmd == 'L') { /* LIST */
            sem_op(semid, -1); /* P */
            int n = list->count;
            /* simple serialization: index:entry\n */
            char *p = resp.payload;
            for (int i=0;i<n && strlen(p) < sizeof(resp.payload)-100;i++) {
                snprintf(p + strlen(p), sizeof(resp.payload) - strlen(p), "%d:%s\n", i, list->entries[i]);
            }
            sem_op(semid, +1); /* V */
        } else if (req.cmd == 'P') { /* PUT */
            sem_op(semid, -1); /* P */
            if (list->count < MAX_ENTRIES) {
                strncpy(list->entries[list->count++], req.data, MAX_DATA-1);
                resp.status = 0;
                snprintf(resp.payload, sizeof(resp.payload), "OK index=%d", list->count-1);
            } else {
                resp.status = 1;
                snprintf(resp.payload, sizeof(resp.payload), "LIST FULL");
            }
            sem_op(semid, +1);
        } else if (req.cmd == 'G') { /* GET index */
            sem_op(semid, -1);
            if (req.index >=0 && req.index < list->count) {
                snprintf(resp.payload, sizeof(resp.payload), "%s", list->entries[req.index]);
            } else {
                resp.status = 2;
                snprintf(resp.payload, sizeof(resp.payload), "NO SUCH INDEX");
            }
            sem_op(semid, +1);
        } else if (req.cmd == 'D') { /* DEL index */
            sem_op(semid, -1);
            if (req.index >=0 && req.index < list->count) {
                /* simple: shift down */
                for (int i=req.index;i<list->count-1;i++) {
                    strcpy(list->entries[i], list->entries[i+1]);
                }
                list->count--;
                snprintf(resp.payload, sizeof(resp.payload), "DELETED %d", req.index);
            } else {
                resp.status = 3;
                snprintf(resp.payload, sizeof(resp.payload), "NO SUCH INDEX");
            }
            sem_op(semid, +1);
        } else if (req.cmd == 'Q') {
            snprintf(resp.payload, sizeof(resp.payload), "SERVER QUIT");
            msgsnd(msqid, &resp, sizeof(resp) - sizeof(long), 0);
            break;
        } else {
            resp.status = -1;
            snprintf(resp.payload, sizeof(resp.payload), "UNKNOWN CMD %c", req.cmd);
        }

        /* send response to client (mtype = client_pid) */
        if (msgsnd(msqid, &resp, sizeof(resp) - sizeof(long), 0) < 0) {
            perror("msgsnd resp");
        }
    }

    /* cleanup (optional: decide whether to remove IPC objects) */
    shmdt(list);
    /* keep IPC objects around so tests with ipcs can find them, or remove them:
       msgctl(msqid, IPC_RMID, NULL);
       shmctl(shmid, IPC_RMID, NULL);
       semctl(semid, 0, IPC_RMID);
    */
    return 0;
}
