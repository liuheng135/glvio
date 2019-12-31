#include "msgque.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <sys/msg.h>
#include <sys/types.h>
#include <sys/sysinfo.h>

static int g_msgque = -1;


int msgque_init(void)
{
    g_msgque = msgget((key_t)1234, 0666 | IPC_CREAT);
    if(g_msgque < 0) {
        fprintf(stderr, "%s(): msgget() failed: %s\n", __func__, strerror(errno));
        return -1;
    }
    return 0;

}

int msgque_send(struct msgque_flow_info_s *msg)
{
    int ret = msgsnd(g_msgque, msg, sizeof(struct msgque_flow_info_s), IPC_NOWAIT);
    if(ret < 0) {
         printf("msg send failed:%d\r\n",ret);
    }
}