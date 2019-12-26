#ifndef _UDP_H_
#define _UDP_H_

#ifdef __cplusplus 
extern "C" {
#endif

#include <sys/types.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>

struct udp_data_s{
    char ip[20];
    int port;
    int sockfd;
    struct sockaddr_in server;    
    socklen_t sender_addr_len;
};

int udp_init(struct udp_data_s *dat,char *ip,int port);
int udp_send(struct udp_data_s *dat,char *buf,int len);
int udp_recv(struct udp_data_s *dat,char *buf,int len);

#ifdef __cplusplus 
}
#endif

#endif