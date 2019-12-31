#ifndef _NETWORK_H_
#define _NETWORK_H_

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>


#define LW_UDP_PORT            9696
#define LW_UDP_IP              "192.168.100.1"

struct net_data_s{
    struct sockaddr_in client_addr;
    socklen_t client_addr_len;
    int socket_fd;
    int port;
    char ip[20];
    int if_connect;
};

int network_init(struct net_data_s *dat,char *ip,int port);
int network_read(struct net_data_s *dat, void *buffer, int size);
int network_write(struct net_data_s *dat, const void *buffer, int size);

#endif