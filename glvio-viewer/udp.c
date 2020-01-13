#include "udp.h"
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>


int udp_init(struct udp_data_s *dat,char *ip,int port)
{
    int i;
    int flag = 0;

    for(i = 0;i < 20;i++){
        if(ip[i] != '\0'){
            dat->ip[i] = ip[i];
        }else{
            break;
        }
    }

    dat->port = port;
    
    dat->sockfd = socket( AF_INET, SOCK_DGRAM, 0 );
    if ( -1 == dat->sockfd ) {
        perror("failed to create socket\r\n");
        return -1;
    }

    memset( &dat->server, 0, sizeof( struct sockaddr_in ) );
    dat->server.sin_family = AF_INET;
    dat->server.sin_port = htons(dat->port);
    dat->server.sin_addr.s_addr = inet_addr(dat->ip);
    flag = fcntl(dat->sockfd , F_GETFL , 0);
	fcntl(dat->sockfd,F_SETFL,flag | O_NONBLOCK);
    return 0;
}

int udp_send(struct udp_data_s *dat,char *buf,int len)
{
    return sendto(dat->sockfd, buf, len, 0,(struct sockaddr *)&dat->server,  (socklen_t)sizeof(dat->server));
}

int udp_recv(struct udp_data_s *dat,char *buf,int len)
{
    return recvfrom(dat->sockfd, buf, len, 0,(struct sockaddr *)&dat->server, &dat->sender_addr_len);
}