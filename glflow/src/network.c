#include "network.h"
#include "stdio.h"
#include "string.h"
#include "errno.h"

int network_init(struct net_data_s *dat,char *ip,int port)
{
    int flag = 0;
    if((dat->socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		return -1;
	}
	memcpy(dat->ip,ip,strlen(ip));
	dat->port = port;

	bzero(&dat->client_addr, sizeof(dat->client_addr));
	dat->client_addr.sin_family = AF_INET;
	dat->client_addr.sin_port = htons(dat->port);
	dat->client_addr.sin_addr.s_addr = htonl(INADDR_ANY) ;
	flag = fcntl(dat->socket_fd , F_GETFL , 0);
	fcntl(dat->socket_fd,F_SETFL,flag | O_NONBLOCK);
	
	if(bind(dat->socket_fd, (struct sockaddr *)&(dat->client_addr), sizeof(dat->client_addr))<0){
		return -2;
	}
	dat->if_connect = 0;
	return 0;
}

int network_read(struct net_data_s *dat, void *buffer, int size)
{
	int recvd;

	recvd = recvfrom(dat->socket_fd, buffer, size, 0,(struct sockaddr *)&dat->client_addr, (socklen_t*)&dat->client_addr_len);
	if(recvd > 0){
		if(dat->if_connect == 0){
			printf("[NET] Client connected\r\n");
			dat->if_connect  = 1;
		}
	}
    return recvd;
}

int network_write(struct net_data_s *dat, const void *buffer, int size)
{
	int ret;
	if(dat->if_connect > 0 ){
    	ret = sendto(dat->socket_fd,buffer, size, 0,(struct sockaddr *)&dat->client_addr, dat->client_addr_len);
		if(ret < 0){
			perror("send failed");
		}
        return ret;
	}else{
		return 0;
	}
}


struct net_data_s net0_data;
struct net_data_s net1_data;



