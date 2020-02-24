#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <string.h>
#include "hal_device.h"


struct net_data_s{
    struct sockaddr_in addr;
    int socket_fd;
    int addr_len;
    int port;
    char ip[20];
};


int network_init(struct hal_dev_s *dev)
{
    int flag = 0;
    struct net_data_s *dat = (struct net_data_s *)dev->priv_data;

    if((dat->socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		return -1;
	}
	
	bzero(&dat->addr, sizeof(dat->addr));
	dat->addr.sin_family = AF_INET;
	dat->addr.sin_port = htons(dat->port);
	//dat->addr.sin_addr.s_addr = inet_addr(dat->ip);
	dat->addr.sin_addr.s_addr = htonl(INADDR_ANY);
	flag = fcntl(dat->socket_fd , F_GETFL , 0);
	fcntl(dat->socket_fd,F_SETFL,flag | O_NONBLOCK);
	
	if(bind(dat->socket_fd, (struct sockaddr *)&(dat->addr), sizeof(dat->addr))<0){
		return -2;
	}

	return 0;
}

int network_open(struct hal_dev_s *dev,uint16_t oflag)
{
    return 0;
}

int network_read(struct hal_dev_s *dev, void *buffer, int size,int pos)
{
    struct net_data_s *dat = (struct net_data_s *)dev->priv_data;
    return recvfrom(dat->socket_fd, buffer, size, 0,(struct sockaddr *)&dat->addr, (socklen_t*)&dat->addr_len);   
}

int network_write(struct hal_dev_s *dev, const void *buffer, int size,int pos)
{
    struct net_data_s *dat = (struct net_data_s *)dev->priv_data;
    return sendto(dat->socket_fd,buffer, size, 0,(struct sockaddr *)&dat->addr, dat->addr_len);
}



#define AW_UDP_PORT            9696
#define AW_UDP_IP              "192.168.100.1"

#define LW_UDP_PORT            14550
#define LW_UDP_IP              "192.168.0.255"

struct net_data_s net0_data;
struct hal_dev_s  net0_dev;

struct net_data_s net1_data;
struct hal_dev_s  net1_dev;


int net_register(void)
{
	net0_dev.init  = network_init;
	net0_dev.open  = network_open;
	net0_dev.close = NULL;
	net0_dev.read  = network_read;
	net0_dev.write = network_write;
	net0_dev.ioctl = NULL;
	net0_dev.priv_data = &net0_data;

	//memcpy(net0_data.ip,AW_UDP_IP,sizeof(AW_UDP_IP));
	net0_data.port = AW_UDP_PORT;

	net1_dev.init  = network_init;
	net1_dev.open  = network_open;
	net1_dev.close = NULL;
	net1_dev.read  = network_read;
	net1_dev.write = network_write;
	net1_dev.ioctl = NULL;
	net1_dev.priv_data = &net1_data;

    memcpy(net1_data.ip,LW_UDP_IP,sizeof(LW_UDP_IP));
	net1_data.port = LW_UDP_PORT;

	//hal_dev_register(&net0_dev,"net0",HAL_O_RDWR);
	hal_dev_register(&net1_dev,"net1",HAL_O_RDWR);
	return 0;
}


