#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "lwlink.h"

unsigned char recvBuf[12000] = { 0 };

int main(int argc, char *argv[])
{
    int i;
    int sockfd = -1;
    int recved_len;

    struct lwlink_data_handler_s link_handler;


    printf("viewer start...\r\n");

    sockfd = socket( AF_INET, SOCK_DGRAM, 0 );
    if ( -1 == sockfd ) {
        perror("failed to create socket\r\n");
        exit( -1 );
    }
    
    struct sockaddr_in server;    
    socklen_t sender_addr_len;

    memset( &server, 0, sizeof( struct sockaddr_in ) );
    server.sin_family = AF_INET;
    server.sin_port = htons(3366);
    server.sin_addr.s_addr = inet_addr( "192.168.0.1");
    
    printf("server connected,waiting for message\r\n");

    lwlink_data_handler_init(&link_handler,0x02);

    sendto(sockfd, "hello", 5, 0,(struct sockaddr *)&server,  (socklen_t)sizeof(server));
    sendto(sockfd, "hello", 5, 0,(struct sockaddr *)&server,  (socklen_t)sizeof(server));

    while(1) {
        recved_len =  recvfrom(sockfd, recvBuf, sizeof( recvBuf ), 0,&server, &sender_addr_len);
        if(recved_len > 0){
			for(i = 0; i < recved_len; i++){
                if(lwlink_data_handler_parse(&link_handler,recvBuf[i]) > 0){
                    printf("msg recvd,type is %d\r\n",lwlink_data_handler_get_type(&link_handler));
                }          
            }
        }
        usleep(20000);
    }

    close( sockfd );

    return 0;
}
