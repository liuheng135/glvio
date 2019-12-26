#include "lwlink.h"
#include "stdio.h"

int lwlink_msg_pack(struct lwlink_data_handler_s *handler,uint8_t type,uint8_t *data,uint32_t len)
{
    int i;
    
    handler->txbuf[0] = MSG_HEAD1;
    handler->txbuf[1] = MSG_HEAD2;
    handler->txbuf[2] = handler->id;
    handler->txbuf[3] = type;
    handler->txbuf[4] = (len & 0xff);
    handler->txbuf[5] = (len >> 8) & 0xff;
    
    for(i = 0; i < len; i++){
        handler->txbuf[i+6] = data[i];
    }
    
    handler->txbuf[len + 6] = 0;
    
    if(type >= 0x20){
        for(i = 0; i < len+3; i++){
            handler->txbuf[len+6] ^= handler->txbuf[i + 3];
        }
    }
    handler->txbuf[len + 7] = MSG_END1;
    handler->txbuf[len + 8] = MSG_END2;
    
    return len+9;
}

int lwlink_data_handler_init(struct lwlink_data_handler_s *handler,uint8_t id)
{
    //printf("AAA\r\n");
    handler->status = HANDLER_STATUS_WHEAD1;
    handler->id = id;
    handler->rxbuf_ptr = 0;
    //printf("BBB\r\n");
    return 0;
}

uint8_t lwlink_data_checksum_calc(struct lwlink_data_handler_s *handler)
{
    int16_t i;
    uint8_t checksum=0;
    uint16_t length =(int16_t)((uint16_t)handler->rxbuf[4]|(uint16_t)handler->rxbuf[5]<<8);
    
    if(length > MSG_LENGTH_MAX){
        length = 0;
    }
    for(i=0; i<length+4; i++)
    {
        checksum ^= (handler->rxbuf[2+i]&0xFF);
    }
    return checksum;
}

int lwlink_data_handler_parse(struct lwlink_data_handler_s *handler,uint8_t ch)
{
    switch(handler->status){
        case HANDLER_STATUS_WHEAD1:
            if(ch == MSG_HEAD1){
                handler->status  = HANDLER_STATUS_WHEAD2;
                handler->rxbuf_ptr = 0;
                handler->rxbuf[handler->rxbuf_ptr++] = ch;
                printf("H1 OK\r\n");
            }
            break;
        case HANDLER_STATUS_WHEAD2:
            if(ch == MSG_HEAD2){
                handler->status  = HANDLER_STATUS_DATA;
                handler->rxbuf[handler->rxbuf_ptr++] = ch;
                printf("H2 OK\r\n");
            }else{
                  handler->status  = HANDLER_STATUS_WHEAD1;
                  printf("H1 F\r\n");
            }
            break;
        case HANDLER_STATUS_DATA:
            handler->rxbuf[handler->rxbuf_ptr++] = ch;    
            if(ch == MSG_END1){
                handler->status = HANDLER_STATUS_WEND2;
                printf("E1 OK:%d\r\n",handler->rxbuf_ptr-1);
            }
            break;
        case HANDLER_STATUS_WEND2:
            handler->rxbuf[handler->rxbuf_ptr++] = ch;    
            if(ch == MSG_END2){
                if(handler->rxbuf[3] >= 0x20 ){
                    if(handler->rxbuf[handler->rxbuf_ptr - 3] == lwlink_data_checksum_calc(handler)){
                        handler->status  = HANDLER_STATUS_WHEAD1;
                        printf(" P  E2 OK\r\n");
                        return 1;
                    }else{
                        handler->status  = HANDLER_STATUS_WHEAD1;
                        printf(" P  E2 F\r\n");
                        return -2;
                    }
                }else{
                    uint16_t length =(int16_t)((uint16_t)handler->rxbuf[4]|(uint16_t)handler->rxbuf[5]<<8);
                    if(length == (handler->rxbuf_ptr - 9)){
                        handler->status  = HANDLER_STATUS_WHEAD1;
                        printf("E2 OK\r\n");
                        return 2;
                    }else{
                        handler->status  = HANDLER_STATUS_WHEAD1;
                        printf("E2 CK F \r\n");
                        return -3;
                    }
                }
            }else{
                handler->status  = HANDLER_STATUS_DATA;
            }
            break;
        default:
            break;
    }
    
    return 0;
}

uint8_t lwlink_data_handler_get_type(struct lwlink_data_handler_s *handler)
{
    return handler->rxbuf[3];
}

uint8_t lwlink_data_handler_get_id(struct lwlink_data_handler_s *handler)
{
    return handler->rxbuf[2];
}

uint8_t lwlink_data_handler_get_length(struct lwlink_data_handler_s *handler)
{
    return (int16_t)((uint16_t)handler->rxbuf[4]|(uint16_t)handler->rxbuf[5]<<8);
}

uint8_t* lwlink_data_handler_get_data(struct lwlink_data_handler_s *handler)
{
    return &(handler->rxbuf[6]);
}
