#include "anoc_link.h"

uint8_t anoc_data_checksum_calc(unsigned char *buffer)
{
    int16_t i;
    uint8_t checksum=0;
    uint16_t length =buffer[3];
    
    if(length > ANOC_MSG_LENGTH_MAX){
        length = 0;
    }

    for(i=0; i<length+4; i++)
    {
        checksum += buffer[i];
    }
    return checksum;
}

int      anoc_msg_pack(struct anoc_data_handler_s *handler,uint8_t type,uint8_t *data,uint32_t len)
{
    int i;
    
    handler->txbuf[0] = ANOC_HEAD;
    handler->txbuf[1] = ANOC_DIR_FROM_PILOT;
    handler->txbuf[2] = type;
    handler->txbuf[3] = len;
   
    for(i = 0; i < len; i++){
        handler->txbuf[i+4] = data[i];
    }
    
    handler->txbuf[len + 4] = anoc_data_checksum_calc(handler->txbuf);
    
    return len+5;
}

int      anoc_data_handler_init(struct anoc_data_handler_s *handler)
{
    handler->heartbeat_rate = 0;
    handler->status = ANOC_HANDLER_STATUS_WHEAD1;

    return 0;
}

int      anoc_data_handler_parse(struct anoc_data_handler_s *handler,uint8_t ch);
uint8_t  anoc_data_handler_get_type(struct anoc_data_handler_s *handler)
{
    return handler->rxbuf[2];
}

uint8_t  anoc_data_handler_get_length(struct anoc_data_handler_s *handler)
{
    return handler->rxbuf[3];
}

uint8_t* anoc_data_handler_get_data(struct anoc_data_handler_s *handler)
{
    return &handler->txbuf[4];
}


