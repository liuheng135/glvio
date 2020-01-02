#include "hal_i2c.h"
#include "hal_sensor.h"


int stm8s_init(struct hal_dev_s *dev)
{
    uint8_t buf[5] = {0};
    if(hal_i2c_master_recv(dev,buf,5) == 5){
	    return 0;
	}else{
        return -1;
	}
}


int stm8s_open(struct hal_dev_s *dev, uint16_t oflag)
{
	return 0;
}

int stm8s_close(struct hal_dev_s *dev)
{
	return 0;
}

int stm8s_read(struct hal_dev_s *dev, void *buffer, int size,int pos)
{
	return hal_i2c_master_recv(dev,buffer,size);
}

int stm8s_write(struct hal_dev_s *dev, const void *buffer, int size,int pos)
{
	return hal_i2c_master_send(dev,(uint8_t*)buffer,size);
}

int stm8s_ioctl(struct hal_dev_s *dev, uint8_t cmd, void *args)
{
	return 0;
}

struct hal_i2c_dev_s stm8s;

int stm8s_register(void)
{
	stm8s.port	    = 0;
	stm8s.address   = 0x33;
	stm8s.cfg.flags = HAL_I2C_WR | HAL_I2C_RD;
	stm8s.cfg.speed = 400000;
	stm8s.cfg.width = 8;

	stm8s.dev.init  = stm8s_init;
	stm8s.dev.open  = stm8s_open;
	stm8s.dev.close = stm8s_close;
	stm8s.dev.read  = stm8s_read;
	stm8s.dev.write = stm8s_write;
	stm8s.dev.ioctl = stm8s_ioctl;

	hal_i2c_device_register(&stm8s,"stm8s",HAL_O_RDWR);
	return 0;
}



