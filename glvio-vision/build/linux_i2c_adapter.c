#include "hal_i2c.h"
#include <fcntl.h>
#include <unistd.h>
#include "i2c-vbus.h"
#include <stdlib.h>
#include <stdio.h>
#include <sys/ioctl.h>    /* BSD and Linux */


struct linux_i2c_adapter_s{
	int port;
	int bus_fd;
};

struct hal_i2c_adapter_s   i2c_bus0;
struct linux_i2c_adapter_s linux_i2c_adapter0;

int linux_i2c_init(struct hal_i2c_adapter_s *adpt)
{
	struct linux_i2c_adapter_s *linux_adapter = (struct linux_i2c_adapter_s *)adpt->user_data;
	linux_adapter->bus_fd = open("/dev/i2c-bus-0",O_RDWR);
	if(linux_adapter->bus_fd > -1){
		return 0;
	}
	return -1;
}

int linux_i2c_configure(struct hal_i2c_adapter_s *adpt,struct hal_i2c_cfg_s *cfg)
{
	return 0;
}

int linux_i2c_transfer (struct hal_i2c_adapter_s *adpt,struct hal_i2c_msg_s *msg,int num)
{
	int i;
	int msg_num = 0;
	struct linux_i2c_adapter_s *linux_adapter = (struct linux_i2c_adapter_s *)adpt->user_data;
	struct i2c_vbus_msg vmsg[I2C_VBUS_MSG_NUM_MAX];
	struct i2c_vbus_msg_packet msg_packet;

	msg_num = num > I2C_VBUS_MSG_NUM_MAX ? I2C_VBUS_MSG_NUM_MAX : num;
	memset(&vmsg,0x00,sizeof(vmsg));
	memset(&msg_packet,0x00,sizeof(msg_packet));

	for(i = 0;i < msg_num;i++){
		vmsg[i].addr = msg[i].addr;
		if(msg[i].flags & HAL_I2C_RD){
			vmsg[i].flags |= I2C_M_RD;
		}
		if(msg[i].flags & HAL_I2C_ADDR_10BIT){
			vmsg[i].flags |= I2C_M_TEN;
		}
		vmsg[i].len = msg[i].length;
		vmsg[i].speed = msg[i].speed;
		memcpy(vmsg[i].buf,msg[i].buff,vmsg[i].len);
	}

	msg_packet.msg_num = msg_num;
	msg_packet.msgs    = vmsg;
	if(linux_adapter->bus_fd > -1){
		ioctl(linux_adapter->bus_fd,I2C_XFER,&msg_packet);
		for(i = 0;i < msg_num;i++){
			memcpy(msg[i].buff,vmsg[i].buf,vmsg[i].len);
		}
	}else{
		return -1;
	}
	return msg_num;
}

int linux_i2c_register(void)
{
	linux_i2c_adapter0.port = 0;

	i2c_bus0.init      = linux_i2c_init;
	i2c_bus0.transfer  = linux_i2c_transfer;
	i2c_bus0.configure = linux_i2c_configure;
	i2c_bus0.user_data = &linux_i2c_adapter0;

	return hal_i2c_adapter_register(&i2c_bus0,0);
}
