#ifndef _I2C_VBUS_H_
#define _I2c_VBUS_H_

#include <linux/types.h>

#define I2C_VBUS_MSG_LENGTH_MAX   128
#define I2C_VBUS_MSG_NUM_MAX        5


struct i2c_vbus_msg{
	__u16 addr;	/* slave address			*/
	__u16 flags;
	__u32 speed;
	
#define I2C_M_TEN		0x0010	/* this is a ten bit chip address */
#define I2C_M_RD		0x0001	/* read data, from slave to master */
#define I2C_M_STOP		0x8000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NOSTART		0x4000	/* if I2C_FUNC_NOSTART */
#define I2C_M_REV_DIR_ADDR	0x2000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NO_RD_ACK		0x0800	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_RECV_LEN		0x0400	/* length will be first received byte */

	__u16 len;		/* msg length				*/
	__u8  buf[I2C_VBUS_MSG_LENGTH_MAX];		/* pointer to msg data			*/
};

struct i2c_vbus_msg_packet{
	__u16 msg_num;
	struct i2c_vbus_msg *msgs;
};



#define I2C_XFER	0x0701	/* transfer a message using adapter */
#define I2C_CFG  	0x0702	/* configure adapter */


#endif
