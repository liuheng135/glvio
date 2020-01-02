#include "hal_device.h"

int mpu6050_register(void);
int spl06_register(void);
int dps280_register(void);
int ist8310_register(void);
int stm8s_register(void);
int ubx_register(void);
int linux_i2c_register(void);
int net_register(void);


void board_initialize(void)
{
    linux_i2c_register();

    mpu6050_register();
    //dps280_register();
    spl06_register();
    ist8310_register();
    stm8s_register();
    //ubx_register();
    net_register();
}


