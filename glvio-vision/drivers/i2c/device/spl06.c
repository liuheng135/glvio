#include "hal_i2c.h"
#include "hal_sensor.h"

#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1

#define SPL06_REG_PRE		0x00
#define SPL06_REG_TEMP		0x03
#define SPL06_REG_PRS_CFG	0x06
#define SPL06_REG_TMP_CFG	0x07
#define SPL06_REG_MEAS_CFG	0x08
#define SPL06_REG_CFG		0x09
#define SPL06_REG_RST		0x0C
#define SPL06_REG_ID		0x0D
#define SPL06_REG_CONF		0x10

struct spl06_calib_data
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;       
    uint8_t id;
    int32_t kP;    
    int32_t kT;
} ;

int spl06_get_id(struct hal_dev_s *dev)
{
	uint8_t id;
	int ret;
	struct spl06_calib_data *calib = (struct spl06_calib_data *)dev->priv_data;

	ret =  hal_i2c_read_from_addr(dev, SPL06_REG_ID, &id, 1);
	if(ret < 0){
		printf("[SPL06]get id failed:%x\r\n",id);
		return -1;
	}

	calib->id = id;

	return 0;
}


int spl06_readCoefficients(struct hal_dev_s *dev)
{
    int ret;
	uint8_t buf[18];
    uint32_t h;
    uint32_t m;
    uint32_t l; 
    struct spl06_calib_data *calib = (struct spl06_calib_data *)dev->priv_data;

	ret =  hal_i2c_read_from_addr(dev, SPL06_REG_CONF,buf, 18);

	if(ret < 0){
		printf("[SPL06]get coefficients failed\r\n");
		return -1;
	}

    h = buf[0];
    l = buf[1];
    calib->c0 = (int16_t)h<<4 | l>>4;
    calib->c0 = (calib->c0&0x0800)?(0xF000|calib->c0):calib->c0;
    h = buf[1];
    l = buf[2];
    calib->c1 = (int16_t)(h&0x0F)<<8 | l;
    calib->c1 = (calib->c1&0x0800)?(0xF000|calib->c1):calib->c1;
    h = buf[3];
    m = buf[4];
    l = buf[5];
    calib->c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
    calib->c00 = (calib->c00&0x080000)?(0xFFF00000|calib->c00):calib->c00;
    h = buf[5];
    m = buf[6];
    l = buf[7];
    calib->c10 = (int32_t)(h&0x0F)<<16 | (int32_t)m<<8 | l;
    calib->c10 = (calib->c10&0x080000)?(0xFFF00000|calib->c10):calib->c10;
    h = buf[8];
    l = buf[9];
	calib->c01 = (int16_t)h<<8 | l;
    h = buf[10];
    l = buf[11];
	calib->c11 = (int16_t)h<<8 | l;
    h = buf[12];
    l = buf[13];
	calib->c20 = (int16_t)h<<8 | l;
    h = buf[14];
    l = buf[15];
	calib->c21 = (int16_t)h<<8 | l;
    h = buf[16];
    l = buf[17];
	calib->c30 = (int16_t)h<<8 | l;

	return 0;
}

void spl06_rateset(struct hal_dev_s *dev,uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
    uint8_t reg = 0;
    int32_t i32kPkT = 0;	
    struct spl06_calib_data *calib = (struct spl06_calib_data *)dev->priv_data;

    switch(u8SmplRate){
        case 2:
            reg |= (1<<4);
            break;
        case 4:
            reg |= (2<<4);
            break;
        case 8:
            reg |= (3<<4);
            break;
        case 16:
            reg |= (4<<4);
            break;
        case 32:
            reg |= (5<<4);
            break;
        case 64:
            reg |= (6<<4);
            break;
        case 128:
            reg |= (7<<4);
            break;
        case 1:
        default:
            break;
    }

	switch(u8OverSmpl){
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    if(iSensor == PRESSURE_SENSOR){
        calib->kP = i32kPkT;
        hal_i2c_write_to_addr(dev,SPL06_REG_PRS_CFG,&reg,1);
       
        if(u8OverSmpl > 8){
            hal_i2c_read_from_addr(dev, SPL06_REG_CFG, &reg, 1);
            reg |= 0x04;
            hal_i2c_write_to_addr(dev,SPL06_REG_CFG,&reg,1);
        }else{
            hal_i2c_read_from_addr(dev, SPL06_REG_CFG, &reg, 1);
            reg &= (~0x04);
            hal_i2c_write_to_addr(dev,SPL06_REG_CFG,&reg,1);
        }
    }
	
    if(iSensor == TEMPERATURE_SENSOR){
        calib->kT = i32kPkT;
        reg |= 0x80;
        hal_i2c_write_to_addr(dev,SPL06_REG_TMP_CFG,&reg,1);
        if(u8OverSmpl > 8){
            hal_i2c_read_from_addr(dev, SPL06_REG_CFG, &reg, 1);
            reg |= 0x08;
            hal_i2c_write_to_addr(dev,SPL06_REG_CFG,&reg,1);
        }else{
            hal_i2c_read_from_addr(dev, SPL06_REG_CFG, &reg, 1);
            reg &= (~0x08);
            hal_i2c_write_to_addr(dev,SPL06_REG_CFG,&reg,1);
        }
    }

}

static int spl06_init_client(struct hal_dev_s *dev)
{
	int ret;
	uint8_t reg = 0;
	
	ret = spl06_readCoefficients(dev);
	if(ret < 0){
		return -1;
	}

    spl06_rateset(dev,PRESSURE_SENSOR,32,16);
    spl06_rateset(dev,TEMPERATURE_SENSOR,4,4);

    reg = CONTINUOUS_P_AND_T + 4;
	hal_i2c_write_to_addr(dev,SPL06_REG_MEAS_CFG,&reg,1);

	return 0;
}

void spl06_convert(struct spl06_calib_data *calib,int rawPre,int rawTemp,float *pres,float *temp)
{
    float fTCompensate;
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = rawTemp / (float)calib->kT;
    fPsc = rawPre / (float)calib->kP;

    fTCompensate = calib->c0 * 0.5 + calib->c1 * fTsc;

    qua2 = calib->c10 + fPsc * (calib->c20 + fPsc* calib->c30);
    qua3 = fTsc * fPsc * (calib->c11 + fPsc * calib->c21);
    fPCompensate = calib->c00 + fPsc * qua2 + fTsc * calib->c01 + qua3;

    *pres = fPCompensate / 100.0f;
    *temp = fTCompensate;
}

int spl06_init(struct hal_dev_s *dev)
{
    if(spl06_get_id(dev) < 0){
        return -1;
    }
	if(spl06_init_client(dev) < 0){
		return -1;
	}
    return 0;
}


int spl06_open(struct hal_dev_s *dev, uint16_t oflag)
{
	int ret;
	uint8_t id;
	
	ret =  hal_i2c_read_from_addr(dev, SPL06_REG_ID, &id, 1);
	
	if(ret < 0){
        return -1;
	}
	
	return 0; 
}

int spl06_read(struct hal_dev_s *dev, void *buffer, int size,int pos)
{
	int ret;
	uint8_t buf[6];
	uint8_t h,m,l;
	int32_t rawPre;
	int32_t rawTemp;
    
    struct spl06_calib_data *calib = (struct spl06_calib_data *)dev->priv_data;
	struct baro_report_s *report = (struct baro_report_s *)buffer;

	if(size < sizeof(struct baro_report_s)){
        return -1;
	}
	
	ret = hal_i2c_read_from_addr(dev, SPL06_REG_PRE,buf, 6);

	if(ret < 0){
		return -1;
	}

	h = buf[3];
	m = buf[4];
	l = buf[5];
	rawTemp = (int32_t)h<<16 | (int32_t)m<<8 | (int32_t)l;
	rawTemp = (rawTemp&0x800000) ? (0xFF000000|rawTemp) : rawTemp;

	h = buf[0];
	m = buf[1];
	l = buf[2];
	rawPre = (int32_t)h<<16 | (int32_t)m<<8 | (int32_t)l;
	rawPre = (rawPre&0x800000) ? (0xFF000000|rawPre) : rawPre;
	
	spl06_convert(calib,rawPre,rawTemp,&report->pressure,&report->temperature);
	
	return 1;
}

struct spl06_calib_data spl06_calib;
struct hal_i2c_dev_s spl06_0;

int spl06_register(void)
{

	spl06_0.port	   = 0;
	spl06_0.address   = 0x76;
	spl06_0.cfg.flags = HAL_I2C_WR | HAL_I2C_RD;
	spl06_0.cfg.speed = 600000;
	spl06_0.cfg.width = 8;
	

	spl06_0.dev.init  = spl06_init;
	spl06_0.dev.open  = spl06_open;
	spl06_0.dev.close = NULL;
	spl06_0.dev.read  = spl06_read;
	spl06_0.dev.write = NULL;
	spl06_0.dev.ioctl = NULL;
	spl06_0.dev.priv_data = &spl06_calib;
	hal_i2c_device_register(&spl06_0,"spl06-0",HAL_O_RDWR | HAL_DEV_STANDALONE);
	return 0;
}


