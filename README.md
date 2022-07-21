NK-980IOT I2C测试：读取BMP180[https://club.rt-thread.org/ask/article/58d5593496be9547.html](https://club.rt-thread.org/ask/article/58d5593496be9547.html)
@[toc](【Renesas RA6M4开发板之I2C读取BMP180气压温度】)

# 1.0 BMP180
![在这里插入图片描述](https://img-blog.csdnimg.cn/91b4c58b2d8f4b07bc567059b1c9a759.png)
**此图转载钦源盛数码专营店**

*本篇通过Renesas RA6M4开发板采用I2C读取BMP180传感器的气压温度示例程序演示。*
## 1.1 BMP180介绍
1、BMP180是一款高精度、小体积、低能耗的压力传感器，可以应用在移动设备中；
2、精度低，可以达到0.03hPa；
3、BMP180采用强大的8-pin陶瓷无引线芯片承载(LCC)超薄封装，可以通过I2C总线直接与各种微处理器相连。

## 1.2 BMP180特点
1.压力范围:300---1100hPa(海拔9000米一500米)
2.电源电压:5V
3.低功耗:在标准模式下5uA
4.高精度:低功耗模式下，分辨率为0.06hPa(0.5米)
5.高线性模式下，分辨率为0.03hPa(0.25米)
6.带温度输出
7.I2C通信方式8.带温度补偿
9.MSL1秒响应
10.待机电流:0.1uA
11.需在气体环境中工作，不可测量液体和反接电源😅😅😅
![在这里插入图片描述](https://img-blog.csdnimg.cn/d7fbba3d7c7245ac98a7d667d9fd7bf8.png)

尺寸大小如下：
![在这里插入图片描述](https://img-blog.csdnimg.cn/dbaa40ceacae412790c6610c009dd525.png)
## 1.3 产品应用
1.GPS导航(航位推算，上下桥检测等)
⒉.室内室外导航
3.休闲、体育等监测
4.天气预报
5.垂直速度指示(上升/下沉速度)



# 2. RT-theard配置
## 2.1 硬件需求
1、需要BMP180采集气体环境下的气压和温度，I2C通讯接线**SDA---p503;SCL---p504**，不需要关注地址后面库自带配置了，与[ssd1306](https://blog.csdn.net/VOR234/article/details/125742886)不同

> 实现功能：
> I2C读取BMP180传感器的气压温度


2、RA6M4开发板
![在这里插入图片描述](https://img-blog.csdnimg.cn/4c5dcda23c6d4afaacb393dc46a7ae51.png)
3、USB下载线，ch340串口和附带6根母母线，**rx---p613;tx---p614**        
![在这里插入图片描述](https://img-blog.csdnimg.cn/fd57ca4397984f7caa355496c08cfffc.png)




## 2.2 软件配置
Renesas RA6M4开发板环境配置参照：[【基于 RT-Thread Studio的CPK-RA6M4 开发板环境搭建】](https://blog.csdn.net/vor234/article/details/125634313)
1、新建项目RA6M4-bmp180工程
![在这里插入图片描述](https://img-blog.csdnimg.cn/cce4bacd566e47889f8b06f28e0783be.png)

2、点击RT-theard Setting，在软件包下添加软件包，然后搜索bmp相关软件支持包，点击添加即可,然后出现对应包。
![在这里插入图片描述](https://img-blog.csdnimg.cn/532992c65fa9489893d93961a082bacb.png)

3、配置ssd306，右键选择配置项
![在这里插入图片描述](https://img-blog.csdnimg.cn/7209c6e06c19446d9caff6feb8a1bd2d.png)

4、在软件包中开启示例程序。
![在这里插入图片描述](https://img-blog.csdnimg.cn/fdb9bd27060b428b8acf2788794dcc5e.png)

5、在硬件中，启动I2C，设置端口SDA---p503;SCL---p504
![在这里插入图片描述](https://img-blog.csdnimg.cn/52dce4383ada4fcf88d4c2518c38bed8.png)
6、全部保存刚刚的配置，更新当前配置文件

**保存完是灰色，没有保存是蓝色。**
# 3. 代码分析
1、刚刚加载软件包在packages文件夹下，bmp180.c和bmp180_sample.c示例代码更改为如下
（或者头文件添加`#include "bsp_api.h"`,否则会报错`unitx_t`，根据提示全部改为`rt_unitx_t`也OK，下面是第二种方法）😅😅😅
*bmp180.c*

```cpp

/*
 * Copyright (c) 2020 panrui <https://github.com/Prry/rtt-bmp180>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-12     panrui      the first version
 */
 
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h>
#include "sensor.h"
#include "bmp180.h"

#define PKG_USING_BMP180

#ifdef PKG_USING_BMP180

#define BMP180_ADDR				0x77 	/* i2c slave address */

#define BMP_REG_CHIP_ID			0xD0	
#define BMP_REG_RESET			0xE0
#define BMP_REG_CTRL_ADDR		0xF4
#define BMP_REG_AD_MSB			0xF6
#define BMP_REG_AD_LSB			0xF7
#define BMP_REG_AD_XLSB			0xF8
#define BMS_CAL_AC1				0xAA

#define BMP_REG_CTRL_TEMP		0x2E
#define BMP_REG_CTRL_POSS0		0x34
#define BMP_REG_CTRL_POSS1		0x74
#define BMP_REG_CTRL_POSS2		0xB4
#define BMP_REG_CTRL_POSS3		0xF4
#define BMP_REG_RESET_VALUE		0xB6
		
#define	BMP180_I2C_BUS			"i2c1"		/* i2c linked */
#define BMP180_DEVICE_NAME		"bmp180"	/* register device name */


/* bmp clalc param */
struct bmp180_calc
{
	short ac1;
	short ac2;
	short ac3;
	short b1;
	short b2;
	short mb;
	short mc;
	short md;
	unsigned short ac4;
	unsigned short ac5;
	unsigned short ac6;	
};

/* bmp180 private data */
struct bmp180_dev
{
	struct bmp180_calc calc_param;
	struct rt_i2c_bus_device *i2c_bus;	/* linked i2c bus */
};

static rt_err_t  bmp180_read_regs(rt_sensor_t psensor, rt_uint8_t reg, rt_uint8_t *data, rt_uint8_t data_size)
{
    struct rt_i2c_msg msg[2];
	struct bmp180_dev *dev = RT_NULL;
	struct rt_i2c_bus_device *i2c_bus = RT_NULL;
	rt_uint32_t slave_addr = 0;
	
	slave_addr = (rt_uint32_t)psensor->config.intf.user_data;	/* get i2c slave address */
	dev = (struct bmp180_dev*)psensor->parent.user_data;/* bmp180 private data */
	i2c_bus = (struct rt_i2c_bus_device*)dev->i2c_bus; /* get i2c bus device */
	
    msg[0].addr  = (rt_uint8_t)slave_addr;
    msg[0].flags = RT_I2C_WR;
    msg[0].len   = 1;
    msg[0].buf   = &reg;
    msg[1].addr  = (rt_uint8_t)slave_addr;
    msg[1].flags = RT_I2C_RD;
    msg[1].len   = data_size;
    msg[1].buf   = data;

    if(rt_i2c_transfer(i2c_bus, msg, 2) == 2)
	{
        return RT_EOK;
    }
    else
    {
	  	LOG_E("i2c bus read failed!\r\n");
        return -RT_ERROR;
    }
}

static rt_err_t  bmp180_write_regs(rt_sensor_t psensor, rt_uint8_t reg, rt_uint8_t *data, rt_uint8_t data_size)
{
    struct rt_i2c_msg msg[2];
	struct bmp180_dev *dev = RT_NULL;
	struct rt_i2c_bus_device *i2c_bus;
	rt_uint32_t slave_addr = 0;

	slave_addr = (rt_uint32_t)psensor->config.intf.user_data;
	dev = (struct bmp180_dev*)psensor->parent.user_data;
	i2c_bus = (struct rt_i2c_bus_device*)dev->i2c_bus;  
	
    msg[0].addr		= (rt_uint8_t)slave_addr;
    msg[0].flags	= RT_I2C_WR;
    msg[0].len   	= 1;
    msg[0].buf   	= &reg;
    msg[1].addr  	= (rt_uint8_t)slave_addr;
    msg[1].flags	= RT_I2C_WR | RT_I2C_NO_START;
    msg[1].len   	= data_size;
    msg[1].buf   	= data;
    if(rt_i2c_transfer(i2c_bus, msg, 2) == 2)
	{
        return RT_EOK;
    }
    else
    {
	  	LOG_E("i2c bus write failed!\r\n");
        return -RT_ERROR;
    }
}

static rt_err_t bmp180_write_reg(rt_sensor_t psensor, rt_uint8_t reg, rt_uint8_t data)
{
	return bmp180_write_regs(psensor, reg, &data, 1);
}

static long bmp180_read_ut(rt_sensor_t psensor)
{
    rt_uint8_t buf[2] = {0};
	long   data = 0;
	
	bmp180_write_reg(psensor, BMP_REG_CTRL_ADDR, BMP_REG_CTRL_TEMP);
	rt_thread_delay(1);	/* max conversion time: 4.5ms */
	bmp180_read_regs(psensor, BMP_REG_AD_MSB, buf, 2);
	data = (buf[0]<<8) | buf[1];
	
	return data;
}

static long bmp180_read_up(rt_sensor_t psensor)
{
    rt_uint8_t buf[2] = {0};
	long   data = 0;
	
	bmp180_write_reg(psensor, BMP_REG_CTRL_ADDR, BMP_REG_CTRL_POSS0);
	rt_thread_delay(1);	/* max conversion time: 4.5ms */
	bmp180_read_regs(psensor, BMP_REG_AD_MSB, buf, 2);
	data = (buf[0]<<8) | buf[1];
	
	return data;
}

static rt_size_t bmp180_polling_get_data(rt_sensor_t psensor, struct rt_sensor_data *sensor_data)
{
	long x1, x2, b5, b6, x3, b3, p;
	unsigned long b4, b7;
	short temperature=0;
	long ut,up,pressure=0;
	struct bmp180_dev *dev = RT_NULL;
	struct bmp180_calc *param = RT_NULL;
	
	ut = bmp180_read_ut(psensor);
	up = bmp180_read_up(psensor);
	
	
	dev = (struct bmp180_dev*)psensor->parent.user_data;/* bmp180 private data */
	param = &dev->calc_param;	/* get calc param */
	
	/* temperature calc */
	x1 = (((long)ut - (long)param->ac6)*(long)param->ac5) >> 15;
  	x2 = ((long)param->mc << 11) / (x1 + param->md);
  	b5 = x1 + x2;
  	temperature = ((b5 + 8) >> 4);

	/* pressure calc */
	b6 = b5 - 4000;
	x1 = (param->b2 * (b6 * b6)>>12)>>11;
	x2 = (param->ac2 * b6)>>11;
	x3 = x1 + x2;
	b3 = (((((long)param->ac1)*4 + x3)<<0) + 2)>>2;
	
	x1 = (param->ac3 * b6)>>13;
	x2 = (param->b1 * ((b6 * b6)>>12))>>16;
	x3 = ((x1 + x2) + 2)>>2;
	b4 = (param->ac4 * (unsigned long)(x3 + 32768))>>15;
	b7 = ((unsigned long)(up - b3) * (50000>>0));
	if (b7 < 0x80000000)
	{
		p = (b7<<1)/b4;
	}
	else
	{
		p = (b7/b4)<<1;
	}
	x1 = (p>>8) * (p>>8);
	x1 = (x1 * 3038)>>16;
	x2 = (-7357 * p)>>16;
	pressure = p+((x1 + x2 + 3791)>>4);	
	
	if(psensor->info.type == RT_SENSOR_CLASS_BARO)
	{/* actual barometric */
	  	sensor_data->type = RT_SENSOR_CLASS_BARO;
		sensor_data->data.baro = pressure;
		sensor_data->timestamp = rt_sensor_get_ts();
	}
	else if(psensor->info.type == RT_SENSOR_CLASS_TEMP)
	{/* actual temperature */
		sensor_data->type = RT_SENSOR_CLASS_TEMP;
		sensor_data->data.temp = temperature;
		sensor_data->timestamp = rt_sensor_get_ts();
	}
	else
	{
		return 0;
	}
	
    return 1;
}

static rt_size_t bmp180_fetch_data(struct rt_sensor_device *psensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);
	RT_ASSERT(psensor);
	
	//if(psensor->parent.open_flag & RT_DEVICE_FLAG_RDONLY)
	if(psensor->config.mode == RT_SENSOR_MODE_POLLING)
	{
        return bmp180_polling_get_data(psensor, buf);
    }

    return 0;
}

static rt_err_t bmp180_control(struct rt_sensor_device *psensor, int cmd, void *args)
{
	rt_err_t	ret = RT_EOK;
    rt_uint8_t 	*chip_id;
	
    RT_ASSERT(psensor);

    switch (cmd)
    {
    	/* read bmp180 id */
        case RT_SENSOR_CTRL_GET_ID:
		  	chip_id = (rt_uint8_t*)args;
	       	ret = bmp180_read_regs(psensor, BMP_REG_CHIP_ID, chip_id, 1);
        break;

        default:
        break;
	}
    return ret;
}

static struct rt_sensor_ops bmp180_ops =
{
    bmp180_fetch_data,
    bmp180_control,
};

int rt_hw_bmp180_init(const char *name, struct rt_sensor_config *cfg)
{
  	rt_err_t ret = RT_EOK;
	rt_sensor_t sensor_baro = RT_NULL, sensor_temp = RT_NULL;
    struct rt_sensor_module *module = RT_NULL;
	struct bmp180_dev 		*bmp180 = RT_NULL;
	rt_uint8_t bmbuf[22] = {0};
	
	bmp180 = rt_calloc(1, sizeof(struct bmp180_dev));
	if(bmp180 == RT_NULL)
	{
	  	LOG_E("malloc memory failed\r\n");
		ret = -RT_ERROR;
		goto __exit;
	}
	
    bmp180->i2c_bus = rt_i2c_bus_device_find(cfg->intf.dev_name);
    if(bmp180->i2c_bus == RT_NULL)
    {
        LOG_E("i2c bus device %s not found!\r\n", cfg->intf.dev_name);
		ret = -RT_ERROR;
		goto __exit;
    }	
	
	module = rt_calloc(1, sizeof(struct rt_sensor_device));
    if(module == RT_NULL)
	{
	  	LOG_E("malloc memory failed\r\n");
	  	ret = -RT_ERROR;
		goto __exit;
	}
	module->sen[0] = sensor_baro;
    module->sen[1] = sensor_temp;
    module->sen_num = 2;
	
	/*  barometric pressure sensor register */
    {
        sensor_baro = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_baro == RT_NULL)
		{
		  	goto __exit;
		}
		rt_memset(sensor_baro, 0x0, sizeof(struct rt_sensor_device));
        sensor_baro->info.type       = RT_SENSOR_CLASS_BARO;
        sensor_baro->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_baro->info.model      = "bmp180_baro";
        sensor_baro->info.unit       = RT_SENSOR_UNIT_PA;
        sensor_baro->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_baro->info.range_max  = 110000;	/* 1Pa */
        sensor_baro->info.range_min  = 30000;
        sensor_baro->info.period_min = 100;	/* read ten times in 1 second */

        rt_memcpy(&sensor_baro->config, cfg, sizeof(struct rt_sensor_config));
        sensor_baro->ops = &bmp180_ops;
        sensor_baro->module = module;
        
        ret = rt_hw_sensor_register(sensor_baro, name, RT_DEVICE_FLAG_RDWR, (void*)bmp180);
        if (ret != RT_EOK)
        {
            LOG_E("device register err code: %d", ret);
            goto __exit;
        }
    }
    /* temperature sensor register */
    {
        sensor_temp = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_temp == RT_NULL)
		{
			goto __exit;
		}
		rt_memset(sensor_temp, 0x0, sizeof(struct rt_sensor_device));
        sensor_temp->info.type       = RT_SENSOR_CLASS_TEMP;
        sensor_temp->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_temp->info.model      = "bmp180_temp";
        sensor_temp->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
        sensor_temp->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_baro->info.range_max  = 850;	/* 0.1C */
        sensor_baro->info.range_min  = -400;
        sensor_temp->info.period_min = 100;	/* read ten times in 1 second */

        rt_memcpy(&sensor_temp->config, cfg, sizeof(struct rt_sensor_config));
        sensor_temp->ops = &bmp180_ops;
        sensor_temp->module = module;
        
        ret = rt_hw_sensor_register(sensor_temp, name, RT_DEVICE_FLAG_RDWR, (void*)bmp180);
        if (ret != RT_EOK)
        {
            LOG_E("device register err code: %d", ret);
            goto __exit;
        }
    }
   
	/* bmp180 read calc param */
	ret = bmp180_read_regs(sensor_baro, BMS_CAL_AC1, bmbuf, 22);
	if(ret == RT_EOK)
	{
		bmp180->calc_param.ac1 = (bmbuf[0]<<8)|bmbuf[1];
		bmp180->calc_param.ac2 = (bmbuf[2]<<8)|bmbuf[3];
		bmp180->calc_param.ac3 = (bmbuf[4]<<8)|bmbuf[5];
		bmp180->calc_param.ac4 = (bmbuf[6]<<8)|bmbuf[7];
		bmp180->calc_param.ac5 = (bmbuf[8]<<8)|bmbuf[9];
		bmp180->calc_param.ac6 = (bmbuf[10]<<8)|bmbuf[11];
		bmp180->calc_param.b1 = (bmbuf[12]<<8)|bmbuf[13];
		bmp180->calc_param.b2 = (bmbuf[14]<<8)|bmbuf[15];
		bmp180->calc_param.mb = (bmbuf[16]<<8)|bmbuf[17];
		bmp180->calc_param.mc = (bmbuf[18]<<8)|bmbuf[19];
		bmp180->calc_param.md = (bmbuf[20]<<8)|bmbuf[21];
	}
	else
	{
		LOG_E("bmp180 read calc param failed\r\n");
		goto __exit;
	}
    return RT_EOK;

__exit:
  	if(sensor_baro)
	{
		rt_free(sensor_baro);
	}
	
	if(sensor_temp)
	{
		rt_free(sensor_temp);
	}
	
    if(module)
	{
	 	rt_free(module);
	}
	
	if (bmp180)
	{
		rt_free(bmp180);
	}
    return ret;
}

#endif /* PKG_USING_BMP180 */

```
*bmp180_sample.c*

```cpp

/*
 * Copyright (c) 2020 panrui <https://github.com/Prry/rtt-bmp180>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-12     panrui      the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "sensor.h"
#include "bmp180.h"

static void read_baro_entry(void *parameter)
{
    rt_device_t baro_dev = RT_NULL, temp_dev = RT_NULL;
    struct rt_sensor_data baro_data,temp_data;
    rt_size_t res0 = 0, res1 = 1;
	rt_uint8_t chip_id;
	
    baro_dev = rt_device_find("baro_bmp180");
    if (baro_dev == RT_NULL)
    {
	  	 rt_kprintf("not found baro_bmp180 device\r\n");
        return;
    }

    if (rt_device_open(baro_dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        rt_kprintf("open baro_180 failed\r\n");
        return;
    }

	temp_dev = rt_device_find("temp_bmp180");
    if (temp_dev == RT_NULL)
    {
	  	 rt_kprintf("not found temp_bmp180 device\r\n");
        return;
    }

    if (rt_device_open(temp_dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        rt_kprintf("open temp_bmp180 failed\r\n");
        return;
    }
	
	rt_device_control(baro_dev, RT_SENSOR_CTRL_SET_ODR, (void *)(1));/* 1Hz read */
    rt_device_control(temp_dev, RT_SENSOR_CTRL_SET_ODR, (void *)(1));/* 1Hz read */
	
	rt_device_control(temp_dev, RT_SENSOR_CTRL_GET_ID, (void*)&chip_id);
	rt_kprintf("bmp180 chip ID [0x%X]\n", chip_id);
	while (1)
    {
        res0 = rt_device_read(baro_dev, 0, &baro_data, 1);
		res0 = rt_device_read(temp_dev, 0, &temp_data, 1);
        if (res0==0 || res1==0)
        {
            rt_kprintf("read data failed! result is %d,%d\n", res0, res1);
            rt_device_close(baro_dev);
			rt_device_close(temp_dev);
            return;
        }
        else
        {
        	rt_kprintf("baro[%dPa],temp[%d.%dC],timestamp[%d]\r\n", baro_data.data.baro, 
					   temp_data.data.temp/10-42, temp_data.data.temp%10,
					   temp_data.timestamp);
        }

        rt_thread_delay(500);
    }
}

static int baro_read_sample(void)
{
    rt_thread_t baro_thread;

    baro_thread = rt_thread_create("baro_r",
                                     read_baro_entry,
                                     RT_NULL,
                                     1024,
                                     RT_THREAD_PRIORITY_MAX / 2,
                                     20);
    if (baro_thread != RT_NULL)
    {
        rt_thread_startup(baro_thread);
    }

    return RT_EOK;
}
INIT_APP_EXPORT(baro_read_sample);

static int rt_hw_bmp180_port(void)
{
    struct rt_sensor_config cfg;
    	
	cfg.intf.dev_name = "i2c1"; 		/* i2c bus */
    cfg.intf.user_data = (void *)0x77;	/* i2c slave addr */
    rt_hw_bmp180_init("bmp180", &cfg);	/* bmp180 */

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_bmp180_port);

```

2、此库包含读取I2C信息，解调，并且添加一个进程、每秒自动打印温度和气压以及时间。
![在这里插入图片描述](https://img-blog.csdnimg.cn/bcf10b9772b043408e473466af95ad8b.png)

关键打印代码以及自行校准两项参数

```cpp
rt_kprintf("baro[%dPa],temp[%d.%dC],timestamp[%d]\r\n", baro_data.data.baro, 
					   temp_data.data.temp/10-42, temp_data.data.temp%10,
					   temp_data.timestamp);
```

3、main.c文件在re_gen文件夹下，主程序围绕“hal_entry();”函数（在src文件夹），这些默认不变


# 4. 下载验证
1、编译重构

![在这里插入图片描述](https://img-blog.csdnimg.cn/82c91b60678f4276ad109e002c6896b9.png)

编译成功

2、下载程序
![在这里插入图片描述](https://img-blog.csdnimg.cn/fed4020b4402480bbd28ec75c3cf346e.png)

下载成功


3、CMD串口调试

![在这里插入图片描述](https://img-blog.csdnimg.cn/181227ee2ed64ef2801477ece50cf41c.png)
然后板载复位，开始串口打印显示！🎉🎉🎉
效果如下
![请添加图片描述](https://img-blog.csdnimg.cn/873cc7d5abf24198b2f33e2af068024f.gif)

这样我们就可以天马行空啦!![请添加图片描述](https://img-blog.csdnimg.cn/92099d4d054b4b2cbd39b95719739a90.gif)

参考文献；
[【基于 RT-Thread Studio的CPK-RA6M4 开发板环境搭建】](https://blog.csdn.net/vor234/article/details/125634313)
