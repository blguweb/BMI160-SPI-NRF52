/**
 *
 *  Copyright (C) Waveshare November 2018.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documnetation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to  whom the Software is
 * furished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "bmi160.h"
#include "bmi160_defs.h"
#include "boards.h"

#include "app_util_platform.h"
#include "nrf_delay.h"
#include "app_error.h"
#include <string.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "spi.h"
#include "SEGGER_RTT.h"

static struct bmi160_dev sensor; // BMI160 sensor struct
static uint8_t       no_use = 0xFF ;
static nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE( 0 );  // SPI instance

void user_delay_ms(uint32_t period)
{
    // delay time
    nrf_delay_ms(period);
}


int8_t spi_read_transfer(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	ret_code_t ret ;
	uint8_t read_temp[ length + 1 ] ;
	reg_addr = reg_addr | 0x80; 
  	ret = nrf_drv_spi_transfer(&spi, &reg_addr, 1, read_temp, length + 1 ) ;	
	nrf_delay_ms(5); 
	for( int i = 1 ; i < length + 1 ; i ++ )
	  reg_data[i-1] = read_temp[i] ;
	
	return (int8_t)ret;	
}

int8_t spi_write_transfer(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	ret_code_t ret;
	uint8_t write_temp[ length + 1 ] ;
	reg_addr = reg_addr & 0x7F;
	write_temp[0] = reg_addr ;
	for( int i = 1 ; i < length + 1 ; i ++ )
	  write_temp[i] = reg_data[i-1] ;

	ret = nrf_drv_spi_transfer(&spi, write_temp, length + 1, &no_use, 1 ) ;
	nrf_delay_ms(5) ;
	
	return (int8_t)ret;	
}


int main(void)
{

		nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
		// 配置传感器的加速度和角速度参数
		struct bmi160_sensor_data accel;
		struct bmi160_sensor_data gyro;
		

//		bsp_board_init(BSP_INIT_LEDS);
		APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
		NRF_LOG_DEFAULT_BACKENDS_INIT();

		spi_config.ss_pin   = CS_PIN;
		spi_config.miso_pin = MISO_PIN;
		spi_config.mosi_pin = MOSI_PIN;
		spi_config.sck_pin  = SCK_PIN;
    
    /*
    Doesn't print what to send or what received in SPI
    */
    //APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL, NULL)); 
    
    /*
    Print what to send or what received in SPI
    */
		APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
//		UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());//打印调试信息
//		NRF_LOG_INFO("SPI example started.");
//		NRF_LOG_FLUSH();
		
		 // init bmi160 sensor
		sensor.id = 1 ;
		sensor.intf = BMI160_SPI_INTF;
		sensor.read = spi_read_transfer;
		sensor.write = spi_write_transfer;
		sensor.delay_ms = user_delay_ms;

		int8_t rslt = BMI160_OK;

		rslt = bmi160_init(&sensor);

		
		// 获取传感器的chip id
		uint8_t data;
		rslt = bmi160_get_regs(BMI160_CHIP_ID_ADDR , &data , 1 , &sensor);
		if (rslt == BMI160_OK) {
			SEGGER_RTT_printf(0,"Chip ID: :%d\n",data);
		} else {
			SEGGER_RTT_printf(0,"error get regs :%hhd\n",rslt);
		}

		
		// 设置加速度计和陀螺仪的配置
		sensor.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ; // 加速度计采样率为100Hz
		sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G; // 加速度计量程为±4g
		sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4; // 加速度计滤波器为正常模式
		sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE; // power mode

		sensor.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ; // 陀螺仪采样率为100Hz
		sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS; // 陀螺仪量程为±2000°/s
		sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE; // 陀螺仪滤波器为正常模式
		sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; // power mode
		// set configuration
		rslt = bmi160_set_sens_conf(&sensor);
    if (rslt != BMI160_OK)
    {
			NRF_LOG_INFO("error config: ");
			NRF_LOG_FLOAT(rslt);
			SEGGER_RTT_printf(0,"error config: %d\n",rslt);
    }


    while (1)
    {
			        // 读取加速度计和陀螺仪的数据
        rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, &sensor);
        if (rslt != BMI160_OK)
        {
            // 处理读取失败的情况
//					NRF_LOG_INFO("error read: ");
//					NRF_LOG_FLOAT(rslt);
//					SEGGER_RTT_printf(0,"error read: %d\n",rslt);
					continue;
        }
				else
				{

					// 转换原始数据为物理值，单位为mg和°/s
					float ax = (float)(accel.x * 9.8) / 16384;
					float ay = (float)(accel.y * 9.8) / 16384;
					float az = (float)(accel.z * 9.8) / 16384;

					float gx = (float)(gyro.x * 2000)/0x8000;
					float gy = (float)(gyro.y * 2000)/0x8000;
					float gz = (float)(gyro.z * 2000)/0x8000;
//					float ay = (float)accel.y * 0.122f;
//					float az = (float)accel.z * 0.122f;

//					float gx = (float)gyro.x * 0.061f;
//					float gy = (float)gyro.y * 0.061f;
//					float gz = (float)gyro.z * 0.061f;

					// 在这里处理或显示数据


					char axs[32];
					char ays[32];
					char azs[32];
					char gxs[32];
					char gys[32];
					char gzs[32];
					sprintf(axs,"%.4f",ax);
					sprintf(ays,"%.4f",ay);
					sprintf(azs,"%.4f",az);
					sprintf(gxs,"%.4f",gx);
					sprintf(gys,"%.4f",gy);
					sprintf(gzs,"%.4f",gz);
					SEGGER_RTT_printf(0,"accx: %s\n",axs);
					SEGGER_RTT_printf(0,"accy: %s\n",ays);
					SEGGER_RTT_printf(0,"accz: %s\n",azs);
					SEGGER_RTT_printf(0,"gyrox: %s\n",gxs);
					SEGGER_RTT_printf(0,"gyroy: %s\n",gys);
					SEGGER_RTT_printf(0,"gyroz: %s\n",gzs);

					sensor.delay_ms(10); // 延时10ms
				}
			
    }
}
