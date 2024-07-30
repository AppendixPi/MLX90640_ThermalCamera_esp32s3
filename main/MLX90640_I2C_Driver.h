/*
 * MLX90640_I2C_Driver.h
 *
 *  Created on: 29 Jul 2024
 *      Author: gabri
 */

#ifndef MAIN_MLX90640_I2C_DRIVER_H_
#define MAIN_MLX90640_I2C_DRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SDA_PIN GPIO_NUM_16
#define SCL_PIN GPIO_NUM_15
#define I2C_BUS_FREQUENCY_HZ 1000000
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1
#define I2C_MASTER_TIMEOUT_MS 1000

#define MLX90640_I2C_ADD	0x33

void MLX90640_I2CInit(void);
int MLX90640_I2CGeneralReset(void);
int MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data);
int MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data);
//void MLX90640_I2CFreqSet(int freq);


#endif /* MAIN_MLX90640_I2C_DRIVER_H_ */
