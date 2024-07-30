/**
 * @file st7789.h
 *
 * Mostly taken from lbthomsen/esp-idf-littlevgl github.
 */

#ifndef ST7789_H
#define ST7789_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "lvgl.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

/*Definitions for SlimStack connector P1*/
#define P1_1_PIN	GPIO_NUM_45
#define P1_4_PIN	GPIO_NUM_10
#define P1_5_PIN	GPIO_NUM_11
#define P1_6_PIN	GPIO_NUM_12
#define P1_7_PIN	GPIO_NUM_13
#define P1_8_PIN	GPIO_NUM_16
#define P1_9_PIN	GPIO_NUM_15
#define P1_10_PIN	GPIO_NUM_42
#define P1_11_PIN	GPIO_NUM_17
#define P1_12_PIN	GPIO_NUM_18
#define P1_13_PIN	GPIO_NUM_21
#define P1_14_PIN	GPIO_NUM_26
#define P1_15_PIN	GPIO_NUM_47
#define P1_16_PIN	GPIO_NUM_33
#define P1_17_PIN	GPIO_NUM_34
#define P1_20_PIN	GPIO_NUM_46

/*Definitions for SlimStack connector P2*/
#define P2_3_PIN	GPIO_NUM_43
#define P2_4_PIN	GPIO_NUM_44
#define P2_6_PIN	GPIO_NUM_0
#define P2_7_PIN	GPIO_NUM_41
#define P2_8_PIN	GPIO_NUM_40
#define P2_13_PIN	GPIO_NUM_39
#define P2_14_PIN	GPIO_NUM_48
#define P2_15_PIN	GPIO_NUM_35
#define P2_16_PIN	GPIO_NUM_36
#define P2_17_PIN	GPIO_NUM_37
#define P2_18_PIN	GPIO_NUM_38

#define CONFIG_MOSI_GPIO	P1_13_PIN
#define CONFIG_SCLK_GPIO	P1_12_PIN
#define CONFIG_CS_GPIO		P1_15_PIN
#define CONFIG_DC_GPIO		P1_14_PIN
#define CONFIG_RESET_GPIO	P1_16_PIN
#define CONFIG_BL_GPIO		P2_8_PIN

typedef enum _disp_spi_send_flag_t {
    DISP_SPI_SEND_QUEUED        = 0x00000000,
    DISP_SPI_SEND_POLLING       = 0x00000001,
    DISP_SPI_SEND_SYNCHRONOUS   = 0x00000002,
    DISP_SPI_SIGNAL_FLUSH       = 0x00000004,
    DISP_SPI_RECEIVE            = 0x00000008,
    DISP_SPI_CMD_8              = 0x00000010, /* Reserved */
    DISP_SPI_CMD_16             = 0x00000020, /* Reserved */
    DISP_SPI_ADDRESS_8          = 0x00000040,
    DISP_SPI_ADDRESS_16         = 0x00000080,
    DISP_SPI_ADDRESS_24         = 0x00000100,
    DISP_SPI_ADDRESS_32         = 0x00000200,
    DISP_SPI_MODE_DIO           = 0x00000400,
    DISP_SPI_MODE_QIO           = 0x00000800,
    DISP_SPI_MODE_DIOQIO_ADDR   = 0x00001000,
	DISP_SPI_VARIABLE_DUMMY		= 0x00002000,
} disp_spi_send_flag_t;


#define ST7789_DC       CONFIG_DC_GPIO
#define ST7789_RST      CONFIG_RESET_GPIO

#define CONFIG_LV_DISPLAY_ORIENTATION 0 //(landscape)

#define SPI_TRANSACTION_POOL_SIZE 50	/* maximum number of DMA transactions simultaneously in-flight */
/* DMA Transactions to reserve before queueing additional DMA transactions. A 1/10th seems to be a good balance. Too many (or all) and it will increase latency. */
#define SPI_TRANSACTION_POOL_RESERVE_PERCENTAGE 10
#if SPI_TRANSACTION_POOL_SIZE >= SPI_TRANSACTION_POOL_RESERVE_PERCENTAGE
#define SPI_TRANSACTION_POOL_RESERVE (SPI_TRANSACTION_POOL_SIZE / SPI_TRANSACTION_POOL_RESERVE_PERCENTAGE)
#else
#define SPI_TRANSACTION_POOL_RESERVE 1	/* defines minimum size */
#endif

#if CONFIG_LV_DISP_USE_RST
  #if CONFIG_LV_DISP_ST7789_SOFT_RESET
    #define ST7789_SOFT_RST
  #endif
#else
  #define ST7789_SOFT_RST
#endif

#define ST7789_INVERT_COLORS            1//CONFIG_LV_INVERT_COLORS

#define MY_DISP_HOR_RES	320//240
#define MY_DISP_VER_RES 240//320
#define DISP_BUF_SIZE MY_DISP_HOR_RES * MY_DISP_VER_RES
#define SPI_BUS_MAX_TRANSFER_SZ (DISP_BUF_SIZE * 2)

/* ST7789 commands */
#define ST7789_NOP      0x00
#define ST7789_SWRESET  0x01
#define ST7789_RDDID    0x04
#define ST7789_RDDST    0x09

#define ST7789_RDDPM        0x0A    // Read display power mode
#define ST7789_RDD_MADCTL   0x0B    // Read display MADCTL
#define ST7789_RDD_COLMOD   0x0C    // Read display pixel format
#define ST7789_RDDIM        0x0D    // Read display image mode
#define ST7789_RDDSM        0x0E    // Read display signal mode
#define ST7789_RDDSR        0x0F    // Read display self-diagnostic result (ST7789V)

#define ST7789_SLPIN        0x10
#define ST7789_SLPOUT       0x11
#define ST7789_PTLON        0x12
#define ST7789_NORON        0x13

#define ST7789_INVOFF       0x20
#define ST7789_INVON        0x21
#define ST7789_GAMSET       0x26    // Gamma set
#define ST7789_DISPOFF      0x28
#define ST7789_DISPON       0x29
#define ST7789_CASET        0x2A
#define ST7789_RASET        0x2B
#define ST7789_RAMWR        0x2C
#define ST7789_RGBSET       0x2D    // Color setting for 4096, 64K and 262K colors
#define ST7789_RAMRD        0x2E

#define ST7789_PTLAR        0x30
#define ST7789_VSCRDEF      0x33    // Vertical scrolling definition (ST7789V)
#define ST7789_TEOFF        0x34    // Tearing effect line off
#define ST7789_TEON         0x35    // Tearing effect line on
#define ST7789_MADCTL       0x36    // Memory data access control
#define ST7789_IDMOFF       0x38    // Idle mode off
#define ST7789_IDMON        0x39    // Idle mode on
#define ST7789_RAMWRC       0x3C    // Memory write continue (ST7789V)
#define ST7789_RAMRDC       0x3E    // Memory read continue (ST7789V)
#define ST7789_COLMOD       0x3A

#define ST7789_RAMCTRL      0xB0    // RAM control
#define ST7789_RGBCTRL      0xB1    // RGB control
#define ST7789_PORCTRL      0xB2    // Porch control
#define ST7789_FRCTRL1      0xB3    // Frame rate control
#define ST7789_PARCTRL      0xB5    // Partial mode control
#define ST7789_GCTRL        0xB7    // Gate control
#define ST7789_GTADJ        0xB8    // Gate on timing adjustment
#define ST7789_DGMEN        0xBA    // Digital gamma enable
#define ST7789_VCOMS        0xBB    // VCOMS setting
#define ST7789_LCMCTRL      0xC0    // LCM control
#define ST7789_IDSET        0xC1    // ID setting
#define ST7789_VDVVRHEN     0xC2    // VDV and VRH command enable
#define ST7789_VRHS         0xC3    // VRH set
#define ST7789_VDVSET       0xC4    // VDV setting
#define ST7789_VCMOFSET     0xC5    // VCOMS offset set
#define ST7789_FRCTR2       0xC6    // FR Control 2
#define ST7789_CABCCTRL     0xC7    // CABC control
#define ST7789_REGSEL1      0xC8    // Register value section 1
#define ST7789_REGSEL2      0xCA    // Register value section 2
#define ST7789_PWMFRSEL     0xCC    // PWM frequency selection
#define ST7789_PWCTRL1      0xD0    // Power control 1
#define ST7789_VAPVANEN     0xD2    // Enable VAP/VAN signal output
#define ST7789_CMD2EN       0xDF    // Command 2 enable
#define ST7789_PVGAMCTRL    0xE0    // Positive voltage gamma control
#define ST7789_NVGAMCTRL    0xE1    // Negative voltage gamma control
#define ST7789_DGMLUTR      0xE2    // Digital gamma look-up table for red
#define ST7789_DGMLUTB      0xE3    // Digital gamma look-up table for blue
#define ST7789_GATECTRL     0xE4    // Gate control
#define ST7789_SPI2EN       0xE7    // SPI2 enable
#define ST7789_PWCTRL2      0xE8    // Power control 2
#define ST7789_EQCTRL       0xE9    // Equalize time control
#define ST7789_PROMCTRL     0xEC    // Program control
#define ST7789_PROMEN       0xFA    // Program mode enable
#define ST7789_NVMSET       0xFC    // NVM setting
#define ST7789_PROMACT      0xFE    // Program action

void st7789_init(void);
void spi_display_init(void);
void st7789_flush( lv_display_t * disp, const lv_area_t * area, uint8_t * pixmap);

void st7789_send_cmd(uint8_t cmd);
void st7789_send_data(void *data, uint16_t length);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ST7789_H  */
