/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define FC_TARGET_MCU     STM32F7X2 // name of our micro 

#define BOARD_NAME        MAFLY     // any random name i could think of
#define MANUFACTURER_ID   CUST      // i folled this link https://github.com/betaflight/config/blob/master/Manufacturers.md 

#define USE_ACC                     // dont know why we need this line                 
#define USE_ACC_SPI_ICM42688P       // our IMU https://betaflight.com/docs/wiki/getting-started/hardware#gyroscopeaccelerometer

#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P      // our gyro (which is in the same IC as the IMU)

#define USE_MAG
#define USE_MAG_LIS3MDL             // note that this is only using i2c https://betaflight.com/docs/wiki/guides/current/magnetometer#hardware-and-connection
#define MAG_I2C_INSTANCE             (I2CDEV_1)

#define USE_BARO
#define USE_BARO_DPS310             // we use the DPS368 which is the direct replacment of the 310 and has the same default address  https://betaflight.com/docs/wiki/guides/current/barometer
#define BARO_I2C_INSTANCE            (I2CDEV_1)

#define USE_FLASH
#define USE_FLASH_W25Q128FV         // so we are using the USE_FLASH_W25Q128JV and not the ..FV since the FV was out of stock. but there is no define for the JV so i will just use the FV

#define MOTOR1_PIN           PA9    // TIM1_CH2
#define MOTOR2_PIN           PA8    // TIM1_CH1
#define MOTOR3_PIN           PC9    // TIM3_CH4
#define MOTOR4_PIN           PC8    // TIM3_CH3

#define SERVO1_PIN           PA15   // TIM2_CH1
#define SERVO2_PIN           PB3    // TIM2_CH2
#define SERVO3_PIN           PB10   // TIM2_CH3
#define SERVO4_PIN           PB11   // TIM2_CH4

#define UART1_TX_PIN         PB6    // used for the GPS
#define UART1_RX_PIN         PA10

#define UART2_TX_PIN         PA2    // since this is the Sbus it was not recomneded to use the Tx line for anything else 
#define UART2_RX_PIN         PA3    // SBus

#define UART3_TX_PIN         PC10   // not used at the moment
#define UART3_RX_PIN         PC11   // not used at the moment

#define UART4_TX_PIN         PA0    // not used at the moment
#define UART4_RX_PIN         PA1    // not used at the moment

#define UART5_TX_PIN         PC12   // so if the SmartPort on UART6 dont work we can add a resistor and move it to this UART 
#define UART5_RX_PIN         PD2    // not used at the moment

#define UART6_TX_PIN         PC6    // SmartPort this is a single wire half-duplex used for the telemetry protocol

#define I2C1_SCL_PIN         PB8
#define I2C1_SDA_PIN         PB7

#define LED0_PIN             PC3    // this LED is required and should be blue https://betaflight.com/docs/development/manufacturer/manufacturer-design-guidelines#3146-leds
#define LED1_PIN             PC7    

// SPI-1 ACC/GYRI
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE  SPI1
#define GYRO_1_EXTI_PIN      PC4    // external interrupt 
#define GYRO_1_CS_PIN        PA4
#define SPI1_SCK_PIN         PA5   
#define SPI1_SDI_PIN         PA6
#define SPI1_SDO_PIN         PA7

// SPI-2 FLASH
#define FLASH_SPI_INSTANCE   SPI2
#define FLASH_CS_PIN         PB12
#define SPI2_SCK_PIN         PB13
#define SPI2_SDI_PIN         PB14
#define SPI2_SDO_PIN         PB15

#define ADC_VBAT_PIN         PB0

#define USE_LED_STRIP
#define LED_STRIP_PIN        PA2    // TIM5_CH3 this need to be on its own timer and DMA channel https://betaflight.com/docs/development/ledstrip#supported-hardware

/* TIMERS
* https://betaflight.com/docs/development/manufacturer/creating-configuration#timer-and-dma-resources
*
*    TIMER_PIN_MAP(WW, XX, YY, ZZ)

*    WW - zero-indexed counter, increment by 1 for each line in the timer map
*    XX - pin number to set the timer on
*    YY - selected instance of the timer configuration where multiple options are available. See your MCU's target timer code for the list of options.
*    ZZ - DMA setting for that timer, same number as used in the dma pin <pin number> <dma> command. -1 means do not use DMA

*   need this: https://github.com/betaflight/betaflight/blob/master/src/main/drivers/mcu/stm32/timer_stm32f7xx.c
*   new link https://github.com/betaflight/betaflight/blob/fc52b6b4ffa4a5407e2d59e9cd3cd6d73c51a2aa/src/platform/STM32/timer_stm32f7xx.c#L4
*    e.g. if you wanted to set Pin A00 to TIM2_CH1, YY would be "1". For TIM5_CH1, YY would be "2"
*/

// TIMERS
#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA9 , 1,  1) \
    TIMER_PIN_MAP( 1, PA8 , 1,  1) \
    TIMER_PIN_MAP( 2, PC9 , 1,  0) \
    TIMER_PIN_MAP( 3, PC8 , 1,  0) \
    TIMER_PIN_MAP( 4, PA15 , 1,  -1) \
    TIMER_PIN_MAP( 5, PB3 , 1,  -1) \
    TIMER_PIN_MAP( 6, PB10 , 1,  -1) \
    TIMER_PIN_MAP( 7, PB11 , 1,  -1) \
    TIMER_PIN_MAP( 8, PA2 , 2,  0)
     

#define ADC1_DMA_OPT        1

// DEFAULTS
#define DEFAULT_BLACKBOX_DEVICE         BLACKBOX_DEVICE_FLASH 
#define DEFAULT_DSHOT_BURST             DSHOT_DMAR_AUTO
#define DEFAULT_DSHOT_BITBANG           DSHOT_BITBANG_ON // TODO turn this on here instead of having to do it in the CLI
#define DEFAULT_DSHOT_TELEMETRY         DSHOT_TELEMETRY_ON
#define USE_DSHOT // TODO remove if not needed
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC