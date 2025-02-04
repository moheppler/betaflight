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

#define FC_TARGET_MCU     STM32H743 // name of our micro 

#define BOARD_NAME        FCHAT     // any random name i could think of
#define MANUFACTURER_ID   CUST      // i folled this link https://github.com/betaflight/config/blob/master/Manufacturers.md 

#define USE_ACC                     // dont know why we need this line                 
#define USE_ACC_SPI_ICM42688P       // our IMU https://betaflight.com/docs/wiki/getting-started/hardware#gyroscopeaccelerometer

#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P      // our gyro (which is in the same IC as the IMU)

#define USE_MAG
#define USE_MAG_LIS2MDL             // note that this is only using i2c https://betaflight.com/docs/wiki/guides/current/magnetometer#hardware-and-connection
#define MAG_I2C_INSTANCE             (I2CDEV_1)

#define USE_BARO
#define USE_BARO_DPS310             // we use the DPS368 which is the direct replacment of the 310 and has the same default address  https://betaflight.com/docs/wiki/guides/current/barometer
#define BARO_I2C_INSTANCE            (I2CDEV_1)

#define USE_FLASH
#define USE_FLASH_W25Q128FV         // so we are using the USE_FLASH_W25Q128JV and not the ..FV since the FV was out of stock. but there is no define for the JV so i will just use the FV

#define MOTOR1_PIN           PA3    // TIM5_CH4
#define MOTOR2_PIN           PA2    // TIM5_CH3
#define MOTOR3_PIN           PA1    // TIM5_CH2
#define MOTOR4_PIN           PA0    // TIM5_CH1

#define MOTOR5_PIN           PB0    // TIM3_CH3
#define MOTOR6_PIN           PB1    // TIM3_CH4
#define MOTOR7_PIN           PD12   // TIM4_CH1
#define MOTOR8_PIN           PD13   // TIM4_CH2

#define SERVO1_PIN           PE9    // TIM1_CH1
#define SERVO2_PIN           PE11   // TIM1_CH2
#define SERVO3_PIN           PE13   // TIM1_CH3
#define SERVO4_PIN           PE14   // TIM1_CH4

//#define SERVO5_PIN           PB14    // TIM12_CH1 // NB: there is no define for servos over 5 so this has to be assigned in the CLI 
//#define SERVO6_PIN           PB15    // TIM12_CH2

#define UART1_TX_PIN         PB6    // not used
#define UART1_RX_PIN         PB7    // Sbus input 

#define UART2_TX_PIN         PD5    // SmartPort output 
#define UART2_RX_PIN         PD6    // not used

#define UART3_TX_PIN         PB10   // GPS
#define UART3_RX_PIN         PB11    

#define UART4_TX_PIN         PC10   // Spare
#define UART4_RX_PIN         PC11    

#define I2C1_SCL_PIN         PB8
#define I2C1_SDA_PIN         PB9

#define LED0_PIN             PE0    // this LED is required and should be blue https://betaflight.com/docs/development/manufacturer/manufacturer-design-guidelines#3146-leds
#define LED1_PIN             PE1    

#define PINIO1_PIN           PD2    
#define PINIO2_PIN           PD3   
#define PINIO3_PIN           PD4    

//#define USE_LED_STRIP
#define LED_STRIP_PIN        PC6    // TIM8_CH1 this need to be on its own timer and DMA channel https://betaflight.com/docs/development/ledstrip#supported-hardware

#define ADC_VBAT_PIN         PC0
#define ADC_EXTERNAL1_PIN    PC1    // spare

// not sure how the CAN should be initialized 
// #define FDCAN1_RX            PD0    
// #define FDCAN1_TX            PD1

// SPI-4 ACC/GYRI
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE  SPI4
#define GYRO_1_EXTI_PIN      PE3    // external interrupt 
#define GYRO_1_CS_PIN        PE4
#define SPI4_SCK_PIN         PE2   
#define SPI4_SDI_PIN         PE5    // MISO
#define SPI4_SDO_PIN         PE6    // MOSI

// SPI-1 FLASH
#define FLASH_SPI_INSTANCE   SPI1
#define FLASH_CS_PIN         PA4
#define SPI1_SCK_PIN         PA5
#define SPI1_SDI_PIN         PA6   // MISO
#define SPI1_SDO_PIN         PA7   // MOSI

/* TIMERS
* https://betaflight.com/docs/development/manufacturer/creating-configuration#timer-and-dma-resources
*
*    TIMER_PIN_MAP(WW, XX, YY, ZZ)

*    WW - zero-indexed counter, increment by 1 for each line in the timer map
*    XX - pin number to set the timer on
*    YY - selected instance of the timer configuration where multiple options are available. See your MCU's target timer code for the list of options.
*    ZZ - DMA setting for that timer, same number as used in the dma pin <pin number> <dma> command. -1 means do not use DMA
*   
*   e.g. for YY if you wanted to set Pin A00 to TIM2_CH1, YY would be "1". For TIM5_CH1, YY would be "2"
*   
*   link: 
*   STM32F7
*   https://github.com/betaflight/betaflight/blob/fc52b6b4ffa4a5407e2d59e9cd3cd6d73c51a2aa/src/platform/STM32/timer_stm32f7xx.c#L4
*   
*   STM32H7
*   https://github.com/betaflight/betaflight/blob/fc52b6b4ffa4a5407e2d59e9cd3cd6d73c51a2aa/src/platform/STM32/timer_stm32h7xx.c
*    
*   For STMH7 the DMA number is as follow 
*   DMA1_Stream_x -> DMAMUX1_Channel_x
*   DMA2_Stream_x -> DMAMUX1_Channel_(x+8)
*   Look up each pin and DMA using STM32CubeMX
*/

// TIMERS
#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA3 , 2,  3) \
    TIMER_PIN_MAP( 1, PA2 , 2,  2) \
    TIMER_PIN_MAP( 2, PA1 , 2,  1) \
    TIMER_PIN_MAP( 3, PA0 , 2,  0) \
    TIMER_PIN_MAP( 4, PB0 , 2,  10) \
    TIMER_PIN_MAP( 5, PB1 , 2,  11) \
    TIMER_PIN_MAP( 6, PD12 , 1, 4) \
    TIMER_PIN_MAP( 7, PD13 , 1, 5) \
    TIMER_PIN_MAP( 8, PE9 , 1,  9) \
    TIMER_PIN_MAP( 9, PE11 , 1,  7) \
    TIMER_PIN_MAP( 10, PE13 , 1, -1) \
    TIMER_PIN_MAP( 11, PE14 , 1, -1) \
    TIMER_PIN_MAP( 11, PC6 , 2, 8)


#define ADC1_DMA_OPT        12 // DMA2 Stream 4
//#define TIMUP1_DMA_OPT      0
//#define TIMUP3_DMA_OPT      0
//#define TIMUP4_DMA_OPT      0
//#define TIMUP5_DMA_OPT      0


#define PINIO1_BOX 40   // BOX: BOXUSER1 MODE:USER1 
#define PINIO2_BOX 41   // BOX: BOXUSER2 MODE:USER2
#define PINIO3_BOX 42   // BOX: BOXUSER3 MODE:USER3

// DEFAULTS
#define DEFAULT_BLACKBOX_DEVICE         BLACKBOX_DEVICE_FLASH 
#define DEFAULT_DSHOT_BURST             DSHOT_DMAR_AUTO
#define DEFAULT_DSHOT_BITBANG           DSHOT_BITBANG_ON        // need to be ON so we can get back RPM data from the ESC
#define DEFAULT_DSHOT_TELEMETRY         DSHOT_TELEMETRY_ON

#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC