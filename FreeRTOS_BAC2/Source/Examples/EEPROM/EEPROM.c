/*
    FreeRTOS V7.3.0 - Copyright (C) 2012 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTO+IO includes. */
#include "FreeRTOS_IO.h"

/* Library includes. */
#include "lpc17xx_gpio.h"

/* Example includes. */
#include "EEPROM.h"
#include "I2C.h"
#include "I2C-to-OLED.h"
#include "I2C-to-and-from-EEPROM.h"
#include "oled.h"

/*
 * The task that uses the I2C to communicate with the OLED.
 */
static void prvEEPROMTask( void *pvParameters );

extern Peripheral_Descriptor_t xI2CPort;
/*-----------------------------------------------------------*/

/* Place holder for calls to ioctl that don't use the value parameter. */
#define i2cPARAMETER_NOT_USED           ( ( void * ) 0 )

/* The size, in bytes, of the circular buffer used when the I2C port is using
circular buffer Rx mode. */
#define i2cCIRCULAR_BUFFER_SIZE         ( ( void * ) 50 )

/* The maximum amount of time to wait to receive the requested number of bytes
when using zero copy receive mode. */
#define i2c200MS_TIMEOUT                ( ( void * ) ( 200UL / portTICK_RATE_MS ) )
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

void vEEPROMTaskStart( void )
{
	/* Just create the I2C task, then return.  The I2C task communicates with
	the OLED and the EEPROM. */
	xTaskCreate( 	prvEEPROMTask,								/* The task that uses the I2C peripheral. */
					( const int8_t * const ) "ROM",	/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
					configEEPROM_TASK_STACK_SIZE,				/* The size of the stack allocated to the task. */
					NULL,									/* The parameter is not used, so NULL is passed. */
					configEEPROM_TASK_PRIORITY,				/* The priority allocated to the task. */
					NULL );									/* A handle to the task being created is not required, so just pass in NULL. */
}
/*-----------------------------------------------------------*/

static void prvEEPROMTask(void *pvParameters)
{
    const uint32_t ulMaxDelay = 10000UL / portTICK_RATE_MS;

    (void) pvParameters;

    for (;;)
    {
//        FreeRTOS_ioctl(xI2CPort, ioctlRELEASE_WRITE_MUTEX,
//                i2cPARAMETER_NOT_USED );
        /* Write and read-back operations are to be performed on the EEPROM while
         the I2C bus is in interrupt driven zero copy Tx, and interrupt driven
         circular buffer Rx mode.  Indicate this on the OLED. */

        if (xSemaphoreTake( xOLEDMutex, portMAX_DELAY ) == pdTRUE)
        {
            vI2C_OLEDInitialize(xI2CPort);
            vOLEDPutString(0U, (uint8_t *) "Testing EEPROM", OLED_COLOR_WHITE,
                    OLED_COLOR_BLACK);
            vOLEDPutString(oledCHARACTER_HEIGHT, (uint8_t *) "in intrpt mode",
                    OLED_COLOR_WHITE, OLED_COLOR_BLACK);

            /* Using zero copy Tx mode means the write mutex must be obtained prior to
             calling vOLEDRefreshDisplay().  The write mutex is retained when
             vOLEDRefreshDisplay() returns. */
            vOLEDRefreshDisplay();

            /* Perform the interrupt driven mode EEPROM tests/examples. */
            vI2C_EEPROMTest(xI2CPort);

            xSemaphoreGive(xOLEDMutex);
        }
        vTaskDelay(ulMaxDelay);
    }
}
/*-----------------------------------------------------------*/

