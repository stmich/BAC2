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

/* FreeRTOS+IO includes. */
#include "FreeRTOS_IO.h"

/* Library includes. */
#include "lpc17xx_gpio.h"

/* Example includes. */
#include "I2C.h"
#include "I2C-to-OLED.h"

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

extern Peripheral_Descriptor_t xI2CPort;
extern xSemaphoreHandle xOLEDMutex;

/*-----------------------------------------------------------*/

void vI2C_Initialize( void )
{

    /* Open the I2C port used for writing to both the OLED and the EEPROM.  The
     second parameter (ulFlags) is not used in this case.  The port is opened in
     polling mode.  It is changed to interrupt driven mode later in this
     function. */
    xI2CPort = FreeRTOS_open(board_I2C_PORT,
            (uint32_t) i2cPARAMETER_NOT_USED);
    configASSERT( xI2CPort );

    /* Switch to interrupt driven zero copy Tx mode and interrupt driven
     circular buffer Rx mode (with a limited time out). */
    FreeRTOS_ioctl(xI2CPort, ioctlUSE_ZERO_COPY_TX, i2cPARAMETER_NOT_USED);
    FreeRTOS_ioctl(xI2CPort, ioctlUSE_CIRCULAR_BUFFER_RX,
            i2cCIRCULAR_BUFFER_SIZE);
    FreeRTOS_ioctl(xI2CPort, ioctlSET_RX_TIMEOUT, i2c200MS_TIMEOUT);

    /* By default, the I2C interrupt priority will have been set to
     the lowest possible.  It must be kept at or below
     configMAX_LIBRARY_INTERRUPT_PRIORITY, but can be raised above
     its default priority using a FreeRTOS_ioctl() call with the
     ioctlSET_INTERRUPT_PRIORITY command. */
    FreeRTOS_ioctl(xI2CPort, ioctlSET_INTERRUPT_PRIORITY,
            (void *) configI2C_INTERRUPT_PRIORITY);

    xOLEDMutex = xSemaphoreCreateMutex();
}
