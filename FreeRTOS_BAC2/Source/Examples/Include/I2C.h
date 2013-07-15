/*
 * I2C.h
 *
 *  Created on: Jun 23, 2013
 *      Author: karl
 */
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+IO includes. */
#include "FreeRTOS_IO.h"

#ifndef I2C
#define I2C

void vI2C_Initialize( void );

Peripheral_Descriptor_t xI2CPort;
xSemaphoreHandle xOLEDMutex;

#endif /* I2C */
