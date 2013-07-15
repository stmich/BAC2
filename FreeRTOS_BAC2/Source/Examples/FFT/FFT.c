/*
 * FFT.c
 *
 *  Created on: Jun 23, 2013
 *      Author: Michael Steinberger
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+IO includes. */
#include "FreeRTOS_IO.h"

/* Library includes. */
#include "lpc17xx_gpio.h"
#include "arm_math.h"
#include <stdlib.h>

/* Example includes. */
#include "FFT.h"
#include "I2C.h"
#include "I2C-to-OLED.h"
#include "oled.h"

/* Place holder for calls to ioctl that don't use the value parameter. */
#define i2cPARAMETER_NOT_USED   ( ( void * ) 0 )


#define TEST_LENGTH_SAMPLES 2048


/*-----------------------------------------------------------*/

/*
 * The task that demonstrates how to create and read files using all supported
 * FreeRTOS+IO transfer methods.
 */
static void prvCalculateFFTTask(void *pvParameters);
void prvPutResults(uint32_t lMaxIndex, float32_t xMaxValue );

/*-----------------------------------------------------------*/
extern Peripheral_Descriptor_t xI2CPort;

/* -------------------------------------------------------------------
* External Input and Output buffer Declarations for FFT Bin Example
* ------------------------------------------------------------------- */
extern float32_t xSqrSignal[TEST_LENGTH_SAMPLES];

static float32_t xFFTMagOutput[TEST_LENGTH_SAMPLES/2];


/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */
uint32_t fftSize = 1024;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

/*-----------------------------------------------------------*/
void vFFTTaskStart(void)
{
  /* Create the prvCalculateFFTTask task.   */
  xTaskCreate(    prvCalculateFFTTask,                     /* The task that uses the SSP in SPI mode to implement a file system disk IO driver. */
                    ( const int8_t * const ) "FFT_Clc",        /* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
                    configFFT_TASK_STACK_SIZE,      /* The size of the stack allocated to the task. */
                    NULL,                                   /* The parameter is not used, so NULL is passed. */
                    configFFT_TASK_PRIORITY,   /* The priority allocated to the task. */
                    NULL );
}

/*-----------------------------------------------------------*/
static void prvCalculateFFTTask(void *pvParameters)
{
  (void) pvParameters;
  const uint32_t ulMaxDelay = 10000UL / portTICK_RATE_MS;
  arm_status status;
  arm_cfft_radix4_instance_f32 xCFTTStructure;
  float32_t * xFFTInput;

  /* Index at which max energy of bin occurs */
  uint32_t lMaxIndex = 0;
  float32_t xMaxValue;

  for (;;)
  {
    status = ARM_MATH_SUCCESS;

    /* Copy data to save original input data (inplace transform)*/
    xFFTInput = (float32_t*) pvPortMalloc( sizeof( xSqrSignal ) );
    memcpy( xFFTInput, &xSqrSignal, sizeof(xSqrSignal) );

    /* Initialize the CFFT/CIFFT module */
    status = arm_cfft_radix4_init_f32(&xCFTTStructure, fftSize, ifftFlag,
            doBitReverse);

    /* Process the data through the CFFT/CIFFT module */
    arm_cfft_radix4_f32(&xCFTTStructure, xFFTInput);

    /* Process the data through the Complex Magnitude Module for
     calculating the magnitude at each bin */
    arm_cmplx_mag_f32(xFFTInput, xFFTMagOutput, fftSize);
    vPortFree(xFFTInput);

    /* Calculates maxValue and returns corresponding BIN value */
    arm_max_f32(xFFTMagOutput, fftSize, &xMaxValue, &lMaxIndex);


    if (xSemaphoreTake( xOLEDMutex, portMAX_DELAY ) == pdTRUE)
    {
        /* The OLED must be initialized before it is used. */
        vI2C_OLEDInitialize(xI2CPort);

        prvPutResults( lMaxIndex, xMaxValue );
        vOLEDRefreshDisplay();
        xSemaphoreGive(xOLEDMutex);
    }
    vTaskDelay(ulMaxDelay);
  }
}
/*-----------------------------------------------------------*/
void prvPutResults(uint32_t lMaxIndex, float32_t xMaxValue )
{
    int8_t cCharBufferValue[20];
    int8_t cCharBufferText[20];

    strcpy(cCharBufferText, "max.Bin: ");
    //uint8_t * cTextIndex = "max. BinIndex: ", "max. Magnitude: " };

    vOLEDPutString(0U, (uint8_t *) "Calculating FFT", OLED_COLOR_WHITE,
            OLED_COLOR_BLACK);
    vOLEDPutString(oledCHARACTER_HEIGHT, (uint8_t *) "-------------------------------",
            OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    itoa(lMaxIndex, cCharBufferValue, 10);
    strcat(cCharBufferText, cCharBufferValue);
    vOLEDPutString(2*oledCHARACTER_HEIGHT, (uint8_t *) cCharBufferText,
            OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    strcpy(cCharBufferText, "max.Magn.: ");
    itoa(xMaxValue, cCharBufferValue, 10);
    strcat(cCharBufferText, cCharBufferValue);
    vOLEDPutString(3*oledCHARACTER_HEIGHT, (uint8_t *) cCharBufferText,
            OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}
