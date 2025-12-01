/***************************************************************************//**
 * @file	app.c
 * @brief
 * @date	2025/11/27
 * @author	sata
 * @version	1.00
*******************************************************************************/
#include <stdio.h>
#include "common.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"
#include <math.h>
#include <string.h>

extern QSPI_HandleTypeDef hqspi;
extern SAI_HandleTypeDef hsai_BlockA1;
extern TIM_HandleTypeDef htim13;
extern int16_t WaveBuffer[WAVE_BUFFER_SIZE];

static void waveGeneration(Note_t note);


void waveGeneration(Note_t note)
{
	int16_t lv = 0;
	size_t i = 0;
	while (i < WaveBuffer) {
		lv = (int16_t)note;

		WaveBuffer[i] = lv;		// L
		i++;
		WaveBuffer[i] = lv;		// R
	}
}


void app(void)
{
	printf("Hello, from CM7/app()\n");

	printf("Wave form generation\n");
	waveGeneration(NOTE_C0);

	printf("\n");
	printf("\n");

	return;
}
