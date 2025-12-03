/***************************************************************************//**
 * @file	pcm5102a.h
 * @brief
 * @date	2025/12/03
 * @author	sata
 * @version	1.00
*******************************************************************************/

#ifndef BSP_CUSTOM_PCM5102A_H_
#define BSP_CUSTOM_PCM5102A_H_

#include "common.h"
#include "main.h"


typedef struct {
	SAI_HandleTypeDef *hsai;
	GPIO_TypeDef *fltPort;
	uint16_t fltPin;
} PCM5102A_HandleTypeDef;


extern int PCM5102A_Initialize(PCM5102A_HandleTypeDef *h);
extern void PCM5102A_Deinitialize(void);

#endif /* BSP_CUSTOM_PCM5102A_H_ */
