/***************************************************************************//**
 * @file	pcm5102a.c
 * @brief
 * @date	2025/12/03
 * @author	sata
 * @version	1.00
*******************************************************************************/
#include "common.h"
#include "main.h"
#include "pcm5102a.h"

static PCM5102A_HandleTypeDef deviceHandler = {0};

static bool isNull(PCM5102A_HandleTypeDef *p)
{
	if (p->hsai != NULL) return false;
	if (p->fltPort != NULL) return false;
	if (p->fltPin != 0) return false;

	return true;
}

/**
 * @brief	PCM5102A用ハンドラーを指定する
 *
 * @param	handle
 * @return	0: OK
 * @example
 * 	PCM5102A_HandleTypeDef h;
 * 	h.hsai = &hsai_BlockA1;
 * 	h.fltPort = &GPIOB;
 * 	h.fltPin = GPIO_PIN0;
 * 	if (!PCM5102A_Initialize(&h)){エラー処理}
 *
 * @todo	ここでGPIOやI2Sの初期化もすべき。今はmain()で正しく設定されている前提で動いてる
 */
int PCM5102A_Initialize(PCM5102A_HandleTypeDef *h)
{
	if (isNull(&deviceHandler) == false) {
		return -1;
	}

	deviceHandler.hsai = h->hsai;
	deviceHandler.fltPort = h->fltPort;
	deviceHandler.fltPin = h->fltPin;

	return 0;
}

/**
 * @brief	PCM5102A用ハンドラーを無効化する
 *
 * @todo	ここでGPIOやI2Sの機能停止した方がいい
 */
void PCM5102A_Deinitialize(void)
{
	deviceHandler = (PCM5102A_HandleTypeDef){0};
}


