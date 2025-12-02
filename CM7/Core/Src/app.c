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
#include <stdlib.h>

#include "st7789.h"
#include "pcm5102a.h"


extern QSPI_HandleTypeDef hqspi;
extern SAI_HandleTypeDef hsai_BlockA1;
extern TIM_HandleTypeDef htim13;
extern int16_t WaveBuffer[WAVE_BUFFER_SIZE];

static void waveGeneration(Note_t note);

#define TABLE_SIZE 256
#define PI 3.1415926535f
#define SAMPLE_RATE 48000
#define BUFFER_SIZE 480  // 1kHzなら48サンプルで1周期、480で10周期分

int16_t wavetable[TABLE_SIZE];

// テスト用バッファ（16bitステレオ）
int16_t audio_buffer[BUFFER_SIZE * 2]; // L,R交互で2倍
float phase = 0.0f;
float phase_inc = 0.0f;

uint8_t is_playing = 0; // 再生中フラグ
extern volatile uint8_t b1state;    // B1を押されたらインクリメント

const float scale_freqs[] = {
	261.63, // ド (C4)
	293.66, // レ (D4)
	329.63, // ミ (E4)
	349.23, // ファ (F4)
	392.00, // ソ (G4)
	440.00, // ラ (A4)
	493.88, // シ (B4)
	523.25  // ド (C5)
};
uint8_t current_note = 0;

void set_frequency(float freq)
{
	phase_inc = 2.0f * M_PI * freq / SAMPLE_RATE;
}


/// @brief 波形を生成して波形バッファに格納する
/// @param freq 周波数
void generate_sinewave(float freq)
{
#if 1
	float sample;   // 時間における波の高さ（電位？）
	int16_t val;    // バッファに格納する値
	size_t i;

	for (i = 0; i < BUFFER_SIZE; i++) {
		sample = sinf(2.0f * M_PI * freq * i / SAMPLE_RATE);    // y = Asin(2πft + ϕ)
		val = (int16_t)(sample * 0x7fff);                       // シリアライズ
		audio_buffer[2 * i] = val;                              // LRのデータを連続して格納
		audio_buffer[2 * i + 1] = val;
	}
#else
	for (int i = 0; i < BUFFER_SIZE; i++) {
		float sample = sinf(phase);
		phase += phase_inc;
		if (phase >= 2.0f * M_PI)
			phase -= 2.0f * M_PI;

		int16_t val = (int16_t)(sample * 32767);
		audio_buffer[2 * i] = val;
		audio_buffer[2 * i + 1] = val;
	}
#endif
}

void waveGeneration(Note_t note)
{
	int16_t lv = 0;
	size_t i = 0;
	while (i < WAVE_BUFFER_SIZE) {
		lv = (int16_t)note;

		WaveBuffer[i] = lv;		// L
		i++;
		WaveBuffer[i] = lv;		// R
	}
}


void test(void)
{
	uint32_t tick = HAL_GetTick();
	generate_sinewave(scale_freqs[0]);
	while (1)
	{
		if (HAL_GetTick() - tick > 1000) {
			current_note = (current_note + 1) % 8;
			// set_frequency(scale_freqs[current_note]);
			generate_sinewave(scale_freqs[current_note]);
			tick = HAL_GetTick();
		}
		if (b1state % 2) {
			HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)audio_buffer, BUFFER_SIZE * 2);
		}
		else {
			HAL_SAI_DMAStop(&hsai_BlockA1);
			// current_note = (current_note + 1) % 8;
			// generate_sinewave(scale_freqs[current_note]);
		}
	}
}

void app(void)
{
	test();

	printf("Hello, from CM7/app()\n");

	printf("Wave form generation\n");
	// waveGeneration(NOTE_C0);

	printf("\n");
	printf("\n");

	return;
}
