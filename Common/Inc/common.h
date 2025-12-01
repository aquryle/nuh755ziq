/***************************************************************************//**
 * @file	common.h
 * @brief
 * @date	2025/11/27
 * @author	sata
 * @version	1.00
*******************************************************************************/

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define AUDIO_CHANNEL_NUM	(2)
#define WAVE_BUFFER_SIZE	(480 * AUDIO_CHANNEL_NUM)



#define BIT0	 (1ul << 0)
#define BIT1	 (1ul << 1)
#define BIT2	 (1ul << 2)
#define BIT3	 (1ul << 3)
#define BIT4	 (1ul << 4)
#define BIT5	 (1ul << 5)
#define BIT6	 (1ul << 6)
#define BIT7	 (1ul << 7)
#define BIT8	 (1ul << 8)
#define BIT9	 (1ul << 9)
#define BIT10	 (1ul << 10)
#define BIT11	 (1ul << 11)
#define BIT12	 (1ul << 12)
#define BIT13	 (1ul << 13)
#define BIT14	 (1ul << 14)
#define BIT15	 (1ul << 15)
#define BIT16	 (1ul << 16)
#define BIT17	 (1ul << 17)
#define BIT18	 (1ul << 18)
#define BIT19	 (1ul << 19)
#define BIT20	 (1ul << 20)
#define BIT21	 (1ul << 21)
#define BIT22	 (1ul << 22)
#define BIT23	 (1ul << 23)
#define BIT24	 (1ul << 24)
#define BIT25	 (1ul << 25)
#define BIT26	 (1ul << 26)
#define BIT27	 (1ul << 27)
#define BIT28	 (1ul << 28)
#define BIT29	 (1ul << 29)
#define BIT30	 (1ul << 30)
#define BIT31	 (1ul << 31)

typedef enum {
	NOTE_A0 = 0,
	NOTE_B0 = 0,
	NOTE_C0 = 0,
	NOTE_D0 = 0,
	NOTE_E0 = 0,
	NOTE_F0 = 0,
	NOTE_G0 = 0,
} Note_t;

#endif /* INC_COMMON_H_ */
