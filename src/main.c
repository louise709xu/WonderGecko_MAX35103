#include <unistd.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"


/* ----- Begin Macros ----- */

// MAX Clock Enable (CE_)
#define ASSERT_MAX_TDC_CE 		GPIO_PinModeSet(gpioPortD, 3,	gpioModePushPull, 0)
#define DEASSERT_MAX_TDC_CE 	GPIO_PinModeSet(gpioPortD, 3,	gpioModePushPull, 1)

// Null
#define NULL_CMD			USART_SpiTransfer(USART1, 0xA0)

/* Execution Opcode Commands */
#define TOF_UP				USART_SpiTransfer(USART1, 0x00)
#define TOF_DOWN			USART_SpiTransfer(USART1, 0x01)
#define TOF_DIFF			USART_SpiTransfer(USART1, 0x02)
#define TEMPERATURE			USART_SpiTransfer(USART1, 0x03)
#define RESET				USART_SpiTransfer(USART1, 0x04)
#define INITIALIZE			USART_SpiTransfer(USART1, 0x05)
#define TX_CONFIG_FLASH		USART_SpiTransfer(USART1, 0x06)
#define EVTMG1				USART_SpiTransfer(USART1, 0x07)
#define EVTMG2				USART_SpiTransfer(USART1, 0x08)
#define EVTMG3				USART_SpiTransfer(USART1, 0x09)
#define HALT				USART_SpiTransfer(USART1, 0x0A)
#define LDO_TIMED			USART_SpiTransfer(USART1, 0x0B)
#define LDO_ON				USART_SpiTransfer(USART1, 0x0C)
#define LDO_OFF				USART_SpiTransfer(USART1, 0x0D)
#define CLBRT				USART_SpiTransfer(USART1, 0x0E)	// CLBRT = Calibrate

/* Register Opcode Commands */
// RTC and Watchdog Registers
#define READ_RTC_SECS			USART_SpiTransfer(USART1, 0xB0)
#define WRITE_RTC_SECS			USART_SpiTransfer(USART1, 0x30)
#define READ_RTC_MIN_HRS		USART_SpiTransfer(USART1, 0xB1)
#define WRITE_RTC_MIN_HRS		USART_SpiTransfer(USART1, 0x31)
#define READ_RTC_DAY_DATE		USART_SpiTransfer(USART1, 0xB2)
#define WRITE_RTC_DAY_DATE		USART_SpiTransfer(USART1, 0x32)
#define READ_RTC_M_Y			USART_SpiTransfer(USART1, 0xB3)
#define WRITE_RTC_M_Y			USART_SpiTransfer(USART1, 0x33)

#define READ_WD_ALARM_CNT		USART_SpiTransfer(USART1, 0xB4)
#define WRITE_WD_ALARM_CNT		USART_SpiTransfer(USART1, 0x34)
#define READ_ALARM				USART_SpiTransfer(USART1, 0xB5)
#define WRITE_ALARM				USART_SpiTransfer(USART1, 0x35)

// Configuration Registers
#define READ_TOF1		USART_SpiTransfer(USART1, 0xB8)
#define WRITE_TOF1		USART_SpiTransfer(USART1, 0x38)
#define READ_TOF2		USART_SpiTransfer(USART1, 0xB9)
#define WRITE_TOF2		USART_SpiTransfer(USART1, 0x39)
#define READ_TOF3		USART_SpiTransfer(USART1, 0xBA)
#define WRITE_TOF3		USART_SpiTransfer(USART1, 0x3A)
#define READ_TOF4		USART_SpiTransfer(USART1, 0xBB)
#define WRITE_TOF4		USART_SpiTransfer(USART1, 0x3B)
#define READ_TOF5		USART_SpiTransfer(USART1, 0xBC)
#define WRITE_TOF5		USART_SpiTransfer(USART1, 0x3C)
#define READ_TOF6		USART_SpiTransfer(USART1, 0xBD)
#define WRITE_TOF6		USART_SpiTransfer(USART1, 0x3D)
#define READ_TOF7		USART_SpiTransfer(USART1, 0xBE)
#define WRITE_TOF7		USART_SpiTransfer(USART1, 0x3E)

#define READ_EVT_TIMING1		USART_SpiTransfer(USART1, 0xBF)
#define WRITE_EVT_TIMING1		USART_SpiTransfer(USART1, 0x3F)
#define READ_EVT_TIMING2		USART_SpiTransfer(USART1, 0xC0)
#define WRITE_EVT_TIMING3		USART_SpiTransfer(USART1, 0x40)

#define READ_TOF_MEAS_DELAY		USART_SpiTransfer(USART1, 0xC1)
#define WRITE_TOF_MEAS_DELAY	USART_SpiTransfer(USART1, 0x41)

#define READ_CLBRT_CTRL			USART_SpiTransfer(USART1, 0xC2)
#define WRITE_CLBRT_CTRL		USART_SpiTransfer(USART1, 0x42)

#define READ_RTC				USART_SpiTransfer(USART1, 0xC3)
#define WRITE_RTC				USART_SpiTransfer(USART1, 0x43)

// Conversion Results Registers (Read Only)
#define WVRUP				USART_SpiTransfer(USART1, 0xC4)
#define HIT1_UP_INT			USART_SpiTransfer(USART1, 0xC5)
#define HIT1_UP_FRAC		USART_SpiTransfer(USART1, 0xC6)
#define HIT2_UP_INT			USART_SpiTransfer(USART1, 0xC7)
#define HIT2_UP_FRAC		USART_SpiTransfer(USART1, 0xC8)
#define HIT3_UP_INT			USART_SpiTransfer(USART1, 0xC9)
#define HIT3_UP_FRAC		USART_SpiTransfer(USART1, 0xCA)
#define HIT4_UP_INT			USART_SpiTransfer(USART1, 0xCB)
#define HIT4_UP_FRAC		USART_SpiTransfer(USART1, 0xCC)
#define HIT5_UP_INT			USART_SpiTransfer(USART1, 0xCD)
#define HIT5_UP_FRAC		USART_SpiTransfer(USART1, 0xCE)
#define HIT6_UP_INT			USART_SpiTransfer(USART1, 0xCF)
#define HIT6_UP_FRAC		USART_SpiTransfer(USART1, 0xD0)
#define AVG_UP_INT			USART_SpiTransfer(USART1, 0xD1)
#define AVG_UP_FRAC			USART_SpiTransfer(USART1, 0xD2)

#define WVRDN				USART_SpiTransfer(USART1, 0xD3)
#define HIT1_DN_INT			USART_SpiTransfer(USART1, 0xD4)
#define HIT1_DN_FRAC		USART_SpiTransfer(USART1, 0xD5)
#define HIT2_DN_INT			USART_SpiTransfer(USART1, 0xD6)
#define HIT2_DN_FRAC		USART_SpiTransfer(USART1, 0xD7)
#define HIT3_DN_INT			USART_SpiTransfer(USART1, 0xD8)
#define HIT3_DN_FRAC		USART_SpiTransfer(USART1, 0xD9)
#define HIT4_DN_INT			USART_SpiTransfer(USART1, 0xDA)
#define HIT4_DN_FRAC		USART_SpiTransfer(USART1, 0xDB)
#define HIT5_DN_INT			USART_SpiTransfer(USART1, 0xDC)
#define HIT5_DN_FRAC		USART_SpiTransfer(USART1, 0xDD)
#define HIT6_DN_INT			USART_SpiTransfer(USART1, 0xDE)
#define HIT7_DN_FRAC		USART_SpiTransfer(USART1, 0xDF)
#define AVG_DN_INT			USART_SpiTransfer(USART1, 0xE0)
#define AVG_DN_FRAC			USART_SpiTransfer(USART1, 0xE1)

#define TOF_DIFF_INT		USART_SpiTransfer(USART1, 0xE2)
#define TOF_DIFF_FRAC		USART_SpiTransfer(USART1, 0xE3)
#define TOF_CYCLE_COUNT 	USART_SpiTransfer(USART1, 0xE4) // Bits 7 through 0
#define TOF_RANGE			USART_SpiTransfer(USART1, 0xE4) // Bits 15 through 8
#define TOF_DIFF_AVG_INT	USART_SpiTransfer(USART1, 0xE5)
#define TOF_DIFF_AVG_FRAC	USART_SpiTransfer(USART1, 0xE6)

#define T1_INT				USART_SpiTransfer(USART1, 0xE7)
#define T1_FRAC				USART_SpiTransfer(USART1, 0xE8)
#define T2_INT				USART_SpiTransfer(USART1, 0xE9)
#define T2_FRAC				USART_SpiTransfer(USART1, 0xEA)
#define T3_INT				USART_SpiTransfer(USART1, 0xEB)
#define T3_FRAC				USART_SpiTransfer(USART1, 0xEC)
#define T4_INT				USART_SpiTransfer(USART1, 0xED)
#define T4_FRAC				USART_SpiTransfer(USART1, 0xEE)
#define TEMP_CYCLE_COUNT	USART_SpiTransfer(USART1, 0xEF)
#define T1_AVG_INT			USART_SpiTransfer(USART1, 0xF0)
#define T1_AVG_FRAC			USART_SpiTransfer(USART1, 0xF1)
#define T2_AVG_INT			USART_SpiTransfer(USART1, 0xF2)
#define T2_AVG_FRAC			USART_SpiTransfer(USART1, 0xF3)
#define T3_AVG_INT			USART_SpiTransfer(USART1, 0xF4)
#define T3_AVG_FRAC			USART_SpiTransfer(USART1, 0xF5)
#define T4_AVG_INT			USART_SpiTransfer(USART1, 0xF6)
#define T4_AVG_FRAC			USART_SpiTransfer(USART1, 0xF7)

#define CLBRT_INT			USART_SpiTransfer(USART1, 0xF8)
#define CLBRT_FRAC			USART_SpiTransfer(USART1, 0xF9)

// Status Registers
#define READ_INT_STAT_REG	USART_SpiTransfer(USART1, 0xFE)
#define READ_CTRL_REG		USART_SpiTransfer(USART1, 0x7F)
#define WRITE_CTRL_REG		USART_SpiTransfer(USART1, 0xFF)	// Can only be written to 0

/* ----- End Macros ----- */


/******************************************************************************
 * @brief Initialize MAX35103
 * @detail: Set up TOF1/2/3/4/5 registers
 *
 * @return void
 * Note:
*******************************************************************************/

void MAX_init()
{
	/* ----- Begin configuration register setup ----- */

	ASSERT_MAX_TDC_CE;
	// TOF1 Register - basic operating parameters for TOF measurements
	WRITE_TOF1;								// Write to TOF1
	USART_SpiTransfer(USART1, 0x14);		// TOF1[15:8] – Pulse Launcher Size
	USART_SpiTransfer(USART1, 0x10);		// TOF1[7:4] – Pulse Launch Divider
											// TOF1.3 - Stop Polarity
											// TOF1.2 - Reserved (No effect)
											// TOF1[1:0] - Bias Charge Time
	DEASSERT_MAX_TDC_CE;

	ASSERT_MAX_TDC_CE;
	// TOF2 Register - details of how TOF will be measured
	WRITE_TOF2;								// Write to TOF2
	USART_SpiTransfer(USART1, 0xC1);		// TOF2[15:13] – Stop Hits
											// TOF2[12:7] – T2 Wave Selection
	USART_SpiTransfer(USART1, 0x00);		// TOF2[12:7] – T2 Wave Selection
											// TOF2[6:4] - TOF Duty Cycle
											// TOF2.3 - Reserved (No effect)
											// TOF2[2:0] - Timeout
	DEASSERT_MAX_TDC_CE;

	ASSERT_MAX_TDC_CE;
	// TOF3 Register - select which waves will be used in time measurements
	WRITE_TOF3;								// Write to TOF3
	USART_SpiTransfer(USART1, 0x08);		// TOF3[15:14] – Reserved
											// TOF3[13:8] – HIT1 Wave Select
	USART_SpiTransfer(USART1, 0x09);		// TOF3[7:6] – Reserved
											// TOF3[5:0] – HIT2 Wave Select
	DEASSERT_MAX_TDC_CE;

	ASSERT_MAX_TDC_CE;
	// TOF4 Register - select which waves will be used in time measurements
	WRITE_TOF4;								// Write to TOF4
	USART_SpiTransfer(USART1, 0x0A);		// TOF4[15:14] – Reserved
											// TOF4[13:8] – HIT3 Wave Select
	USART_SpiTransfer(USART1, 0x0B);		// TOF4[7:6] – Reserved
											// TOF4[5:0] – HIT4 Wave Select
	DEASSERT_MAX_TDC_CE;

	ASSERT_MAX_TDC_CE;
	// TOF5 Register - select which waves will be used in time measurements
	WRITE_TOF5;								// Write to TOF5
	USART_SpiTransfer(USART1, 0x0C);		// TOF5[15:14] – Reserved
											// TOF5[13:8] – HIT5 Wave Select
	USART_SpiTransfer(USART1, 0x0D);		// TOF5[7:6] – Reserved
											// TOF5[5:0] – HIT6 Wave Select
	DEASSERT_MAX_TDC_CE;

	ASSERT_MAX_TDC_CE;
	// TOF6 Register - comparator upstream
	WRITE_TOF6;								// Write to TOF6
	USART_SpiTransfer(USART1, 0x06);		// TOF6[15:8] – Comparator Return Offset Upstream
	USART_SpiTransfer(USART1, 0x03);		// TOF6[7:0] – Comparator Offset Upstream
	DEASSERT_MAX_TDC_CE;

	ASSERT_MAX_TDC_CE;
	// TOF7 Register - comparator downstream
	WRITE_TOF7;								// Write to TOF7
	USART_SpiTransfer(USART1, 0x06);		// TOF7[15:8] – Comparator Return Offset Downstream
	USART_SpiTransfer(USART1, 0x03);		// TOF7[7:0] – Comparator Offset Downstream
	DEASSERT_MAX_TDC_CE;

	/* ----- End configuration register setup ----- */

	ASSERT_MAX_TDC_CE;
	WRITE_TOF_MEAS_DELAY;			// Set TOF Measurement Delay
	USART_SpiTransfer(USART1, 0x00);
	USART_SpiTransfer(USART1, 0xE0);
	DEASSERT_MAX_TDC_CE;

	ASSERT_MAX_TDC_CE;
	WRITE_CLBRT_CTRL;				// Set Calibration and Control Register
	USART_SpiTransfer(USART1, 0x02);
	USART_SpiTransfer(USART1, 0x7F);
	DEASSERT_MAX_TDC_CE;

	ASSERT_MAX_TDC_CE;
	TX_CONFIG_FLASH;				// Transfer Configuration to Flash Command
	DEASSERT_MAX_TDC_CE;

	ASSERT_MAX_TDC_CE;
	INITIALIZE;						// Initialize
	DEASSERT_MAX_TDC_CE;

}


/******************************************************************************
 * @brief Initialize WonderGecko SPI
 * @detail: Enable high frequency RC oscillator, set baud rate/endian/clock mode
 *
 * @return void
 * Note:
*******************************************************************************/

void SPI_init()
{
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

	/* Enabling clock to USART1 */
	CMU_ClockEnable(cmuClock_USART1, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

	/* Initialize USART in SPI master mode. */
	init.baudrate = 14000000;			// baudRate defined in common.h
	init.msbf     = true; 			// Analog devices is big enDian
	init.clockMode = usartClockMode1;
	USART_InitSync(USART1, &init);

	/* Enabling pins and setting location, SPI /CS bus controlled independently */
	USART1->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN |
			USART_ROUTE_CLKPEN | USART_ROUTE_LOCATION_LOC1;

	/* IO configuration */
	GPIO_PinModeSet(gpioPortD, 0,	gpioModePushPull, 0);	/* MOSI */
	GPIO_PinModeSet(gpioPortD, 1,	gpioModeInput, 0);		/* MISO */
	GPIO_PinModeSet(gpioPortD, 2,	gpioModePushPull, 1);	/* CLK */
	GPIO_PinModeSet(gpioPortD, 3,	gpioModePushPull, 0);	/* CE */

}


/*******************************************************************************
 * @brief :
 * @detail:
 *
 * @return void
 * Note:
*******************************************************************************/
/*
void setupGPIOInt() {

	GPIO_PinModeSet(gpioPortD, 4, gpioModeInput, 1);	// MAX Interrupt
	GPIO_IntConfig(gpioPortD, 4, true, true, true);

	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);

}
*/

/******************************************************************************
 * @brief
 * @detail:
 *
 * @return void
 * Note:
*******************************************************************************/
/*
void GPIO_EVEN_IRQHandler(void) {

	GPIO_IntDisable(0x0010);

	uint8_t readbufInt[6];

	ASSERT_MAX_TDC_CE;
	READ_INT_STAT_REG;			 		// Read status register
	readbufInt[0] = NULL_CMD;
	readbufInt[1] = NULL_CMD;
	DEASSERT_MAX_TDC_CE;

	// TOF Interrupt (bit 12)
	if((0x10 & readbufInt[0]) == 0x10)
	{
		ASSERT_MAX_TDC_CE;
		TOF_DIFF_INT; 					// Read TOF_DIFFInt
		readbufInt[2] = NULL_CMD;
		readbufInt[3] = NULL_CMD;
		DEASSERT_MAX_TDC_CE;

		ASSERT_MAX_TDC_CE;
		TOF_DIFF_FRAC; 					// Read TOF_DIFFFrac
		readbufInt[4] = NULL_CMD;
		readbufInt[5] = NULL_CMD;
		DEASSERT_MAX_TDC_CE;
	}

	GPIO_IntEnable(0x0010);
}
*/


int main(void)
{
	/* Chip errata */
	CHIP_Init();

	SPI_init();
	//setupGPIOInt();
	GPIO_IntEnable(0x0010);

	uint8_t readbuf[8];
	// float mydata;
	int i;

	MAX_init();

	while (1) {

		ASSERT_MAX_TDC_CE;
		READ_INT_STAT_REG;			 	// Read status register
		readbuf[0] = NULL_CMD;
		readbuf[1] = NULL_CMD;
		DEASSERT_MAX_TDC_CE;

		ASSERT_MAX_TDC_CE;
		TOF_DIFF;					 	// TOF Diff
		DEASSERT_MAX_TDC_CE;

		ASSERT_MAX_TDC_CE;
		READ_RTC_SECS; 					// Read RTC Seconds
		readbuf[2] = NULL_CMD;
		readbuf[3] = NULL_CMD;
		DEASSERT_MAX_TDC_CE;

		ASSERT_MAX_TDC_CE;
		TOF_DIFF_INT; 					// Read TOF_DIFFInt
		readbuf[4] = NULL_CMD;
		readbuf[5] = NULL_CMD;
		DEASSERT_MAX_TDC_CE;

		ASSERT_MAX_TDC_CE;
		TOF_DIFF_FRAC; 					// Read TOF_DIFFFrac
		readbuf[6] = NULL_CMD;
		readbuf[7] = NULL_CMD;
		DEASSERT_MAX_TDC_CE;

		// mydata = *((float*)(&readbuf[2]));

		for(i = 0; i < 1000; i++){}
		i = 0;

	}
}
