#include <unistd.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "max_macros.h"

// SPI Transfer
#define MAX_SPI_CMD(x)          USART_SpiTransfer(USART1, x)

// MAX Clock Enable (CE_)
#define ASSERT_MAX_TDC_CE       GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0)
#define DEASSERT_MAX_TDC_CE     GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 1)

uint8_t spi_return[20];  // spi_return[0] reserved for meaningless values (values returned by MAX during opcode commands or writing to registers)
                         // spi_return[1:2] Interrupt Status Register
                         // spi_return[3:4] TOF Int
                         // spi_return[5:6] TOF Frac
                         // spi_return[7:8] RTC Seconds

/******************************************************************************
 * @brief Initialize MAX35103 settings
 * @detail: Set up TOF1/2/3/4/5 registers through SPI
 *
 * @return void
 * Note:
*******************************************************************************/

void MAX_init()
{
    /* ----- Begin configuration register setup ----- */

	ASSERT_MAX_TDC_CE;
	    // TOF1 Register - basic operating parameters for TOF measurements
	    spi_return[0] = MAX_SPI_CMD(WRITE_TOF1);    // Write to TOF1
	    spi_return[0] = MAX_SPI_CMD(0x0C);          // TOF1[15:8]   Pulse Launcher Size
	    spi_return[0] = MAX_SPI_CMD(0x10);          // TOF1[7:4]    Pulse Launch Divider
	                                                // TOF1.3       Stop Polarity
	                                                // TOF1.2       Reserved (No effect)
	                                                // TOF1[1:0]    Bias Charge Time
	    DEASSERT_MAX_TDC_CE;

	    ASSERT_MAX_TDC_CE;
	    // TOF2 Register - details of how TOF will be measured
	    spi_return[0] = MAX_SPI_CMD(WRITE_TOF2);    // Write to TOF2
	    spi_return[0] = MAX_SPI_CMD(0xE1);          // TOF2[15:13]  Stop Hits
	                                                // TOF2[12:7]   T2 Wave Selection
	    spi_return[0] = MAX_SPI_CMD(0x00);          // TOF2[12:7]   T2 Wave Selection
	                                                // TOF2[6:4]    TOF Duty Cycle
	                                                // TOF2.3       Reserved (No effect)
	                                                // TOF2[2:0]    Timeout
	    DEASSERT_MAX_TDC_CE;

	    ASSERT_MAX_TDC_CE;
	    // TOF3 Register - select which waves will be used in time measurements
	    spi_return[0] = MAX_SPI_CMD(WRITE_TOF3);    // Write to TOF3
	    spi_return[0] = MAX_SPI_CMD(0x05);          // TOF3[15:14]  Reserved
	                                                // TOF3[13:8]   HIT1 Wave Select
	    spi_return[0] = MAX_SPI_CMD(0x06);          // TOF3[7:6]    Reserved
	                                                // TOF3[5:0]    HIT2 Wave Select
	    DEASSERT_MAX_TDC_CE;

	    ASSERT_MAX_TDC_CE;
	    // TOF4 Register - select which waves will be used in time measurements
	    spi_return[0] = MAX_SPI_CMD(WRITE_TOF4);    // Write to TOF4
	    spi_return[0] = MAX_SPI_CMD(0x07);          // TOF4[15:14]  Reserved
	                                                // TOF4[13:8]   HIT3 Wave Select
	    spi_return[0] = MAX_SPI_CMD(0x08);          // TOF4[7:6]    Reserved
	                                                // TOF4[5:0]    HIT4 Wave Select
	    DEASSERT_MAX_TDC_CE;

	    ASSERT_MAX_TDC_CE;
	    // TOF5 Register - select which waves will be used in time measurements
	    spi_return[0] = MAX_SPI_CMD(WRITE_TOF5);    // Write to TOF5
	    spi_return[0] = MAX_SPI_CMD(0x09);          // TOF5[15:14]  Reserved
	                                                // TOF5[13:8]   HIT5 Wave Select
	    spi_return[0] = MAX_SPI_CMD(0x0A);          // TOF5[7:6]    Reserved
	                                                // TOF5[5:0]    HIT6 Wave Select
	    DEASSERT_MAX_TDC_CE;

	    ASSERT_MAX_TDC_CE;
	    // TOF6 Register - comparator upstream
	    spi_return[0] = MAX_SPI_CMD(WRITE_TOF6);    // Write to TOF6
	    spi_return[0] = MAX_SPI_CMD(0x23);          // TOF6[15:8]   Comparator Return Offset Upstream
	    spi_return[0] = MAX_SPI_CMD(0x0A);          // TOF6[7:0]    Comparator Offset Upstream
	    DEASSERT_MAX_TDC_CE;

	    ASSERT_MAX_TDC_CE;
	    // TOF7 Register - comparator downstream
	    spi_return[0] = MAX_SPI_CMD(WRITE_TOF7);    // Write to TOF7
	    spi_return[0] = MAX_SPI_CMD(0x23);          // TOF7[15:8]   Comparator Return Offset Downstream
	    spi_return[0] = MAX_SPI_CMD(0x0A);          // TOF7[7:0]    Comparator Offset Downstream
	    DEASSERT_MAX_TDC_CE;

	    ASSERT_MAX_TDC_CE;
	    spi_return[0] = MAX_SPI_CMD(WRITE_TOF_MEAS_DELAY);  // Set TOF Measurement Delay
	    spi_return[0] = MAX_SPI_CMD(0x00);
	    spi_return[0] = MAX_SPI_CMD(0xC8);
	    DEASSERT_MAX_TDC_CE;

	    ASSERT_MAX_TDC_CE;
	    spi_return[0] = MAX_SPI_CMD(WRITE_CLBRT_CTRL);      // Set Calibration and Control Register
	    spi_return[0] = MAX_SPI_CMD(0x0E);
	    spi_return[0] = MAX_SPI_CMD(0xFF);
	    DEASSERT_MAX_TDC_CE;

	    /* ----- End configuration register setup ----- */

	    ASSERT_MAX_TDC_CE;
	    spi_return[0] = MAX_SPI_CMD(TX_CONFIG_FLASH);       // Transfer Configuration to Flash Command
	    DEASSERT_MAX_TDC_CE;

	    ASSERT_MAX_TDC_CE;
	    spi_return[0] = MAX_SPI_CMD(INITIALIZE);            // Initialize
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
    init.baudrate  = 14000000;      // baudRate defined in common.h
    init.msbf      = true;          // Analog devices is big endian
    init.clockMode = usartClockMode1;
    USART_InitSync(USART1, &init);

    /* Enabling pins and setting location, SPI /CS bus controlled independently */
    USART1->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN |
            USART_ROUTE_CLKPEN | USART_ROUTE_LOCATION_LOC1;

    /* IO configuration */
    GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 0);	 /* MOSI */
    GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 0);     /* MISO */
    GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 1);  /* CLK */
    GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);  /* CE */

}

/******************************************************************************
 * @brief Set up UART
 * @detail:
 *
 * @return void
 * Note:
*******************************************************************************/
void UART_init() {

  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_USART0, true); // Enable clock for USART0 module
  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 1); // TX
  GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 0); // RX

  USART_InitAsync_TypeDef usartInitUSART0 = {
    .enable = usartDisable, 					// Initially disabled
    .refFreq = 0,								// configured reference frequency
    .baudrate = 115200, 				        // Baud rate defined in common.h
    .oversampling = usartOVS16, 				// overSampling rate x16
    .databits = USART_FRAME_DATABITS_EIGHT, 	// 8-bit frames
    .parity = USART_FRAME_PARITY_NONE,			// parity - none
    .stopbits = USART_FRAME_STOPBITS_ONE,		// 1 stop bit
  };

  /*Initialize UART registers*/
  USART_InitAsync(USART0, &usartInitUSART0);

  USART0 -> ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC5;

  /* Inform NVIC of IRQ changes*/
  NVIC_ClearPendingIRQ(USART0_TX_IRQn);
  NVIC_EnableIRQ(USART0_TX_IRQn);

  USART_Enable(USART0, usartEnable);
  NVIC_SetPriority(USART0_TX_IRQn, 1);
}

/*******************************************************************************
 * @brief :
 * @detail:
 *
 * @return void
 * Note:
*******************************************************************************/

void setupGPIOInt() {

    GPIO_PinModeSet(gpioPortD, 4, gpioModeInput, 1);  // MAX Interrupt
    GPIO_IntConfig(gpioPortD, 4, true, true, true);

    NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);

}


/******************************************************************************
 * @brief
 * @detail:
 *
 * @return void
 * Note:
*******************************************************************************/

void GPIO_EVEN_IRQHandler(void) {
    // TODO Multiple interrupts on EVEN_IRQHandler
    // http://community.silabs.com/t5/32-bit-MCU/GPIO-and-USART-interrupt/td-p/105651
    // http://community.silabs.com/t5/Bluetooth-Wi-Fi-Knowledge-Base/BGM111-button-press-detection-with-interrupts-C-project/ta-p/184967
    // http://community.silabs.com/t5/32-bit-MCU/EFM32WG-multiple-external-interrupts/td-p/145825
    // https://github.com/hrshygoodness/EFM32-Library/blob/master/v2/kits/EFM32TG_STK3300/examples/Modified/gpiointerrupt/main.c

    GPIO_IntDisable(0x0010);

    ASSERT_MAX_TDC_CE;
    spi_return[0] = MAX_SPI_CMD(READ_INT_STAT_REG);     // Read status register
    spi_return[1] = MAX_SPI_CMD(NULL_CMD);
    spi_return[2] = MAX_SPI_CMD(NULL_CMD);
    DEASSERT_MAX_TDC_CE;

    GPIO_IntClear(0x0010);
}


int main(void)
{
    /* Chip errata */
    CHIP_Init();

    SPI_init();
    UART_init();
    MAX_init();  // Initialize MAX settings through SPI

    setupGPIOInt();
    GPIO_IntEnable(0x0010);

    // float mydata;

    int i;

    ASSERT_MAX_TDC_CE;
    spi_return[0] = MAX_SPI_CMD(TOF_DIFF);          // TOF Diff
    DEASSERT_MAX_TDC_CE;

    ASSERT_MAX_TDC_CE;
    spi_return[0] = MAX_SPI_CMD(READ_INT_STAT_REG);     // Read status register
    spi_return[1] = MAX_SPI_CMD(NULL_CMD);
    spi_return[2] = MAX_SPI_CMD(NULL_CMD);
    DEASSERT_MAX_TDC_CE;

    while (1) {

        ASSERT_MAX_TDC_CE;
        spi_return[0] = MAX_SPI_CMD(READ_RTC_SECS);     // Read RTC Seconds
        spi_return[7] = MAX_SPI_CMD(NULL_CMD);
        spi_return[8] = MAX_SPI_CMD(NULL_CMD);
        DEASSERT_MAX_TDC_CE;

        // TOF Interrupt (bit 12)
        if((0x10 & spi_return[1]) == 0x10)
        {
        	ASSERT_MAX_TDC_CE;
        	spi_return[0] = MAX_SPI_CMD(TOF_DIFF_INT); 	    // Read TOF_DIFFInt
        	spi_return[3] = MAX_SPI_CMD(NULL_CMD);
        	spi_return[4] = MAX_SPI_CMD(NULL_CMD);
        	DEASSERT_MAX_TDC_CE;

        	ASSERT_MAX_TDC_CE;
        	spi_return[0] = MAX_SPI_CMD(TOF_DIFF_FRAC);     // Read TOF_DIFFFrac
        	spi_return[5] = MAX_SPI_CMD(NULL_CMD);
        	spi_return[6] = MAX_SPI_CMD(NULL_CMD);
        	DEASSERT_MAX_TDC_CE;

        	USART_Tx(USART0, 0x46);

        	spi_return[1] = 0x00;
            GPIO_IntEnable(0x0010);

            ASSERT_MAX_TDC_CE;
            spi_return[0] = MAX_SPI_CMD(TOF_DIFF);          // TOF Diff
            DEASSERT_MAX_TDC_CE;
        }

        // mydata = *((float*)(&readbuf[2]));

        for(i = 0; i < 1000; i++){}
        i = 0;

    }
}
