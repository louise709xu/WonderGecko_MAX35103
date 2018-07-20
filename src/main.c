#include <stdio.h>
#include <unistd.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "spidrv.h"
#include "uartdrv.h"
#include "string.h"
#include "gpiointerrupt.h"

#include "max_macros.h"

#include <time.h>

/* ----- SPI Declarations ----- */

/* @var SPI_TX_CONFIG_GUF_LENGTH Configuration requires 3 bytes transferred */
#define SPI_TX_CONFIG_BUF_LENGTH 3
/* @var SPI_TX_BUF_LENGTH OP code commands only transfer 1 byte and receive 2 bytes */
#define SPI_TX_BUF_LENGTH 1
#define SPI_RX_BUF_LENGTH 21

SPIDRV_HandleData_t spi_handleData;
SPIDRV_Handle_t spi_handle = &spi_handleData;

uint8_t spi_tx_config_buffer[SPI_TX_CONFIG_BUF_LENGTH];
uint8_t spi_tx_buffer[SPI_TX_BUF_LENGTH];

/*******************************************************************************
 * @var spi_rx_buffer
 * @abstract Stores information received from the MAX board
 * @discussion The measurements and values from the MAX board are stored in the
 *             address locations:
 *             spi_rx_buffer[0:2]   Interrupt Status Register
 *             spi_rx_buffer[3:5]   TOF Int
 *             spi_rx_buffer[6:8]   TOF Frac
 *             spi_rx_buffer[9:11]  RTC Month_Year
 *             spi_rx_buffer[12:14] RTC Day_Date
 *             spi_rx_buffer[15:17] RTC Min_Hours
 *             spi_rx_buffer[18:20] RTC Seconds
 ******************************************************************************/
uint8_t spi_rx_buffer[SPI_RX_BUF_LENGTH];


// MAX SPI Transfer
#define MAX_SPI_TX_Config(x)    SPIDRV_MTransmitB(spi_handle, x, 3);
#define MAX_SPI_TX(x)           SPIDRV_MTransmitB(spi_handle, x, 1);
#define MAX_SPI_RX(x)           SPIDRV_MReceiveB(spi_handle, x, 1);
#define MAX_SPI_TXRX(x,y)       SPIDRV_MTransferB(spi_handle, x, y, 3);


/* ----- UART Declarations ----- */
#define UART_TX_BUF_LENGTH 21

DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_RX_BUFS, rxBufferQueue);
DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_TX_BUFS, txBufferQueue);

UARTDRV_HandleData_t uart_handleData;
UARTDRV_Handle_t uart_handle = &uart_handleData;

/*******************************************************************************
 * @var uart_tx_buffer
 * @abstract Stores information received from the MAX board
 * @discussion The measurements and values from the MAX board are stored in the
 *             address locations:
 *             uart_tx_buffer[0]    10 Month
 *             uart_tx_buffer[1]    Month
 *             uart_tx_buffer[2]    '/'
 *             uart_tx_buffer[3]    10 Date
 *             uart_tx_buffer[4]    Date
 *             uart_tx_buffer[5]    '/'
 *             uart_tx_buffer[6]    10 Year
 *             uart_tx_buffer[7]    Year
 *             uart_tx_buffer[8]    ' '
 *             uart_tx_buffer[9]    10 Hour
 *             uart_tx_buffer[10]   Hour
 *             uart_tx_buffer[11]   ':'
 *             uart_tx_buffer[12]   10 Minute
 *             uart_tx_buffer[13]   Minute
 *             uart_tx_buffer[14]   ':'
 *             uart_tx_buffer[15]   10 Seconds
 *             uart_tx_buffer[16]   Seconds
 *             uart_tx_buffer[17]   ':'
 *             uart_tx_buffer[18]   Tenths of Seconds
 *             uart_tx_buffer[19]   Hundredths of Seconds
 *             uart_tx_buffer[20]   '\n'
 ******************************************************************************/
uint8_t uart_tx_buffer[UART_TX_BUF_LENGTH];

RTCDRV_TimerID_t rtc_id;


/*******************************************************************************
 * @function    MAX_Init()
 * @abstract    Initialize MAX35103 settings
 * @discussion  Set up TOF1/2/3/4/5/6/7 registers and initialize MAX board
 *
 * @return      void
 ******************************************************************************/
void MAX_Init()
{
    /* ----- Begin configuration register setup ----- */

    /***************************************************************************
     * TOF1 Register - basic operating parameters for TOF measurements
     * TOF1[15:8]   Pulse Launcher Size
     * TOF1[7:4]    Pulse Launch Divider
     * TOF1[3]      Stop Polarity
     * TOF1[2]      Reserved (No effect)
     * TOF1[1:0]    Bias Charge Time
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF1;
    spi_tx_config_buffer[1] = 0x0C;
    spi_tx_config_buffer[2] = 0x10;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF2 Register - details of how TOF will be measured
     * TOF2[15:13]  Stop Hits
     * TOF2[12:7]   T2 Wave Selection
     * TOF2[6:4]    TOF Duty Cycle
     * TOF2[3]      Reserved (No effect)
     * TOF2[2:0]    Timeout
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF2;
    spi_tx_config_buffer[1] = 0xE1;
    spi_tx_config_buffer[2] = 0x00;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF3 Register - select which waves will be used in time measurements
     * TOF3[15:14]  Reserved
     * TOF3[13:8]   HIT1 Wave Select
     * TOF3[7:6]    Reserved
     * TOF3[5:0]    HIT2 Wave Select
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF3;
    spi_tx_config_buffer[1] = 0x05;
    spi_tx_config_buffer[2] = 0x06;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF4 Register - select which waves will be used in time measurements
     * TOF4[15:14]  Reserved
     * TOF4[13:8]   HIT3 Wave Select
     * TOF4[7:6]    Reserved
     * TOF4[5:0]    HIT4 Wave Select
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF4;
    spi_tx_config_buffer[1] = 0x07;
    spi_tx_config_buffer[2] = 0x08;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF5 Register - select which waves will be used in time measurements
     * TOF5[15:14]  Reserved
     * TOF5[13:8]   HIT5 Wave Select
     * TOF5[7:6]    Reserved
     * TOF5[5:0]    HIT6 Wave Select
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF5;
    spi_tx_config_buffer[1] = 0x09;
    spi_tx_config_buffer[2] = 0x0A;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF6 Register - comparator upstream
     * TOF6[15:8]   Comparator Return Offset Upstream
     * TOF6[7:0]    Comparator Offset Upstream
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF6;
    spi_tx_config_buffer[1] = 0x23;
    spi_tx_config_buffer[2] = 0x0A;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF7 Register - comparator downstream
     * TOF7[15:8]   Comparator Return Offset Downstream
     * TOF7[7:0]    Comparator Offset Downstream
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF7;
    spi_tx_config_buffer[1] = 0x23;
    spi_tx_config_buffer[2] = 0x0A;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF Measurement Delay - delay between start of pulse launch and receiver
     *                         enable
     * DLY[15:8]    Delay
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF_MEAS_DELAY;
    spi_tx_config_buffer[1] = 0x00;
    spi_tx_config_buffer[2] = 0xC8;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * Calibration and Control Register - calibration settings
     * CLBRT[15:11] Temperature Measurement Cycles
     * CLBRT[10]    Calibration Usage
     * CLBRT[9:7]   Calibration Configuration
     * CLBRT[6:5]   Temperature Port
     * CLBRT[4:2]   Preamble Temperature Cycle
     * CLBRT[1:0]   Port Cycle Time
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_CLBRT_CTRL;
    spi_tx_config_buffer[1] = 0x0E;
    spi_tx_config_buffer[2] = 0xFF;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);


    /* ----- End configuration register setup ----- */

    spi_tx_buffer[0] = TX_CONFIG_FLASH;       // Transfer Configuration to Flash Command
    MAX_SPI_TX(&spi_tx_buffer[0]);

    spi_tx_buffer[0] = INITIALIZE;            // Initialize
    MAX_SPI_TX(&spi_tx_buffer[0]);
}

/*******************************************************************************
 * @function    SPI_Init()
 * @abstract    Set up SPI
 * @discussion
 *
 * @return      void
 ******************************************************************************/
void SPI_Init() {

    SPIDRV_Init_t initData = {                                                        \
              USART1,                       /* USART port                       */    \
              _USART_ROUTE_LOCATION_LOC1,   /* USART pins location number       */    \
              1000000,                      /* Bitrate                          */    \
              8,                            /* Frame length                     */    \
              0,                            /* Dummy tx value for rx only funcs */    \
              spidrvMaster,                 /* SPI mode                         */    \
              spidrvBitOrderMsbFirst,       /* Bit order on bus                 */    \
              spidrvClockMode1,             /* SPI clock/phase mode             */    \
              spidrvCsControlAuto,          /* CS controlled by the driver      */    \
              spidrvSlaveStartImmediate     /* Slave start transfers immediately*/    \
    };

    // Initialize a SPI driver instance
    SPIDRV_Init( spi_handle, &initData );

}


/*******************************************************************************
 * @function    UART_Init()
 * @abstract    Set up UART
 * @discussion  Enable clocks, set GPIO pinmode, clear interrupts, enable USART
 *
 * @return      void
 ******************************************************************************/
void UART_Init() {
	UARTDRV_InitUart_t uartInitData = {
		USART0,                                             \
		115200,                                             \
		_USART_ROUTE_LOCATION_LOC5,                         \
		usartStopbits1,                                     \
		usartNoParity,                                      \
		usartOVS16,                                         \
		false,                                              \
		uartdrvFlowControlNone,                             \
		gpioPortE,                                          \
		12,                                                 \
		gpioPortE,                                          \
		13,                                                 \
		(UARTDRV_Buffer_FifoQueue_t *)&rxBufferQueue,       \
		(UARTDRV_Buffer_FifoQueue_t *)&txBufferQueue,       \
	};

	UARTDRV_InitUart(uart_handle, &uartInitData);
}

void callback_UARTTX(UARTDRV_Handle_t handle,
                           Ecode_t transferStatus,
                           uint8_t *data,
                           UARTDRV_Count_t transferCount)
{
  (void)handle;
  (void)transferStatus;
  (void)data;
  (void)transferCount;
}

void callback_UARTRX(UARTDRV_Handle_t handle,
                           Ecode_t transferStatus,
                           uint8_t *data,
                           UARTDRV_Count_t transferCount)
{
  (void)handle;
  (void)transferStatus;
  (void)data;
  (void)transferCount;
}

/*
void callback_RTC( RTCDRV_TimerID_t id, void * user )
{
  (void) user; // unused argument in this example


}
*/

void GPIOINT_callback(void) {
    // TODO Multiple interrupts on EVEN_IRQHandler

    GPIO_IntDisable(0x0010);

    spi_tx_buffer[0] = READ_INT_STAT_REG;
    MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[0]);      // Read status register

    GPIO_IntClear(0x0010);
}


/*******************************************************************************
 * @function    setupGPIOInt()
 * @abstract    Enable GPIO Interrupts
 * @discussion
 *
 * @return      void
 ******************************************************************************/
void setupGPIOInt() {

    GPIO_PinModeSet(gpioPortD, 4, gpioModeInput, 1);  // MAX Interrupt
    GPIO_ExtIntConfig(gpioPortD, 4, 4, true, true, true);

    GPIOINT_Init();
    GPIOINT_CallbackRegister(4, (GPIOINT_IrqCallbackPtr_t) GPIOINT_callback);

    GPIO_IntEnable(0x0010);
}

void delay(int number_of_seconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * number_of_seconds;

    // Stroing start time
    clock_t start_time = clock();

    // looping till required time is not acheived
    while (clock() < start_time + milli_seconds);
}

/*******************************************************************************
 * @function    main()
 * @abstract    Set up communication with MAX board, poll for measurements
 * @discussion  Initialize WonderGecko, SPIDRV, UART, MAX board, check interrupt
 *              status, record TOF measurements when available
 *
 * @return      void
 ******************************************************************************/
int main(void) {

    /* Chip errata */
    CHIP_Init();

    SPI_Init();
    UART_Init();
    MAX_Init();
    setupGPIOInt();

    /*
    RTCDRV_Init();

    // Reserve a timer
    Ecode_t max_rtc = RTCDRV_AllocateTimer( &rtc_id );
    // Start a periodic timer with 1000 millisecond timeout
    RTCDRV_StartTimer( rtc_id, rtcdrvTimerTypePeriodic, 1000, callback_RTC, NULL );
    */

    uart_tx_buffer[2] = uart_tx_buffer[5] = '/';
    uart_tx_buffer[8] = ' ';
    uart_tx_buffer[11] = uart_tx_buffer[14] = uart_tx_buffer[17] = ':';
    uart_tx_buffer[20] = 0x09;

    spi_tx_buffer[0] = TOF_DIFF;
    MAX_SPI_TX(&spi_tx_buffer[0]);

    spi_tx_buffer[0] = READ_INT_STAT_REG;
    MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[0]);

    while (1) {

    	// For debugging purposes
    	spi_tx_buffer[0] = READ_RTC_M_Y;
        MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[9]);

    	spi_tx_buffer[0] = READ_RTC_DAY_DATE;
        MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[12]);

    	spi_tx_buffer[0] = READ_RTC_MIN_HRS;
        MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[15]);

    	spi_tx_buffer[0] = READ_RTC_SECS;
        MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[18]);

        // Bitwise operations to separate data in each register
        uart_tx_buffer[0] = ((spi_rx_buffer[10] & 0x10) >> 4) + 0x30;   // 10 Month
        uart_tx_buffer[1] = (spi_rx_buffer[10] & 0x0F) + 0x30;          // Month
        uart_tx_buffer[3] = ((spi_rx_buffer[14] & 0x30) >> 4) + 0x30;   // 10 Date
        uart_tx_buffer[4] = (spi_rx_buffer[14] & 0x0F) + 0x30;          // Date
        uart_tx_buffer[6] = ((spi_rx_buffer[11] & 0xF0) >> 4) + 0x30;   // 10 Year
        uart_tx_buffer[7] = (spi_rx_buffer[11] & 0x0F) + 0x30;          // Year
    	uart_tx_buffer[9] = ((spi_rx_buffer[17] & 0x30) >> 4) + 0x30;   // 10 Hour (tens digit stays the same regardless 12/24 hr)
        if((spi_rx_buffer[17] & 0x40) == 0x40){                         // if 12 hour mode
        	uart_tx_buffer[10] = (spi_rx_buffer[17] & 0x0F) + 0x32;      // Hour (add 2)
        }
        else {                                                          // if 24 hour mode
        	uart_tx_buffer[10] = (spi_rx_buffer[17] & 0x0F) + 0x30;      // Hour
        }
        uart_tx_buffer[12] = ((spi_rx_buffer[16] & 0x70) >> 4) + 0x30;   // 10 Minute
        uart_tx_buffer[13] = (spi_rx_buffer[16] & 0x0F) + 0x30;          // Minute
        uart_tx_buffer[15] = ((spi_rx_buffer[20] & 0x70) >> 4) + 0x30;  // 10 Second
        uart_tx_buffer[16] = (spi_rx_buffer[20] & 0x0F) + 0x30;         // Second
        uart_tx_buffer[18] = ((spi_rx_buffer[19] & 0xF0) >> 4) + 0x30;  // Tenth of Second
        uart_tx_buffer[19] = (spi_rx_buffer[19] & 0x0F) + 0x30;         // Hundredth of Second

        UARTDRV_Transmit(uart_handle, uart_tx_buffer, UART_TX_BUF_LENGTH, callback_UARTTX);

        delay(10);

        // TOF Interrupt (bit 12)
        if((0x10 & spi_rx_buffer[1]) == 0x10) {

        	// Read data from MAX board registers
        	spi_tx_buffer[0] = READ_RTC_SECS;
            MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[18]);

        	spi_tx_buffer[0] = READ_RTC_MIN_HRS;
            MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[15]);

        	spi_tx_buffer[0] = READ_RTC_DAY_DATE;
            MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[12]);

        	spi_tx_buffer[0] = READ_RTC_M_Y;
            MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[9]);

        	spi_tx_buffer[0] = TOF_DIFF_INT;
            MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[3]);

            spi_tx_buffer[0] = TOF_DIFF_FRAC;
            MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[6]);

            // Bitwise operations to separate data in each register
            uart_tx_buffer[0] = ((spi_rx_buffer[10] & 0x10) >> 4) + 0x30;   // 10 Month
            uart_tx_buffer[1] = (spi_rx_buffer[10] & 0x0F) + 0x30;          // Month
            uart_tx_buffer[2] = ((spi_rx_buffer[14] & 0x30) >> 4) + 0x30;   // 10 Date
            uart_tx_buffer[3] = (spi_rx_buffer[14] & 0x0F) + 0x30;          // Date
            uart_tx_buffer[4] = ((spi_rx_buffer[11] & 0xF0) >> 4) + 0x30;   // 10 Year
            uart_tx_buffer[5] = (spi_rx_buffer[11] & 0x0F) + 0x30;          // Year
        	uart_tx_buffer[6] = ((spi_rx_buffer[17] & 0x30) >> 4) + 0x30;   // 10 Hour (tens digit stays the same regardless 12/24 hr)
            if((spi_rx_buffer[17] & 0x40) == 0x40){                         // if 12 hour mode
            	uart_tx_buffer[7] = (spi_rx_buffer[17] & 0x0F) + 0x32;      // Hour (add 2)
            }
            else {                                                          // if 24 hour mode
            	uart_tx_buffer[7] = (spi_rx_buffer[17] & 0x0F) + 0x30;      // Hour
            }
            uart_tx_buffer[8] = ((spi_rx_buffer[16] & 0x70) >> 4) + 0x30;   // 10 Minute
            uart_tx_buffer[9] = (spi_rx_buffer[16] & 0x0F) + 0x30;          // Minute
            uart_tx_buffer[10] = ((spi_rx_buffer[20] & 0x70) >> 4) + 0x30;  // 10 Second
            uart_tx_buffer[11] = (spi_rx_buffer[20] & 0x0F) + 0x30;         // Second
            uart_tx_buffer[12] = ((spi_rx_buffer[19] & 0xF0) >> 4) + 0x30;  // Tenth of Second
            uart_tx_buffer[12] = (spi_rx_buffer[19] & 0x0F) + 0x30;         // Hundredth of Second

            UARTDRV_Transmit(uart_handle, uart_tx_buffer, UART_TX_BUF_LENGTH, callback_UARTTX);

        	spi_rx_buffer[1] = 0x00;
            GPIO_IntEnable(0x0010);

            spi_tx_buffer[0] = TOF_DIFF;
            MAX_SPI_TX(&spi_tx_buffer[0]);

        }

    }

}
