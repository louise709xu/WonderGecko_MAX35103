#include <stdio.h>
#include <unistd.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "spidrv.h"
#include "uartdrv.h"
#include "gpiointerrupt.h"

#include "max_macros.h"
#include "int_2hex.h"
#include <time.h>

/* ----- SPI Declarations ----- */

/* @var SPI_TX_CONFIG_BUF_LENGTH  Configuration requires 3 bytes transferred */
#define SPI_TX_CONFIG_BUF_LENGTH 3
/* @var SPI_TX_BUF_LENGTH  OP code commands only transfer 1 byte and receive 2 bytes */
#define SPI_TX_BUF_LENGTH 1
/* @var SPI_RX_BUF_LENGTH  Arbitrary, 3 bytes per data register read */
// Needs 3 bytes per register despite 2 bytes of actual data because otherwise data gets overwritten
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
#define UART_TX_BUF_LENGTH 6

DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_RX_BUFS, rxBufferQueue);
DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_TX_BUFS, txBufferQueue);

UARTDRV_HandleData_t uart_handleData;
UARTDRV_Handle_t uart_handle = &uart_handleData;

/*******************************************************************************
 * @var max_reg_buffer
 * @abstract Stores information received from the MAX board
 * @discussion The measurements and values from the MAX board are stored in the
 *             address locations:
 *             uart_rx_buffer[0] TOF Int
 *             uart_rx_buffer[1] TOF Frac
 *             uart_rx_buffer[2] RTC Month_Year
 *             uart_rx_buffer[3] RTC Day_Date
 *             uart_rx_buffer[4] RTC Min_Hours
 *             uart_rx_buffer[5] RTC Seconds
 ******************************************************************************/
uint32_t max_reg_buffer[UART_TX_BUF_LENGTH];


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
 * @discussion  SPI transfer between Wonder Gecko and MAX35103
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
 * @abstract    Set up USART
 * @discussion  USART transfer between Wonder Gecko and computer/other devices
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



void pollRTC() {
	spi_tx_buffer[0] = READ_RTC_M_Y;
    MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[9]);

	spi_tx_buffer[0] = READ_RTC_DAY_DATE;
    MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[12]);

	spi_tx_buffer[0] = READ_RTC_MIN_HRS;
    MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[15]);

	spi_tx_buffer[0] = READ_RTC_SECS;
    MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[18]);
}

void pollTOF() {
	spi_tx_buffer[0] = TOF_DIFF_INT;
    MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[3]);

    spi_tx_buffer[0] = TOF_DIFF_FRAC;
    MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[6]);
}

void processRTC(){
	max_reg_buffer[2] = int16_2hex(*((uint16_t*)&spi_rx_buffer[10]));
	max_reg_buffer[3] = int16_2hex(*((uint16_t*)&spi_rx_buffer[13]));
	max_reg_buffer[4] = int16_2hex(*((uint16_t*)&spi_rx_buffer[16]));
	max_reg_buffer[5] = int16_2hex(*((uint16_t*)&spi_rx_buffer[19]));
}

void processTOF(){
	max_reg_buffer[0] = int16_2hex(*((uint16_t*)&spi_rx_buffer[4]));
	max_reg_buffer[1] = int16_2hex(*((uint16_t*)&spi_rx_buffer[7]));
}



// For debugging purposes
void delay(int number_of_seconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * number_of_seconds;

    // Storing start time
    clock_t start_time = clock();

    // looping till required time is not achieved
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

    spi_tx_buffer[0] = TOF_DIFF;
    MAX_SPI_TX(&spi_tx_buffer[0]);

    spi_tx_buffer[0] = READ_INT_STAT_REG;
    MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[0]);

    uint8_t delimiter = '\t';

    uint8_t test_buffer[10];
    test_buffer[0] = 'a';
    UARTDRV_Transmit(uart_handle, test_buffer, 10, callback_UARTTX);

    int i;

    while (1) {

    	// For debugging purposes
    	// pollRTC();
    	// processRTC();
        // UARTDRV_Transmit(uart_handle, (uint8_t*)&max_reg_buffer, UART_TX_BUF_LENGTH * 4, callback_UARTTX);

        delay(100);

        // TOF Interrupt (bit 12)
        if((0x10 & spi_rx_buffer[1]) == 0x10) {

        	// Read data from MAX board registers
        	pollRTC();
        	pollTOF();

            processRTC();
            processTOF();

            for(i = 0; i < UART_TX_BUF_LENGTH; i++) {
            	UARTDRV_Transmit(uart_handle, (uint8_t*)(&max_reg_buffer[i]), sizeof(uint32_t), callback_UARTTX);
            	UARTDRV_Transmit(uart_handle, &delimiter, sizeof(uint8_t), callback_UARTTX);
            }

        	spi_rx_buffer[1] = 0x00;
            GPIO_IntEnable(0x0010);

            spi_tx_buffer[0] = TOF_DIFF;
            MAX_SPI_TX(&spi_tx_buffer[0]);

        }

    }

}



