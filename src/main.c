#include <unistd.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "spidrv.h"
#include "uartdrv.h"
#include "string.h"

#include "max_macros.h"

/* @var SPI_TX_CONFIG_GUF_LENGTH Configuration requires 3 bytes transferred */
#define SPI_TX_CONFIG_BUF_LENGTH 3
/* @var SPI_TX_BUF_LENGTH OP code commands only transfer 1 byte and receive 2 bytes */
#define SPI_TX_BUF_LENGTH 1
#define SPI_RX_BUF_LENGTH 20

SPIDRV_HandleData_t spi_handleData;
SPIDRV_Handle_t spi_handle = &spi_handleData;

uint8_t spi_tx_config_buffer[SPI_TX_CONFIG_BUF_LENGTH];
uint8_t spi_tx_buffer[SPI_TX_BUF_LENGTH];

/*******************************************************************************
 * @var spi_rx_buffer
 * @abstract Stores information received from the MAX board
 * @discussion The measurements and values from the MAX board are stored in the
 *             address locations:
 *             spi_rx_buffer[0:1]   Reserved
 *             spi_rx_buffer[2:5] Interrupt Status Register
 *             spi_rx_buffer[6:9] TOF Int
 *             spi_rx_buffer[10:13] TOF Frac
 *             spi_rx_buffer[14:17] RTC Seconds
 ******************************************************************************/
uint8_t spi_rx_buffer[SPI_RX_BUF_LENGTH];


// MAX SPI Transfer
#define MAX_SPI_TX_Config(x)    SPIDRV_MTransmitB(spi_handle, x, 3);
#define MAX_SPI_TX(x)           SPIDRV_MTransmitB(spi_handle, x, 1);
#define MAX_SPI_RX(x)           SPIDRV_MReceiveB(spi_handle, x, 1);
#define MAX_SPI_TXRX(x,y)       SPIDRV_MTransferB(spi_handle, x, y, 3);


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
     * CLBRT[9:7]   Calibration COnfiguration
     * CLBRT[6:5]   Temperature POrt
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

    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_USART0, true); // Enable clock for USART0 module
    CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 1); // TX
    GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 0); // RX

    USART_InitAsync_TypeDef usartInitUSART0 = {
        .enable = usartDisable,                     // Initially disabled
        .refFreq = 0,                               // configured reference frequency
        .baudrate = 115200,                         // Baud rate defined in common.h
        .oversampling = usartOVS16,                 // overSampling rate x16
        .databits = USART_FRAME_DATABITS_EIGHT,     // 8-bit frames
        .parity = USART_FRAME_PARITY_NONE,          // parity - none
        .stopbits = USART_FRAME_STOPBITS_ONE,       // 1 stop bit
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
 * @function    setupGPIOInt()
 * @abstract    Enable GPIO Interrupts
 * @discussion
 *
 * @return      void
 ******************************************************************************/
void setupGPIOInt() {

    GPIO_PinModeSet(gpioPortD, 4, gpioModeInput, 1);  // MAX Interrupt
    GPIO_IntConfig(gpioPortD, 4, true, true, true);

    NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);

}


/*******************************************************************************
 * @function    GPIO_EVEN_IRQHandler()
 * @abstract    Called after interrupt occurs
 * @discussion  Clear interrupts, read MAX status register
 *
 * @return      void
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void) {
    // TODO Multiple interrupts on EVEN_IRQHandler

    GPIO_IntDisable(0x0010);

    spi_tx_buffer[0] = READ_INT_STAT_REG;
    MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[2]);      // Read status register

    GPIO_IntClear(0x0010);
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
    GPIO_IntEnable(0x0010);

    int i;

    spi_tx_buffer[0] = TOF_DIFF;
    MAX_SPI_TX(&spi_tx_buffer[0]);

    spi_tx_buffer[0] = READ_INT_STAT_REG;
    MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[2]);

    while (1) {

    	spi_tx_buffer[0] = READ_RTC_MIN_HRS;
    	//spi_tx_buffer[0] = READ_RTC_SECS;
        MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[14]);

        // TOF Interrupt (bit 12)
        if((0x10 & spi_rx_buffer[3]) == 0x10)
        {
        	spi_tx_buffer[0] = TOF_DIFF_INT;
            MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[6]);

            spi_tx_buffer[0] = TOF_DIFF_FRAC;
            MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[10]);

        	USART_Tx(USART0, 0x46);

        	spi_rx_buffer[1] = 0x00;
            GPIO_IntEnable(0x0010);

            spi_tx_buffer[0] = TOF_DIFF;
            MAX_SPI_TX(&spi_tx_buffer[0]);
        }

        for(i = 0; i < 1000; i++){}
        i = 0;

    }
}
