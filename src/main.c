#include <unistd.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"


/* ----- Begin Macros ----- */
// SPI Transfer
#define MAX_SPI_CMD(x)          USART_SpiTransfer(USART1, x)

// MAX Clock Enable (CE_)
#define ASSERT_MAX_TDC_CE       GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0)
#define DEASSERT_MAX_TDC_CE     GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 1)

// Null
#define NULL_CMD                0xA0

/* Execution Opcode Commands */
#define TOF_UP                  0x00
#define TOF_DOWN                0x01
#define TOF_DIFF                0x02
#define TEMPERATURE             0x03
#define RESET                   0x04
#define INITIALIZE              0x05
#define TX_CONFIG_FLASH         0x06
#define EVTMG1                  0x07
#define EVTMG2                  0x08
#define EVTMG3                  0x09
#define HALT                    0x0A
#define LDO_TIMED               0x0B
#define LDO_ON                  0x0C
#define LDO_OFF                 0x0D
#define CLBRT                   0x0E  // CLBRT = Calibrate

/* Register Opcode Commands */
// RTC and Watchdog Registers
#define READ_RTC_SECS           0xB0
#define WRITE_RTC_SECS          0x30
#define READ_RTC_MIN_HRS        0xB1
#define WRITE_RTC_MIN_HRS       0x31
#define READ_RTC_DAY_DATE       0xB2
#define WRITE_RTC_DAY_DATE      0x32
#define READ_RTC_M_Y            0xB3
#define WRITE_RTC_M_Y           0x33

#define READ_WD_ALARM_CNT       0xB4
#define WRITE_WD_ALARM_CNT      0x34
#define READ_ALARM              0xB5
#define WRITE_ALARM             0x35

// Configuration Registers
#define READ_TOF1               0xB8
#define WRITE_TOF1              0x38
#define READ_TOF2               0xB9
#define WRITE_TOF2              0x39
#define READ_TOF3               0xBA
#define WRITE_TOF3              0x3A
#define READ_TOF4               0xBB
#define WRITE_TOF4              0x3B
#define READ_TOF5               0xBC
#define WRITE_TOF5              0x3C
#define READ_TOF6               0xBD
#define WRITE_TOF6              0x3D
#define READ_TOF7               0xBE
#define WRITE_TOF7              0x3E

#define READ_EVT_TIMING1        0xBF
#define WRITE_EVT_TIMING1       0x3F
#define READ_EVT_TIMING2        0xC0
#define WRITE_EVT_TIMING3       0x40

#define READ_TOF_MEAS_DELAY     0xC1
#define WRITE_TOF_MEAS_DELAY    0x41

#define READ_CLBRT_CTRL         0xC2
#define WRITE_CLBRT_CTRL        0x42

#define READ_RTC                0xC3
#define WRITE_RTC               0x43

// Conversion Results Registers (Read Only)
#define WVRUP                   0xC4
#define HIT1_UP_INT             0xC5
#define HIT1_UP_FRAC            0xC6
#define HIT2_UP_INT             0xC7
#define HIT2_UP_FRAC            0xC8
#define HIT3_UP_INT             0xC9
#define HIT3_UP_FRAC            0xCA
#define HIT4_UP_INT             0xCB
#define HIT4_UP_FRAC            0xCC
#define HIT5_UP_INT             0xCD
#define HIT5_UP_FRAC            0xCE
#define HIT6_UP_INT             0xCF
#define HIT6_UP_FRAC            0xD0
#define AVG_UP_INT              0xD1
#define AVG_UP_FRAC             0xD2

#define WVRDN                   0xD3
#define HIT1_DN_INT             0xD4
#define HIT1_DN_FRAC            0xD5
#define HIT2_DN_INT             0xD6
#define HIT2_DN_FRAC            0xD7
#define HIT3_DN_INT             0xD8
#define HIT3_DN_FRAC            0xD9
#define HIT4_DN_INT             0xDA
#define HIT4_DN_FRAC            0xDB
#define HIT5_DN_INT             0xDC
#define HIT5_DN_FRAC            0xDD
#define HIT6_DN_INT             0xDE
#define HIT7_DN_FRAC            0xDF
#define AVG_DN_INT              0xE0
#define AVG_DN_FRAC             0xE1

#define TOF_DIFF_INT            0xE2
#define TOF_DIFF_FRAC           0xE3
#define TOF_CYCLE_COUNT         0xE4  // Bits 7 through 0
#define TOF_RANGE               0xE4  // Bits 15 through 8
#define TOF_DIFF_AVG_INT        0xE5
#define TOF_DIFF_AVG_FRAC       0xE6

#define T1_INT                  0xE7
#define T1_FRAC                 0xE8
#define T2_INT                  0xE9
#define T2_FRAC                 0xEA
#define T3_INT                  0xEB
#define T3_FRAC                 0xEC
#define T4_INT                  0xED
#define T4_FRAC                 0xEE
#define TEMP_CYCLE_COUNT        0xEF
#define T1_AVG_INT              0xF0
#define T1_AVG_FRAC             0xF1
#define T2_AVG_INT              0xF2
#define T2_AVG_FRAC             0xF3
#define T3_AVG_INT              0xF4
#define T3_AVG_FRAC             0xF5
#define T4_AVG_INT              0xF6
#define T4_AVG_FRAC             0xF7

#define CLBRT_INT               0xF8
#define CLBRT_FRAC              0xF9

// Status Registers
#define READ_INT_STAT_REG       0xFE
#define READ_CTRL_REG           0x7F
#define WRITE_CTRL_REG          0xFF  // Can only be written to 0

/* ----- End Macros ----- */


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
    MAX_SPI_CMD(WRITE_TOF1);    // Write to TOF1
    MAX_SPI_CMD(0x14);          // TOF1[15:8]   Pulse Launcher Size
    MAX_SPI_CMD(0x10);          // TOF1[7:4]    Pulse Launch Divider
                                // TOF1.3       Stop Polarity
                                // TOF1.2       Reserved (No effect)
                                // TOF1[1:0]    Bias Charge Time
    DEASSERT_MAX_TDC_CE;

    ASSERT_MAX_TDC_CE;
    // TOF2 Register - details of how TOF will be measured
    MAX_SPI_CMD(WRITE_TOF2);    // Write to TOF2
    MAX_SPI_CMD(0xC1);          // TOF2[15:13]  Stop Hits
                                // TOF2[12:7]   T2 Wave Selection
    MAX_SPI_CMD(0x00);          // TOF2[12:7]   T2 Wave Selection
                                // TOF2[6:4]    TOF Duty Cycle
                                // TOF2.3       Reserved (No effect)
                                // TOF2[2:0]    Timeout
    DEASSERT_MAX_TDC_CE;

    ASSERT_MAX_TDC_CE;
    // TOF3 Register - select which waves will be used in time measurements
    MAX_SPI_CMD(WRITE_TOF3);    // Write to TOF3
    MAX_SPI_CMD(0x08);          // TOF3[15:14]  Reserved
                                // TOF3[13:8]   HIT1 Wave Select
    MAX_SPI_CMD(0x09);          // TOF3[7:6]    Reserved
                                // TOF3[5:0]    HIT2 Wave Select
    DEASSERT_MAX_TDC_CE;

    ASSERT_MAX_TDC_CE;
    // TOF4 Register - select which waves will be used in time measurements
    MAX_SPI_CMD(WRITE_TOF4);    // Write to TOF4
    MAX_SPI_CMD(0x0A);          // TOF4[15:14]  Reserved
                                // TOF4[13:8]   HIT3 Wave Select
    MAX_SPI_CMD(0x0B);          // TOF4[7:6]    Reserved
                                // TOF4[5:0]    HIT4 Wave Select
    DEASSERT_MAX_TDC_CE;

    ASSERT_MAX_TDC_CE;
    // TOF5 Register - select which waves will be used in time measurements
    MAX_SPI_CMD(WRITE_TOF5);    // Write to TOF5
    MAX_SPI_CMD(0x0C);          // TOF5[15:14]  Reserved
                                // TOF5[13:8]   HIT5 Wave Select
    MAX_SPI_CMD(0x0D);          // TOF5[7:6]    Reserved
                                // TOF5[5:0]    HIT6 Wave Select
    DEASSERT_MAX_TDC_CE;

    ASSERT_MAX_TDC_CE;
    // TOF6 Register - comparator upstream
    MAX_SPI_CMD(WRITE_TOF6);    // Write to TOF6
    MAX_SPI_CMD(0x06);          // TOF6[15:8]   Comparator Return Offset Upstream
    MAX_SPI_CMD(0x03);          // TOF6[7:0]    Comparator Offset Upstream
    DEASSERT_MAX_TDC_CE;

    ASSERT_MAX_TDC_CE;
    // TOF7 Register - comparator downstream
    MAX_SPI_CMD(WRITE_TOF7);    // Write to TOF7
    MAX_SPI_CMD(0x06);          // TOF7[15:8]   Comparator Return Offset Downstream
    MAX_SPI_CMD(0x03);          // TOF7[7:0]    Comparator Offset Downstream
    DEASSERT_MAX_TDC_CE;

    ASSERT_MAX_TDC_CE;
    MAX_SPI_CMD(WRITE_TOF_MEAS_DELAY);  // Set TOF Measurement Delay
    MAX_SPI_CMD(0x00);
    MAX_SPI_CMD(0xE0);
    DEASSERT_MAX_TDC_CE;

    ASSERT_MAX_TDC_CE;
    MAX_SPI_CMD(WRITE_CLBRT_CTRL);      // Set Calibration and Control Register
    MAX_SPI_CMD(0x02);
    MAX_SPI_CMD(0x7F);
    DEASSERT_MAX_TDC_CE;

    /* ----- End configuration register setup ----- */

    ASSERT_MAX_TDC_CE;
    MAX_SPI_CMD(TX_CONFIG_FLASH);       // Transfer Configuration to Flash Command
    DEASSERT_MAX_TDC_CE;

    ASSERT_MAX_TDC_CE;
    MAX_SPI_CMD(INITIALIZE);            // Initialize
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

    GPIO_IntDisable(0x0010);

    uint8_t readbufInt[6];

    ASSERT_MAX_TDC_CE;
    MAX_SPI_CMD(READ_INT_STAT_REG);     // Read status register
    readbufInt[0] = MAX_SPI_CMD(NULL_CMD);
    readbufInt[1] = MAX_SPI_CMD(NULL_CMD);
    DEASSERT_MAX_TDC_CE;

    // TOF Interrupt (bit 12)
    if((0x10 & readbufInt[0]) == 0x10)
    {
        ASSERT_MAX_TDC_CE;
        MAX_SPI_CMD(TOF_DIFF_INT); 	    // Read TOF_DIFFInt
        readbufInt[2] = MAX_SPI_CMD(NULL_CMD);
        readbufInt[3] = MAX_SPI_CMD(NULL_CMD);
        DEASSERT_MAX_TDC_CE;

        ASSERT_MAX_TDC_CE;
        MAX_SPI_CMD(TOF_DIFF_FRAC);     // Read TOF_DIFFFrac
        readbufInt[4] = MAX_SPI_CMD(NULL_CMD);
        readbufInt[5] = MAX_SPI_CMD(NULL_CMD);
        DEASSERT_MAX_TDC_CE;
    }

    GPIO_IntClear(0x0010);
    GPIO_IntEnable(0x0010);
}


int main(void)
{
    /* Chip errata */
    CHIP_Init();

    SPI_init();
    MAX_init();  // Initialize MAX settings through SPI

    setupGPIOInt();
    GPIO_IntEnable(0x0010);

    uint8_t readbuf[8];
    // float mydata;
    int i;

    while (1) {

        ASSERT_MAX_TDC_CE;
        MAX_SPI_CMD(TOF_DIFF);          // TOF Diff
        DEASSERT_MAX_TDC_CE;

        ASSERT_MAX_TDC_CE;
        MAX_SPI_CMD(READ_RTC_SECS);     // Read RTC Seconds
        readbuf[2] = MAX_SPI_CMD(NULL_CMD);
        readbuf[3] = MAX_SPI_CMD(NULL_CMD);
        DEASSERT_MAX_TDC_CE;

        // mydata = *((float*)(&readbuf[2]));

        for(i = 0; i < 1000; i++){}
        i = 0;

    }
}
