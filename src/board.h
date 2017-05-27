/**
 * PINS Definitions
 */

#ifndef _SAM4S_BOARD_H
#define _SAM4S_BOARD_H

#include <conf_board.h>
#include <stdint.h>
#include <sam4sd32b.h>

extern void board_init(void);
extern void resetup_system_clocks(uint32_t sclk);

/**
 * \def CONF_BOARD_KEEP_WATCHDOG_AT_INIT
 * \brief Let watchdog remain enabled
 *
 * If this symbol is defined, the watchdog is left running with its current
 * configuration. Otherwise, it gets disabled during board initialization.
 */
# ifndef CONF_BOARD_KEEP_WATCHDOG_AT_INIT
#  define CONF_BOARD_KEEP_WATCHDOG_AT_INIT
# endif


//! Name string macro
#define BOARD_NAME                "ROCKHOPPER SAM4S"

//! Resonator definitions
#define BOARD_FREQ_SLCK_XTAL      (32768U)
#define BOARD_FREQ_SLCK_BYPASS    (32768U)
#define BOARD_FREQ_MAINCK_XTAL    (10000000U)
#define BOARD_FREQ_MAINCK_BYPASS  (10000000U)
#define BOARD_OSC_STARTUP_US      15625

#define PIN_PA1    IOPORT_CREATE_PIN(PIOA, 1)
#define PIN_PA2    IOPORT_CREATE_PIN(PIOA, 2)
#define PIN_PA3    IOPORT_CREATE_PIN(PIOA, 3)
#define PIN_PA4     IOPORT_CREATE_PIN(PIOA, 4)
#define PIN_PA5     IOPORT_CREATE_PIN(PIOA, 5)
#define PIN_PA6     IOPORT_CREATE_PIN(PIOA, 6)
#define PIN_PA7     IOPORT_CREATE_PIN(PIOA, 7)
#define PIN_PA8     IOPORT_CREATE_PIN(PIOA, 8)
#define PIN_PA14    IOPORT_CREATE_PIN(PIOA, 14)
#define PIN_PA15    IOPORT_CREATE_PIN(PIOA, 15)
#define PIN_PA16    IOPORT_CREATE_PIN(PIOA, 16)
#define PIN_PA17    IOPORT_CREATE_PIN(PIOA, 17)
#define PIN_PA21    IOPORT_CREATE_PIN(PIOA, 21)
#define PIN_PA22    IOPORT_CREATE_PIN(PIOA, 22)
#define PIN_PA23    IOPORT_CREATE_PIN(PIOA, 23)
#define PIN_PA24    IOPORT_CREATE_PIN(PIOA, 24)
#define PIN_PA25    IOPORT_CREATE_PIN(PIOA, 25)
#define PIN_PA27    IOPORT_CREATE_PIN(PIOA, 27)
#define PIN_PB0     IOPORT_CREATE_PIN(PIOB, 0)
#define PIN_PB1     IOPORT_CREATE_PIN(PIOB, 1)
#define PIN_PB2     IOPORT_CREATE_PIN(PIOB, 2)
#define PIN_PB3     IOPORT_CREATE_PIN(PIOB, 3)
#define PIN_PB4     IOPORT_CREATE_PIN(PIOB, 4)
#define PIN_PB12     IOPORT_CREATE_PIN(PIOB, 12)
#define PIN_PB13     IOPORT_CREATE_PIN(PIOB, 13)


#define PIN_PI_GPIO23    PIN_PA3
#define PIN_PI_GPIO24    PIN_PA2
#define PIN_PI_GPIO25    PIN_PA25
#define PIN_PI_GPIO27    PIN_PA7
#define PIN_PI_GPIO22    PIN_PA8
#define PIN_PI_GPIO7    PIN_PA24

#define PIN_J5_2    PIN_PA4
#define PIN_J5_3    PIN_PA1
#define PIN_J5_4    PIN_PB12
#define PIN_J5_5    PIN_PB13

// Pins for testing only
//#define SSC_ADC_BUF_PIN PIN_PI_GPIO23
//#define SD_CARD_WR_PIN PIN_PI_GPIO27
//#define SD_CARD_RD_PIN PIN_PI_GPIO22

// SPI handshaking
#define PI_ON_PIN PIN_PI_GPIO22
#define SPI_SLAVE_READY_PIN PIN_PI_GPIO24
#define SPI_MASTER_READY_PIN PIN_PI_GPIO25

#define PIN_ENABLE_TVDD   PIN_PA21
#define PIN_ENABLE_3V3_OUT   PIN_PA6
#define PIN_ENABLE_5V_OUT   PIN_PA5
#define PIN_ENABLE_ADC   PIN_PA23

#define PIN_PREAMP_SHDN PIN_J5_2
#define PIN_PREAMP_G1 PIN_J5_3
#define PIN_PREAMP_G0 PIN_J5_4

// LED definitions
#define LED_PIN             PIN_PA22
#define LED_ON              true
#define LED_OFF             !LED_ON

//! Number of on-board LEDs
#define LED_COUNT 1

//! Number of on-board buttons
#define BUTTON_COUNT              0

/** UART0 pins (UTXD0 and URXD0) definitions, PA9,10. */
#define PINS_UART0         (PIO_PA9A_URXD0 | PIO_PA10A_UTXD0)
#define PINS_UART0_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_UART0_MASK    PIO_PA9A_URXD0|PIO_PA10A_UTXD0
#define PINS_UART0_PIO     PIOA
#define PINS_UART0_ID      ID_PIOA
#define PINS_UART0_TYPE    PIO_PERIPH_A
#define PINS_UART0_ATTR    PIO_DEFAULT

/** UART1 pins (UTXD1 and URXD1) definitions, PB2,PB3. */
#define PINS_UART1         (PIO_PB2A_URXD1 | PIO_PB3A_UTXD1)
#define PINS_UART1_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_UART1_MASK    PIO_PB2A_URXD1 | PIO_PB3A_UTXD1
#define PINS_UART1_PIO     PIOB
#define PINS_UART1_ID      ID_PIOB
#define PINS_UART1_TYPE    PIO_PERIPH_A
#define PINS_UART1_ATTR    PIO_DEFAULT

// UARTS
#define CONSOLE_UART              UART0
#define CONSOLE_UART_ID           ID_UART0
#define CONSOLE_UART_BAUDRATE     9600

//! \name HSMCI pins definition
/*! Number of slot connected on HSMCI interface */
#define SD_MMC_HSMCI_MEM_CNT      1
#define SD_MMC_HSMCI_SLOT_0_SIZE  4
#define PINS_HSMCI   {0x3fUL << 26, PIOA, ID_PIOA, PIO_PERIPH_C, PIO_PULLUP}
/** HSMCI MCCDA pin definition. */
#define PIN_HSMCI_MCCDA_GPIO            (PIO_PA28_IDX)
#define PIN_HSMCI_MCCDA_FLAGS           (PIO_PERIPH_C | PIO_DEFAULT)
/** HSMCI MCCK pin definition. */
#define PIN_HSMCI_MCCK_GPIO             (PIO_PA29_IDX)
#define PIN_HSMCI_MCCK_FLAGS            (PIO_PERIPH_C | PIO_DEFAULT)
/** HSMCI MCDA0 pin definition. */
#define PIN_HSMCI_MCDA0_GPIO            (PIO_PA30_IDX)
#define PIN_HSMCI_MCDA0_FLAGS           (PIO_PERIPH_C | PIO_DEFAULT)
/** HSMCI MCDA1 pin definition. */
#define PIN_HSMCI_MCDA1_GPIO            (PIO_PA31_IDX)
#define PIN_HSMCI_MCDA1_FLAGS           (PIO_PERIPH_C | PIO_DEFAULT)
/** HSMCI MCDA2 pin definition. */
#define PIN_HSMCI_MCDA2_GPIO            (PIO_PA26_IDX)
#define PIN_HSMCI_MCDA2_FLAGS           (PIO_PERIPH_C | PIO_DEFAULT)
/** HSMCI MCDA3 pin definition. */
#define PIN_HSMCI_MCDA3_GPIO            (PIO_PA27_IDX)
#define PIN_HSMCI_MCDA3_FLAGS           (PIO_PERIPH_C | PIO_DEFAULT)
/** SD/MMC card detect pin definition. */

/** SSC pin Receiver Data (RD) */
#define PIN_SSC_RD        (PIO_PA18_IDX)
#define PIN_SSC_RD_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
/** SSC pin Transmitter Clock (TK) */
#define PIN_SSC_RK        (PIO_PA19_IDX)
#define PIN_SSC_RK_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
/** SSC pin Transmitter FrameSync (TF) */
#define PIN_SSC_RF        (PIO_PA20_IDX)
#define PIN_SSC_RF_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)

/** SSC pin Transmitter Data (TD) */
#define PIN_SSC_TD        (PIO_PA17_IDX)
#define PIN_SSC_TD_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
/** SSC pin Transmitter Clock (TK) */
#define PIN_SSC_TK        (PIO_PA16_IDX)
#define PIN_SSC_TK_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
/** SSC pin Transmitter FrameSync (TF) */
#define PIN_SSC_TF        (PIO_PA15_IDX)
#define PIN_SSC_TF_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)


// Timer Counters Pins
#define PIN_TC0_TIOA0 (PIO_PA0_IDX)
#define PIN_TC0_TIOA0_MUX   (IOPORT_MODE_MUX_B)
#define PIN_TC0_TIOA0_FLAGS (PIO_PERIPH_B | PIO_DEFAULT)


// SPI
/** SPI MISO pin definition. */
#define SPI_MISO_GPIO         (PIO_PA12_IDX)
#define SPI_MISO_FLAGS       (PIO_PERIPH_A | PIO_PULLUP)
/** SPI MOSI pin definition. */
#define SPI_MOSI_GPIO         (PIO_PA13_IDX)
#define SPI_MOSI_FLAGS       (PIO_PERIPH_A | PIO_PULLUP)
/** SPI SPCK pin definition. */
#define SPI_SPCK_GPIO         (PIO_PA14_IDX)
#define SPI_SPCK_FLAGS       (PIO_PERIPH_A | PIO_PULLUP)

/** SPI chip select 0 pin definition. (Only one configuration is possible) */
#define SPI_NPCS0_GPIO         (PIO_PA11_IDX)
#define SPI_NPCS0_FLAGS           (PIO_PERIPH_A | PIO_DEFAULT)


// SD Card
//#define SD_MMC_0_CD_GPIO            (PIO_PC12_IDX)
//#define SD_MMC_0_CD_PIO_ID          ID_PIOC
//#define SD_MMC_0_CD_FLAGS           (PIO_INPUT | PIO_PULLUP)

//#define SD_MMC_0_CD_DETECT_VALUE    0

//#define SD_MMC_0_CD    {PIO_PC12, PIOC, ID_PIOC, PIO_INPUT, PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE}
//#define SD_MMC_0_CD_MASK PIO_PC12
//#define SD_MMC_0_CD_PIO PIOC
//#define SD_MMC_0_CD_ID ID_PIOC
//#define SD_MMC_0_CD_TYPE PIO_INPUT
//#define SD_MMC_0_CD_ATTR (PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_EDGE)




#endif /* SAM4S_H */
