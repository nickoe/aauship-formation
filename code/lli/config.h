#ifndef CONFIG_H
#define CONFIG_H
/**
 * @file
 * @author Nick Østergaard
 *
 * @section DESCRIPTION
 * This is the configuration file for the LLI interface for AAUSHIP1
 */

#define DEBUG 
#define LOG_ENABLE
#define AUTO_SHUTDOWN_ENABLE
//#define RF_TEST_IDX

extern int awake_flag;
extern uint8_t rmc_idx;

/**
 * Serial rates
 */
// USB connection
/* IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT !!!
 * The uart baud tate of 115200 is desired to achieve higher sample
 * rate of the IMU, currently it is observed that the LLI cannot
 * handæle communicatio form the HLI when using 115200, so for now we
 * are using 57600, which works, but we have slow IMU sampling. Should
 * be good enough so far.
 */
//#define UART_BAUD_RATE   115200
#define UART_BAUD_RATE    57600
// APC220/RF7020 radio
#define UART2_BAUD_RATE   38400
// UP-501 GPS
#define UART3_BAUD_RATE    9600


/**
 * Blinkenlights
 */
#define LED1 PL0 // Radio RX indicator
#define LED2 PL1 // Radio TX indicator
#define LED3 PL2 // GPS message indicator
#define LED4 PL3 // IMU indicator


/**
 * PWM outputs
 */ 
// OC3 timer
#define DCPWM1 PE4 // OC3B
#define DCPWM2 PE5 // OC3C
#define DCPWM3 PE3 // OC3A
// OC1 timer
#define RCPWM1 PB5 // OC1A
#define RCPWM2 PB6 // OC1B
#define RCPWM3 PB7 // OC1C
// OC4 timer
#define RCPWM4 PH4 // OC4B
#define RCPWM5 PH5 // OC4C

/**
 * Message related stuff
 */
#define MAX_MSG_SIZE 1024  // This should probably be 255 + len + devid + msgid + checksum
#define TX_BUFF_SIZE 350

/* Defines sample rate from the ADIS16405 bt counting interrupt ticks
 * from sample ready pin on device */
#define ADIS_READY 164 // 10 Hz
//#define ADIS_READY 82 // 20Hz
//#define ADIS_READY 42 // 40Hz

/* A hacky timer used to be able to send data in bulk, such the radio
 * can have time to switch from RX to TX and vice versa, to gain
 * better throughput */
#define TX_READY 546 // 3Hz

/* Defines for how many control input packets we can loose without it
 * shutting down */
#define AWAKE_THRESHOLD 10

#endif // CONFIG_H 
