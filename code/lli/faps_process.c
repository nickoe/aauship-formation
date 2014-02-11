#include "faps_parse.h"
#include "faps_process.h"
#include "pwm.h"
#include <avr/io.h>
#include "config.h"
#include "crc16.h"
#include <string.h>

int awake_flag;
/*
 * Decide what to do with a given command
 */
int process(msg_t *msg) 
{
	char buildtime[] =  __DATE__ " " __TIME__;
	char gitcommit[sizeof(__GIT_COMMIT__)] = __GIT_COMMIT__;
	char buildinfo[sizeof(buildtime)+40+2];
	int16_t duty = 0;
	int i = 0;

	switch (msg->devid) {
		case 0:
			switch (msg->msgid) {
				case 0:
					grs_send(package(0, 0x00, 0x08, 0),0); // NACK
					break;
				case 1:
					grs_send(package(0, 0x00, 0x08, 0),0); // NACK
					break;
				case 2:
					grs_send(package(0, 0x00, 0x08, 0),0); // NACK
					break;			
				case 5:
					grs_send(package(0, 0x00, 0x05, 0),0); // PING
					break;		
				case 6:
					grs_send(package(0, 0x00, 0x06, 0),0); // PONG
					break;
				case 9:
					memcpy(buildinfo,buildtime,sizeof(buildtime));
					for (i = sizeof(buildtime); i < sizeof(buildinfo); i++) {
						buildinfo[i] = gitcommit[i];
					}
					grs_send(package(sizeof(buildinfo), 0x00, 0x09, buildtime),sizeof(buildinfo));
					break;
			}


		case 10: 
			switch (msg->msgid) {
				case 0:
					break;
				case 1:
					break;
				case 2:
					break;
				case 3:
					duty = (int16_t) (msg->data[0]);
					duty = (duty << 8) & 0xFF00; 
					duty = (duty | ((msg->data[1])&0xFF));
					pwm_set_duty(RC1, duty );
					grs_ack();
					awake_flag = 0;
					break;
				case 4:
				case 5:
					duty = (int16_t) (msg->data[0]);
					duty = (duty << 8) & 0xFF00; 
					duty = (duty | ((msg->data[1])&0xFF));
					pwm_set_duty(RC2, duty );
					grs_ack();
					awake_flag = 0;
					break;
				case 6:
				case 7:
					duty = (int16_t) (msg->data[0]);
					duty = (duty << 8) & 0xFF00; 
					duty = (duty | ((msg->data[1])&0xFF));
					pwm_set_duty(RC3, duty );
					grs_ack();
					awake_flag = 0;
					break;
				case 8:
				case 9:
					duty = (int16_t) (msg->data[0]);
					duty = (duty << 8) & 0xFF00; 
					duty = (duty | ((msg->data[1])&0xFF));
					pwm_set_duty(RC4, duty );
					grs_ack();
					awake_flag = 0;
					break;
				case 10:
				case 11:
					duty = (int16_t) (msg->data[0]);
					duty = (duty << 8) & 0xFF00; 
					duty = (duty | ((msg->data[1])&0xFF));
					pwm_set_duty(RC5, duty );
					grs_ack();
					awake_flag = 0;
					break;
				case 12:
				case 13:
					pwm_set_duty(DC1, msg->data[0]);
					grs_ack();
					awake_flag = 0;
					break;
				case 14:
				case 15:
					pwm_set_duty(DC2, msg->data[0]);
					grs_ack();
					awake_flag = 0;
					break;
				case 16:
				case 17:
					pwm_set_duty(DC3, msg->data[0]);
					grs_ack();
					awake_flag = 0;
					break;
				case 18:
					break;
				case 19:
					duty = (int16_t) (msg->data[0]);
					duty = (duty << 8) & 0xFF00; 
					duty = (duty | ((msg->data[1])&0xFF));
					pwm_set_duty(RC1, duty );
					duty = (int16_t) (msg->data[2]);
					duty = (duty << 8) & 0xFF00; 
					duty = (duty | ((msg->data[3])&0xFF));
					pwm_set_duty(RC2, duty );
					awake_flag = 0;
					break;
				case 20:
					break;

			}

		case 101: 
			pwm_set_duty(DC2, msg->data[0]);
			break;
		case 102: 
			pwm_set_duty(DC3, msg->data[0]);
			break;
		case 104: 
			pwm_set_duty(RC2, msg->data[0]);
			break;
		case 105: 
			pwm_set_duty(RC3, msg->data[0]);
			break;
		case 106: 
			pwm_set_duty(RC4, msg->data[0]);
			break;
		case 107: 
			pwm_set_duty(RC5, msg->data[0]);
			break;
		case 108: 
			PORTL ^= (1<<LED4);
			break;
	}
}

/*
 * Prepare messages
 */
char *package(uint8_t len, uint8_t devid, uint8_t msgid, int8_t data[]) {
	uint8_t i = 0;
	uint16_t crc = 0x0000;

	pack[0] = '$';
	pack[1] = len;
	pack[2] = devid;
	pack[3] = msgid;
	for (i = 0; i < len; i++) {
		pack[i+4] = data[i];
	}
	
	crc = crc16_ccitt_calc(pack+1,len+3);
	
	pack[i+4] = (crc >> 8) & 0x00FF;
	pack[i+5] = crc & 0x00FF;

	return pack;
}

/*
 * Send to HLI
 */
void hli_send(uint8_t ptr[], uint8_t len) {
	int i;

	for (i=0; i<len+6; i++) {
		uart_putc(*(ptr+i));
	}
}

/*
 * Send to GRS
 */
void grs_send(uint8_t ptr[], uint8_t len) {
	int i;

	for (i=0; i<len+6; i++) {
		uart2_putc(*(ptr+i));
	}
}

/*
 * GRS ACK
 */
void grs_ack(void) {
	grs_send(package(0, 0x00, 0x07, NULL), 0); // GRS ACK
}

/*
 * GRS NACK
 */
void grs_nack(void) {
	grs_send(package(0, 0x00, 0x08, NULL), 0); // GRS NACK
}
