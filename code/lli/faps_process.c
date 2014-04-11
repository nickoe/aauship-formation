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
    char buildinfo[sizeof(buildtime)+sizeof(gitcommit)];
	int16_t duty = 0;
	int i = 0;

	switch (msg->devid) {
		// GENERAL LLI
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
          memcpy(buildinfo,buildtime,sizeof(buildtime)-1);
          for (i = sizeof(buildtime); i < sizeof(buildinfo); i++) {
	          buildinfo[i-1] = gitcommit[i-sizeof(buildtime)];
          }
					grs_send(package(sizeof(buildinfo), 0x00, 0x09, buildinfo),sizeof(buildinfo));
					hli_send(package(sizeof(buildinfo), 0x00, 0x09, buildinfo),sizeof(buildinfo));
					break;
			}

		// ACTUATORS
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
				case 21:
					duty = (int16_t) ((msg->data[0] << 8) & 0xFF00) | (msg->data[1]&0xFF);
              if (duty == 0)
                PORTF &= ~(1<<DCDIR1);
              else
	    					PORTF |= (1<<DCDIR1);
					pwm_set_duty(DC1, duty );
					awake_flag = 0;
					break;
				case 22:
					duty = (int16_t) ((msg->data[0] << 8) & 0xFF00) | (msg->data[1]&0xFF);
              if (duty == 0)
                PORTF &= ~(1<<DCDIR2);
              else
	    					PORTF |= (1<<DCDIR2);
					pwm_set_duty(DC2, duty );
					awake_flag = 0;
					break;
				case 23:
					duty = (int16_t) ((msg->data[0] << 8) & 0xFF00) | (msg->data[1]&0xFF);
              if (duty == 0)
                PORTF &= ~(1<<DCDIR3);
              else
	    					PORTF |= (1<<DCDIR3);
					pwm_set_duty(DC3, duty );
					awake_flag = 0;
					break;
				case 34:
					PORTF &= ~(1<<DCDIR1);
					break;
				case 35:
					PORTF &= ~(1<<DCDIR2);
					break;
				case 36:
					PORTF &= ~(1<<DCDIR3);
					break;

			}
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
