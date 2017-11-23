#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include <stdbool.h>

#define EOL_S "\r\n"
#define EOL_CHAR (EOL_S[0])

struct {
	uint16_t carrierPeriod;					// 00 Период несущей частоты
	uint16_t sequenceLength;				// 01 Количество бит в кодовой последовательности
	uint16_t startPulseLength1;				// 02 Длительность стартового импульса
	uint16_t startPauseLength1;				// 03 Длительность паузы после стартового импульса
	uint16_t reserved04;					// 04 reserved;
	uint16_t reserved05;					// 05 reserved;
	uint16_t stopPulseLength1;				// 06 Длительность завершающего импульса
	uint16_t stopPauseLength1;				// 07 Длительность паузы после завершающего импульса
	uint16_t reserved08;					// 08 reserved;
	uint16_t reserved09;					// 09 reserved;
	uint16_t zeroPulseLength;				// 0A Длительность импульса "0"
	uint16_t zeroPauseLength;				// 0B Длительность паузы после импульса "0"
	uint16_t onePulseLength;				// 0C Длительность импульса "1"
	uint16_t onePauseLength;				// 0D Длительность паузы после импульса "1"
	uint16_t repeatStartPulseLength1;		// 0E Длительность стартового импульса последовательности повтора
	uint16_t repeatStartPauseLength1;		// 0F Длительность паузы после стартового импульса последовательности повтора
	uint16_t reserved10;					// 10 reserved
	uint16_t reserved11;					// 11 reserved
	uint16_t repeatStopPulseLength1;		// 12 Длительность завершающего импульса последовательности повтора
	uint16_t repeatStopPauseLength1;		// 13 Длительность паузы после завершающего импульса последовательности повтора
	uint16_t reserved14;					// 14 reserved
	uint16_t reserved15;					// 15 reserved
} settings = {
	210,	// 00
	32,		// 01
	684,	// 02
	342,	// 03
	0,		// 04
	0,		// 05
	42,		// 06
	1710,	// 07
	0,		// 08
	0,		// 09
	42,		// 0A
	42,		// 0B
	42,		// 0C
	126,	// 0D
	684,	// 0E
	171,	// 0F
	0,		// 10
	0,		// 11
	42,		// 12
	3800,	// 13
	0,		// 14
	0		// 15
};

#define REG_INDEX(x) ((((void*)&settings.x) - ((void*)&settings)) / sizeof(uint16_t))


volatile uint16_t delay = 0;

volatile uint16_t pulseLen = 0;
volatile uint16_t pauseLen = 0;


#define STATE_IDLE			0
#define STATE_CMD_START		1
#define STATE_CMD_DATA		2
#define STATE_REPEAT_START	3
#define STATE_REPEAT_END	4
#define STATE_DELAY			5
#define STATE_DELAY_END		6

volatile uint8_t state = STATE_IDLE;

volatile uint8_t bitCounter;

volatile uint8_t sequence[32];
volatile uint8_t sequenceShift;

void nextPulse(void) {
	switch (state) {
	case STATE_CMD_START:
		pulseLen = settings.startPulseLength1;
		pauseLen = settings.startPauseLength1;
		state = STATE_CMD_DATA;
		bitCounter = 0;
		break;
	case STATE_CMD_DATA:
		if (bitCounter < (settings.sequenceLength & 0xFF)) {
			if ((bitCounter & 0x07) == 0) {
				sequenceShift = sequence[bitCounter >> 3];
			}
			if (sequenceShift & 0x01) {
				pulseLen = settings.onePulseLength;
				pauseLen = settings.onePauseLength;
			} else {
				pulseLen = settings.zeroPulseLength;
				pauseLen = settings.zeroPauseLength;
			}
			sequenceShift = sequenceShift >> 1;
			bitCounter++;
		} else {
			pulseLen = settings.stopPulseLength1;
			pauseLen = settings.stopPauseLength1;
			state = STATE_IDLE;
		}
		break;
	case STATE_REPEAT_START:
		pulseLen = settings.repeatStartPulseLength1;
		pauseLen = settings.repeatStartPauseLength1;
		state = STATE_REPEAT_END;
		break;
	case STATE_REPEAT_END:
		pulseLen = settings.repeatStopPulseLength1;
		pauseLen = settings.repeatStopPauseLength1;
		state = STATE_IDLE;
		break;
	case STATE_DELAY:
		pulseLen = 0;
		pauseLen = delay;
		state = STATE_DELAY_END;
		break;
	case STATE_DELAY_END:
		pulseLen = 0;
		pauseLen = 0;
		state = STATE_IDLE;
		break;
	default:
		pulseLen = 0;
		pauseLen = 0;
		break;
	}
}

ISR(TIMER1_COMPA_vect) {
	if (pulseLen > 0) {
		pulseLen--;
		if (pulseLen == 0) {
			TCCR1A = (0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<FOC1A)|(0<<FOC1B)|(0<<WGM11)|(0<<WGM10);
		}
	} else if (pauseLen > 0) {
			pauseLen--;
	}

	if (pauseLen == 0) {
		nextPulse();
		if (pulseLen > 0) {
			TCCR1A = (0<<COM1A1)|(1<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<FOC1A)|(0<<FOC1B)|(0<<WGM11)|(0<<WGM10);
		}
	}
}


volatile uint8_t rxbuf[256];
volatile uint8_t rxbufhead = 0;
volatile uint8_t rxbuftail = 0;
volatile uint8_t rxbufsize = 0;


ISR(USART_RXC_vect) {
	uint8_t c = UDR;
	if (rxbufhead != rxbuftail-1) {
		rxbuf[rxbufhead] = c;
		rxbufhead++;
		rxbufsize++;
		if (rxbufsize >= 192) {
			// CTS = 1
			PORTD |= (1<<2);
		}
	}
}


uint8_t getByte(void) {
	while (rxbufsize == 0);
	cli();
	uint8_t r = rxbuf[rxbuftail];
	rxbuftail++;
	rxbufsize--;
	if (rxbufsize < 192) {
		// CTS = 0
		PORTD &= ~(1<<2);
	}
	sei();
	return r;
}

void sendByte(uint8_t c) {
	while (!(UCSRA & (1<<UDRE)));
	UDR = c;
}

void sendStringPGM(const prog_char * str) {
	char c;
	while ((c = pgm_read_byte(str++)) != 0) {
		sendByte(c);
	}
}

volatile bool error = false;

void setError(void) {
	if (!error) {
		error = true;
		while (getByte() != EOL_CHAR);
	}
}

uint8_t expectChar(void) {
	if (error) {
		return 0;
	} else {
		uint8_t c = getByte();
		if (c == EOL_CHAR) {
			error = true;
			return 0;
		} else {
			return c;
		}
	}
}

void expectEol(void) {
	if (error) {
		return;
	} else {
		uint8_t c = getByte();
		if (c != EOL_CHAR) {
			setError();
		}
	}
}

void expectConst(uint8_t s) {
	if (expectChar() != s) {
		setError();
	}
}

uint8_t expectHex4(void) {
	uint8_t c = expectChar();
	if (c >= '0' && c <= '9') {
		return c - '0';
	} else if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	} else if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	} else {
		setError();
		return 0;
	}
}

uint8_t expectHex8(void) {
	uint8_t high = expectHex4();
	uint8_t low = expectHex4();
	return (high << 4) | (low);
}

uint16_t expectHex16(void) {
	uint8_t high = expectHex8();
	uint8_t low = expectHex8();
	return (high << 8) | (low);
}

void sendHexNibble(uint8_t n) {
	n = n & 0x0F;
	if (n <= 9) {
		sendByte('0' + n);
	} else {
		sendByte('A' - 10 + n);
	}
}

void sendHexByte(uint8_t n) {
	sendHexNibble(n >> 4);
	sendHexNibble(n >> 0);
}

void sendFail(uint8_t fail) {
	sendStringPGM(PSTR("ERROR "));
	sendHexByte(fail);
	sendStringPGM(PSTR(EOL_S));
}

void sendOk(void) {
	sendStringPGM(PSTR("OK" EOL_S));
}

void cmdVarSet(void) {
	uint8_t regAddr = expectHex8();
	expectConst('=');
	uint16_t regValue = expectHex16();
	expectEol();
	
	if (error) {
		sendFail(0x02);
	} else {
		if (regAddr < (sizeof(settings) / sizeof(uint16_t))) {
			((uint16_t *)&settings)[regAddr] = regValue;
			switch (regAddr) {
			case REG_INDEX(carrierPeriod):
				OCR1A = regValue;
				break;
			}
			sendOk();
		} else {
			sendFail(0x03);
		}
	}	
}

void cmdTransmit(void) {

	uint8_t sequenceByteCount = ((settings.sequenceLength & 0xFF) + 7) >> 3;
	
	while (sequenceByteCount) {
		sequence[--sequenceByteCount] = expectHex8();
	}
	expectEol();

	if (error) {
		sendFail(0x02);
	} else {
		state = STATE_CMD_START;
		while(state != STATE_IDLE);
		sendOk();
	}

}

void cmdRepeat(void) {
	expectEol();

	if (error) {
		sendFail(0x02);
	} else {
		state = STATE_REPEAT_START;
		while(state != STATE_IDLE);
		sendOk();
	}
}

void cmdDelay(void) {
	delay = expectHex16();
	expectEol();

	if (error) {
		sendFail(0x02);
	} else {
		state = STATE_DELAY;
		while(state != STATE_IDLE);
		sendOk();
	}
}

void cmdList(void) {
	expectEol();
	
	if (error) {
		sendFail(0x02);
	} else {
		for (uint8_t i = 0; i < sizeof(settings) / sizeof(uint16_t); i++) {
			uint16_t val = ((uint16_t *)&settings)[i];
			sendByte('S');
			sendHexByte(i);
			sendByte('=');
			sendHexByte((val >> 8) & 0xFF);
			sendHexByte(val & 0xFF);
			sendStringPGM(PSTR(EOL_S));
		}
		sendOk();
	}
}

void cmdVer(void) {
	expectEol();
	
	if (error) {
		sendFail(0x02);
	} else {
		sendStringPGM(PSTR("Build " __DATE__ " " __TIME__ EOL_S));
		sendOk();
	}
}

int main(void) {


	UCSRA = (0<<U2X)|(0<<MPCM);
	UCSRB = (1<<RXCIE)|(0<<TXCIE)|(0<<UDRIE)|(1<<RXEN)|(1<<TXEN)|(0<<UCSZ2)|(0<<TXB8);
	UCSRC = (1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0)|(0<<UCPOL);
	UBRRH = 0;
	UBRRL = 51;


	TCCR1A = (0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<FOC1A)|(0<<FOC1B)|(0<<WGM11)|(0<<WGM10);
	TCCR1B = (0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(1<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10);
	OCR1A = 210;
	TIMSK |= (1<<OCIE1A);

	DDRB |= (1<<1)|(1<<0);

	DDRD |= (1<<2);
	
	
	/*rxbuf[rxbufhead++] = 'S';
	rxbuf[rxbufhead++] = '0';
	rxbuf[rxbufhead++] = '1';
	rxbuf[rxbufhead++] = '=';
	rxbuf[rxbufhead++] = '9';
	rxbuf[rxbufhead++] = '8';
	rxbuf[rxbufhead++] = '2';
	rxbuf[rxbufhead++] = '0';
	rxbuf[rxbufhead++] = EOL_CHAR;
	rxbuf[rxbufhead++] = 'T';
	rxbuf[rxbufhead++] = 'A';
	rxbuf[rxbufhead++] = 'B';
	rxbuf[rxbufhead++] = '5';
	rxbuf[rxbufhead++] = '4';
	rxbuf[rxbufhead++] = 'F';
	rxbuf[rxbufhead++] = 'E';
	rxbuf[rxbufhead++] = '0';
	rxbuf[rxbufhead++] = '1';
	rxbuf[rxbufhead++] = EOL_CHAR;
	rxbuf[rxbufhead++] = 'R';
	rxbuf[rxbufhead++] = EOL_CHAR;
	rxbuf[rxbufhead++] = 'T';
	rxbuf[rxbufhead++] = 'A';
	rxbuf[rxbufhead++] = 'B';
	rxbuf[rxbufhead++] = '5';
	rxbuf[rxbufhead++] = '4';
	rxbuf[rxbufhead++] = 'F';
	rxbuf[rxbufhead++] = 'E';
	rxbuf[rxbufhead++] = '0';
	rxbuf[rxbufhead++] = '1';
	rxbuf[rxbufhead++] = EOL_CHAR;

	rxbufsize = rxbufhead;*/
	

	sei();

	for (;;) {
		error = false;
		uint8_t c = expectChar();
		if (!error) {
			switch(c) {
			case 'S':
			case 's':
				cmdVarSet();
				break;
			case 'T':
			case 't':
				cmdTransmit();
				break;
			case 'R':
			case 'r':
				cmdRepeat();
				break;
			case 'D':
			case 'd':
				cmdDelay();
				break;
			case 'L':
			case 'l':
				cmdList();
				break;
			case 'V':
			case 'v':
				cmdVer();
				break;
			default:
				expectEol();
				sendFail(0x01);
				break;
			}
		}
	}

}



