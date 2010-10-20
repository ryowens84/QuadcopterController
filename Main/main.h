#ifndef main_h
#define main_h

#define LED	(1<<15)
#define XBEE_EN	(1<<10)

#define LED_ON()		IOSET0 = LED
#define LED_OFF()		IOCLR0 = LED

#define XBEEon()		IOSET0 = XBEE_EN
#define XBEEoff()		IOCLR0 = XBEE_EN

#endif
