#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "Function_prototype.h"  /*Header file contain all function prototype*/

typedef unsigned char uint8;

typedef struct
{
	uint8 sec1;     /*Units of seconds number*/
	uint8 sec2;     /*Tens of seconds number*/
	uint8 min1;     /*Units of minutes number*/
	uint8 min2;     /*Tens of minutes number*/
	uint8 hours1;   /*Units of hours number*/
	uint8 hours2;   /*Tens of hours number*/
}Time;

/*Global structure of all used variables*/
Time g_watch = {0};  /*initialize all members to 0*/


ISR(TIMER1_COMPA_vect)   /*interrupt will generated every one second*/
{
	g_watch.sec1++;

	if(g_watch.sec1 > 9)
	{
		g_watch.sec1 = 0;
		g_watch.sec2++;
	}

	if(g_watch.sec2 == 6)  /*if we finished 60 seconds*/
	{
		g_watch.sec2 = 0;
		g_watch.min1++;
	}

	if(g_watch.min1 > 9)
	{
		g_watch.min1 = 0;
		g_watch.min2++;
	}

	if(g_watch.min2 == 6)  /*if we finished 60 minutes*/
	{
		g_watch.min2 = 0;
		g_watch.hours1++;
	}

	if(g_watch.hours1 > 9)
	{
		g_watch.hours1 = 0;
		g_watch.hours2++;
	}

	if(g_watch.hours2 == 1 && g_watch.hours1 == 2)  /*if we finished 12 hours*/
	{
		g_watch.hours1=0;
		g_watch.hours2=0;
		g_watch.sec1 = 0;
		g_watch.sec2 = 0;
		g_watch.min1 = 0;
		g_watch.min2 = 0;
	}
}


/*When IRQ is occurred reset the stop watch to start count again from 0*/
ISR (INT0_vect)
{
	g_watch.sec1 = 0;
	g_watch.sec2 = 0;
	g_watch.min1 = 0;
	g_watch.min2 = 0;
	g_watch.hours1 = 0;
	g_watch.hours2 = 0;
}


/*When IRQ is occurred Pause the stop watch (by clear the clock bits of TIMER1)*/
ISR (INT1_vect)
{
	TCCR1B = 0;
}


/*When IRQ is occurred Resume the stop watch*/
ISR (INT2_vect)
{
	/* Configure TIMER1 control register TCCR1B
		  1. CTC Mode WGM12=1 WGM13=0
		  2. Prescaler = F_CPU/1024 CS10=1 CS11=0 CS12=1
	 */
	TCCR1B = (1<<WGM12) | (1<<CS10) | (1<<CS12);
}


int main(void)
{
	DDRC |= 0x0F;    /*Make first four pins in PORTC is output for decoder*/

	DDRA |= 0x3F;    /*Make first six pins in PORTA as output for NPN
	                   transistors to enable and disable the 7-segments*/

	PORTA &=0xC0;    /*Initially disable all 7-segments*/

	INT0_Init();     /*To RESET the stop watch if button is pushed PD2*/

	INT1_Init();     /*To pause the stop watch if button is pressed PD3*/

	INT2_Init();     /*To resume the stop watch if button is pressed PB2*/

	TIMER1_CTC_Init();  /*To set TIMER1*/

	while(1)
	{
		/*Disable all segments except first one to display units of seconds*/
		PORTA &=0xC1;
		PORTA|=(1<<0);
		PORTC =(PORTC & 0xF0) | (g_watch.sec1 & 0x0F);
		_delay_ms(2);

		/*Disable all segments except second one to display tens of seconds*/
		PORTA &=0xC2;
		PORTA|=(1<<1);
		PORTC =(PORTC & 0xF0) | (g_watch.sec2 & 0x0F);
		_delay_ms(2);

		/*Disable all segments except third one to display units of minutes*/
		PORTA &=0xC4;
		PORTA|=(1<<2);
		PORTC =(PORTC & 0xF0) | (g_watch.min1 & 0x0F);
		_delay_ms(2);

		/*Disable all segments except fourth one to display tens of minutes*/
		PORTA &=0xC8;
		PORTA|=(1<<3);
		PORTC =(PORTC & 0xF0) | (g_watch.min2 & 0x0F);
		_delay_ms(2);

		/*Disable all segments except fifth one to display units of hours*/
		PORTA &=0xD0;
		PORTA|=(1<<4);
		PORTC =(PORTC & 0xF0) | (g_watch.hours1 & 0x0F);
		_delay_ms(2);

		/*Disable all segments except last one to display tens of hours*/
		PORTA &=0xE0;
		PORTA|=(1<<5);
		PORTC =(PORTC & 0xF0) | (g_watch.hours2 & 0x0F);
		_delay_ms(2);
	}
}


void TIMER1_CTC_Init(void)
{
	TCNT1 = 0;    /*Set the initial count for TIMER1 is 0*/

	OCR1A = 977;  /*Set the compare value to 977 (one second will passed every 977 counts)*/

	/* Configure TIMER1 control register TCCR1A
	  1. Disconnect OC1A and OC1B  COM1A1=0 COM1A0=0 COM1B0=0 COM1B1=0
	  2. FOC1A=1 FOC1B=0  (non-PWM mode)
	  3. CTC Mode WGM10=0 WGM11=0
	 */
	TCCR1A = (1<<FOC1A);

	/* Configure TIMER1 control register TCCR1B
	  1. CTC Mode WGM12=1 WGM13=0
	  2. Prescaler = F_CPU/1024 CS10=1 CS11=0 CS12=1
	 */
	TCCR1B = (1<<WGM12) | (1<<CS10) | (1<<CS12);

	TIMSK |= (1<<OCIE1A);  /*Enable the interrupt of TIMER1A compare mode*/
	SREG |= (1<<7);        /*Set global interrupt enable (I-bit)*/
}


void INT0_Init(void)
{
	DDRD &= ~(1<<PD2);    /*Configure PD2/INT0 as input pin for button*/
	PORTD |= (1<<PD2);    /*Set internal pull up resistor for PD2*/

	/*Falling edge of INT0 generate IRQ*/
	MCUCR = (1<<ISC01);

	GICR |= (1<<INT0);    /*Enable external interrupt 0*/

	sei();  /*Set global interrupt enable (I-bit)*/
}


void INT1_Init(void)
{
	DDRD &= ~(1<<PD3);    /*Configure PD3/INT1 as input pin for button*/

	MCUCR |= (1<<ISC10) | (1<<ISC11);  /*Rising edge of INT0 generate IRQ*/

	GICR |= (1<<INT1);    /*Enable external interrupt 1*/

	sei();  /*Set global interrupt enable (I-bit)*/
}


void INT2_Init(void)
{
	DDRB &= ~(1<<PB2);    /*Configure PB2/INT2 as input pin for button*/
	PORTB |= (1<<PB2);    /*Set internal pull resistor up for PD2*/

	MCUCSR &= ~(1<<ISC2); /*Falling edge of INT0 generate IRQ*/

	GICR |= (1<<INT2);    /*Enable external interrupt 1*/

	sei();  /*Set global interrupt enable (I-bit)*/
}
