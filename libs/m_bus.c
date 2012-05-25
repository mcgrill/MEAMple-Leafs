// -----------------------------------------------------------------------------
// M2 data bus subsystem
// version: 2.0
// date: May 30, 2011
// author: J. Fiene
// -----------------------------------------------------------------------------

#include "m_bus.h"

void m_bus_init(void)
{
	// ENABLE PULLUPS
	set(PORTD,0);
	set(PORTD,1);
	set(PORTD,2);
	
	// CONFIGURE THE CLOCK
	TWBR = 12;	// CLK freq = CPU clock / (16 + 2*TWBR*(4^TWPS)), 16MHz clock, TWBR=12, TWPS=00 -> 400kHz

	// ENABLE interrupts in INT2 (D2)
	set(EICRA,ISC21); clear(EICRA,ISC20); // trigger on falling edge
	set(EIMSK,INT2); // demask the interrupt
	sei(); // enable global interrupts
}