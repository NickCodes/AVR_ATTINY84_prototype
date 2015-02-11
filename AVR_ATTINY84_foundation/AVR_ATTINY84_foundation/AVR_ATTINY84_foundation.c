#include <avr/io.h>
#include "interrupt.h"
#define NOP() asm volatile ("nop" ::)

#pragma region Prescalar & Setup

// Define ports
#define USI_OUT_REG	PORTA	// USI output register
#define USI_IN_REG	PINA	// USI input register
#define USI_DIR_REG	DDRA	// USI direction register
#define USI_CLOCK_PIN PA4	// USI clock I/O pin
#define USI_DATAIN_PIN PA5	// USI data input pin
#define USI_DATAOUT_PIN	PA6	// USI data output pin
#define DIAGNOSTIC_LED PA1	// Diagnostic LED

// Todo - better understand prescaler below
// Bits per second = CPUSPEED / PRESCALER / (COMPAREVALUE+1) / 2... Maximum = CPUSPEED / 64.
#define TC0_PRESCALER_VALUE 8	// 1, 8, 64, 256 or 1024
#define TC0_COMPARE_VALUE 255	// 0 to 255. > 31 with prescaler CLK/1

#if TC0_PRESCALER_VALUE == 1 
#define TC0_PS_SETTING (1<<CS00)
#elif TC0_PRESCALER_VALUE == 8 
#define TC0_PS_SETTING (1<<CS01)
#elif TC0_PRESCALER_VALUE == 64	
#define TC0_PS_SETTING (1<<CS01)|(1<<CS00)
#elif TC0_PRESCALER_VALUE == 256
 #define TC0_PS_SETTING (1<<CS02)
#elif TC0_PRESCALER_VALUE == 1024 
#define TC0_PS_SETTING (1<<CS02)|(1<<CS00)
#else #error Invalid T/C0 prescaler setting
#endif

struct usidriverStatus_t {
	unsigned char masterMode : 1;       // True if in master mode
	unsigned char transferComplete : 1; // True when transfer completed
	unsigned char writeCollision : 1;   // True if put attempted during transfer
};
volatile struct usidriverStatus_t spiX_status;
unsigned char storedUSIDR;

// Generate clock by toggling USCK or writing 1 to USITC bit in USICR
ISR(TIM0_COMPA_vect){	USICR |= (1<<USITC);}

#pragma endregion

#pragma region SPI via 3 wire USI helper methods

// Clock overflow interrupt (fires when full byte is transferred)
ISR(USI_OVF_vect){
	if( spiX_status.masterMode == 1 ) {	TIMSK0 &= ~(1<<OCIE0A); }		// Disable compare match, prevent new USI clocks
	USISR = (1<<USIOIF);												// Clear USI counter
	spiX_status.transferComplete = 1;									// Set transfer complete flag
	storedUSIDR = USIDR;												// Store transferred USIDR data
}
void spiX_initmaster( char spi_mode ){
	// Configure port directions
	USI_DIR_REG |= (1<<USI_DATAOUT_PIN) | (1<<USI_CLOCK_PIN); // Outputs
	USI_DIR_REG &= ~(1<<USI_DATAIN_PIN);                      // Inputs
	USI_OUT_REG |= (1<<USI_DATAIN_PIN);                       // Pull-ups
		
	// Configure USI to 3-wire master mode with overflow interrupt.
	USICR = (1<<USIOIE) | (1<<USIWM0) |
	(1<<USICS1) | (spi_mode<<USICS0) |
	(1<<USICLK);

	TCCR0A |= (1<<WGM01) ;				// Enable clearing the timer automatically on compare match
	TCCR0B |= TC0_PS_SETTING;			// TC0_PS_SETTING is the bit mask value of TC0_PRESCALER_VALUE (1,8,64,256,1024)
	
	//TCCR0A |= TC0_PS_SETTING;			// These bits are the ATTiny84 in TCCR0B (original atmel comment?)
	
	// Init Output Compare Register
	OCR0A = TC0_COMPARE_VALUE;

	spiX_status.masterMode       = 1;
	spiX_status.transferComplete = 0;
	spiX_status.writeCollision   = 0;
	storedUSIDR = 0;
}
// Setup USI SPI slave
void spiX_initslave( char spi_mode ){
	// Configure port directions.
	USI_DIR_REG |= (1<<USI_DATAOUT_PIN);                      // Outputs.
	USI_DIR_REG &= ~(1<<USI_DATAIN_PIN) | (1<<USI_CLOCK_PIN); // Inputs.
	USI_OUT_REG |= (1<<USI_DATAIN_PIN) | (1<<USI_CLOCK_PIN);  // Pull-ups.
	
	// Configure USI to 3-wire slave mode with overflow interrupt.
	USICR = (1<<USIOIE) | (1<<USIWM0) |
	(1<<USICS1) | (spi_mode<<USICS0);

	spiX_status.masterMode       = 0;	// Slave mode on
	spiX_status.transferComplete = 0;
	spiX_status.writeCollision   = 0;
	storedUSIDR = 0;
}
// Put one BYTE in the USIDR, initiating transfer if master
char spiX_put( unsigned char val ){
	// Return if 4 bit USISR counter is not 0000 (0x0f = 00001111) - collision
	// aka don't put another bit on the bus until the entire buffer is ready to be turned over?
	if( (USISR & 0x0F) != 0 ) {	spiX_status.writeCollision = 1;	return 0; }
	spiX_status.transferComplete = 0;
	spiX_status.writeCollision = 0;
	// Put 8bit char on the USIDR register
	USIDR = val;
	if( spiX_status.masterMode == 1 ) {
		TIFR0 |= (1<<OCF0A);   // Clear compare match flag
		TIMSK0 |= (1<<OCIE0A); // Master enable compare match interrupt
	}
	if( spiX_status.writeCollision == 0 ) return 1;
	return 0;
}
// Return the most recently transferred BYTE from USI/SPI bus
unsigned char spiX_get(){	return storedUSIDR;}
// Wait until transfer complete flag is set
void spiX_wait(){	do {} while( spiX_status.transferComplete == 0 ); }
// Swap shift register with USI/SPI slave
unsigned char read_register(unsigned char adress)	{
	PORTA &= ~(1<<PA7);			// Disable pullup resistor for PA7
	spiX_put( adress++ );		// choose register
	spiX_wait();				// Wait for spiX_status.transferComplete flag to be set
	adress = spiX_get();		// returns storedUSIDR value
	PORTA |= (1<<PA7);			// Enable pullup resistor for PA7
	return adress;
}
// Pause for ms milliseconds
void long_delay(uint16_t ms){	for (; ms>0; ms--) NOP();	}
#pragma endregion



// Implementation ------------------------------------------------------------------------

#define SPIMODE 0	// Sample on leading _rising_ edge, setup on trailing _falling_ edge.
//#define SPIMODE 1	// Sample on leading _falling_ edge, setup on trailing _rising_ edge.

int main(void)
{
	spiX_initmaster(SPIMODE);
	//spiX_initslave(SPIMODE);
	
	sei();												// Enable interrupts
	PortSetup();										// Setup I/O
	unsigned char byte_payload = 0b11111111;			// Send this to slave
	unsigned char communication_flags = 0x00;			// Set flag to begin transmission
	read_register(byte_payload);						// Swap one BYTE with slave over USI/SPI
	uint16_t delay = 400;
		
	while(1){

		// Reset flag while button on PA1 is pressed
		if( PINA & ~(1 << PA1) )
		{	
			communication_flags = 0x00;
			PORTB |= (1<<PB0);
			PORTB &= ~(1<<PB1);
			PORTB &= ~(1<<PB2);
		} 
		else 
		{
			PORTB &= ~(1<<PB0);
			long_delay(400);
			// When button is no longer pressed AND flag has been reset
			if (communication_flags == 0x00)
			{
				PORTB |= (1<<PB1);		
				long_delay(400);
				communication_flags = 0x01;			// Set last bit of comm flags to prevent another transmission
				read_register(byte_payload);		// Swap one BYTE with slave			
				PORTB |= (1<<PB2);		
			}
		} // end else
	} // end while
} // end main


// If slave transferred expected payload, blink to indicate success
void CheckForCorrectPayload()
{
	if (storedUSIDR & 0b10101010 )
	{
		while(1){
			PORTB |= (1<<PB0);	
			long_delay(400);
			PORTB &= ~(1<<PB0);	
			long_delay(400);
		}
	}
}

void PortSetup()
{
	DDRA |= (1<<PA1);	// PA1 as input switch
	PORTA |= (1<<PA1);	// Enable pull up resistor on PA1, switch forces pin to ground/low
	DDRB &= ~(1<<PB0);	// PB0 as output diagnostic LED
	DDRB &= ~(1<<PB1);	// PB1 as output diagnostic LED
	DDRB &= ~(1<<PB2);	// PB2 as output diagnostic LED	
}