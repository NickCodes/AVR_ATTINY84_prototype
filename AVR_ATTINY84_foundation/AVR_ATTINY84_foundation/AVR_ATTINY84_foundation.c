#include <avr/io.h>
#include "interrupt.h"
#define NOP() asm volatile ("nop" ::)

// Define ports
#define USI_OUT_REG	PORTA			// USI output register
#define USI_IN_REG	PINA			// USI input register
#define USI_DIR_REG	DDRA			// USI direction register
#define USI_CLOCK_PIN PA4			// USI clock I/O pin
#define USI_DATAIN_PIN PA5			// USI data input pin
#define USI_DATAOUT_PIN	PA6			// USI data output pin
#define DIAGNOSTIC_LED PA1				// Diagnostic LED

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

unsigned char storedUSIDR;

// Driver status struct - master mode flag, transfer complete flag, write during put collision flag
struct usidriverStatus_t {
	unsigned char masterMode : 1;       // True if in master mode.
	unsigned char transferComplete : 1; // True when transfer completed.
	unsigned char writeCollision : 1;   // True if put attempted during transfer.
};
volatile struct usidriverStatus_t spiX_status;

// Generate clock by toggling USCK or writing 1 to USITC bit in USICR
ISR(TIM0_COMPA_vect){	USICR |= (1<<USITC);}

// Clock overflow interrupt (when full byte is transferred)
ISR(USI_OVF_vect){
	if( spiX_status.masterMode == 1 ) {	TIMSK0 &= ~(1<<OCIE0A); } // Disable compare match, prevent new USI clocks
	USISR = (1<<USIOIF);																					// Clear USI counter
	spiX_status.transferComplete = 1;															// Set transfer complete flag
	storedUSIDR = USIDR;																					// Store transferredd USIDR
}

// Setup USI SPI master
void spiX_initmaster( char spi_mode ){
	// Configure port directions.
	USI_DIR_REG |= (1<<USI_DATAOUT_PIN) | (1<<USI_CLOCK_PIN); // Outputs.
	USI_DIR_REG &= ~(1<<USI_DATAIN_PIN);                      // Inputs.
	USI_OUT_REG |= (1<<USI_DATAIN_PIN);                       // Pull-ups.
	
	// Configure USI to 3-wire master mode with overflow interrupt.
	USICR = (1<<USIOIE) | (1<<USIWM0) |
	(1<<USICS1) | (spi_mode<<USICS0) |
	(1<<USICLK);

	TCCR0A |= (1<<WGM01) ;				// Enable clearing the timer automatically on compare match
	//TCCR0A |= TC0_PS_SETTING;		// These bits are the ATTiny84 in TCCR0B (original atmel comment?)
	TCCR0B |= TC0_PS_SETTING;			// TC0_PS_SETTING is the bit mask value of TC0_PRESCALER_VALUE (1,8,64,256,1024)
	
	// Init Output Compare Register.
	OCR0A = TC0_COMPARE_VALUE;
	
	// Init driver status register.
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
	
	// Init driver status register.
	spiX_status.masterMode       = 0;
	spiX_status.transferComplete = 0;
	spiX_status.writeCollision   = 0;
	storedUSIDR = 0;
}

// Put one BYTE in the USIDR, initiating transfer if master
char spiX_put( unsigned char val ){
	// Return if 4 bit USISR counter is not 0000 (0x0f = 00001111) - collision
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




// Implementation ------------------------------------------------------------------------


#define SPIMODE 0	// Sample on leading _rising_ edge, setup on trailing _falling_ edge.
//#define SPIMODE 1	// Sample on leading _falling_ edge, setup on trailing _rising_ edge.

// A function that causes to processor to do nothing for ms milliseconds
void long_delay(uint16_t ms){	for (; ms>0; ms--) NOP();	} // Subtract one from ms until it is zero

unsigned char read_register(unsigned char adress);

int main(void)
{
	spiX_initmaster(SPIMODE);
	//spiX_initslave(SPIMODE);
	
	sei();							// Enable interrupts
	DDRA |= (1<<PA7);				// Set PA7 as input - Pin 7 (PA6) is DI - Data Input for USI in 3 wire SPI mode
	PORTA |= (1<<PA7);				// Enable pull up resistor on PA7

	//DDRA |= (0<<PA3);				// Set PA2 as output
	//PORTA |= (0<<PA3);				// Enable pull up resistor on PA7


	//unsigned char var = 0x1c;							// GPIO register memory address?
	unsigned char byte_payload = 0b11111111;			// Send this to slave
	unsigned char communication_flags = 0b00000000;
	read_register(byte_payload);						// Swap one BYTE with slave
	long_delay(0x3000);
	
	// Status light needs to start red

	while(1){
		// This should return NON ZERO if bit 1 in storedUSDIR is set, ZERO if bit one is not set
		if ( storedUSIDR & 0b00000010 ){
			// If slave returned 0b00000010, turn off error light, turn on success light
			long_delay(0x3000);
		}
		
		// Pressing the button sets flags and requests another transmission, comparing the storedUSDIR again
		
		// PA2 is high by default due to pullup resistor - button is not pressed
		if( PINA & (1 << PA2) )
		{	
			communication_flags &= 0b00000000; // Clear button press flag
		} 
		else 
		{
			if (communication_flags & 0b00000000)
			{
				communication_flags |= 0b00000001;	// Set last bit of comm flags to prevent another transmission
				long_delay(0x3000);					// Turn off status led
				read_register(byte_payload);		// Swap one BYTE with slave			
				long_delay(0x3000);					// Turn off status led
			}
		} // end else
	} // end while
} // end main

unsigned char read_register(unsigned char adress)	{
	PORTA &= ~(1<<PA7);			// Disable pullup resistor for PA7
	spiX_put( adress++ );		// choose resgister
	// Check for collision
	// Clear flags
	// Put (address++)++ in USIDR data register
	// Clear and enable compare match flag
	// Enable compare match interrupt routine (ISR)
	// Return 1 (0 if collision)
	// Compare match cycles 8 bits into USIDR
	// Compare match interrupt fires and USIDR is stored in temp
	spiX_wait();				// Wait for spiX_status.transferComplete flag to be set
	adress = spiX_get();		// returns storedUSIDR value
	PORTA |= (1<<PA7);			// Enable pullup resistor for PA7

	// Interrupt handler for USI counter overflow -----
	// Check mastermode
	// Disable compare match interript (to prevent more clocks since it has triggered the overflow interrupt)
	// Clear USISR 4 bit counter (indicating transmission no longer in progress)
	// Set spiX_status.transferComplete flag
	// Buffer the USIDR register so next transfer will not overwrite

	// Theory of USI / SPI operation -----
	// Compare match is being used as a counter for 8 bit cycles, triggering the overflow interrupt to indicate that the
	// entire 8 bit USIDR has been swapped through USI/SPI
	
	// TODO - why enable and disable pull up resistor?
	// (1<<PA7) returns the bitmask for PA7 (all 0's and a 1 where PA7 occupies the byte)
	// When you OR PORTA against the bit mask, you end up with the bit for PA7 turned on with nothing else affected
	// Using PORTA &= (1<<PA7) disables the PA7 bit without affecting anything else
	
	return adress;
}