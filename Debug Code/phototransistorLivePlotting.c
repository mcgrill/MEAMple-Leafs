// the meample leafs
// ir logging code
// base by nick mcgill

// header files
#include "m_general.h"
#include "m_usb.h"
#include "m_wii.h"


// subroutines
void set_ADC(void);
void update_ADC(void);


// global variables
#define DEBUG 1
#define CLOCK 0
int f0val = 0;
int f1val = 0;
int f4val = 0;
int f5val = 0;

int main(void){

	m_red(ON);
	m_green(ON);


	if (DEBUG){
		m_usb_init(); // connect usb
		while(!m_usb_isconnected()){};  //wait for connection
	}

	m_red(OFF);
	m_green(OFF);

	char rx_buffer; //computer interactions
	unsigned int ir_data[12] = {0,0,0,0,0,0,0,0,0,0,0,0};


	set_ADC();
  
	while(1){
	
		update_ADC();
		while(!m_usb_rx_available());  	//wait for an indication from the computer
		rx_buffer = m_usb_rx_char();  	//grab the computer packet
		m_usb_rx_flush();  				//clear buffer		

		if(rx_buffer == 1) {  			//computer wants ir data
			//write ir data as concatenated hex:  f0f1f4f5
			m_usb_tx_hex(f0val);
			m_usb_tx_hex(f1val);
			m_usb_tx_hex(f4val);
			m_usb_tx_hex(f5val);
			m_usb_tx_char('\n');  //MATLAB serial command reads 1 line at a time
		}
		if (ADC > 512){
			m_green(ON);
			m_red(OFF);
		}	
		else{
			m_red(ON);
			m_green(OFF);
		}
		
		/*if(DEBUG){
			m_usb_tx_string("f0 - FR: ");
			m_usb_tx_int(f0val);
			m_usb_tx_string("     f1 - FL: ");
			m_usb_tx_int(f1val);
			m_usb_tx_string("     f4 - BL: ");
			m_usb_tx_int(f4val);
			m_usb_tx_string("     f5 - BR: ");
			m_usb_tx_int(f5val);
			m_usb_tx_string("\n");
		}*/
	}
}


//_______________________________________ Subroutine for setting ADCs
void set_ADC(void){
	//****************** set ADC values
	clear(ADMUX, REFS1); // voltage Reference - set to VCC
	set(ADMUX, REFS0);   // ^
	
	//clear(ADMUX, REFS1); // voltage Reference - set to Vref, the Aref pin, 3.4V
	//clear(ADMUX, REFS0); // ^

	set(ADCSRA, ADPS2); // set the ADC clock prescaler, divide 16 MHz by 128 (set, set, set)
	set(ADCSRA, ADPS1); // ^
	set(ADCSRA, ADPS0); // ^
	
	set(DIDR0, ADC0D); // disable the 0 digital input
	set(DIDR2, ADC1D); // disable the 1 digital input
	set(DIDR2, ADC4D); // disable the 4 digital input
	set(DIDR2, ADC5D); // disable the 5 digital input
}



//_______________________________________ Subroutine for updating ADCs
void update_ADC(){ 		//update to current ADC values, set to ADC_F0, B4

	//-------------------> set pin F0 to read ADC values
	clear(ADCSRB, MUX5); // single-ended channel selection
	clear(ADMUX, MUX2); // ^
	clear(ADMUX, MUX1); // ^
	clear(ADMUX, MUX0); // ^
	
	set(ADCSRA, ADEN); // start conversion process
	set(ADCSRA, ADSC); // ^
	while(!check(ADCSRA,ADIF));

	f0val = ADC;
	set(ADCSRA, ADIF); // sets flag after conversion


	//-------------------> set pin F1 to read ADC values
	clear(ADCSRB, MUX5);  // F1
	clear(ADMUX, MUX2); 	// ^
	clear(ADMUX, MUX1);   // ^
	set(ADMUX, MUX0);   	// ^

	set(ADCSRA, ADEN); // start conversion process
	set(ADCSRA, ADSC); // ^
	
	while(!check(ADCSRA,ADIF));

	f1val = ADC;
	set(ADCSRA, ADIF); // sets flag after conversion
  
  
	//-------------------> set pin F4 to read ADC values
	clear(ADCSRB, MUX5);  // F4
	set(ADMUX, MUX2); 		// ^
	clear(ADMUX, MUX1);   // ^
	clear(ADMUX, MUX0);   // ^

	set(ADCSRA, ADEN); // start conversion process
	set(ADCSRA, ADSC); // ^
	
	while(!check(ADCSRA,ADIF));
	f4val = ADC;
	set(ADCSRA, ADIF); // sets flag after conversion
	
	
	//-------------------> set pin F5 to read ADC values
	clear(ADCSRB, MUX5);  // F5
	set(ADMUX, MUX2); 		// ^
	clear(ADMUX, MUX1);   // ^
	set(ADMUX, MUX0);   // ^

	set(ADCSRA, ADEN); // start conversion process
	set(ADCSRA, ADSC); // ^
	
	while(!check(ADCSRA,ADIF));
	f5val = ADC;
	set(ADCSRA, ADIF); // sets flag after conversion
}
