/*
// the meample leafs [[ meampleleafs.com ]]
// ir logging code
// by Nick McGill [[ nmcgill.com ]]

Based on code by:
	Michael Posner
	Levi Cai
	Sydney Jackopin

MEAM 510
ROBOCKEY
*/

#include "m_general.h"
#include "m_usb.h"
#include "m_wii.h"


int main(void) {
	
	m_red(ON);
	m_green(ON);

	//initialize serial communications
	m_usb_init();

	while(!m_usb_isconnected()){};  //wait for connection
	m_red(OFF);

	//initialize m_wii
	while(!m_wii_open());  //wait for m_wii to connect
	m_green(OFF);


	char rx_buffer; //computer interactions
	unsigned int wii_data[12] = {0,0,0,0,0,0,0,0,0,0,0,0};


	while(1) {

		//read m_wii data
		m_wii_read(wii_data);

		while(!m_usb_rx_available());  //wait for an indication from the computer

		rx_buffer = m_usb_rx_char();  //grab the computer packet
		m_usb_rx_flush();  //clear buffer

		if(rx_buffer == 1) {  //computer wants m_wii data
			
			//write m_wii data as concatenated hex:  x1y1s1x2y2s2x3y3s3x4y4s4
			int i;
			for(i=0; i < 12; i++) {
				m_usb_tx_hex(wii_data[i]);
			}
			
			m_usb_tx_char('\n');  //MATLAB serial command reads 1 line at a time

		}

	}

}
