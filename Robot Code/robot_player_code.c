// MEAM 510 - ROBOCKEY
// The meamtreal MEAMpleleafs
//
// bot #1
// by Mike Kofron and Nick McGill


// define frequency of microcontroller
#ifndef F_CPU
#define F_CPU 16000000UL
#endif


// HEADER FILES
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>
#include "m_general.h"
#include "m_usb.h"
#include "m_rf.h"
#include "m_usb.h"
#include "m_wii.h"


// SUBROUTINES
void init(void);
void init_timer(void);
void init_ADC(void);
void track(unsigned int*);
char search(void);
void pause(void);
char approach_puck(void);
char approach_net(void);
void shoot(void);
void home(void);
void celebrate(void);
void comm_test(void);
void update_ADC(void);
void check_goal(void);
void debug_goal(void);
void debug_localization(void);
void debug_photo(void);


// GLOBAL VARIABLES
#define X_GOAL_A 115		// global X-position of Team A's net [cm]
#define Y_GOAL_A 10			// global Y-position of Team A's net [cm]
#define X_GOAL_B -115		// global X-position of Team B's net [cm]
#define Y_GOAL_B 10			// global Y-position of Team B's net [cm]

int which_goal;		// a variable to tell the bot which is his team

#define PI 3.14		// the number, pi

#define LD 0		// left motor direction, pin B
#define RD 1		// right motor direction, pin B
#define LDR 4		// left motor reverse-direction, pin B
#define RDR 5		// right motor reverse-direction, pin B
#define LE 6		// left motor enable, pin B
#define RE 6		// right motor enable, pin C
#define SOL 3		// solenoid firing, pin D
#define GOAL 7		// goal switch, pin D

#define CHANNEL 1				// channel for wireless communication
#define RXADDRESS 0x04			// address of wireless transmissions
#define TXADDRESS 0x04			// ^
#define PACKET_LENGTH 12		// length of wireless packets
#define BLOB_LENGTH 12			// length of Wii sensor packets

#define CLOCK 0		// clock prescaler

#define DEBUG_GOAL 0				// if you want to debug the goal switch, set this to 1
#define DEBUG_LOCALIZATION 0		// if you want to debug the bot's localization routine, set this to 1
#define DEBUG_PHOTO 0				// if you want to debug the phototransistor ADC inputs, set this to 1
#define WII 1						// if the bot is equipped with a Wii sensor, set this to 1

#define SEARCH 1			// robot states
#define PAUSE 2				// ^
#define APPROACH_PUCK 3		// ^
#define APPROACH_NET 4		// ^
#define SHOOT 5				// ^
#define CELEBRATE 6			// ^
#define COMM 7				// ^

#define COMM_TEST 0xA0			// wireless communication states
#define COMM_PLAY 0xA1			// ^
#define COMM_GOAL_A 0xA2		// ^
#define COMM_GOAL_B 0xA3		// ^
#define COMM_PAUSE 0xA4			// ^
#define COMM_HALFTIME 0xA6		// ^
#define COMM_GAME_OVER 0xA7		// ^

#define V_vert 29			// distance between the vertically-arranged stars [cm]
#define x_center 512		// x-direction center offset of the Wii sensor [pixels]
#define y_center 384		// y-direction center offset of the Wii sensor [pixels]

#define net_tol 30				// maximum shooting range [cm]
#define beta_tol 9				// maximum shooting angle [degrees]
#define photo_ltol 350			// low threshold for puck sensing, ADC value
#define photo_htol 1001			// high threshold for puck sensing, ADC value
#define puck_scaler_max 0.5		// maximum puck scaler value
#define net_scaler_max 0.3		// maximum net scaler value

#define speed 0.75		// maximum duty cycle of motors
#define time 100		// duration of solenoid fire [ms]

int XBOT;			// global bot x position [cm]
int YBOT;			// global bot y position [cm]
double theta;		// global bot orientation [rad]

int photo1;		// front left IR sensor
int photo2;		// front right IR sensor

unsigned int blobs[BLOB_LENGTH] = {0,0,0,0,0,0,0,0,0,0,0,0};		// Wii blob buffer
char buffer[PACKET_LENGTH] = {0,0,0,0,0,0,0,0,0,0,0,0};				// wifi buffer
volatile char pflag = 0;											// wifi flag

char state = 0;		// state set initially to default
int comm;			// communication signal from wifi packet	

char found_puck;		// a variable to denote whether the bot can see the puck
char got_puck;			// a variable to denote whether the bot possesses the puck
char close_enough;		// a variable to denote whether the bot is in shooting range of the opposing net


//_______________________________________ MAIN ROUTINE
int main(void)
{
	m_red(ON);			// initialization identifier
	m_green(ON);		// ^
	
	init();
	
	m_red(OFF);			// initialization identifier
	m_green(OFF);		// ^
		
	while(1){

		if(DEBUG_GOAL){				// if the user wishes to debug the goal switch,
			while(1){				// this loop will endlessly repeat the debug_goal routine
				m_wait(500);		// ^
				debug_goal();		// ^
			}						//
		}							//
		
		if(DEBUG_LOCALIZATION){				// if the user wishes to debug localization,
			while(1){						// this loop will endlessly repeat the debug_localization routine
				m_wait(500);				// ^
				debug_localization();		// ^
			}								//
		}									//
		
		if(DEBUG_PHOTO){			// if the user wishes to debug the IR sensors,
			while(1){				// this loop will endlessly repeat the debug_photo routine
				m_wait(500);		// ^
				debug_photo();		// ^
			}						//
		}							//

		m_wii_read(blobs);		// read the mWii
		track(blobs);			// define XBOT, YBOT, theta
		update_ADC();			// define photo1, photo2
		check_goal();			// define which_goal

		// WIRELESS COMMUNICATION
		if (pflag){								// if a wireless communication has been received...
			m_rf_read(buffer,PACKET_LENGTH);	// pull the packet
			comm = buffer[0];					// set the comm state
			
			switch (comm){			// WIRELESS COMMUNICATION STATES
				default:				// if signal received is none of the below...
					state = COMM;		// set the bot's state to COMM
					break;				//
				case COMM_TEST:				// if the Comm Test command is received...
					state = COMM;			// set the bot's state to COMM
					break;					//
				case COMM_PLAY:			// if the Play command is recieved...
					state = SEARCH;		// set the bot's state to SEARCH
					break;				//
				case COMM_GOAL_A:						// if the Goal A signal is received...
					if( which_goal == X_GOAL_A ){		// if the bot is on Team A...
						state = CELEBRATE;				// set the bot's state to CELEBRATE
					}									//
					else{								// if the bot is on Team B...
						state = PAUSE;					// set the bot's state to PAUSE
					}									//
					break;								//
				case COMM_GOAL_B:							// if the Goal B signal is received...
					if( which_goal == X_GOAL_B ){			// if the bot is on Team B...
						state = CELEBRATE;					// set the bot's state to CELEBRATE
					}										//
					else{									// if the bot is on Team A...
						state = PAUSE;						// set the bot's state to PAUSE
					}										//
					break;									//
				case COMM_PAUSE:		// if the Pause command is received...
					state = PAUSE;		// set the bot's state to PAUSE
					break;				//
				case COMM_HALFTIME:			// if the Halftime signal is received...
					state = PAUSE;			// set the bot's state to PAUSE
					break;					//
				case COMM_GAME_OVER:	// if the Game Over signal is received...
					state = PAUSE;		// set the bot's state to PAUSE
					break;				//
			}
			pflag=0;	// clear the wifi flag
		}			

		// STATE MACHINE
		switch (state){
			default:				// if state is none of the below...
				state = PAUSE;		// set the bot's state to PAUSE
				break;				//
			case SEARCH:						// SEARCH FOR THE PUCK
				found_puck = search();			// call the search routine
				if(found_puck){					// if the bot can see the puck...
					state = APPROACH_PUCK;		// tell the bot to approach the puck
				}								//
				break;							// if the bot cannot see the puck, repeat
			case PAUSE:			// STOP
				pause();		// call the pause routine
				break;			//
			case APPROACH_PUCK:					// APPROACH THE PUCK
				got_puck = approach_puck();		// call the approach_puck routine
				if(got_puck==1){				// if the bot has obtained the puck...
					state = APPROACH_NET;		// tell the bot to approach the net
				}								//
				else if(got_puck==2){			// if the bot has lost sight of the puck...
					state = SEARCH;				// tell the bot to search for the puck
				}								//
				break;							// if the bot can still see the puck but does not yet have it, repeat
			case APPROACH_NET:						// APPROACH THE NET
				close_enough = approach_net();		// call the approach_net routine
				if(close_enough==1){				// if the bot is in shooting range...
					state = SHOOT;					// tell the bot to shoot the puck
				}									//
				else if( close_enough == 2 ){		// if the bot has lost the puck...
					state = APPROACH_PUCK;			// tell the bot to approach the puck
				}									//
				break;								// if the bot is not yet in shooting range but still has the puck, repeat
			case SHOOT:				// SHOOT THE PUCK
				shoot();			// run the shoot routine
				state = SEARCH;		// tell the bot to search for the puck
				break;				//
			case CELEBRATE:				// CELEBRATE A GOAL
				celebrate();			// run the celebrate routine
				state = PAUSE;			// tell the bot to stop
				break;					//
			case COMM:				// COMMUNICATE
				comm_test();		// run the comm_test routine
				state = PAUSE;		// tell the bot to stop
				break;				//
		}				
	}
}


//_______________________________________ INITIALIZATION ROUTINE
void init(void){
	// initialize the entire system
	
	m_clockdivide(CLOCK);							// clock frequency = F_CPU / (2^CLOCK)
	m_bus_init();									// initialize mBUS
	init_timer();									// initialize timers
	init_ADC();										// initialize analog-to-digital conversion
	m_rf_open(CHANNEL,RXADDRESS,PACKET_LENGTH);		// open wireless communications
	sei();											// enable global interrupts
	
	if(WII){ while(!m_wii_open()); }		// initialize mWii
		
	if ( DEBUG_GOAL || DEBUG_LOCALIZATION || DEBUG_PHOTO ){		// if you are debugging...
		m_usb_init(); 											// connect USB
		while(!m_usb_isconnected()){};							// wait for connection
	}															//
	
	set(DDRB,LD);			// enable left motor direction line for output
	set(DDRB, LDR);			// enable left motor reverse-direction line for output
	set(DDRB,RD);			// enable right motor direction line for output
	set(DDRB, RDR);			// enable right motor reverse-direction line for output
	set(DDRD,SOL);			// enable solenoid-firing line for output
	clear(DDRD,GOAL);		// enable goal-switching line for input
	set(PORTD,GOAL);		// set internal pull-up resistor for goal switch
}	


//_______________________________________ TIMER INITIALIZATION ROUTINE
void init_timer(void){
	// initialize two different timers, one for each motor's PWM enable signal
	
	// TIMER 1
	clear(TCCR1B,CS12);		// timer prescaler = 1 --> timer frequency = clock frequency / 1
	clear(TCCR1B,CS11);		// ^
	set(TCCR1B,CS10);		// ^
	
	set(TCCR1B,WGM13);		// mode 15 --> PWM, count up to OCR1A
	set(TCCR1B,WGM12);		// ^
	set(TCCR1A,WGM11);		// ^
	set(TCCR1A,WGM10);		// ^
	
	set(TCCR1A,COM1B1);			// set at OCR1A, clear at OCR1B --> duty cycle = OCR1B / OCR1A
	clear(TCCR1A,COM1B0);		// ^
	
	set(DDRB,LE);		// enable B6 for left motor enable -- this is for PWM output
	
	OCR1A = 0x03FF;		// 10-bit to match timer 3 max
	OCR1B = 1;			// set initial duty cycle to 0
	
	// timer 3
	clear(TCCR3B,CS32);		// timer prescaler = 1 --> timer frequency = clock frequency / 1
	clear(TCCR3B,CS31);		// ^
	set(TCCR3B,CS30);		// ^
	
	clear(TCCR3B,WGM33);	// mode 7 --> PWM, count up to 0x03FF
	set(TCCR3B,WGM32);		// ^
	set(TCCR3A,WGM31);		// ^
	set(TCCR3A,WGM30);		// ^
	
	set(TCCR3A,COM3A1);			// set at 0x03FF, clear at OCR3A --> duty cycle = OCR3A / 0x03FF
	clear(TCCR3A,COM3A0);		// ^
	
	set(DDRC,RE);		// enable C6 for right motor enable -- this is for PWM output
	
	OCR3A = 1;			// set initial duty cycle to 0
}	


//_______________________________________ ADC INITIALIZATION ROUTINE
void init_ADC(void){
	// initialize the analog-to-digital conversion system
	
	clear(ADMUX,REFS1); 	// voltage reference = Vcc
	set(ADMUX,REFS0);		// ^

	set(ADCSRA,ADPS2);		// ADC prescaler = 128 --> ADC frequency = clock frequency / 128
	set(ADCSRA,ADPS1);		// ^
	set(ADCSRA,ADPS0);		// ^
	
	set(DIDR0,ADC0D);		// disable F0 digital input
	set(DIDR0,ADC1D);		// disable F1 digital input
	set(DIDR0,ADC4D);		// disable F4 digital input
	set(DIDR0,ADC5D);		// disable F5 digital input
}


//_______________________________________ LOCALIZATION ROUTINE
void track(unsigned int *blob){
	// use the mWii data to find the position and orientation of the bot relative to rink center
	
	// SUBROUTINE VARIABLES
	int x1; int x2; int x3; int x4; int y1; int y2; int y3; int y4;
	long v21; long v31; long v41; long v32; long v42; long v43;
	double v_vert;
	int x_vert1; int x_vert2; int y_vert1; int y_vert2;
	double scale;
	double xO; double yO;
	int x_other1; int x_other2;
	int x_actually2; int x_actually3; int y_actually2; int y_actually3;
	double alpha; double phi; double r;
	
	x1 = blob[0] - x_center;		// pull variables from IR blobs, shift using center offset
	y1 = blob[1] - y_center;		// ^
	x2 = blob[3] - x_center;		// ^
	y2 = blob[4] - y_center;		// ^
	x3 = blob[6] - x_center;		// ^
	y3 = blob[7] - y_center;		// ^
	x4 = blob[9] - x_center;		// ^
	y4 = blob[10] - y_center;		// ^
	
	v21 = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);		// these are actually the squares of the vector magnitudes
	v31 = (x3-x1)*(x3-x1) + (y3-y1)*(y3-y1);		// ^
	v41 = (x4-x1)*(x4-x1) + (y4-y1)*(y4-y1);		// ^
	v32 = (x3-x2)*(x3-x2) + (y3-y2)*(y3-y2);		// ^
	v42 = (x4-x2)*(x4-x2) + (y4-y2)*(y4-y2);		// ^
	v43 = (x4-x3)*(x4-x3) + (y4-y3)*(y4-y3);		// ^
	
	// determine the longest inter-blob vector (this corresponds to the IR blob vertical)
	if( v21>v31 && v21>v41 && v21>v32 && v21>v42 && v21>v43 ){ v_vert = v21; x_vert1 = x1; x_vert2 = x2; y_vert1 = y1; y_vert2 = y2; }
	else if( v31>v21 && v31>v41 && v31>v32 && v31>v42 && v31>v43 ){ v_vert = v31; x_vert1 = x1; x_vert2 = x3; y_vert1 = y1; y_vert2 = y3; }
	else if( v41>v21 && v41>v31 && v41>v32 && v41>v42 && v41>v43 ){ v_vert = v41; x_vert1 = x1; x_vert2 = x4; y_vert1 = y1; y_vert2 = y4; }
	else if( v32>v21 && v32>v31 && v32>v41 && v32>v42 && v32>v43 ){ v_vert = v32; x_vert1 = x2; x_vert2 = x3; y_vert1 = y2; y_vert2 = y3; }
	else if( v42>v21 && v42>v31 && v42>v41 && v42>v32 && v42>v43 ){ v_vert = v42; x_vert1 = x2; x_vert2 = x4; y_vert1 = y2; y_vert2 = y4; }
	else if( v43>v21 && v43>v31 && v43>v41 && v43>v32 && v43>v42 ){ v_vert = v43; x_vert1 = x3; x_vert2 = x4; y_vert1 = y3; y_vert2 = y4; }
	else{ v_vert = 1; x_vert1 = 0; x_vert2 = 0; y_vert1 = 0; y_vert2 = 0; }
	v_vert = sqrt(v_vert);
	
	scale = V_vert/v_vert;			// convert all variables from [pixels] to [cm]
	x1 = scale*x1;					// ^
	x2 = scale*x2;					// ^
	x3 = scale*x3;					// ^
	x4 = scale*x4;					// ^
	y1 = scale*y1;					// ^
	y2 = scale*y2;					// ^
	y3 = scale*y3;					// ^
	y4 = scale*y4;					// ^
	x_vert1 = scale*x_vert1;		// ^
	x_vert2 = scale*x_vert2;		// ^
	y_vert1 = scale*y_vert1;		// ^
	y_vert2 = scale*y_vert2;		// ^
	
	xO = (x_vert1 + x_vert2) / 2.0;		// define the local position of the origin
	yO = (y_vert1 + y_vert2) / 2.0;		// ^
	
	// determine the shortest inter-blob vector (this corresponds to V31)
	if( v21<v31 && v21<v41 && v21<v32 && v21<v42 && v21<v43 ){ x_other1 = x1; x_other2 = x2; }
	else if( v31<v21 && v31<v41 && v31<v32 && v31<v42 && v31<v43 ){ x_other1 = x1; x_other2 = x3; }
	else if( v41<v21 && v41<v31 && v41<v32 && v41<v42 && v41<v43 ){ x_other1 = x1; x_other2 = x4; }
	else if( v32<v21 && v32<v31 && v32<v41 && v32<v42 && v32<v43 ){ x_other1 = x2; x_other2 = x3; }
	else if( v42<v21 && v42<v31 && v42<v41 && v42<v32 && v42<v43 ){ x_other1 = x2; x_other2 = x4; }
	else if( v43<v21 && v43<v31 && v43<v41 && v43<v32 && v43<v42 ){ x_other1 = x3; x_other2 = x4; }
	else{ x_other1 = 0; x_other2 = 0; }
	
	// determine the blobs 1, 2, and 3
	if( x_vert1 == x_other1 ){ x_actually2 = x_vert2; x_actually3 = x_vert1; y_actually2 = y_vert2; y_actually3 = y_vert1; }
	else if( x_vert2 == x_other1 ){ x_actually2 = x_vert1; x_actually3 = x_vert2; y_actually2 = y_vert1; y_actually3 = y_vert2; }
	else if( x_vert1 == x_other2 ){ x_actually2 = x_vert2; x_actually3 = x_vert1; y_actually2 = y_vert2; y_actually3 = y_vert1; }
	else if( x_vert2 == x_other2 ){ x_actually2 = x_vert1; x_actually3 = x_vert2; y_actually2 = y_vert1; y_actually3 = y_vert2; }
	else{ x_actually2 = 0; x_actually3 = 0; y_actually2 = 0; y_actually3 = 0; }

	// find the angle between the local frame and the global frame
	theta = atan2( (x_actually3 - x_actually2), (y_actually3 - y_actually2) );

	// find the magnitude of the vector to the origin
	r = xO*xO + yO*yO;
	r = sqrt(r);
	
	// find the angle between the local x-axis and the vector to the origin
	alpha = -atan2(yO,xO);
	
	// find the angle between the global x-axis and the vector to the origin
	phi = theta - alpha;
	
	XBOT = -r*cos(phi);		// determine the bot's global position
	YBOT = -r*sin(phi);		// ^
}	


//_______________________________________ SEARCH ROUTINE
char search(void){
	// use the input data from the phototransistors to generally orient the bot toward the puck
	
	m_green(OFF);		// visual cues
	m_red(OFF);			// ^

	if( photo1 > photo_ltol || photo2 > photo_ltol ){		// if the bot can see the puck...
		return 1;											// return 1
	}														//
	else{							// if the bot cannot see the puck...
		set(PORTB,LD);				// turn clockwise
		clear(PORTB, LDR);			// ^
									//
		clear(PORTB,RD);			// ^
		set(PORTB, RDR);			// ^

		OCR1B = OCR1A*speed;		// speed = max
		OCR3A = OCR1B;				// ^
		return 0;					// return 0
	}
}


//_______________________________________ PAUSE ROUTINE
void pause(void){
	// run the motors at minimum duty cycle
	
	m_green(ON);		// visual cues
	m_red(ON);			// ^
	
	OCR1B = 1;			// speed = 0
	OCR3A = OCR1B;		// ^
}


//_______________________________________ PUCK APPROACH ROUTINE
char approach_puck(void){
	// use the input data from the phototransistors to drive toward and obtain the puck

	m_green(OFF);		// visual cues
	m_red(ON);			// ^

	if( photo1 < photo_ltol && photo2 < photo_ltol ){		// if the bot has lost sight of the puck...
		return 2;											// return 2
	}														//
	else{													// if the bot still sees the puck...
		
		if( photo1 < photo_htol || photo2 < photo_htol ){		// if the bot does not yet have the puck...
			double puck_scaler;									// define a variable to drive the wheels at different speeds

			if( photo1 > photo2 ){							// if the puck is in front and to the left of the bot...
				puck_scaler = photo1/(photo2+1)/5.0;		// ~1/5 < puck_scaler < ~5
				if( puck_scaler > puck_scaler_max ){		// if the puck scaler is too big...
					puck_scaler = puck_scaler_max;			// set it at its maximum
				}											//
				
				set(PORTB,LD);			// drive forward
				clear(PORTB, LDR);		// ^
										//
				set(PORTB,RD);			// ^
				clear(PORTB, RDR);		// ^
		
				OCR1B = OCR1A*(speed-puck_scaler);		// bank left
				OCR3A = OCR1A*speed;					// ^
			}											//
			else{											// if the puck is in front and to the right of the bot...
				puck_scaler = photo2/(photo1+1)/5.0;		// ~1/5 < puck_scaler < ~5
				if( puck_scaler > puck_scaler_max ){		// if the puck scaler is too big...
					puck_scaler = puck_scaler_max;			// set it at its maximum
				}											//
				
				set(PORTB,LD);			// drive forward
				clear(PORTB, LDR);		// ^
										// ^
				set(PORTB,RD);			// ^
				clear(PORTB, RDR);		// ^

				OCR1B = OCR1A*speed;					// bank right
				OCR3A = OCR1A*(speed-puck_scaler);		// ^
			}											//
			return 0;		// return 0
		}			
		else{				// if the bot has obtained the puck...
			return 1;		// return 1
		}
	}
}


//_______________________________________ NET APPROACH ROUTINE
char approach_net(void){
	// use the input data from the mWii to drive toward the opposing goal		
	
	m_red(OFF);			// visual cues
	m_green(ON);		// ^

	if( photo1 < photo_htol && photo2 < photo_htol ){		// if the bot has lost possession of the puck...
		return 2;											// return 2
	}														//
	else{										// if the bot still has possession of the puck...
		// SUBROUTINE VARIABLES
		double net_dist;
		double Xnet; double Ynet;
		double Xnet_bot; double Ynet_bot;
		double gamma; double beta;
		double net_scaler;
	
		if( which_goal == X_GOAL_A ){					// if the bot is on Team A...
			Xnet = X_GOAL_B;	Ynet = Y_GOAL_B;		// set the coordinates of the opposing net to those of Goal B
		}												//
		else{												// if the bot is on Team B...
			Xnet = X_GOAL_A;	Ynet = Y_GOAL_A;			// set the coordinates of the opposing net to those of Goal A
		}													//

		Xnet_bot = Xnet - XBOT;		// vector from bot to net
		Ynet_bot = Ynet - YBOT;		// ^
	
		net_dist = Xnet_bot*Xnet_bot + Ynet_bot*Ynet_bot;		// distance to net
		net_dist = sqrt(net_dist);								// ^
	
		gamma = atan2( Ynet_bot, Xnet_bot );		// angle between global X and net
		beta = theta - gamma;						// angle between orientation and vector to net

		beta = beta*180.0/PI;		// convert from [rad] to [degrees]
		if( beta > 180 ){			// 0 < beta < 180
			beta = beta - 360;		// ^
		}							//
		else if( beta < -180 ){		// ^
			beta = beta + 360;		// ^
		}							//
		
		net_scaler = fabs(beta)/180.0;			// 0 < net_scaler < 1.0
		if( net_scaler > net_scaler_max ){		// if net scaler is too big...
			net_scaler = net_scaler_max;		// set it at its maximum
		}										//
		
		if( net_dist > net_tol ){		// if the bot is not within shooting range...
			if( beta > 0 ){				// if there is a positive angle between the net and the bot...
				set(PORTB,LD);			// drive forward
				clear(PORTB, LDR);		// ^
				set(PORTB,RD);			// ^
				clear(PORTB, RDR);		// ^
				
				OCR1B = OCR1A*(speed-net_scaler);		// bank right
				OCR3A = OCR1A*speed;					// ^
				
				return 0;		// return 0
			}					//
			else{						// if there is a negative angle between the net and the bot...
				set(PORTB,LD);			// drive forward
				clear(PORTB, LDR);		// ^
				set(PORTB,RD);			// ^
				clear(PORTB, RDR);		// ^	
				
				OCR1B = OCR1A*speed;					// bank left
				OCR3A = OCR1A*(speed-net_scaler);		// ^
				
				return 0;		// return 0
			}
		}			
		else{	// if the bot is within shooting range...
			if( beta > beta_tol ){		// if the positive angle between the net and bot is too great...
				set(PORTB,LD);			// spin CW
				clear(PORTB, LDR);		// ^
				clear(PORTB,RD);		// ^
				set(PORTB, RDR);		// ^
				
				OCR1B = OCR1A*speed;		// speed = max
				OCR3A = OCR1A*speed;		// ^
				
				return 0;		// return 0
			}					//
			else if( beta < -beta_tol ){		// if the negative angle between the net and bot is too great...
				clear(PORTB,LD);				// spin CCW
				set(PORTB, LDR);				// ^
				set(PORTB,RD);					// ^
				clear(PORTB, RDR);				// ^	
				
				OCR1B = OCR1A*speed;		// speed = max
				OCR3A = OCR1A*speed;		// ^
				
				return 0;		// return 0
			}					//
			else{					// if the bot is oriented correctly...
				return 1;			// return 1
			}
		}			
	}		
}


//_______________________________________ SHOOT ROUTINE
void shoot(void){
	// fire the solenoid and kick the puck into the goal
	
	set(PORTD,SOL);		// turn on the signal to the solenoid
	m_wait(time);		// apply power for specified duration
	clear(PORTD,SOL);	// turn off the signal to the solenoid
}


//_______________________________________ CELEBRATE ROUTINE
void celebrate(void){
	// spin around and flash the red on-board LED for 5 seconds
	
	set(PORTB,LD);			// spin CW
	clear(PORTB, LDR);		// ^
	clear(PORTB,RD);		// ^
	set(PORTB, RDR);		// ^
	
	OCR1B = OCR1A*speed;	// speed = max
	OCR3A = OCR1B;			// ^
	
	int i;					// flash both LEDs alternatively at a frequency of 5 Hz
	for(i=0;i<25;i++){		// ^
		m_green(ON);		// ^
		m_red(OFF);			// ^
		m_wait(100);		// ^
		m_green(OFF);		// ^
		m_red(ON);			// ^
		m_wait(100);		// ^
	}						//
	m_red(OFF);				// ensure that both LEDs are turned off
	m_green(OFF);			// ^
}


//_______________________________________ COMMUNICATION ROUTINE
void comm_test(void){
	// flash the red on-board LED for 5 seconds

	int i;					// flash both LEDs simultaneously at a frequency of 5 Hz
	for(i=0;i<5;i++){		// ^
		m_green(ON);		// ^
		m_red(ON);			// ^
		m_wait(100);		// ^
		m_red(OFF);			// ^
		m_green(OFF);		// ^
		m_wait(100);		// ^
	}
}							


//_______________________________________ ADC ROUTINE
void update_ADC(void){ 		
	// pull current ADC values from two sources

	clear(ADCSRB, MUX5);		// set pin to F0
	clear(ADMUX, MUX2);			// ^
	clear(ADMUX, MUX1);			// ^
	clear(ADMUX, MUX0);			// ^
	
	set(ADCSRA, ADEN);		// start conversion process
	set(ADCSRA, ADSC);		// ^
	
	while(!check(ADCSRA,ADIF));		// wait to finish
	photo1 = (1023-ADC);			// pull value
	set(ADCSRA, ADIF);				// clear flag after conversion


	clear(ADCSRB, MUX5);		// set pin to F1
	clear(ADMUX, MUX2);			// ^
	clear(ADMUX, MUX1);			// ^
	set(ADMUX, MUX0);			// ^

	set(ADCSRA, ADEN);		// start conversion process
	set(ADCSRA, ADSC);		// ^
	
	while(!check(ADCSRA,ADIF));		// wait to finish
	photo2 = (1023-ADC);			// pull value
	set(ADCSRA, ADIF);				// clear flag after conversion
}


//_______________________________________ GOAL SWITCH CHECK ROUTINE
void check_goal(void){
	// pull current goal value
	
	if( check(PIND,GOAL) ){			// if the switching pin is high...
		which_goal = X_GOAL_A;		// the bot is on Team A
	}								//
	else{							// if the switching pin is low...
		which_goal = X_GOAL_B;		// the bot is on Team B
	}
}


//_______________________________________ DEBUG ROUTINE FOR GOAL SWITCH
void debug_goal(void){
	// read the value from the goal switch and print it to the computer's terminal using USB communications
	
	check_goal();							// define which_goal
	m_usb_tx_string("which goal: ");		// print to the screen "which goal: "
	m_usb_tx_int(which_goal);				// print to the screen the value of which_goal in integer form
	m_usb_tx_string("\n");					// print to the screen a new line
}


//_______________________________________ DEBUG ROUTINE FOR LOCALIZATION
void debug_localization(void){
	// read the values from the localization routine and print them to the computer's terminal using USB communications
	
	m_wii_read(blobs);					// read the mWii
	track(blobs);						// define XBOT, YBOT, theta
	m_usb_tx_string("XBOT: ");			// print to the screen "XBOT: "
	m_usb_tx_int(XBOT);					// print to the screen the value of XBOT in integer form
	m_usb_tx_string("\n YBOT: ");		// print to the screen a new line, followed by "YBOT: "
	m_usb_tx_int(YBOT);					// print to the screen the value of YBOT in integer form
	m_usb_tx_string("\n theta: ");		// print to the screen a new line, followed by "theta: "
	m_usb_tx_int(theta);				// print to the screen the value of theta in integer form
	m_usb_tx_string("\n");				// print to the screen a new line
}


//_______________________________________ DEBUG ROUTINE FOR IR SENSORS
void debug_photo(void){
	// read the values from the ADC conversions and print them to the computer's terminal using USB communications
	
	update_ADC();						// define photo1, photo2
	m_usb_tx_string("photo1: ");		// print to the screen "photo1: "
	m_usb_tx_int(photo1);				// print to the screen the value of photo1 in integer form
	m_usb_tx_string("\n photo2: ");		// print to the screen a new line, followed by "photo2: "
	m_usb_tx_int(photo2);				// print to the screen the value of photo2 in integer form
	m_usb_tx_string("\n");				// print to the screen a new line
}


//_______________________________________ WIFI INTERRUPT
ISR(INT2_vect){
	// upon receipt of a wifi packet, execute this routine
	
	pflag=1;		// set the wifi flag
}
