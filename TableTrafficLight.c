/*******************************************************************
 * TableTrafficLight.c
 * Instructor: Devinder Kaur
 * Runs on LM4F120/TM4C123
 * Index implementation of a Moore finite state machine to operate a traffic light.  
 * Authors: Daniel Valvano,
 *					Jonathan Valvano,
 * 					Thomas Royko
 * Student: Quinn Kleinfelter
 * Section: 001 / 003
 * Date:    11/03/2020
 *
 * east/west red light connected to PB5
 * east/west yellow light connected to PB4
 * east/west green light connected to PB3
 * north/south facing red light connected to PB2
 * north/south facing yellow light connected to PB1
 * north/south facing green light connected to PB0
 * pedestrian detector connected to PE2 (1=pedestrian present)
 * north/south car detector connected to PE1 (1=car present)
 * east/west car detector connected to PE0 (1=car present)
 * "walk" light connected to PF3 (built-in green LED)
 * "don't walk" light connected to PF1 (built-in red LED)
 *******************************************************************/

#include "TExaS.h"
#include "inc\tm4c123gh6pm.h"
#include "SysTick.h"

// Label inputs and outputs
#define LIGHT                   (*((volatile uint32_t *)0x400050FC))
#define SENSOR                  (*((volatile uint32_t *)0x4002401C))
#define GPIO_PORTF_OUT          (*((volatile uint32_t *)0x40025028))
	
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// Define digital inputs and outputs
#define GPIO_PORTB_DIR_R        (*((volatile uint32_t *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile uint32_t *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile uint32_t *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile uint32_t *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile uint32_t *)0x4000552C))
#define GPIO_PORTE_DIR_R        (*((volatile uint32_t *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile uint32_t *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile uint32_t *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile uint32_t *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile uint32_t *)0x4002452C))
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08))
#define SYSCTL_RCGC2_R          (*((volatile uint32_t *)0x400FE108))
#define GPIO_PORTF_AFSEL_R      (*((volatile uint32_t *)0x40025420))
#define GPIO_PORTF_DIR_R        (*((volatile uint32_t *)0x40025400))
#define GPIO_PORTF_DEN_R        (*((volatile uint32_t *)0x4002551C))
#define GPIO_PORTF_PCTL_R       (*((volatile uint32_t *)0x4002552C))
#define GPIO_PORTF_AMSEL_R      (*((volatile uint32_t *)0x40025528))

struct State {
	uint32_t Out; // 6-bit output
	uint32_t Time; // 10ms
	uint8_t Next[8]; // depends on 3-bit input
};
typedef const struct State STyp;

#define goS 0
#define waitS 1
#define goW 2
#define waitW 3
#define stopWaitS 4
#define stopWaitW 5
#define stopSW 6
#define stopBlink1SW 7
#define stopBlink2SW 8
#define stopBlink3SW 9
#define stopBlink4SW 10
#define stopSolidSW 11

// Port I/O
// PE2-0 = inputs (south, west, walk respectively)
// PB5-0 = traffic light outputs
// PF3 & PF1 = walking LEDs

// Define FSM as global
// The walk sensor will be PE2
// The last state added will run for 10 seconds (how long pedestrians have to cross), both lights are red in this case
// Walk light will stay on for 2 seconds, then don't walk light will blink for 2 seconds, and then don't walk light will
// Stay solid for 2 seconds

// State Machine Output:
// Bit 7 - PF3 - green LED, walk light
// Bit 6 - PF1 - red LED, don't walk light
// Bit 5 - PB5 - South Red
// Bit 4 - PB4 - South Yellow
// Bit 3 - PB3 - South Green
// Bit 2 - PB2 - West Red
// Bit 1 - PB1 - West Yellow
// Bit 0 - PB0 - West Green

STyp FSM[12]={
 {0x61, 10, {goS, waitS, goS, waitS, stopWaitS, stopWaitS, stopWaitS, stopWaitS}}, 																						// goS
 {0x62, 10, {goW, goW, goW, goW, stopSW, stopSW, stopSW, stopSW}},																														// waitS
 {0x4C, 10, {goW, goW, waitW, waitW, stopWaitW, stopWaitW, stopWaitW, stopWaitW}},																						// goW
 {0x54, 10, {goS, goS, goS, goS, stopSW, stopSW, stopSW, stopSW}},																														// waitW
 {0x62, 10, {stopSW, stopSW, stopSW, stopSW, stopSW, stopSW, stopSW, stopSW}},																								// stopWaitS
 {0x54, 10, {stopSW, stopSW, stopSW, stopSW, stopSW, stopSW, stopSW, stopSW}},																								// stopWaitW
 {0xA4, 10, {stopBlink1SW, stopBlink1SW, stopBlink1SW, stopBlink1SW, stopBlink1SW, stopBlink1SW, stopBlink1SW, stopBlink1SW}},// stopSW
 {0x64, 10, {stopBlink2SW, stopBlink2SW, stopBlink2SW, stopBlink2SW, stopBlink2SW, stopBlink2SW, stopBlink2SW, stopBlink2SW}},// stopBlink1SW
 {0x24, 10, {stopBlink3SW, stopBlink3SW, stopBlink3SW, stopBlink3SW, stopBlink3SW, stopBlink3SW, stopBlink3SW, stopBlink3SW}},// stopBlink2SW
 {0x64, 10, {stopBlink4SW, stopBlink4SW, stopBlink4SW, stopBlink4SW, stopBlink4SW, stopBlink4SW, stopBlink4SW, stopBlink4SW}},// stopBlink3SW
 {0x24, 10, {stopSolidSW, stopSolidSW, stopSolidSW, stopSolidSW, stopSolidSW, stopSolidSW, stopSolidSW, stopSolidSW}},				// stopBlink4SW
 {0x64, 10, {goS, goW, goS, goW, goW, goW, goS, goS}}																																				  // stopSolidSW
};
	

void Init_PortsEBF(void) {
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x32; // Activate clocks for Port E, F, and B
	// allow time for clock to stabilize
	delay = SYSCTL_RCGC2_R;
	
	// Port B
	GPIO_PORTB_DIR_R |= 0x3F; // make PB5-0 out
	GPIO_PORTB_AFSEL_R &= ~0x3F; // Disable alternate functions
	GPIO_PORTB_DEN_R |= 0x3F; // enable digital i/o
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFF000000) + 0x0000000;
	GPIO_PORTB_AMSEL_R &= ~0x3F; // disable analog functionality
	
	// Port E
	GPIO_PORTE_DIR_R &= ~0x07; // Make PE2-0 inputs
	GPIO_PORTE_AFSEL_R &= ~0x07; // disable alternate functions
	GPIO_PORTE_DEN_R |= 0x07; // Enable digital i/o
	GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R & 0xFFFF0000) + 0x00000000;
	GPIO_PORTE_AMSEL_R &= ~0x07; // diable analog functionality
	
	// Port F
	GPIO_PORTF_DIR_R |= 0x0A; // make PF3 and PF1 outputs
	GPIO_PORTF_AFSEL_R &= ~0x0A; // disable alternate functions
	GPIO_PORTF_DEN_R |= 0x0A; // Enable digital i/o
	GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R & 0xFFFF0F0F) + 0x00000000;
	GPIO_PORTF_AMSEL_R &= ~0x0A; // disable analog functionality
}

int main(void){
	uint8_t n; // state number
	uint32_t Input;
	
	// activate grader and set system clock to 80 MHz
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff);
	
	SysTick_Init();
	Init_PortsEBF();
	
	// Establish initial state of traffic lights
	n = goS; // Initial state: Green South, Red West
	GPIO_PORTF_OUT = 0x02; // turn on dont walk light
	
	EnableInterrupts();
  
  while(1){
    LIGHT = FSM[n].Out; // Set traffic lights to current state's Out Value
		// Set the walk and don't walk lights based on the current states output
		// we only need the left 2 bits for it
		// first we shift right 6 so that the 2 LSB are PF3 and PF1
		// then shift it left one bit so that PF1 is in its correct spot
		// finally, take the original output shift it 7 to the right, and back to the left
		// 3 bits to finalize PF3
		GPIO_PORTF_OUT = ((FSM[n].Out >> 7) << 3) | ((FSM[n].Out >> 6) << 1);
		
		SysTick_Wait10ms(FSM[n].Time);
		Input = SENSOR;
		
		n = FSM[n].Next[Input];
  }
}

