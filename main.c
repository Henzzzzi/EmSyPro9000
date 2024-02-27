// EmSyPro9000

#include <xscugic.h>
#include <xttcps.h>
#include <xgpio.h>
#include <stdio.h>
#include <stdlib.h>

#include "zynq_registers.h"
#include "uart_communication.h"
#include "interrupts.h"
#include "pi_controller.h"
#include "system.h"


#define BUTTONS_channel 	2
#define BUTTONS_AXI_ID 		XPAR_AXI_GPIO_SW_BTN_DEVICE_ID
#define SWITCHES_channel	1
#define SWITCHES_AXI_ID		XPAR_AXI_GPIO_SW_BTN_DEVICE_ID
#define LEDS_channel		1
#define LEDS_AXI_ID			XPAR_AXI_GPIO_LED_DEVICE_ID

#define LD0					0x1
#define LD1					0x2
#define LD2					0x4
#define LD3					0x8


XScuGic GIC; 					// Interrupt controller instance for Interrupt controller driver
XScuGic *pGIC = &GIC; 			// Ptr for above
XTtcPs TTC1;					// Timer instance
XTtcPs *pTTC1 = &TTC1;			// Timer instance pointer
XGpio BTNS_SWTS;				// Buttons and switches instance
XGpio *pBTNS_SWTS = &BTNS_SWTS;	// Buttons and switches instance pointer
XGpio LEDS; 					// LED instance
XGpio *pLEDS = &LEDS;			// LED instance pointer
u8 buttons = 0;					// Variable to save button state when using discreteread in XGpio driver

// state variable and descriptions for the main state machine
volatile uint8_t state = 0;
const char statedesc [3][20] = {"\nConfiguration mode", "\nIdling",  "\nModulating"};

volatile uint8_t timerFlag = FALSE;

// Kp, Ki flags
uint8_t Kp_flag = TRUE;
uint8_t Ki_flag = FALSE;

// Flags for button presses
volatile uint8_t btn0Flag = FALSE;
volatile uint8_t btn1Flag = FALSE;
volatile uint8_t btn2Flag = FALSE;
volatile uint8_t btn3Flag = FALSE;
uint8_t* pbtn0Flag = &btn0Flag;
uint8_t* pbtn1Flag = &btn1Flag;
uint8_t* pbtn2Flag = &btn2Flag;
uint8_t* pbtn3Flag = &btn3Flag;

// Variables / flags for data protection
volatile uint8_t critical_section_flag = FALSE;
uint8_t lock = 0;

int acquire_lock(){
	if (__sync_lock_test_and_set(&lock,1) == 1){
		//printf("Vituix män!\n");
		return 0;
	}
	else {
		//printf("Lukittud!\n");
		return 1;
	}
}

void release_lock(){
    __sync_lock_release(&lock);
    //printf("Vapautettu!\n");
}


// The main program
int main(void) {
	// Flag for starting / stopping modulation when changing state (to do it only once when mode is changed)
	uint8_t timerInterruptsEnabled = 0;

	// Controller parameters and reference value
	double Ki = 132;
	double Kp = 0.00132;
	double Ref = 500;

	// System values
	double PIOut = 0;
	double outputVoltage = 0;

	// Data storage variable for re-entrant PI-controller
	double ui_prev = 0;
	double* pui_prev = &ui_prev;

	// Variables for parsing serial data
	char input;
	char serial_buffer[16];
	uint8_t serial_buffer_index = 0;

	// Initialize UART
	initialize_uart();
	// Send system state
	uart_send_string("\nEmSyPro9000 Started!\n");
	uart_send_string(statedesc[state]);
	uart_send_config(Ref, Ki, Kp);

	// Initialize GPIO, GIC and TTC1
	uint8_t Status;
	Status = InitGPIO(pBTNS_SWTS, pLEDS);
	if (Status != XST_SUCCESS) uart_send_string("GPIO setup error\n");
	Status = InitGic(pGIC);
	if (Status == XST_FAILURE) uart_send_string("Interrupt Controller Initialize error\n");
	Status = InitTTC1(pTTC1);
	if (Status != XST_SUCCESS) uart_send_string("Timer setup error\n");

	// Setup and Enable all interrupts
	InitInterrupts(pGIC, pBTNS_SWTS);

	// Start TTC1
	XTtcPs_Start(pTTC1);

	// Initialize timer TTC0 for LED PWM
	// Set prescaler to 1 and enable it
	TTC0_CLK_CNTRL2 = (0 << XTTCPS_CLK_CNTRL_PS_VAL_SHIFT) | XTTCPS_CLK_CNTRL_PS_EN_MASK;
	// Disable counter, Set counter to match-mode and reset counter, set waveform polarity
	TTC0_CNT_CNTRL2 = XTTCPS_CNT_CNTRL_RST_MASK | XTTCPS_CNT_CNTRL_DIS_MASK | XTTCPS_CNT_CNTRL_MATCH_MASK | XTTCPS_CNT_CNTRL_POL_WAVE_MASK;
	// Disable waveform
	TTC0_CNT_CNTRL2 |= XTTCPS_CNT_CNTRL_EN_WAVE_MASK;


	// Main loop
	while(1){
		// Main state machine (idling / config / modulating)
		switch (state){
			case 0:		// Configuration mode
				// Set LEDs
				AXI_LED_DATA = 0x1;

				// If mode was changed to configuration - stop modulating
				if (timerInterruptsEnabled == 1){
					timerInterruptsEnabled = 0;
					// Reset system and PI controller
					PIOut = 0;
					ui_prev = 0;
					outputVoltage = 0;
					// Reset previous values stored in converter-models variables
					converter(-1);
					//Disable TTC1 interrupts
					TTC1_IER &= ~XTTCPS_IXR_INTERVAL_MASK;
					XTtcPs_ResetCounterValue(pTTC1);
					// Stop PWN counter
					TTC0_CNT_CNTRL2 |= XTTCPS_CNT_CNTRL_DIS_MASK;
					// Disable waveform
					TTC0_CNT_CNTRL2 |= XTTCPS_CNT_CNTRL_EN_WAVE_MASK;
					// Set counter match value to 0
					TTC0_MATCH_1_COUNTER_2 = 0;
				}
				// Mode change requested by buttons
				// If acquire_lock() == TRUE we succesfully acquired the lock if it failed -- do nothing and retry on the next main loop cycle
				if (btn0Flag && acquire_lock()){
					btn0Flag = FALSE;
					release_lock();
					state = 1;
					uart_send_string(statedesc[state]);
				}
				// Which parameter we want to modify?
				if (btn1Flag && acquire_lock()){
					btn1Flag = FALSE;
					release_lock();
					if (Kp_flag) {
						Kp_flag = FALSE;
						Ki_flag = TRUE;
						uart_send_string("\nModifying Ki value\n");

					} else if (Ki_flag) {
						Ki_flag = FALSE;
						Kp_flag = TRUE;
						uart_send_string("\nModifying Kp value\n");

					}
				}
				// Modify Ki/Kp
				if (btn2Flag && acquire_lock()){
					btn2Flag = FALSE;
					release_lock();
					if (Kp_flag){
						if (Kp - 0.0001 >= 0){
							Kp = (float) Kp - 0.0001;
						}
					}
					else if (Ki_flag){
						if (Ki - 1 > 0){
							Ki -= 1;
						}
					}
					uart_send_config(Ref, Ki, Kp);
				}
				if (btn3Flag && acquire_lock()){
					btn3Flag = FALSE;
					release_lock();
					if (Kp_flag){
						Kp = (float) Kp + 0.0001;
					}
					else if (Ki_flag){
						Ki += 1;
					}
					uart_send_config(Ref, Ki, Kp);
				}
				break;

			case 1:		// Idling ...
				// Set LEDs
				AXI_LED_DATA = 0x2;
				// If mode is changed to idling - stop modulating
				if (timerInterruptsEnabled == 1){
					timerInterruptsEnabled = 0;
					// Reset system and PI controller
					PIOut = 0;
					ui_prev = 0;
					outputVoltage = 0;
					// Reset values stored in converter-models static variables
					converter(-1);
					//Disable TTC1 interrupts
					TTC1_IER &= ~XTTCPS_IXR_INTERVAL_MASK;
					XTtcPs_ResetCounterValue(pTTC1);
					// Stop PWN counter
					TTC0_CNT_CNTRL2 |= XTTCPS_CNT_CNTRL_DIS_MASK;
					// Disable waveform
					TTC0_CNT_CNTRL2 |= XTTCPS_CNT_CNTRL_EN_WAVE_MASK;
					// Set counter match value to 0
					TTC0_MATCH_1_COUNTER_2 = 0;
				}
				// Mode change requested by buttons
				if (btn0Flag && acquire_lock()){
					btn0Flag = FALSE;
					release_lock();
					state = 2;
					uart_send_string(statedesc[state]);
				}
				break;

			case 2:		// Modulating
				// Set LEDs
				AXI_LED_DATA = 0x4;
				// Start modulating
				if (timerInterruptsEnabled == 0){
					timerInterruptsEnabled = 1;
					// Enable TTC1 interrupts
					TTC1_IER |= XTTCPS_IXR_INTERVAL_MASK;
					// Start PWM counter
					TTC0_CNT_CNTRL2 &= ~XTTCPS_CNT_CNTRL_DIS_MASK;
					// Enable waveform
					TTC0_CNT_CNTRL2 &= ~XTTCPS_CNT_CNTRL_EN_WAVE_MASK;
				}
				// Update PI output and system output values if flag is set (from interrupt)
				if (timerFlag && acquire_lock()){
					timerFlag = FALSE;
					release_lock();

					//printf("pre PIout: %f  Voltage: %f\n", PIOut, outputVoltage);
					PIOut = PI(Ref, outputVoltage, pui_prev, Ki, Kp);
					outputVoltage = converter(PIOut);
					//printf("PIout: %f  Voltage: %f\n", PIOut, outputVoltage);

					// Set PWM duty cycle based on new outputVoltage
					// (Little bit of error from casting but not a big deal)
					TTC0_MATCH_1_COUNTER_2 = (int) outputVoltage*65;

				}
				// Mode change requested by buttons
				if (btn0Flag && acquire_lock()){
					btn0Flag = FALSE;
					release_lock();
					state = 0;
					uart_send_string(statedesc[state]);
					uart_send_config(Ref, Ki, Kp);
				}
				// Reference change requested by buttons
				if (btn2Flag && acquire_lock()){
					btn2Flag = FALSE;
					release_lock();
					Ref -= 1;

					uart_send_config(Ref, Ki, Kp);
				}
				if (btn3Flag && acquire_lock()){
					btn3Flag = FALSE;
					release_lock();
					Ref += 1;

					uart_send_config(Ref, Ki, Kp);
				}
				break;

			default:
				break;

		}

		// Receive data over UART
		input = uart_receive(); // polling UART receive buffer

		if (input != 0){
			// Depending on the serial terminal used, UART messages can be terminated
			// by either carriage return '\r' or line feed '\n'.
			if (input == '\r' || input == '\n'){

				// Disable button interrupts while doing mode / parameter changes via serial
				XGpio_InterruptDisable(pBTNS_SWTS, 0x3);
				XGpio_InterruptGlobalDisable(pBTNS_SWTS);

				// Add null termination to buffer for strcpm() to function properly
				serial_buffer[serial_buffer_index] = '\0';
				//uart_send_string(serial_buffer);

				char buffer [8];

				// Mode change requested?
				if (strcmp(serial_buffer, "setmode 0") == 0){
					state = 0;
					uart_send_string(statedesc[state]);
					uart_send_config(Ref, Ki, Kp);
				}
				else if  (strcmp(serial_buffer, "setmode 1") == 0){
					state = 1;
					uart_send_string(statedesc[state]);
				}
				else if  (strcmp(serial_buffer, "setmode 2") == 0){
					state = 2;
					uart_send_string(statedesc[state]);
				}
				else{
					// Parameter/reference change requested?
					switch (state){
						case 0:		// Configuration mode
							// Check for Kp
							memcpy(buffer, &serial_buffer, 2);
							buffer[2] = '\0';
							// If message startswith "kp" - copy the rest of the message and convert it to float
							if (strcmp(buffer, "kp") == 0){
								memcpy(buffer, &serial_buffer[2], 8);
								float newKp = (float)atof(buffer);
								// Check if received value is inside borders
								if (newKp < 1000 && newKp >= 0){
									Kp = newKp;
									uart_send_string("Changes applied!\n");
								}
								else{
									uart_send_string("Invalid value for Kp!\n");
								}
								uart_send_config(Ref, Ki, Kp);
								//return
							}
							// Check for Ki
							// If message startswith "ki" - copy the rest of the message and convert it to int
							else if (strcmp(buffer, "ki") == 0){
								memcpy(buffer, &serial_buffer[2], 8);
								float newKi = atof(buffer);
								// Check if received value is inside borders
								if (newKi < 1000 && newKi >= 0){
									Ki = newKi;
									uart_send_string("Changes applied!\n");
								}
								else{
									uart_send_string("Invalid value for Ki!\n");
								}
								uart_send_config(Ref, Ki, Kp);
							}
							break;

						case 1:		// Idling ...
							// No parameter changes when idling
							break;

						case 2:		// Modulating
							// Check for reference change
							// If message startswith "ref" - copy the rest of the message and convert it to float
							memcpy(buffer, &serial_buffer, 3);
							buffer[3] = '\0';
							if (strcmp(buffer, "ref") == 0){
								memcpy(buffer, &serial_buffer[3], 5);
								buffer[5] = '\0';
								float newRef = atof(buffer);
								// Check if received value is inside borders
								if (newRef < 2000 && newRef >= 0){
									Ref = newRef;
									uart_send_string("Changes applied!\n");
								}
								else{
									uart_send_string("Invalid value for Ref!\n");
								}
								uart_send_config(Ref, Ki, Kp);
							}
							break;

						default:
							break;
					}
				}
				// Set buffer index to 0 so the next serial message is saved properly
				serial_buffer_index = 0;
				// Re-Enable button interrupts:
				XGpio_InterruptEnable(pBTNS_SWTS, 0x3);
				XGpio_InterruptGlobalEnable(pBTNS_SWTS);
			}
			// If the received character is not newline or carriage return add it to buffer and continue
			else {
				serial_buffer[serial_buffer_index] = input;
				serial_buffer_index++;
			}
		}
	}

	return 0;
}

// Button interrupt handler
void ButtonISR(void *data){
	XGpio_InterruptClear(&BTNS_SWTS,0xF);
	buttons = XGpio_DiscreteRead(&BTNS_SWTS, BUTTONS_channel);
	switch(buttons){
		// on btn0 press
		case LD0:
			acquire_lock();
			*pbtn0Flag = TRUE;
			release_lock();
			break;
		// On btn1 press
		case LD1:
			acquire_lock();
			*pbtn1Flag = TRUE;
			release_lock();
			break;
		// On btn2 press
		case LD2:
			acquire_lock();
			*pbtn2Flag = TRUE;
			release_lock();
			break;
		// On btn3 press
		case LD3:
			acquire_lock();
			*pbtn3Flag = TRUE;
			release_lock();
			break;

		default:
			break;
	}
}

// TTC1 Interrupt handler (used for timing the control process)
void TimerISR(){
	// Read interrupt status to clear the interrupt
	XTtcPs_ClearInterruptStatus(pTTC1, XTTCPS_IXR_INTERVAL_MASK);
	// Set flag to calculate new values for PI-controller and system output
	acquire_lock();
	timerFlag = TRUE;
	release_lock();
}
