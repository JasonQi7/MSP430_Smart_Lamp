/*  Smart Bedside Lamp
 *  Author: Jason Qi
 *  UCL Electronic and Electrical Engineering
 */

#include "msp430g2553.h"
#define SWITCH_FLAG 1; // Swtich flag mask

// UART pins
#define RXD BIT1
#define TXD BIT2

// States
#define OFF 0
#define LEVEL1 1
#define LEVEL2 2
#define LEVEL3 3
#define RED_ON 4
#define STORE_OFF 5

// Actions
#define ON_LEVEL1 0
#define ON_LEVEL2 1
#define ON_LEVEL3 2
#define OFF_LED 3
#define OFF_INST 4
#define RE_OPEN 5

char currentState = 0;
char storedState = 0;
char numberStr[5];
char transitionTable[6] = {LEVEL1, LEVEL2, LEVEL3, OFF, STORE_OFF, RED_ON}; // Transition table
char actionTable[6] = {ON_LEVEL1, ON_LEVEL2, ON_LEVEL3, OFF_LED, OFF_INST, RE_OPEN}; // Action table
volatile char switchState = 0; // Flag used for restore state after time out
volatile char pressed = 0; // Flag used to indicate a Switch Press
volatile char pressRelease = 0; // Flag used to indicate a Switch Press + Release
volatile char flag = 0;
static volatile char data;
static volatile int count;

// Function prototypes
void ConfigureTimerA (void);
void ConfigureWDT (void);
void ConfigureUART (void);
void ConfigureLEDs (void);
void ConfigureButtons (void);
void UARTSendArray (unsigned char *TxArray, unsigned char ArrayLength);


void main(void)
{
    // System initialisation
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;
    ConfigureWDT();
    ConfigureTimerA();
    ConfigureLEDs();
    ConfigureButtons();
    ConfigureUART();

    // Main routine
    for( ; ; )
    {
        // Triggered by button interrupt
        if (pressRelease)
        {
            switch (actionTable[currentState])
            {
                case OFF_LED: // state 0: LED = OFF
                    TACCR1 = 0;
                    UARTSendArray("currentState: OFF", 17);
                    UARTSendArray("\r\n", 2);
                    break;
                    
                case ON_LEVEL1: // state 1: LED brightness = 1
                    TACCR1 = 200;
                    UARTSendArray("currentState: LEVEL1", 20);
                    UARTSendArray("\r\n", 2);
                    break;

                case ON_LEVEL2: // state 2: LED brightness = 2
                    TACCR1 = 600;
                    UARTSendArray("currentState: LEVEL2", 20);
                    UARTSendArray("\r\n", 2);
                    break;

                case ON_LEVEL3: // state 3: LED brightness = 3
                    TACCR1 = 1000;
                    UARTSendArray("currentState: LEVEL3", 20);
                    UARTSendArray("\r\n", 2);
                    break;

                case OFF_INST: // state 4: Time out, LED = OFF
                    TACCR1 = 0;
                    flag = 1;
                    break;

                case RE_OPEN: // state 5: Reopen, restore brightness
                    switch (storedState)
                    {
                        case LEVEL1:
                            TACCR1 = 200;
                            UARTSendArray("currentState: LEVEL1", 20);
                            UARTSendArray("\r\n", 2);
                            currentState = LEVEL1-1;
                            break;

                        case LEVEL2:
                            TACCR1 = 600;
                            UARTSendArray("currentState: LEVEL2", 20);
                            UARTSendArray("\r\n", 2);
                            currentState = LEVEL2 - 1;
                            break;

                        case LEVEL3:
                            TACCR1 = 1000;
                            UARTSendArray("currentState: LEVEL3", 20);
                            UARTSendArray("\r\n", 2);
                            currentState = LEVEL3 - 1;
                            break;
                    }
                    break;
            }
            currentState=transitionTable[currentState]; // Correct current state
            pressRelease &= ~ SWITCH_FLAG; // Clear switch flag to indicate that the switch press has been serviced
            if (flag != 1)
                storedState=transitionTable[currentState];
            else
                flag = 0;
        }
        __bis_SR_register(LPM0_bits + GIE); // Put CPU into sleep mode
    }
}


// Configure watchdog time for debouncing s1 and s2
void ConfigureWDT (void)
{
    WDTCTL = WDTPW + WDTHOLD; //WDT password + Stop WDT + detect RST button falling edge + set RST/NMI pin to NMI
    IFG1 &= ~ WDTIFG; // Clear the WDT and NMI interrupt flags
    IE1 |= WDTIE; // Enable the WDT and NMI interrupts
}


// Configure timer A as as a clock divider to generate timed interrupt
void ConfigureTimerA (void)
{
    TACCTL0 = CCIE; // Enable counter interrupt on counter compare register 0
    TACCTL1 = OUTMOD_7; // set output on counter reset, clear output on CCR1
    TACTL = TASSEL_2 +ID_3 + MC_1; // Use the SMCLK to clock the counter, SMCLK/8, count up mode
    TACCR0 = 1000 - 1; // Set maximum count (Interrupt frequency 1MHz/8/1000 = 125Hz)
    TACCR1 = 0;
}


// Configure output LED pins
void ConfigureLEDs (void)
{
    P1DIR |= (BIT0+BIT6); // P1.0 and P1.6 to output
    P1OUT &= ~ BIT0; // Set the LEDs P1.0 to indicate correct configuration
    P1SEL |= BIT6; // Select output P1.0 to be TA0.1
    P1SEL2 &= ~BIT6; // Select output P1.0 to be TA0.1
}


// Configure input button pins
void ConfigureButtons (void)
{
     P1DIR &= ~BIT3; // Set button pin as an input pin
     P1OUT |= BIT3; // Set pull up resistor on for button
     P1REN |= BIT3; // Enable pull up resistor for button to keep pin high until pressed
     P1IES |= BIT3; // Enable Interrupt to trigger on the falling edge
     P1IFG &= ~BIT3; // Clear the interrupt flag for the button
     P1IE |= BIT3; // Enable interrupts on port 1 for the button
}


// Configure hardware UART
void ConfigureUART (void)
{
    P1SEL |= RXD + TXD ; // P1.1 = RXD, P1.2=TXD
    P1SEL2 |= RXD + TXD ; // P1.1 = RXD, P1.2=TXD
    UCA0CTL1 |= UCSSEL_2; // Use SMCLK
    UCA0BR0 = 104; // Set baud rate to 9600 with 1MHz clock
    UCA0BR1 = 0; // Set baud rate to 9600 with 1MHz clock
    UCA0MCTL = UCBRS0; // Modulation UCBRSx = 1
    UCA0CTL1 &= ~UCSWRST; // Initialize USCI state machine
    IE2 |= UCA0RXIE; // Enable USCI_A0 RX interrupt
}


// Watchdog timer interrupt handler
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR (void)
{
    P1IFG &= ~BIT3; // Clear the button interrupt flag (in case it has been set by bouncing)
    P1IE |= BIT3; // Re-enable interrupt on Pin 1.3
}


// Timer A interrupt handler
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
    // Switch to time out state only after 4 sec
    if (++count == 500 && currentState != 5 && currentState!= 0)
    {
        // Store current state, next press => OFF
        storedState=transitionTable[currentState]; // Stored current state
        currentState = OFF_INST; // Switch to time out state
        UARTSendArray("Press to turn off.\r\n", 23);
        P1OUT |= BIT0; // Auxiliary LED indicator: ON
    }
}


// Push button interrupt handler
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR (void)
{
    // Act on Pin 1.3 (push button) only
    if (P1IFG & BIT3)
    {
        P1IE &= ~BIT3; // Disable further interrupt
        P1IFG &= ~BIT3; // Clear interrupt flag
        
        // Falling edge detected
        if (P1IES & BIT3)
        {
            P1OUT |= BIT0; // Auxiliary LED indicator: ON
            switchState |= SWITCH_FLAG; // Set SWITCH_FLAG state to press
            pressed |= SWITCH_FLAG; // Set Switch pressed flag
        }
        
        // Rising edge detected
        else
        {
            P1OUT &= ~ BIT0; // Auxiliary LED indicator: OFF
            switchState &= ~SWITCH_FLAG; // Reset Switch state
            pressRelease |= SWITCH_FLAG; // Set 'Press and Released' flag
            count = 0; // reset counter for Timer A
        }
        
        P1IES ^= BIT3; // Toggle edge detect
        IFG1 &= ~WDTIFG; // Clear interrupt flag for WDT
        WDTCTL = WDT_MDLY_32 | (WDTCTL & 0x007F); // Restart WDT
        __bic_SR_register_on_exit(LPM0_bits); // Wake up CPU
    }
}



// UART interrupt handler
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR (void)
{
    data = UCA0RXBUF; // Read from UART buffer
    switch(data)
    {
        case '0': // state 0: LED = OFF
            UARTSendArray("Turned Off.\r\n", 13);
            UARTSendArray("\r\n", 2);
            TACCR1 = 0;
            currentState = OFF;
            break;
   
        case '1': // state 1: LED brightness = 1
            UARTSendArray("Brightness = Level1\r\n", 19);
            UARTSendArray("\r\n", 2);
            TACCR1 = 200;
            currentState = LEVEL1;
            break;
   
        case '2': // state 1: LED brightness = 2
            UARTSendArray("Brightness = Level2\r\n", 19);
            UARTSendArray("\r\n", 2);
            TACCR1 = 600;
            currentState = LEVEL2;
            break;
  
        case '3': // state 1: LED brightness = 3
            UARTSendArray("Brightness = Level3\r\n", 19);
            UARTSendArray("\r\n", 2);
            TACCR1 = 1000;
            currentState = LEVEL3;
            break;
   
        default: // Handle illegal inputs
        {
            UARTSendArray("Error Input: ", 15);
            UARTSendArray(&data, 1);
            UARTSendArray("\n\r", 2);
        }
        break;
    }
}


// Helper function: send array via UART
void UARTSendArray (unsigned char *TxArray, unsigned char ArrayLength)
{
    while (ArrayLength--)
    { // Loop until StringLength == 0 and post decrement
        while (!(IFG2 & UCA0TXIFG)); // Wait for TX buffer to be ready for new data
        UCA0TXBUF = *TxArray; // Write the character at the location specified py the pointer
        TxArray++; // Increment the TxString pointer to point to the next character
    }
}
