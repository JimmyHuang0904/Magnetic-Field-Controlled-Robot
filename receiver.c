/*
 // RECEIVER CODE - PROJECT 2 Magnetic Controlled Robot
 // ELEC 291
*/

#include <C8051F38x.h>
#include <stdlib.h>
#include <stdio.h>

#define SYSCLK          48000000L // SYSCLK frequency in Hz
#define BAUDRATE        115200L   // Baud rate of UART in bps
#define SMB_FREQUENCY   100000L   // I2C SCL clock rate (10kHz to 100kHz)
#define SMB_FREQUENCY   100000L   // I2C SCL clock rate (10kHz to 100kHz)
#define DEFAULT_F       15500L

#define OUT0 P2_0 //left wheel forwards
#define OUT1 P2_1 //left wheel backwards
#define OUT2 P2_2 //right wheel forwards
#define OUT3 P2_3 //right wheel backwards
#define ADC1 P1_7
#define ADC2 P2_4


volatile char_received; //Stores character given by serial port
volatile int movement; //Stores the character describing movement-
/*
0 - resting state
1 - left
2 - right
3 - forward
4 - backwards
*/
volatile int mode; //Tells us wheter its in tracker mode or control mode
/*
0 - control mode
1 - tracker mode
*/


void InitADC (void)
{
    // Init ADC
    ADC0CF = 0xF8;          // SAR clock = 31, Right-justified result
    ADC0CN = 0b_1000_0000;  // AD0EN=1, AD0TM=0
    REF0CN = 0b_0000_1000;  //Select VDD as the voltage reference for the converter
}

void UART1_Init (unsigned long baudrate)
{
    SMOD1 = 0x0C; // no parity, 8 data bits, 1 stop bit
    SCON1 = 0x10;
    if (((SYSCLK/baudrate)/2L)/0xFFFFL < 1){
        SBRL1 = 0x10000L-((SYSCLK/baudrate)/2L);
        SBCON1 |= 0x03; // set prescaler to 1
    }
    else if (((SYSCLK/baudrate)/2L)/0xFFFFL < 4){
        SBRL1 = 0x10000L-(((SYSCLK/baudrate)/2L)/4L);
        SBCON1 &= ~0x03;
        SBCON1 |= 0x01; // set prescaler to 4
    }
    else if (((SYSCLK/baudrate)/2L)/0xFFFFL < 12){
        SBRL1 = 0x10000L-(((SYSCLK/baudrate)/2L)/12L);
        SBCON1 &= ~0x03; // set prescaler to 12
    }
    else{
        SBRL1 = 0x10000L-(((SYSCLK/baudrate)/2L)/48L);
        SBCON1 &= ~0x03;
        SBCON1 |= 0x02; // set prescaler to ?
    }
    SBCON1 |= 0x40; // enable baud rate generator
    SCON1 &= ~0x01; //clearing RX
    EIE2 |= 0x02;   //enables UART1 interrupt
    EIP2 |= 0x02;   //set UART1 interrupt as highest priority
}

void InitPinADC (unsigned char portno, unsigned char pinno)
{
    unsigned char mask;

    mask=1<<pinno;

    switch (portno)
    {
        case 0:
            P0MDIN &= (~mask);  // Set pin as analog input
            P0SKIP |= mask;     // Skip Crossbar decoding for this pin
        break;
        case 1:
            P1MDIN &= (~mask);  // Set pin as analog input
            P1SKIP |= mask;     // Skip Crossbar decoding for this pin
        break;
        case 2:
            P2MDIN &= (~mask);  // Set pin as analog input
            P2SKIP |= mask;     // Skip Crossbar decoding for this pin
        break;
        case 3:
            P3MDIN &= (~mask);  // Set pin as analog input
            P3SKIP |= mask;     // Skip Crossbar decoding for this pin
        break;
        default:
        break;
    }
}

char _c51_external_startup (void)
{
    PCA0MD&=(~0x40) ;                       // DISABLE WDT: clear Watchdog Enable bit
    VDM0CN=0x80;                            // enable VDD monitor
    RSTSRC=0x02|0x04;                       // Enable reset on missing clock detector and VDD

    // Configure UART0
    SCON0 = 0x10; 
    // CLKSEL&=0b_1111_1000;                // Not needed because CLKSEL==0 after reset
    #if (SYSCLK == 12000000L)
        //CLKSEL|=0b_0000_0000;             // SYSCLK derived from the Internal High-Frequency Oscillator / 4 
    #elif (SYSCLK == 24000000L)
        CLKSEL|=0b_0000_0010;               // SYSCLK derived from the Internal High-Frequency Oscillator / 2.
    #elif (SYSCLK == 48000000L)
        CLKSEL|=0b_0000_0011;               // SYSCLK derived from the Internal High-Frequency Oscillator / 1.
    #else
        #error SYSCLK must be either 12000000L, 24000000L, or 48000000L
    #endif
    OSCICN |= 0x03;                         // Configure internal oscillator for its maximum frequency


#if (SYSCLK/BAUDRATE/2L/256L < 1)

    TH1 = 0x10000-((SYSCLK/BAUDRATE)/2L);
    CKCON &= ~0x0B;                         // T1M = 1; SCA1:0 = xx
    CKCON |=  0x08;
#elif (SYSCLK/BAUDRATE/2L/256L < 4)
    TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/4L);
    CKCON &= ~0x0B;                         // T1M = 0; SCA1:0 = 01
    CKCON |=  0x01;
#elif (SYSCLK/BAUDRATE/2L/256L < 12)
    TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/12L);
    CKCON &= ~0x0B;                         // T1M = 0; SCA1:0 = 00
#else
    TH1 = 0x10000-(SYSCLK/BAUDRATE/2/48);
    CKCON &= ~0x0B;                         // T1M = 0; SCA1:0 = 10
    CKCON |=  0x02;
#endif
    TL1 = TH1;                              // Init Timer1
    TMOD &= ~0xf0;                          // TMOD: timer 1 in 8-bit autoreload
    TMOD |=  0x20;                       
    TR1 = 1;                                // START Timer1
    TI = 1;                                 // Indicate TX0 ready

    // Initialize Crossbar and GPIO
    P0MDOUT = 0x10;                         // Enable Uart TX as push-pull output
    P2MDOUT |= 0b0000_0110;                 // Make the LED (P2.2) a push-pull output.  P2.1 used for debuging.
    XBR0 = 0b0000_0101;                     // Enable SMBus pins and UART pins P0.4(TX) and P0.5(RX)
    XBR1 = 0x40;                            // Enable crossbar and weak pull-ups
    XBR2 = 0x01;                            // Enable UART TX1, and RX1 routed to Port Pins
    P2MDOUT    |=0b_0001_0011;
    // Configure Timer 0 as the I2C clock source
    CKCON |= 0x04;                          // Timer0 clock source = SYSCLK
    TMOD &= 0xf0;                           // Mask out timer 1 bits
    TMOD |= 0x02;                           // Timer0 in 8-bit auto-reload mode

    // Timer 0 configured to overflow at 1/3 the rate defined by SMB_FREQUENCY
    TL0 = TH0 = 256-(SYSCLK/SMB_FREQUENCY/3);
    TR0 = 1;                                // Enable timer 0

    // Initialize timer 2 for periodic interrupts
    TMR2CN=0x00;                            // Stop Timer2; Clear TF2;
    CKCON|=0b_0001_0000;
    TMR2RL=(-(SYSCLK/(2*DEFAULT_F)));       // Initialize reload value
    TMR2=0xffff;                            // Set to reload immediately
    ET2=1;                                  // Enable Timer2 interrupts
    TR2=1;                                  // Start Timer2

    EA=1;                                   // Enable interrupts

    // Configure and enable SMBus
    SMB0CF = INH | EXTHOLD | SMBTOE | SMBFTE ;
    SMB0CF |= ENSMB;                        // Enable SMBus

    return 0;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
    AMX0P = pin;             // Select positive input from pin
    AMX0N = LQFP32_MUX_GND;  // GND is negative input (Single-ended Mode)
    // Dummy conversion first to select new pin
    AD0BUSY=1;
    while (AD0BUSY); // Wait for dummy conversion to finish
    // Convert voltage at the pin
    AD0BUSY = 1;
    while (AD0BUSY); // Wait for conversion to complete
    return (ADC0L+(ADC0H*0x100));
}

float Volts_at_Pin(unsigned char pin)
{
     return ((ADC_at_Pin(pin)*3.30)/1024.0);
}

void I2C_write (unsigned char output_data)
{
    SMB0DAT = output_data; // Put data into buffer
    SI0 = 0;
    while (!SI0); // Wait until done with send
}

unsigned char I2C_read (void)
{
    unsigned char input_data;

    SI0 = 0;
    while (!SI0); // Wait until we have data to read
    input_data = SMB0DAT; // Read the data

    return input_data;
}

void I2C_start (void)
{
    ACK0 = 1;
    STA0 = 1;     // Send I2C start
    STO0 = 0;
    SI0 = 0;
    while (!SI0); // Wait until start sent
    STA0 = 0;     // Reset I2C start
}

void I2C_stop(void)
{
    STO0 = 1;      // Perform I2C stop
    SI0 = 0;    // Clear SI
}

void stop() // Stops motors
{
    OUT0 = 0; 
    OUT1 = 0;
    OUT2 = 0;
    OUT3 = 0;
}

void UART1_ISR (void) interrupt 16
{
    while(SCON1 & 0x01){
        char_received = SBUF1;
        SCON1 &= ~0x01;
    }

    if (char_received == 'f'){
        movement = 3 ;
    }
    else if (char_received == 'B'){
        movement = 4 ;
    }
    else if (char_received == 'L'){
        movement = 1 ;
    }
    else if (char_received == 'r'){
        movement = 2 ;
    }
    else if (char_received == 'c'){
        mode = 0 ;
    }
    else if (char_received == 'T'){
        mode = 1 ;
    }
    else
        movement = 0;

}

void main (void)
{
    // Variables
    float d1 = 0.0; 
    float d2 = 0.0; // stores value of how far the receiver inductors are from the transmitter
    float const_distance = 0.0;
    float offset = 0.1;
    mode = 0;

    // Initializations ---------------------------------------------------------//
    UART1_Init(110);
    InitADC();
    InitPinADC(1,7); // Configure P1.7 as analog input
    InitPinADC(2,4); // Configure P2.4 as analog input

    printf("\x1b[2J"); // Clear screen using ANSI escape sequence.
    printf("Testing input into Receiver.\n");
    while(1){ // Forever loop
            printf("char received: %c \r" , char_received);        
        while(mode == 0){ // Control Mode
            switch ( movement ){

                //moving left
                case 1:
                    OUT0 = 1;
                    OUT1 = 0;
                    OUT2 = 0;
                    OUT3 = 0;
                break;

                //moving right;
                case 2:
                    OUT0 = 0;
                    OUT1 = 0;
                    OUT2 = 1;
                    OUT3 = 0;
                break;

                //moving forwards
                case 3:
                    OUT0 = 1;
                    OUT1 = 0;
                    OUT2 = 1;
                    OUT3 = 0;
                break;

                //moving backwards
                case 4:
                    OUT0 = 0;
                    OUT1 = 1;
                    OUT2 = 0;
                    OUT3 = 1;
                break;

                //Stops moving 
                default:
                    stop();
                break;
            }

        }
        //Stuff to clear from control mode before entering tracker mode
        stop();
        const_distance = Volts_at_Pin(LQFP32_MUX_P2_4);

        while(mode == 1){ // Tracker Mode
            d1 = Volts_at_Pin(LQFP32_MUX_P2_4); //getting distance from calculations
            d2 = Volts_at_Pin(LQFP32_MUX_P1_7);
            //printf("\x1b[2J"); // Clear screen using ANSI escape sequence.
            printf("voltage d1: %f  voltage d2: %f  char: %c  constant_distance: %f\r", d1, d2, char_received, const_distance);
            //left wheel
            if(d1>const_distance + offset){
                OUT1 = 1 ;//move left wheel backwards
            }
            else{
                OUT1 = 0;//stop left wheel
            }
            
            if(d2>const_distance + offset){
                OUT3 = 1 ;//move right wheel backwards
            }
            else{
                OUT3 = 1;//stop right wheel
            }
            
            if(d1<const_distance - offset){
                OUT0 = 1;//move left wheel forwards
            }
            else{
                OUT0 = 0;//stop left wheel
            }
            
            if(d2<const_distance - offset){
                OUT2 = 1;//move right wheel forwards
            }
            else{
                OUT2 = 0;//stop right wheel
            }
        }
        //Stuff to clear from tracker mode before entering control mode
        stop();

    }
}
