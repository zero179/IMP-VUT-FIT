//       An example for demonstrating basic principles of FITkit3 usage.
//
// It includes GPIO - inputs from button press/release, outputs for LED control,
// timer in output compare mode for generating periodic events (via interrupt
// service routine) and speaker handling (via alternating log. 0/1 through
// GPIO output on a reasonable frequency). Using this as a basis for IMP projects
// as well as for testing basic FITkit3 operation is strongly recommended.
//
//            (c) 2019 Michal Bidlo, BUT FIT, bidlom@fit.vutbr.cz
//            (c) 2020 Simon  Fenko, xfenko01
////////////////////////////////////////////////////////////////////////////
/* Header file with all the essential definitions for a given type of MCU */
#include "MK60D10.h"

/* Macros for bit-level registers manipulation */
#define GPIO_PIN_MASK 0x1Fu
#define GPIO_PIN(x) (((1)<<(x & GPIO_PIN_MASK)))

/* Mapping of LEDs and buttons to specific port pins: */
// Note: only D9, SW3 and SW5 are used in this sample app
#define LED_D9  0x20      // Port B, bit 5
#define LED_D10 0x10      // Port B, bit 4
#define LED_D11 0x8       // Port B, bit 3
#define LED_D12 0x4       // Port B, bit 2

#define BTN_SW2 0x400     // Port E, bit 10
#define BTN_SW3 0x1000    // Port E, bit 12
#define BTN_SW4 0x8000000 // Port E, bit 27
#define BTN_SW5 0x4000000 // Port E, bit 26
#define BTN_SW6 0x800     // Port E, bit 11

#define SPK 0x10          // Speaker is on PTA4

#define INP_OUT 0x111
#define REG     0x112

int pressed_up = 0, pressed_down = 0, pressed_left = 0, pressed_right = 0, pressed_next = 0;
int beep_flag = 0;
int state = INP_OUT; //starting state in case
unsigned int compare = 0x200;

/* A delay function */
void delay(long long bound) {

  long long i;
  for(i=0;i<bound;i++);
}

/* Initialize the MCU - basic clock settings, turning the watchdog off */
void MCUInit(void)  {
    MCG_C4 |= ( MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x01) );
    SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV1(0x00);
    WDOG_STCTRLH &= ~WDOG_STCTRLH_WDOGEN_MASK;
}

void PortsInit(void)
{
    /* Turn on all port clocks */
    SIM->SCGC5 = SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTA_MASK;

    /* Set corresponding PTB pins (connected to LED's) for GPIO functionality */
    PORTB->PCR[5] = PORT_PCR_MUX(0x01); // D9
    PORTB->PCR[4] = PORT_PCR_MUX(0x01); // D10
    PORTB->PCR[3] = PORT_PCR_MUX(0x01); // D11
    PORTB->PCR[2] = PORT_PCR_MUX(0x01); // D12

    PORTE->PCR[10] = PORT_PCR_MUX(0x01); // SW2
    PORTE->PCR[12] = PORT_PCR_MUX(0x01); // SW3
    PORTE->PCR[27] = PORT_PCR_MUX(0x01); // SW4
    PORTE->PCR[26] = PORT_PCR_MUX(0x01); // SW5
    PORTE->PCR[11] = PORT_PCR_MUX(0x01); // SW6

    PORTA->PCR[4] = PORT_PCR_MUX(0x01);  // Speaker

    /* Change corresponding PTB port pins as outputs */
    PTB->PDDR = GPIO_PDDR_PDD(0x3C);     // LED ports as outputs
    PTA->PDDR = GPIO_PDDR_PDD(SPK);     // Speaker as output
    PTB->PDOR |= GPIO_PDOR_PDO(0x3C);    // turn all LEDs OFF
    PTA->PDOR &= GPIO_PDOR_PDO(~SPK);   // Speaker off, beep_flag is false
}

void LPTMR0_IRQHandler(void)
{
    // Set new compare value set by up/down buttons
    LPTMR0_CMR = compare;                // !! the CMR reg. may only be changed while TCF == 1
    LPTMR0_CSR |=  LPTMR_CSR_TCF_MASK;   // writing 1 to TCF tclear the flag
    beep_flag = !beep_flag;              // see beep_flag test in main()
}

void LPTMR0Init(int count)
{
    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK; // Enable clock to LPTMR
    LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;   // Turn OFF LPTMR to perform setup
    LPTMR0_PSR = ( LPTMR_PSR_PRESCALE(0) // 0000 is div 2
                 | LPTMR_PSR_PBYP_MASK   // LPO feeds directly to LPT
                 | LPTMR_PSR_PCS(1)) ;   // use the choice of clock
    LPTMR0_CMR = count;                  // Set compare value
    LPTMR0_CSR =(  LPTMR_CSR_TCF_MASK    // Clear any pending interrupt (now)
                 | LPTMR_CSR_TIE_MASK    // LPT interrupt enabled
                );
    NVIC_EnableIRQ(LPTMR0_IRQn);         // enable interrupts from LPTMR0
    LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;    // Turn ON LPTMR0 and start counting
}

int main(void)
{
    MCUInit();
    PortsInit();
    LPTMR0Init(compare);

    while (1) {
        // pressing the button next switches states
        if (!pressed_next && !(GPIOE_PDIR & BTN_SW6))
        {
            pressed_next=1;                              // button was pushed on
            // switching states
            if(state == INP_OUT)
            {
                state = REG;
            }
            else{
                state = INP_OUT;
            }
        }
        else if (GPIOE_PDIR & BTN_SW6)
        {
            pressed_next=0;                              // button was released
        }
        switch(state){
            case INP_OUT:
                // pressing the up button makes all led diodes turned on and turned
                // of (blinked) and speaker will make beep sound
                if (!pressed_up && !(GPIOE_PDIR & BTN_SW5))
                {
                    pressed_up=1;                        // button was pushed on
                    GPIOB_PDOR ^= LED_D9;                // invert D9 state
                    GPIOB_PDOR ^= LED_D10;               // invert D10 state
                    GPIOB_PDOR ^= LED_D11;               // invert D11 state
                    GPIOB_PDOR ^= LED_D12;               // invert D12 state
                    for (int i = 0; i<100; i++)            // loop which makes beep
                    {
                        GPIOA_PDOR ^= SPK;               // invert speaker state
                        delay(1000);                     // set frequence
                    }
                    GPIOB_PDOR ^= LED_D9;                // invert D9 state
                    GPIOB_PDOR ^= LED_D10;               // invert D10 state
                    GPIOB_PDOR ^= LED_D11;               // invert D11 state
                    GPIOB_PDOR ^= LED_D12;               // invert D12 state
                }
                else if (GPIOE_PDIR & BTN_SW5)
                {
                    pressed_up = 0;                      // button was released
                }
                
                // pressing the down button makes all led diodes turned on and turned
                // of (blinked) and speaker will make beep sound
                if (!pressed_down && !(GPIOE_PDIR & BTN_SW3))
                {
                    pressed_down=1;                      // button was pushed on
                    GPIOB_PDOR ^= LED_D9;                // invert D9 state
                    GPIOB_PDOR ^= LED_D10;               // invert D10 state
                    GPIOB_PDOR ^= LED_D11;               // invert D11 state
                    GPIOB_PDOR ^= LED_D12;               // invert D12 state
                    for (int i = 0; i<100; i++)            // loop which makes beep
                    {
                        GPIOA_PDOR ^= SPK;               // invert speaker state
                        delay(1000);                     // set frequence
                    }
                    GPIOB_PDOR ^= LED_D9;                // invert D9 state
                    GPIOB_PDOR ^= LED_D10;               // invert D10 state
                    GPIOB_PDOR ^= LED_D11;               // invert D11 state
                    GPIOB_PDOR ^= LED_D12;               // invert D12 state
                }
                else if (GPIOE_PDIR & BTN_SW3)
                {
                    pressed_down = 0;                    // button was push off
                }

                // pressing the right button makes all led diodes turned on and turned
                // of (blinked) and speaker will make beep sound
                if (!pressed_right && !(GPIOE_PDIR & BTN_SW2))
                {
                    pressed_right=1;                     // button was pushed on
                    GPIOB_PDOR ^= LED_D9;                // invert D9 state
                    GPIOB_PDOR ^= LED_D10;               // invert D10 state
                    GPIOB_PDOR ^= LED_D11;               // invert D11 state
                    GPIOB_PDOR ^= LED_D12;               // invert D12 state
                    for (int i = 0; i<100; i++)            // loop which makes beep
                    {
                        GPIOA_PDOR ^= SPK;               // invert speaker state
                        delay(1000);                     // set frequence
                    }
                    GPIOB_PDOR ^= LED_D9;                // invert D9 state
                    GPIOB_PDOR ^= LED_D10;               // invert D10 state
                    GPIOB_PDOR ^= LED_D11;               // invert D11 state
                    GPIOB_PDOR ^= LED_D12;               // invert D12 state
                }
                else if (GPIOE_PDIR & BTN_SW5)
                {
                    pressed_right = 0;                   // button was released
                }
                
                // pressing the left button makes all led diodes turned on and turned
                // of (blinked) and speaker will make beep sound
                if (!pressed_left && !(GPIOE_PDIR & BTN_SW4))
                {
                    pressed_left=1;                      // button was pushed on
                    GPIOB_PDOR ^= LED_D9;                // invert D9 state
                    GPIOB_PDOR ^= LED_D10;               // invert D10 state
                    GPIOB_PDOR ^= LED_D11;               // invert D11 state
                    GPIOB_PDOR ^= LED_D12;               // invert D12 state
                    for (int i = 0; i<100; i++)            // loop which makes beep
                    {
                        GPIOA_PDOR ^= SPK;               // invert speaker state
                        delay(1000);                     // set frequence
                    }
                    GPIOB_PDOR ^= LED_D9;                // invert D9 state
                    GPIOB_PDOR ^= LED_D10;               // invert D10 state
                    GPIOB_PDOR ^= LED_D11;               // invert D11 state
                    GPIOB_PDOR ^= LED_D12;               // invert D12 state
                }
                else if (GPIOE_PDIR & BTN_SW4)
                {
                    pressed_left = 0;                    // button was released
                }
            break;
                            
            case REG:
                ERROR_LABEL: while(1)
                {
                    // beeping signalizing error
                    if (beep_flag)
                    {
                        GPIOA_PDOR ^= SPK;   // invert speaker state
                        delay(1000);         // set frequence
                    }
                    else GPIOA_PDOR &= ~SPK; // logic 0 on speaker port if beep is false
                }
                // checking register0
                __asm__("LDR R0, =0xAAAAAAAA");            // load 0xAAAAAAAA into register 0
                __asm__("CMP R0, 0xAAAAAAAA");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR R0, =0x55555555");            // load 0x55555555 into register 0
                __asm__("CMP R0, 0x55555555");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking register1
                __asm__("LDR R1, =0xAAAAAAAA");            // load 0xAAAAAAAA into register 1
                __asm__("CMP R1, 0xAAAAAAAA");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR R1, =0x55555555");            // load 0x55555555 into register 1
                __asm__("CMP R1, 0x55555555");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking register2
                __asm__("LDR R2, =0xAAAAAAAA");            // load 0xAAAAAAAA into register 2
                __asm__("CMP R2, 0xAAAAAAAA");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR R2, =0x55555555");            // load 0x55555555 into register 2
                __asm__("CMP R2, 0x55555555");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking register3
                __asm__("LDR R3, =0xAAAAAAAA");            // load 0xAAAAAAAA into register 3
                __asm__("CMP R3, 0xAAAAAAAA");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR R3, =0x55555555");            // load 0x55555555 into register 3
                __asm__("CMP R3, 0x55555555");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking register4
                __asm__("LDR R4, =0xAAAAAAAA");            // load 0xAAAAAAAA into register 4
                __asm__("CMP R4, 0xAAAAAAAA");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR R4, =0x55555555");            // load 0x55555555 into register 4
                __asm__("CMP R4, 0x55555555");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking register5
                __asm__("LDR R5, =0xAAAAAAAA");            // load 0xAAAAAAAA into register 5
                __asm__("CMP R5, 0xAAAAAAAA");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR R5, =0x55555555");            // load 0x55555555 into register 5
                __asm__("CMP R5, 0x55555555");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking register6
                __asm__("LDR R6, =0xAAAAAAAA");            // load 0xAAAAAAAA into register 6
                __asm__("CMP R6, 0xAAAAAAAA");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR R6, =0x55555555");            // load 0x55555555 into register 6
                __asm__("CMP R6, 0x55555555");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking register7
                __asm__("LDR R7, =0xAAAAAAAA");            // load 0xAAAAAAAA into register 7
                __asm__("CMP R7, 0xAAAAAAAA");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR R7, =0x55555555");            // load 0x55555555 into register 7
                __asm__("CMP R7, 0x55555555");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking register8
                __asm__("LDR R8, =0xAAAAAAAA");            // load 0xAAAAAAAA into register 8
                __asm__("CMP R8, 0xAAAAAAAA");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR R8, =0x55555555");            // load 0x55555555 into register 8
                __asm__("CMP R8, 0x55555555");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking register9
                __asm__("LDR R9, =0xAAAAAAAA");            // load 0xAAAAAAAA into register 9
                __asm__("CMP R9, 0xAAAAAAAA");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR R9, =0x55555555");            // load 0x55555555 into register 9
                __asm__("CMP R9, 0x55555555");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking register10
                __asm__("LDR R10, =0xAAAAAAAA");           // load 0xAAAAAAAA into register 10
                __asm__("CMP R10, 0xAAAAAAAA");            // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR R10, =0x55555555");           // load 0x55555555 into register 10
                __asm__("CMP R10, 0x55555555");            // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking register11
                __asm__("LDR R11, =0xAAAAAAAA");           // load 0xAAAAAAAA into register 11
                __asm__("CMP R11, 0xAAAAAAAA");            // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR R11, =0x55555555");           // load 0x55555555 into register 11
                __asm__("CMP R11, 0x55555555");            // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking register12
                __asm__("LDR R12, =0xAAAAAAAA");           // load 0xAAAAAAAA into register 12
                __asm__("CMP R12, 0xAAAAAAAA");            // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR R12, =0x55555555");           // load 0x55555555 into register 12
                __asm__("CMP R12, 0x55555555");            // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking registerSP
                __asm__("LDR SP, =0xAAAAAAAA");            // load 0xAAAAAAAA into register SP
                __asm__("CMP SP, 0xAAAAAAAA");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR SP, =0x55555555");            // load 0x55555555 into register SP
                __asm__("CMP SP, 0x55555555");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking registerLR
                __asm__("LDR LR, =0xAAAAAAAA");            // load 0xAAAAAAAA into register LR
                __asm__("CMP LR, 0xAAAAAAAA");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR LR, =0x55555555");            // load 0x55555555 into register LR
                __asm__("CMP LR, 0x55555555");             // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                
                // checking registerPSR
                __asm__("LDR PSR, =0xAAAAAAAA");           // load 0xAAAAAAAA into register PSR
                __asm__("CMP PSR, 0xAAAAAAAA");            // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                __asm__("LDR PSR, =0x55555555");           // load 0x55555555 into register PSR
                __asm__("CMP PSR, 0x55555555");            // comparing
                __asm__("BNE ERROR_LABEL");                // branch if not equal - ERROR
                // endless loop
            break;
        }
    }

    return 0;
}
