/* Pearl Iyayi
** lab 9: implementation of serial and IR interface with rover
*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Uart0.h"
#include "wait.h"
#include "clock.h"
#include "tm4c123gh6pm.h"

// PortC masks
#define LEFT_HALL_MASK  64
#define RIGHT_HALL_MASK 16

// PortE bit banding
#define SLEEP (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))
#define IR    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))

// PortE mask
#define SLEEP_MASK 2
#define IR_MASK    4

// PortF masks
#define LEFT_MOTOR1_MASK  1
#define LEFT_MOTOR2_MASK  2
#define RIGHT_MOTOR1_MASK 4
#define RIGHT_MOTOR2_MASK 8

// Global variables
uint32_t time[50];
uint8_t bitset[32];
uint8_t bi = 0;
uint8_t count = 0;
uint32_t t = 0;
uint8_t code = 0, add = 0, notadd = 0, notcode = 0;
bool valid = false;
char sb[255];

// struct
#define MAX_CHARS 80
#define MAX_FIELDS 5
typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;
USER_DATA data;

//initialize hardware
void inithw()
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R5;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0 | SYSCTL_RCGCWTIMER_R1 |  SYSCTL_RCGCWTIMER_R2;
    _delay_cycles(3);

    // Unlocking PF0
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R |= 1;

    // Configure left and right motors
    GPIO_PORTF_DIR_R |= LEFT_MOTOR1_MASK | LEFT_MOTOR2_MASK | RIGHT_MOTOR2_MASK | RIGHT_MOTOR1_MASK;
    GPIO_PORTF_DEN_R |= LEFT_MOTOR1_MASK | LEFT_MOTOR2_MASK | RIGHT_MOTOR2_MASK | RIGHT_MOTOR1_MASK;
    GPIO_PORTF_AFSEL_R |= LEFT_MOTOR1_MASK | LEFT_MOTOR2_MASK | RIGHT_MOTOR2_MASK | RIGHT_MOTOR1_MASK;
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF0_M | GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M);
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF0_M1PWM4 | GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;

    // Configure PE1 as an output
    GPIO_PORTE_DIR_R |= SLEEP_MASK;
    GPIO_PORTE_DEN_R |= SLEEP_MASK;

    //Configure PE2 as an input from the IR detector
    GPIO_PORTE_DIR_R &= ~IR_MASK;
    GPIO_PORTE_DEN_R |= IR_MASK;
    GPIO_PORTE_IS_R &= ~IR_MASK;
    GPIO_PORTE_IBE_R &= ~IR_MASK;
    GPIO_PORTE_IEV_R &= ~IR_MASK;
    GPIO_PORTE_IM_R |= IR_MASK;
    GPIO_PORTE_PUR_R |= IR_MASK;
    GPIO_PORTE_ICR_R |= IR_MASK;
    NVIC_EN0_R |= 1 << (INT_GPIOE-16);                 // turn-on interrupt 20 (GPIOE)

    // Configure wide timer 1 and wide timer 0 for the hall effect sensors
    GPIO_PORTC_AFSEL_R |= LEFT_HALL_MASK | RIGHT_HALL_MASK;
                                                       // select alternative functions for LEFT_HALL_MASK and RIGHT_HALL_MASK pin
    GPIO_PORTC_PCTL_R &= ~(GPIO_PCTL_PC4_M | GPIO_PCTL_PC6_M);
                                                       // map alt fns to LEFT_HALL_MASK and RIGHT_HALL_MASK
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_WT0CCP0 | GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= LEFT_HALL_MASK | RIGHT_HALL_MASK;
                                                       // enable bit 4 and bit 6 for digital input
    GPIO_PORTC_PUR_R |= LEFT_HALL_MASK | RIGHT_HALL_MASK;
                                                       //pulling up the output from the hall sensors' output

    // Configure PWM module 1 to drive the DRV8833
    // RED   on M1PWM5 (PF1), M1PWM2b
    // BLUE  on M1PWM6 (PF2), M1PWM3a
    // GREEN on M1PWM7 (PF3), M1PWM3b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;                                // turn-off PWM1 generator 2 (drives outs 4 and 5)
    PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3 (drives outs 6 and 7)
    PWM1_2_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                                                     // output 4 on PWM1, gen 2a, cmpa
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 5 on PWM1, gen 2b, cmpb
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                                                     // output 6 on PWM1, gen 3a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 7 on PWM1, gen 3b, cmpb

    PWM1_2_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_3_LOAD_R = 1024;

    PWM1_2_CMPA_R = 0;                               // invert outputs so duty cycle increases with increasing compare values
    PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM1_3_CMPB_R = 0;                               // green off
    PWM1_3_CMPA_R = 0;                               // blue off

    PWM1_2_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 2
    PWM1_3_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM1_ENABLE_R = PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                     // enable outputs

    // Configure wide timers 0 and 1 in count mode for the hall effects
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER1_CTL_R = 0;                               //
    WTIMER1_IMR_R = 0;                               // turn-off interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R &= ~(1 << (INT_WTIMER1A-16-96));      // turn-off interrupt 112 (WTIMER1A)

    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER0_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER0_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER0_CTL_R = 0;                               //
    WTIMER0_IMR_R = 0;                               // turn-off interrupts
    WTIMER0_TAV_R = 0;                               // zero counter for first period
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN2_R &= ~(1 << (INT_WTIMER0A-16-64));      // turn-off interrupt 110 (WTIMER0A)

    // Configure wide timer 2 as a periodic timer
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER2_CFG_R |= 4;                              // configure as 32-bit counter (A only)
    WTIMER2_TAMR_R |= 0x2;                           // configure for Periodic Mode
    WTIMER2_TAMR_R &= ~TIMER_TAMR_TACDIR;            // configure for Count Down Mode
    WTIMER2_IMR_R = 0;                               // turn-off interrupts
    WTIMER2_TAPR_R = 40;                             // decrements at a 1MHz rate
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter

}

//interrupt function called by the IR
void gpiIsr(void)
{
   // putsUart0("gpiisr\n");
    if(count == 0)
    {
        time[count] = WTIMER2_TAV_R;
        count++;
    }
    else if(count == 1)
    {
        time[count] = WTIMER2_TAV_R;
        t = time[count-1] - time[count];
        if(t > 13000 && t < 14000)
        {
            count++;
        }
        else
        {
            count = 0;
        }
    }
    else if (!(count >= 34) && count > 1)
    {
        time[count] = WTIMER2_TAV_R;
        t = time[count-1] - time[count];
        if((t > 844 && t < 2531))
        {
            if(t > 844 && t < 1500)
                bitset[bi++] = 0;
            else if(t > 2000 && t < 2531)
                bitset[bi++] = 1;
            count++;
        }
        else
        {
            count = 0;
        }
    }
    else if(count >= 34)
    {
        putsUart0("all edges prsd\n");
        waitMicrosecond(500000);
        uint8_t b = 0;
        for(; b < 32; b++)
        {
            if(b <= 7)
            {
                if(bitset[b]==0)
                    add = add << 1;
                else
                {
                    add = add << 1;
                    add |= 1;
                }
            }
            else if(b >= 8 && b <= 15)
            {
                if(bitset[b]==0)
                    notadd = notadd << 1;
                else
                {
                    notadd = notadd << 1;
                    notadd |= 1;
                }
            }
            else if(b >= 16 && b <= 23)
            {
                if(bitset[b]==0)
                    code = code << 1;
                else
                {
                    code = code << 1;
                    code |= 1;
                }
            }
            else if(b >= 24 && b <= 31)
            {
                if(bitset[b]==0)
                    notcode = notcode << 1;
                else
                {
                    notcode = notcode << 1;
                    notcode |= 1;
                }//end of nested else
            }//end of else if
        }//end of for loop

        bi = 0;
        count = 0;
        t = 0;
        WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
        WTIMER2_CFG_R |= 4;                              // configure as 32-bit counter (A only)
        WTIMER2_TAMR_R |= 0x2;                           // configure for Periodic Mode
        WTIMER2_TAPR_R = 40;                             // decrements at a 1MHz rate
        WTIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter

        if((code + notcode) == 255 && (add + notadd) == 255)
        {
            valid = true;
        //    char c[200];
         //   sprintf(c,"code: %X\n",code);
          //  putsUart0(c);
        }

    }
    GPIO_PORTE_ICR_R |= IR_MASK;
}

// function to get command from serial connection
void getsUart0(USER_DATA* s)
{
    int count = 0;
    while(true)
    {
        char c = getcUart0();
        if((c == 127 || c == 8) && count > 0)
        {
            count--;
            continue;
        }
        if(c == 13 || count == MAX_CHARS)
        {
            s->buffer[count] = 0;
            return;
        }
        if(c >= 32 && count != MAX_CHARS)
        {
            s->buffer[count] = c;
            count++;
        }
    }
}

// Function parses the command gotten from the Uart
void parseFields(USER_DATA *s)
{
    s->fieldCount = 0;
    char check = 'd';
    int i = 0;

    while(s->buffer[i] != 0)
    {
        if(s->buffer[i]>=48 && s->buffer[i]<=57) //checks if number
        {
            s->fieldType[s->fieldCount] = 'n'; //sets fieldType

            if(s->fieldType[s->fieldCount] != check) //check if coming from delimiter
            {
                s->fieldPosition[s->fieldCount] = i; //sets fieldPosition
                s->fieldCount++; //increment the fieldCount
                check = 'n'; //sets the fieldType for next run
            }
        }
        else if((s->buffer[i]>=65 && s->buffer[i]<=90) ||
                            (s->buffer[i]>=97 && s->buffer[i]<=122))
        {
            s->fieldType[s->fieldCount] = 'a';

            if(s->fieldType[s->fieldCount] != check)
            {
                s->fieldPosition[s->fieldCount] = i;
                s->fieldCount++;
                check = 'a';
            }
        }
        else
        {
            check = 'd'; //if here then its a delimiter
            s->buffer[i] = 0; //over write delimiter to null
        }
        i++;
    }
}

// returns the command entered
char* getFieldString(USER_DATA* s, uint8_t fieldNumber)
{
    char y[50]; int i = fieldNumber; int j = 0;
    if(fieldNumber > s->fieldCount || s->buffer[fieldNumber] == 0)
        return NULL;
    while(true)
    {
        if(s->buffer[i] == 0)
        {
            y[j] = 0;
            break;
        }

        y[j] = s->buffer[i];
        i++;
        j++;
    }
    return y;
}

// returns the distance or degree to move
int32_t getFieldInteger(USER_DATA* s, uint8_t fieldNumber)
{
    int32_t retval = 0; int i = 0;
    if(fieldNumber <= s->fieldCount && s->fieldType[fieldNumber]=='n')
    {
        int j = s->fieldPosition[fieldNumber];
        while(true)
            {
                if(s->buffer[j] == 0)
                {
                    return retval;
                }
                if( i >= 1)
                {
                    retval *= 10;
                    retval += s->buffer[j] - '0';
                }
                else
                {
                    retval = s->buffer[j] - '0';
                }
                i++;
                j++;
            }
    }
    else
        return 0;
}

// functions to make the robot move
void forward(uint16_t dist_cm)
{
    uint16_t total_dist;
  //  char b[200];
    total_dist =  (46/20) * dist_cm;
    bool check_left = false;
    bool check_right = false;

    SLEEP = 1;
    PWM1_2_CMPB_R = 865;
    PWM1_3_CMPB_R = 990;
    while((!check_left) && (!check_right))
    {
       /* sprintf(b,"timer 0: %u\ttimer 1: %u\n",WTIMER0_TAV_R,WTIMER1_TAV_R);
        putsUart0(b);*/
        check_left = (WTIMER1_TAV_R >= total_dist);
        check_right = (WTIMER0_TAV_R >= total_dist);

        if(check_left || check_right)
        {
            PWM1_2_CMPB_R = 0;
            PWM1_3_CMPB_R = 0;
        }
        //if(check_right) PWM1_3_CMPB_R = 0;
    }
    WTIMER1_TAV_R = 0;
    WTIMER0_TAV_R = 0;
    SLEEP = 0;
}

void reverse(uint16_t dist_cm)
{
    uint16_t total_dist;
    total_dist =  (46/20) * dist_cm;
    bool check_left = false;
    bool check_right = false;

    SLEEP = 1;
    PWM1_2_CMPA_R = 760;
    PWM1_3_CMPA_R = 1000;
    while((!check_left) && (!check_right))
    {

        check_left = (WTIMER1_TAV_R >= total_dist);
        check_right = (WTIMER0_TAV_R >= total_dist);

        if(check_left || check_right)
        {
            PWM1_2_CMPA_R = 0;
            PWM1_3_CMPA_R = 0;
        }
    }
    WTIMER1_TAV_R = 0;
    WTIMER0_TAV_R = 0;
    SLEEP = 0;
}

void cw(uint16_t deg)
{
    uint16_t total_spin, temp;
    temp = (58*deg)/360;
    total_spin = (46/20) * temp;
   // sprintf(b,"total distance: %u\n",total_spin);
 //   putsUart0(b);
    SLEEP = 1;
    PWM1_2_CMPB_R = 1000;
    PWM1_3_CMPA_R = 1000;

    while(WTIMER1_TAV_R <= total_spin && WTIMER0_TAV_R <= total_spin);

    PWM1_2_CMPB_R = 0;
    PWM1_3_CMPA_R = 0;
    WTIMER1_TAV_R = 0;
    WTIMER0_TAV_R = 0;
    SLEEP = 0;
}
void ccw(uint16_t deg)
{
    uint16_t total_spin, temp;
    temp = (58*deg)/360;
    total_spin = (46/20) * temp;
    SLEEP = 1;
    PWM1_2_CMPA_R = 1000;
    PWM1_3_CMPB_R = 1000;

    while(WTIMER1_TAV_R <= total_spin && WTIMER0_TAV_R <= total_spin);

    PWM1_2_CMPA_R = 0;
    PWM1_3_CMPB_R = 0;
    WTIMER1_TAV_R = 0;
    WTIMER0_TAV_R = 0;
    SLEEP = 0;
}
void stop(void)
{
   // PWM1_2_CMPA_R = 0;
   // PWM1_2_CMPB_R = 0;
   // PWM1_3_CMPA_R = 0;
   // PWM1_3_CMPB_R = 0;
    SLEEP = 0;
}

// draws a square for extra credit
void extraCredit(void)
{
    forward(20);
    waitMicrosecond(2000);
    cw(105);
    waitMicrosecond(2000);
    forward(30);
    waitMicrosecond(2000);
    cw(115);
    waitMicrosecond(2000);
    forward(30);
    waitMicrosecond(2000);
    cw(105);
    waitMicrosecond(2000);
    forward(20);
    waitMicrosecond(2000);
    cw(105);
}

//function to detect from serial and IR
void calling()
{
    uint16_t value = 0, temp, numcount = 0;
    char command; bool ok = false;
    while(true)
    {
        if(kbhitUart0())
        {
            getsUart0(&data);
            parseFields(&data);
            if(strcmp(getFieldString(&data,0),"forward")==0) forward(getFieldInteger(&data,1));
            else if(strcmp(getFieldString(&data,0),"reverse")==0) reverse(getFieldInteger(&data,1));
            else if(strcmp(getFieldString(&data,0),"cw")==0) cw(getFieldInteger(&data,1));
            else if(strcmp(getFieldString(&data,0),"ccw")==0) ccw(getFieldInteger(&data,1));
            if(strcmp(getFieldString(&data,0),"stop")==0) stop();
        }
        else if(valid)
        {
            switch(code)
            {
                case 0x10: //power
                    putsUart0("0x10\tPower\n");
                    stop();
                    break;
                case 0x88: //1
                    temp = 1;
                    numcount++;
                    putsUart0("0x88\t1\n");
                    break;
                case 0x48: //2
                    temp = 2;
                    numcount++;
                    putsUart0("0x48\t2\n");
                    break;
                case 0xC8: //3
                    temp = 3;
                    numcount++;
                    putsUart0("0xC8\t3\n");
                    break;
                case 0x28: //4
                    temp = 4;
                    numcount++;
                    putsUart0("0x28\t4\n");
                    break;
                case 0xA8: //5
                    temp = 5;
                    numcount++;
                    putsUart0("0xA8\t5\n");
                    break;
                case 0x68: //6
                    temp = 6;
                    numcount++;
                    putsUart0("0x68\t6\n");
                    break;
                case 0xE8: //7
                    temp = 7;
                    numcount++;
                    putsUart0("0xE8\t7\n");
                    break;
                case 0x18: //8
                    temp = 8;
                    numcount++;
                    putsUart0("0x18\t8\n");
                    break;
                case 0x98: //9
                    temp = 9;
                    numcount++;
                    putsUart0("0x98\t9\n");
                    break;
                case 0x8:  //0
                    temp = 0;
                    numcount++;
                    putsUart0("0x8\t0\n");
                    break;
                case 0x22: //ok
                    putsUart0("0x22\tok\n");
                    ok = true;
                    break;
                case 0x2: //up
                    putsUart0("0x2\tup\n");
                    command = 'f';
                    break;
                case 0x82://down
                    putsUart0("0x82\tdown\n");
                    command ='r';
                    break;
                case 0xE0: //left cw
                    putsUart0("0xE0\tleft\n");
                    command ='c';
                    break;
                case 0x60: //right ccw
                    putsUart0("0x60\tright\n");
                    command ='w';
                    break;
                case 0x86: //blue four dots
                    extraCredit();
                    break;
                default:
                    command ='n';
                    putsUart0("default\n");
                    temp = 99;
            }//end of switch

            if(command== 'f')
            {
                if(numcount == 0)
                {
                    if(ok)
                    {
                        forward(30);
                        command = 0;
                    }
                }
                else if(numcount == 1)
                {
                    if(ok)
                    {
                        forward(value);
                        numcount = 0;
                        command = 0;
                    }
                    value = temp;
                }
                else if(numcount > 1)
                {
                    if(ok)
                    {
                        forward(value);
                        numcount = 0;
                        command = 0;
                    }
                    value *= 10;
                    value += temp;
                }
            }
            if(command == 'r')
            {
                if(numcount == 0)
                {
                    if(ok)
                    {
                        reverse(30);
                        command = 0;
                    }
                }
                else if(numcount == 1)
                {
                    if(ok)
                    {
                        reverse(value);
                        numcount = 0;
                        command = 0;
                    }
                    value = temp;
                }
                else if(numcount > 1)
                {
                    if(ok)
                    {
                        reverse(value);
                        numcount = 0;
                        command = 0;
                    }
                    value *= 10;
                    value += temp;
                }
            }
            if(command == 'c')
            {
                if(numcount == 0)
                {
                    if(ok)
                    {
                        cw(30);
                        command = 0;
                    }
                }
                else if(numcount == 1)
                {
                    if(ok)
                    {
                        cw(value);
                        command = 0;
                        numcount = 0;
                    }
                    value = temp;
                }
                else if(numcount > 1)
                {
                    if(ok)
                    {
                        cw(value);
                        command = 0;
                        numcount = 0;
                    }
                    value *= 10;
                    value += temp;
                }
            }
            if(command == 'w')
            {
                if(numcount == 0)
                {
                    if(ok)
                    {
                        ccw(130);
                        command = 0;
                    }
                }
                else if(numcount == 1)
                {
                    if(ok)
                    {
                        ccw(value);
                        numcount = 0;
                    }
                    value = temp;
                }
                else if(numcount > 1)
                {
                    if(ok)
                    {
                        ccw(value);
                        command = 0;
                        numcount = 0;
                    }
                    value *= 10;
                    value += temp;
                }
            }
            ok = false;
            valid = false;
            code = 0;
            add = 0;
            notadd = 0;
            notcode = 0;

        }//end of else if
    }//end of while loop
}//end of function
int main()
{
    inithw();
    initUart0();

    putsUart0("running in main\n");
    calling();
}
