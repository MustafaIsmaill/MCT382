/*
 * MUSTAFA ISMAIL
 *
 * THIS PROJECT IS CREATED TO RUN AT TM4C123GH6PM TIVA C EVALUATION BOARD
 * THE AIM IS TO CONTROL THE SPEED OF THE 6 DC MOTORS USING PWM GENERATORS
 * READ MOTORS RPM FROM ENCODERS USING EDGE TIME CAPTURE
 * RUN 6 PID CONTROL LOOPS
 *
 * USING THE TIVAWARE PERIPHERAL DRIVER LIBRARY AND TIVAWARE UTILITIES LIBRARY
 *
 * MOTORS PIN MAP:
 *
 * M0PWM0 PB6 PWM1 DONE
 * M0PWM1 PB7 PWM2 DONE
 * M0PWM2 PB4 PWM3 DONE
 * M0PWM3 PB5 PWM4 DONE
 * M0PWM4 PE4 PWM5 DONE
 * M0PWM5 PE5 PWM6 DONE
 *
 * MOTOR1 PA2, PA3 DONE
 * MOTOR2 PA4, PA5 DONE
 * MOTOR3 PA6, PA7 DONE
 * MOTOR4 PE1, PE2 DONE
 * MOTOR5 PE3, PC5 DONE
 * MOTOR6 PC6, PC7 DONE
 *
 */


//HEADER FILES
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"
#include "driverlib/i2c.h"
#include <string.h>


//FUNCTIONS PROTOTYPES
void portF_config(void);
void portA_config(void);
void portE_config(void);
void portC_config(void);
void pwm_config(void);
void uart_config(void);
void move_f(void);
void move_b(void);
void move_r(void);
void move_l(void);
void stop(void);
void check_and_move(char k);
void delayMS(int ms);


//GLOBAL VARIABLES
int pwm = 4500;  //USED TO SEND CONSTANT PWM
char c[10];  //BUFFER USED WITH UART

int main(void){
    //Tiva C clock 16 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //    IntMasterEnable();

    pwm_config();
    uart_config();
    portF_config();
    portA_config();
    portE_config();
    portC_config();

    while(1){

        if(UARTCharsAvail(UART0_BASE)){
            UARTgets(c,sizeof(c));
            check_and_move(c[0]);

            //RED LED
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
        }
        else{
            check_and_move('n');

            //BLUE LED
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        }

    }
}

void portF_config(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

//MOTORS
void portA_config(void){
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
                          GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
}

//MOTORS
void portE_config(void){
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
//    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

//MOTORS
void portC_config(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
}

void uart_config(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}

void pwm_config(void){
    //PWM0 clock enable //PWM0 64 divider
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    //GPIOB clock enable
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    //GPIOE clock enable
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    //GPIOB PWM
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);

    //GPIOE PWM
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    GPIOPinConfigure(GPIO_PE5_M0PWM5);

    //PWM GENERATOR CONFIG
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //PWM PERIOD SET
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 5000); //20ms (16Mhz / 64pwm_divider / 50)
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 5000); //20ms (16Mhz / 64pwm_divider / 50)
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 5000); //20ms (16Mhz / 64pwm_divider / 50)

    //PULSE WIDTH SET
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 0);

    //GENERATOR ENABLE
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

    //OUTPUT STATE
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT |
            PWM_OUT_2_BIT | PWM_OUT_3_BIT | PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
}

void delayMS(int ms) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
}

void move_f(void){
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, pwm);

    //MOTOR1
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    //MOTOR2
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);
    //MOTOR3
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
    //MOTOR4
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
    //MOTOR5
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0);
    //MOTOR6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);
}

void move_b(void){
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, pwm);

    //MOTOR1
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
    //MOTOR2
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
    //MOTOR3
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
    //MOTOR4
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);
    //MOTOR5
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
    //MOTOR6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
}

void move_r(void){
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, pwm);

    //MOTOR1
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    //MOTOR2
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);
    //MOTOR3
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
    //MOTOR4
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);
    //MOTOR5
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
    //MOTOR6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
}

void move_l(void){
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pwm);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, pwm);

    //MOTOR1
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
    //MOTOR2
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
    //MOTOR3
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
    //MOTOR4
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
    //MOTOR5
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0);
    //MOTOR6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);
}

void stop(void){
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 0);
    //MOTOR1
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    //MOTOR2
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);
    //MOTOR3
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
    //MOTOR4
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
    //MOTOR5
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0);
    //MOTOR6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);
}

void check_and_move(char k){
    switch(k){
    case 'f':
        move_f();
        break;
    case 'b':
        move_b();
        break;
    case 'r':
        move_r();
        break;
    case 'l':
        move_l();
        break;
    default:
        stop();
    }
}
