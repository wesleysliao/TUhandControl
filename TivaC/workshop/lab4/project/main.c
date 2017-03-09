#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"

uint32_t ui32PulseCount0A;
uint32_t ui32PulseCount0B;
uint32_t ui32PulseCount1A;
uint32_t ui32PulseCount1B;

int main(void)
{
    uint32_t ui32Period0A;
    uint32_t ui32Period0B;
    uint32_t ui32Period1A;
    uint32_t ui32Period1B;

    //SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);   //40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); //80MHz

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR| TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC);

    ui32Period0A = (SysCtlClockGet() / 100000) / 2;
    ui32Period0B = (SysCtlClockGet() / 100000) / 2;
    ui32Period1A = (SysCtlClockGet() / 100000) / 2;
    ui32Period1B = (SysCtlClockGet() / 100000) / 2;
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period0A -1);
    TimerLoadSet(TIMER0_BASE, TIMER_B, ui32Period0B -1);
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period1A -1);
    TimerLoadSet(TIMER1_BASE, TIMER_B, ui32Period1B -1);

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    IntEnable(INT_TIMER0B);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    IntEnable(INT_TIMER1B);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);

    ui32PulseCount0A = 0;
    ui32PulseCount0B = 0;
    ui32PulseCount1A = 0;
    ui32PulseCount1B = 0;

    IntMasterEnable();

    TimerEnable(TIMER0_BASE, TIMER_A|TIMER_B);
    TimerEnable(TIMER1_BASE, TIMER_A);

    while(1)
    {
    }
}

void Timer0IntHandler(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, (1&ui32PulseCount0A++) << 4);
}

void Timer0BIntHandler(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, (1&ui32PulseCount0B++) << 4);
}

void Timer1AIntHandler(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, (1&ui32PulseCount1A++) << 4);
}

void Timer1BIntHandler(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, (1&ui32PulseCount1B++) << 4);
}
