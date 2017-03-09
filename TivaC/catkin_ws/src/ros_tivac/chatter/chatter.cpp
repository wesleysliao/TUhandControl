#include <stdbool.h>
#include <stdint.h>
// TivaC specific includes
extern "C"
{
  #include <driverlib/interrupt.h>
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
}
// ROS includes
#include <ros.h>
#include <std_msgs/String.h>

// ROS nodehandle
ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "Hello world!";
char down[13] = "onButtonDown";
char up[11] = "onButtonUp";


void onButtonDown(void);
void onButtonUp(void);

void onButtonDown(void) {
    if (GPIOIntStatus(GPIO_PORTA_BASE, false) & GPIO_PIN_5) {
        // PA5 was interrupt cause
        str_msg.data = down;
        chatter.publish(&str_msg);
        GPIOIntRegister(GPIO_PORTA_BASE, onButtonUp);   // Register our handler function for port A
        GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5,
            GPIO_RISING_EDGE);          // Configure PA5 for rising edge trigger
        GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5);  // Clear interrupt flag
    }
}

void onButtonUp(void) {
    if (GPIOIntStatus(GPIO_PORTA_BASE, false) & GPIO_PIN_5) {
        // PA5 was interrupt cause
        str_msg.data = up;
        chatter.publish(&str_msg);
        GPIOIntRegister(GPIO_PORTA_BASE, onButtonDown); // Register our handler function for port F
        GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5,
            GPIO_FALLING_EDGE);         // Configure PF4 for falling edge trigger
        GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5);  // Clear interrupt flag
    }
}

int main(void)
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  // TivaC system clock configuration. Set to 80MHz.
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  // Pin F4 setup
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);        // Enable port A
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);  // Init PA5 as input
  GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_5,
      GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor for PA5

  // Interrupt setup
  GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_5);        // Disable interrupt for PA5 (in case it was enabled)
  GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5);      // Clear pending interrupts for PA5
  GPIOIntRegister(GPIO_PORTA_BASE, onButtonDown);     // Register our handler function for port A
  GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5,
      GPIO_FALLING_EDGE);             // Configure PF4 for falling edge trigger
  GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_5);     // Enable interrupt for PF4

  // ROS nodehandle initialization and topic registration
  nh.initNode();
  nh.advertise(chatter);

  while (1)
  {
    // Publish message to be transmitted.
    
    //str_msg.data = hello;

    // Handle all communications and callbacks.
    nh.spinOnce();

    // Delay for a bit.
    nh.getHardware()->delay(100);
  }
}