#include <stdbool.h>
#include <stdint.h>
// TivaC specific includes
extern "C"
{
  #include <driverlib/interrupt.h>
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
  #include <driverlib/adc.h>
}
// ROS includes
#include <ros.h>
#include "adc_joystick_msg/ADC_Joystick.h"

// ROS nodehandle
ros::NodeHandle nh;

adc_joystick_msg::ADC_Joystick js_msg;
ros::Publisher adc_joystick("adc_joystick", &js_msg);


void onButtonDown(void);
void onButtonUp(void);

void onButtonDown(void) {
    if (GPIOIntStatus(GPIO_PORTA_BASE, false) & GPIO_PIN_5) {
        // PA5 was interrupt cause
        js_msg.select = true;
        adc_joystick.publish(&js_msg);
        GPIOIntRegister(GPIO_PORTA_BASE, onButtonUp);   // Register our handler function for port A
        GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5,
            GPIO_RISING_EDGE);          // Configure PA5 for rising edge trigger
        GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5);  // Clear interrupt flag
    }
}

void onButtonUp(void) {
    if (GPIOIntStatus(GPIO_PORTA_BASE, false) & GPIO_PIN_5) {
        // PA5 was interrupt cause
        js_msg.select = false;
        adc_joystick.publish(&js_msg);
        GPIOIntRegister(GPIO_PORTA_BASE, onButtonDown); // Register our handler function for port F
        GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5,
            GPIO_FALLING_EDGE);         // Configure PF4 for falling edge trigger
        GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5);  // Clear interrupt flag
    }
}


void xAxisMoved(void){
  ADCIntClear(ADC1_BASE,0);
  ADCIntClearEx(ADC1_BASE, ADC_INT_DCON_SS0);

  uint32_t intStatus = ADCComparatorIntStatus( ADC1_BASE );
  if( intStatus  != 0)
  {
      ADCComparatorIntClear(ADC1_BASE, intStatus);
  }

  uint32_t deadband = 1;
  uint32_t x_axis_value;
  
  ADCSequenceDataGet(ADC1_BASE, 0, &x_axis_value);
  ADCComparatorRegionSet(ADC1_BASE, 0, x_axis_value-deadband, x_axis_value+deadband);

  js_msg.x_axis_raw = x_axis_value;
  js_msg.y_axis_raw = x_axis_value-deadband;
  adc_joystick.publish(&js_msg);

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


  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1))
  {
  }
  ADCReferenceSet(ADC1_BASE, ADC_REF_INT);
  ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_END | ADC_CTL_CH1 | ADC_CTL_CMP0);
  ADCSequenceEnable(ADC1_BASE, 0);

  ADCComparatorConfigure(ADC1_BASE, 0, ADC_COMP_INT_LOW_ALWAYS|ADC_COMP_INT_HIGH_ALWAYS);
  ADCComparatorIntEnable(ADC1_BASE, 0);

  ADCIntRegister(ADC1_BASE, 0, xAxisMoved);
  ADCIntEnable(ADC1_BASE, 0);
  ADCIntEnableEx(ADC1_BASE, ADC_INT_DCON_SS0);
  
  nh.initNode();
  nh.advertise(adc_joystick);


  ADCProcessorTrigger(ADC1_BASE, 0);

  nh.getHardware()->delay(100);
  
  uint32_t deadband = 1;
  uint32_t x_axis_value;
  
  ADCSequenceDataGet(ADC1_BASE, 0, &x_axis_value);
  ADCComparatorRegionSet(ADC1_BASE, 0, x_axis_value-deadband, x_axis_value+deadband);


  while (1)
  {

    ADCProcessorTrigger(ADC1_BASE, 0);
    
    nh.spinOnce();
    nh.getHardware()->delay(100);
  }

}