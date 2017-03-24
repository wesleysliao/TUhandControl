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


#define GPIO_JOYSTICK_CLICK_PORT GPIO_PORTA_BASE
#define GPIO_JOYSTICK_CLICK_PIN GPIO_PIN_5

#define GPIO_JOYSTICK_X_AXIS_PORT GPIO_PORTE_BASE
#define GPIO_JOYSTICK_X_AXIS_PIN GPIO_PIN_2
#define ADC_CH_JOYSTICK_X_AXIS ADC_CTL_CH1

#define GPIO_JOYSTICK_Y_AXIS_PORT GPIO_PORTE_BASE
#define GPIO_JOYSTICK_Y_AXIS_PIN GPIO_PIN_1
#define ADC_CH_JOYSTICK_Y_AXIS ADC_CTL_CH2

void enableSysPeripherals(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
  {
  }
}


void onButtonDown(void);
void onButtonUp(void);

void onButtonDown(void) {
    if (GPIOIntStatus(GPIO_JOYSTICK_CLICK_PORT, false) & GPIO_JOYSTICK_CLICK_PIN) {
        // PA5 was interrupt cause
        js_msg.select = true;
        adc_joystick.publish(&js_msg);
        GPIOIntRegister(GPIO_JOYSTICK_CLICK_PORT, onButtonUp);   // Register our handler function for port A
        GPIOIntTypeSet(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN,
            GPIO_RISING_EDGE);          // Configure PA5 for rising edge trigger
        GPIOIntClear(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN);  // Clear interrupt flag
    }
}

void onButtonUp(void) {
    if (GPIOIntStatus(GPIO_JOYSTICK_CLICK_PORT, false) & GPIO_JOYSTICK_CLICK_PIN) {
        // PA5 was interrupt cause
        js_msg.select = false;
        adc_joystick.publish(&js_msg);
        GPIOIntRegister(GPIO_JOYSTICK_CLICK_PORT, onButtonDown); // Register our handler function for port F
        GPIOIntTypeSet(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN,
            GPIO_FALLING_EDGE);         // Configure PF4 for falling edge trigger
        GPIOIntClear(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN);  // Clear interrupt flag
    }
}

void readADC(void){

  ADCIntClear(ADC0_BASE,1);

  uint32_t adc_values[8] = {0,0,0,0,0,0,0,0};

  ADCSequenceDataGet(ADC0_BASE, 1, adc_values);

  if( abs(adc_values[0] - js_msg.x_axis_raw) > 100 || abs(adc_values[2] - js_msg.y_axis_raw) > 100)
  {
      js_msg.x_axis_raw = adc_values[0];
      js_msg.y_axis_raw = adc_values[2];
      adc_joystick.publish(&js_msg);
  }

}



int main(void)
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  // TivaC system clock configuration. Set to 80MHz.
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  enableSysPeripherals();

  //Setup Joystick click
  GPIOPinTypeGPIOInput(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN);
  GPIOPadConfigSet(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor

  //Joystick click Interrupt setup
  GPIOIntDisable(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN);        // Disable interrupt for PA5 (in case it was enabled)
  GPIOIntClear(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN);      // Clear pending interrupts for PA5
  GPIOIntRegister(GPIO_JOYSTICK_CLICK_PORT, onButtonDown);     // Register our handler function for port A
  GPIOIntTypeSet(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN, GPIO_FALLING_EDGE);             // Configure PF4 for falling edge trigger
  GPIOIntEnable(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN);     // Enable interrupt for PF4


  //Setup joystic axis pins
  GPIOPinTypeADC(GPIO_JOYSTICK_X_AXIS_PORT, GPIO_JOYSTICK_X_AXIS_PIN);
  GPIOPinTypeADC(GPIO_JOYSTICK_Y_AXIS_PORT, GPIO_JOYSTICK_Y_AXIS_PIN);  

  ADCSequenceDisable( ADC0_BASE, 0 );
  ADCSequenceDisable( ADC0_BASE, 1 );
  ADCSequenceDisable( ADC0_BASE, 2 );
  ADCSequenceDisable( ADC0_BASE, 3 );

  ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CH_JOYSTICK_X_AXIS);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CH_JOYSTICK_X_AXIS);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CH_JOYSTICK_Y_AXIS);
  ADCSequenceStepConfigure(ADC0_BASE,1,3, ADC_CTL_END | ADC_CH_JOYSTICK_Y_AXIS | ADC_CTL_IE);
  ADCSequenceEnable(ADC0_BASE, 1);

  ADCIntRegister(ADC0_BASE, 1, readADC);
  ADCIntEnable(ADC0_BASE, 1);
  
  nh.initNode();
  nh.advertise(adc_joystick);

  while (1)
  {

    ADCProcessorTrigger(ADC0_BASE, 1);
    
    nh.spinOnce();
    nh.getHardware()->delay(100);
  }

}