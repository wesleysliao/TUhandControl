#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include <math.h>


uint32_t x_axis_value;
uint32_t y_axis_value;

void readADC(void){

  ADCIntClear(ADC0_BASE,1);

  uint32_t adc_values[8] = {0,0,0,0,0,0,0,0};

  int vals = ADCSequenceDataGet(ADC0_BASE, 1, adc_values);

  if( abs(adc_values[0] - x_axis_value) > 100 || abs(adc_values[2] - y_axis_value) > 100)
  {
      x_axis_value = adc_values[0];
      y_axis_value = adc_values[2];
      SysCtlDelay(1);
  }

}


int main(void)
{
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2);


  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
  {
  }

  ADCSequenceDisable( ADC0_BASE, 0 );
  ADCSequenceDisable( ADC0_BASE, 1 );
  ADCSequenceDisable( ADC0_BASE, 2 );
  ADCSequenceDisable( ADC0_BASE, 3 );

  ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH1);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2);
  ADCSequenceStepConfigure(ADC0_BASE,1,3, ADC_CTL_END | ADC_CTL_CH2 | ADC_CTL_IE);
  ADCSequenceEnable(ADC0_BASE, 1);

  ADCIntRegister(ADC0_BASE, 1, readADC);
  ADCIntEnable(ADC0_BASE, 1);



  while (1)
  {
      ADCProcessorTrigger(ADC0_BASE, 1);
//      SysCtlDelay(1000);
//      uint32_t value[4] = {0,0,0,0};
//      int vals = ADCSequenceDataGet(ADC0_BASE, 1, value);
      SysCtlDelay(100);
  }
}
