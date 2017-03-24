#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"

uint32_t old_low;
uint32_t old_high;

void xAxisMoved(void){

  ADCIntClear(ADC0_BASE,1);
  ADCIntClearEx(ADC0_BASE, ADC_INT_DCON_SS1);



  uint32_t intStatus = ADCComparatorIntStatus( ADC0_BASE );
  if( intStatus  != 0)
  {
      ADCComparatorIntClear(ADC0_BASE, intStatus);

      uint32_t deadband = 200;
      uint32_t x_axis_value[8] = {0,0,0,0,0,0,0,0};

      int vals = ADCSequenceDataGet(ADC0_BASE, 1, x_axis_value);

      uint32_t low;
      uint32_t high;

      low = x_axis_value[0]-deadband;
      high = x_axis_value[0]+deadband;

      if(x_axis_value[0]<=deadband)
          low = 0;
//      else if(x_axis_value[0]>=4096-deadband)
//          high = 4096;

      ADCComparatorReset(ADC0_BASE, 0, 1, 1);
      ADCComparatorConfigure(ADC0_BASE, 0, ADC_COMP_INT_LOW_ALWAYS);// | ADC_COMP_INT_HIGH_ALWAYS );
      ADCComparatorRegionSet(ADC0_BASE, 0, low, high);//ADCComparatorRegionSet(ADC0_BASE, 0, x_axis_value[0]-deadband, x_axis_value[0]+deadband);

      old_low = low;
      old_high = high;
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
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH1 | ADC_CTL_CMP0);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1 | ADC_CTL_CMP0);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH1 | ADC_CTL_CMP0);
  ADCSequenceStepConfigure(ADC0_BASE,1,3, ADC_CTL_END | ADC_CTL_CH1 | ADC_CTL_CMP0);
  ADCSequenceEnable(ADC0_BASE, 1);

  ADCComparatorConfigure(ADC0_BASE, 0, ADC_COMP_INT_LOW_ALWAYS);// | ADC_COMP_INT_HIGH_ALWAYS );
  ADCComparatorRegionSet(ADC0_BASE, 0, 3072, 5000);
  ADCComparatorIntEnable(ADC0_BASE, 1);

  ADCIntRegister(ADC0_BASE, 1, xAxisMoved);
  //ADCIntEnable(ADC0_BASE, 3);
  ADCIntEnableEx(ADC0_BASE, ADC_INT_DCON_SS1);


  while (1)
  {
      ADCProcessorTrigger(ADC0_BASE, 1);
//      SysCtlDelay(1000);
//      uint32_t value[4] = {0,0,0,0};
//      int vals = ADCSequenceDataGet(ADC0_BASE, 1, value);
      SysCtlDelay(10);
  }
}
