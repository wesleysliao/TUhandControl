#include <stdbool.h>
#include <stdint.h>
#include <algorithm>
#include <queue>
#include <string>

// TivaC specific includes
extern "C"
{
  #include <driverlib/interrupt.h>
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
  #include <driverlib/adc.h>
  #include "inc/hw_ints.h"
  #include "driverlib/pin_map.h"
  #include "driverlib/timer.h"
  #include "driverlib/ssi.h"

  #include "inc/hw_types.h"
  #include "inc/hw_gpio.h"
  #include "inc/hw_memmap.h"

  #define TARGET_IS_BLIZZARD_RB1
  #include "driverlib/rom.h"
}
// extern "C"
// {

//   #include "driverlib/ssi.h"

//   #include <driverlib/gpio.h>

//   #include "inc/hw_types.h"
//   #include "inc/hw_gpio.h"
//   #include "inc/hw_memmap.h"

//   #define TARGET_IS_BLIZZARD_RB1
//   #include "driverlib/rom.h"
// }

//Control registers
#define  WR   0x0
#define  CR0  0x1
#define  CR1  0x2
#define  CR2  0x3
#define  CR3  0x9
//Status registers
#define  SR0  0x4
#define  SR1  0x5
#define  SR2  0x6
#define  SR3  0x7
#define  SR4  0xA

void SetupStepperSPIMaster(void){
	ROM_GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    ROM_GPIOPinConfigure(GPIO_PB7_SSI2TX);
    ROM_GPIOPinConfigure(GPIO_PB6_SSI2RX);
    ROM_GPIOPinTypeSSI(GPIO_PORTB_BASE,GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7);

    ROM_SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 800000, 16);
    ROM_SSIEnable(SSI2_BASE);

    uint32_t buf;
    while(ROM_SSIDataGetNonBlocking(SSI2_BASE, &buf) != 0); //clear spi fifo buffer
}


uint32_t SPIReadByte(uint32_t CSPort, uint8_t CSPins, uint8_t address);

void SPIWriteByte(uint32_t CSPort, uint8_t CSPins, uint8_t address, uint8_t data){
    ROM_GPIOPinWrite(CSPort, CSPins, 0); //Pull CS LOW

    uint32_t ui32Data = 0b1000000000000000 | (address << 8) | data;

    ROM_SSIDataPut(SSI2_BASE, ui32Data);

    while(ROM_SSIBusy(SSI2_BASE))
    {
    }

    ROM_GPIOPinWrite(CSPort, CSPins, 255); //Pull CS HIGH
}

uint32_t SPIReadByte(uint32_t CSPort, uint8_t CSPins, uint8_t address){
    ROM_GPIOPinWrite(CSPort, CSPins, 0); //Pull CS LOW

    uint32_t readRequest = (address << 8);
    uint32_t dataIn =0;

    ROM_SSIDataPut(SSI2_BASE, readRequest);

    while(ROM_SSIBusy(SSI2_BASE))
    {
    }

    while(ROM_SSIDataGetNonBlocking(SSI2_BASE, &dataIn) != 0);

    ROM_GPIOPinWrite(CSPort, CSPins, 255); //Pull CS HIGH

    return dataIn;
}


void ClearStepperRegisters(uint32_t CSPort, uint8_t CSPins){
    SPIWriteByte(CSPort, CSPins, WR, 0);
    SPIWriteByte(CSPort, CSPins, CR0, 0);
    SPIWriteByte(CSPort, CSPins, CR1, 0);
    SPIWriteByte(CSPort, CSPins, CR2, 0);
    SPIWriteByte(CSPort, CSPins, CR3, 0);
}

void SetStepperCurrent(uint32_t CSPort, uint8_t CSPins, uint16_t milliamps)
 {
     // This comes from Table 13 of the AMIS-30543 datasheet.
     uint8_t code = 0;
     if      (milliamps <= 3000) { code = 0b11001; }
     else if (milliamps <= 2845) { code = 0b11000; }
     else if (milliamps <= 2700) { code = 0b10111; }
     else if (milliamps <= 2440) { code = 0b10110; }
     else if (milliamps <= 2240) { code = 0b10101; }
     else if (milliamps <= 2070) { code = 0b10100; }
     else if (milliamps <= 1850) { code = 0b10011; }
     else if (milliamps <= 1695) { code = 0b10010; }
     else if (milliamps <= 1520) { code = 0b10001; }
     else if (milliamps <= 1405) { code = 0b10000; }
     else if (milliamps <= 1260) { code = 0b01111; }
     else if (milliamps <= 1150) { code = 0b01110; }
     else if (milliamps <= 1060) { code = 0b01101; }
     else if (milliamps <=  955) { code = 0b01100; }
     else if (milliamps <=  870) { code = 0b01011; }
     else if (milliamps <=  780) { code = 0b01010; }
     else if (milliamps <=  715) { code = 0b01001; }
     else if (milliamps <=  640) { code = 0b01000; }
     else if (milliamps <=  585) { code = 0b00111; }
     else if (milliamps <=  540) { code = 0b00110; }
     else if (milliamps <=  485) { code = 0b00101; }
     else if (milliamps <=  445) { code = 0b00100; }
     else if (milliamps <=  395) { code = 0b00011; }
     else if (milliamps <=  355) { code = 0b00010; }
     else if (milliamps <=  245) { code = 0b00001; }

     uint8_t CR0_reg = (SPIReadByte(CSPort, CSPins, CR0) & 0b11100000) | code;
     SPIWriteByte(CSPort, CSPins, CR0, CR0_reg);
 }

void SetStepperDirection(uint32_t CSPort, uint8_t CSPins, bool forward){

    uint8_t CR1_reg;

    if(forward){
        CR1_reg = (SPIReadByte(CSPort, CSPins, CR1) & 0b01111111);
    }
    else{
        CR1_reg = (SPIReadByte(CSPort, CSPins, CR1) | 0b10000000);
    }

    SPIWriteByte(CSPort, CSPins, CR1, CR1_reg);
}

void SPIStepperEnable(uint32_t CSPort, uint8_t CSPins){
    uint8_t CR2_reg = (SPIReadByte(CSPort, CSPins, CR2) | 0b10000000);
    SPIWriteByte(CSPort, CSPins, CR2, CR2_reg);
}

void SPIStepperDisable(uint32_t CSPort, uint8_t CSPins){
    SPIWriteByte(CSPort, CSPins, CR2, 0);
}

#define STEPMODE_MICRO_2        2
#define STEPMODE_MICRO_4        4
#define STEPMODE_MICRO_8        8
#define STEPMODE_MICRO_16       16
#define STEPMODE_MICRO_32       32
#define STEPMODE_MICRO_64       64
#define STEPMODE_MICRO_128      128
#define STEPMODE_COMP_HALF      2
#define STEPMODE_COMP_FULL_2PH  1
#define STEPMODE_COMP_FULL_1PH  200
#define STEPMODE_UNCOMP_HALF    201
#define STEPMODE_UNCOMP_FULL    202

void SetStepperStepMode(uint32_t CSPort, uint8_t CSPins, uint8_t stepmode){
    uint8_t sm = 0;
    uint8_t esm = 0;

    if      (stepmode == STEPMODE_MICRO_2)        { sm = 0b100; esm = 0b000; }
    else if (stepmode == STEPMODE_MICRO_4)        { sm = 0b011; esm = 0b000; }
    else if (stepmode == STEPMODE_MICRO_8)        { sm = 0b010; esm = 0b000; }
    else if (stepmode == STEPMODE_MICRO_16)       { sm = 0b001; esm = 0b000; }
    else if (stepmode == STEPMODE_MICRO_32)       { sm = 0b000; esm = 0b000; }
    else if (stepmode == STEPMODE_MICRO_64)       { sm = 0b000; esm = 0b010; }
    else if (stepmode == STEPMODE_MICRO_128)      { sm = 0b000; esm = 0b001; }
    else if (stepmode == STEPMODE_COMP_HALF)      { sm = 0b100; esm = 0b000; }
    else if (stepmode == STEPMODE_COMP_FULL_2PH)  { sm = 0b000; esm = 0b011; }
    else if (stepmode == STEPMODE_COMP_FULL_1PH)  { sm = 0b000; esm = 0b100; }
    else if (stepmode == STEPMODE_UNCOMP_HALF)    { sm = 0b101; esm = 0b000; }
    else if (stepmode == STEPMODE_UNCOMP_FULL)    { sm = 0b111; esm = 0b000; }

    uint8_t CR0_reg = (SPIReadByte(CSPort, CSPins, CR0) & 0b00011111) | (sm << 5);

    SPIWriteByte(CSPort, CSPins, CR0, CR0_reg);
    SPIWriteByte(CSPort, CSPins, CR3, esm);
}

std::string SPIStepperGetErrors(uint32_t CSPort, uint8_t CSPins)
{
    uint32_t sr0_stat = SPIReadByte(CSPort, CSPins, SR0);
    uint32_t sr1_stat = SPIReadByte(CSPort, CSPins, SR1);
    uint32_t sr2_stat = SPIReadByte(CSPort, CSPins, SR2);

    std::string errormsg;

    errormsg = "Error: ";

    if(sr0_stat & 0b01000000)
        errormsg.append("Temp Warning");
    if(sr2_stat & 0b00000100)
        errormsg.append("Temp Shutdown");
    if(sr0_stat & 0b00010000)
        errormsg.append("Watchdog ");
    if(sr0_stat & 0b00001100)
        errormsg.append("Open coil ");
    if((sr1_stat & 0b01111000 ) || (sr2_stat & 0b01111000 ))
        errormsg.append("Overcurrent ");

    return errormsg;
}
