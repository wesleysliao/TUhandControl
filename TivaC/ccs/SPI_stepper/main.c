#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

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

uint8_t WR_reg = 0;
uint8_t CR0_reg = 0;
uint8_t CR1_reg = 0;
uint8_t CR2_reg = 0;
uint8_t CR3_reg = 0;

// Bit-wise reverses a number.
uint8_t Reverse8(uint8_t ui8Number)
{
    uint8_t ui8Index;
    uint8_t ui8ReversedNumber = 0;
    for(ui8Index=0; ui8Index<8; ui8Index++)
    {
        ui8ReversedNumber = ui8ReversedNumber << 1;
        ui8ReversedNumber |= ((1 << ui8Index) & ui8Number) >> ui8Index;
    }
    return ui8ReversedNumber;
}


uint16_t Reverse16(uint16_t ui16Number)
{
    uint16_t ui16Index;
    uint16_t ui16ReversedNumber = 0;
    for(ui16Index=0; ui16Index<16; ui16Index++)
    {
        ui16ReversedNumber = ui16ReversedNumber << 1;
        ui16ReversedNumber |= ((1 << ui16Index) & ui16Number) >> ui16Index;
    }
    return ui16ReversedNumber;
}

#define GPIO_SPI_CS_STEPPER_1_PORT GPIO_PORTA_BASE
#define GPIO_SPI_CS_STEPPER_1_PIN  GPIO_PIN_4

uint32_t SPIReadByte(uint8_t address);

void SPIWriteByte(uint8_t address, uint8_t data){
    GPIOPinWrite(GPIO_SPI_CS_STEPPER_1_PORT, GPIO_SPI_CS_STEPPER_1_PIN, 0); //Pull CS LOW

    uint32_t ui32Data = 0b1000000000000000 | (address << 8) | data;

    SSIDataPut(SSI2_BASE, ui32Data);

    while(SSIBusy(SSI2_BASE))
    {
    }

    GPIOPinWrite(GPIO_SPI_CS_STEPPER_1_PORT, GPIO_SPI_CS_STEPPER_1_PIN, 255); //Pull CS HIGH

    SysCtlDelay(50);


    SPIReadByte(address);
}

uint32_t SPIReadByte(uint8_t address){
    GPIOPinWrite(GPIO_SPI_CS_STEPPER_1_PORT, GPIO_SPI_CS_STEPPER_1_PIN, 0); //Pull CS LOW

    uint32_t readRequest = (address << 8);
    uint32_t dataIn =0;

    SSIDataPut(SSI2_BASE, readRequest);

    while(SSIBusy(SSI2_BASE))
    {
    }

    while(SSIDataGetNonBlocking(SSI2_BASE, &dataIn) != 0);

    GPIOPinWrite(GPIO_SPI_CS_STEPPER_1_PORT, GPIO_SPI_CS_STEPPER_1_PIN, 255); //Pull CS HIGH

    return dataIn;
}


void ClearStepperRegisters(void){
    WR_reg = 0;
    CR0_reg = 0;
    CR1_reg = 0;
    CR2_reg = 0;
    CR3_reg = 0;

    SPIWriteByte(WR, WR_reg);
    SPIWriteByte(CR0, CR0_reg);
    SPIWriteByte(CR1, CR1_reg);
    SPIWriteByte(CR2, CR2_reg);
    SPIWriteByte(CR3, CR3_reg);
}

void SetStepperCurrent(uint16_t milliamps)
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

     CR0_reg = (CR0_reg & 0b11100000) | code;
     SPIWriteByte(CR0, CR0_reg);
 }

void SetStepperDirection(bool forward){

    if(forward){
        CR1_reg = (CR1_reg & 0b01111111);
    }
    else{
        CR1_reg = (CR1_reg | 0b10000000);
    }

    SPIWriteByte(CR1, CR1_reg);
}

void StepperEnable(void){
    CR2_reg = (CR2_reg | 0b10000000);
    SPIWriteByte(CR2, CR2_reg);
}
void StepperDisable(void){
    CR2_reg = 0;
    SPIWriteByte(CR2, CR2_reg);
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

void SetStepperStepMode(uint8_t stepmode){
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

    CR0_reg = (CR0_reg & 0b00011111) | (sm << 5);
    CR3_reg = esm;

    SPIWriteByte(CR0, CR0_reg);
    SPIWriteByte(CR3, CR3_reg);
}

int main(void)
{
    // TivaC system clock configuration. Set to 80MHz.
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_2);
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);

    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinTypeSSI(GPIO_PORTB_BASE,GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7);

    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 100000, 16);
    SSIEnable(SSI2_BASE);

    GPIOPinWrite(GPIO_SPI_CS_STEPPER_1_PORT, GPIO_SPI_CS_STEPPER_1_PIN, 255); //Pull CS HIGH

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 255); //Pull CLR HIGH
    SysCtlDelay(1000);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0); //Pull CLR LOW

    uint32_t buf;
    while(SSIDataGetNonBlocking(SSI2_BASE, &buf) != 0); //clear spi fifo buffer


    ClearStepperRegisters();
    SetStepperCurrent(2800);
    SetStepperStepMode(STEPMODE_MICRO_8);
    StepperEnable();


    while(1)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 255);
        SysCtlDelay(10000);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
        SysCtlDelay(10000);

        uint32_t sr0_stat = SPIReadByte(SR0);
        uint32_t sr1_stat = SPIReadByte(SR1);
        uint32_t sr2_stat = SPIReadByte(SR2);

        if(((sr0_stat | sr1_stat | sr2_stat ) & 0b1111111) > 0){

            StepperDisable();

            uint32_t sr3_stat = SPIReadByte(SR3);
            uint32_t sr4_stat = SPIReadByte(SR4);
            //error detected
            SysCtlDelay(1);

            StepperEnable();
        }

    }
}
