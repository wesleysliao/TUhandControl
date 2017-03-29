#include <stdbool.h>
#include <stdint.h>
#include <algorithm>
#include <queue>

// TivaC specific includes
extern "C"
{
  #include <driverlib/interrupt.h>
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
  #include <driverlib/adc.h>
  #include "inc/hw_ints.h"
  #include "driverlib/ssi.h"
  #include "driverlib/pin_map.h"
  #include "driverlib/timer.h"
}

#include "include/tivacpin.h"

// ROS includes
#include <ros.h>

//message includes
#include "adc_joystick_msg/ADC_Joystick.h"
#include "stepper_msg/Stepper_Target.h"
#include "stepper_msg/Stepper_Status.h"

// ROS nodehandle
ros::NodeHandle nh;

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


//Pin definitions
#define GPIO_JOYSTICK_CLICK_PORT        GPIO_PORTA_BASE
#define GPIO_JOYSTICK_CLICK_PIN         GPIO_PIN_5

#define GPIO_JOYSTICK_X_AXIS_PORT       GPIO_PORTE_BASE
#define GPIO_JOYSTICK_X_AXIS_PIN        GPIO_PIN_2
#define ADC_CH_JOYSTICK_X_AXIS          ADC_CTL_CH1

#define GPIO_JOYSTICK_Y_AXIS_PORT       GPIO_PORTE_BASE
#define GPIO_JOYSTICK_Y_AXIS_PIN        GPIO_PIN_1
#define ADC_CH_JOYSTICK_Y_AXIS          ADC_CTL_CH2

#define GPIO_STEPPER_1_CS_PORT          GPIO_PORTA_BASE
#define GPIO_STEPPER_1_CS_PIN           GPIO_PIN_4
#define GPIO_STEPPER_1_STEP_PORT        GPIO_PORTB_BASE
#define GPIO_STEPPER_1_STEP_PIN         GPIO_PIN_3

#define GPIO_STEPPER_CLRALL_PORT        GPIO_PORTE_BASE
#define GPIO_STEPPER_CLRALL_PIN         GPIO_PIN_0


bool SameSign(int x, int y)
{
    return (x >= 0) ^ (y < 0);
}


void enableSysPeripherals(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
  {
  }
}

void StepperEnable(void);
void StepperDisable(void);
//
//  Joystick code
// 

//globals

int32_t Stepper1_min_speed     = 1;
int32_t Stepper1_max_speed     = 96000;
int32_t Stepper1_accel         = 4;
int32_t Stepper1_target_speed   = 24000;


stepper_msg::Stepper_Status stepper_status_msg;
ros::Publisher stepper_status("stepper_status", &stepper_status_msg);


adc_joystick_msg::ADC_Joystick js_msg;
ros::Publisher adc_joystick("adc_joystick", &js_msg);

//Handlers
void JoystickClicked(void);
void JoystickReleased(void);
void ReadADC(void);

//methods
void setupJoystick(void)
{
  //Setup Joystick click
  GPIOPinTypeGPIOInput(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN);
  GPIOPadConfigSet(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor

  //Joystick click Interrupt setup
  GPIOIntDisable(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN);        // Disable interrupt for PA5 (in case it was enabled)
  GPIOIntClear(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN);      // Clear pending interrupts for PA5
  GPIOIntRegister(GPIO_JOYSTICK_CLICK_PORT, JoystickClicked);     // Register our handler function for port A
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

  ADCIntRegister(ADC0_BASE, 1, ReadADC);
  ADCIntEnable(ADC0_BASE, 1);


  js_msg.x_axis_zero = 2048;
  js_msg.y_axis_zero = 2048;
}

void JoystickClicked(void) {
    if (GPIOIntStatus(GPIO_JOYSTICK_CLICK_PORT, false) & GPIO_JOYSTICK_CLICK_PIN) {
        // PA5 was interrupt cause
        js_msg.select = true;
        adc_joystick.publish(&js_msg);
        GPIOIntRegister(GPIO_JOYSTICK_CLICK_PORT, JoystickReleased);   // Register our handler function for port A
        GPIOIntTypeSet(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN,
            GPIO_RISING_EDGE);          // Configure PA5 for rising edge trigger
        GPIOIntClear(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN);  // Clear interrupt flag

        if(stepper_status_msg.enabled)
            StepperDisable();
        else
            StepperEnable();
    }
}

void JoystickReleased(void) {
    if (GPIOIntStatus(GPIO_JOYSTICK_CLICK_PORT, false) & GPIO_JOYSTICK_CLICK_PIN) {
        // PA5 was interrupt cause
        js_msg.select = false;
        adc_joystick.publish(&js_msg);
        GPIOIntRegister(GPIO_JOYSTICK_CLICK_PORT, JoystickClicked); // Register our handler function for port F
        GPIOIntTypeSet(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN,
            GPIO_FALLING_EDGE);         // Configure PF4 for falling edge trigger
        GPIOIntClear(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN);  // Clear interrupt flag
    }
}

void ReadADC(void){

  ADCIntClear(ADC0_BASE,1);

  uint32_t adc_values[8] = {0,0,0,0,0,0,0,0};

  ADCSequenceDataGet(ADC0_BASE, 1, adc_values);

  if( abs(adc_values[0] - js_msg.x_axis_raw) > 100 || abs(adc_values[2] - js_msg.y_axis_raw) > 100)
  {
      js_msg.x_axis_raw = adc_values[0];
      js_msg.y_axis_raw = adc_values[2];

      adc_joystick.publish(&js_msg);

      Stepper1_target_speed = (Stepper1_max_speed*(js_msg.x_axis_raw-js_msg.x_axis_zero))/2048;
  }
}


uint32_t SPIReadByte(uint8_t address);

void SPIWriteByte(uint8_t address, uint8_t data){
    GPIOPinWrite(GPIO_STEPPER_1_CS_PORT, GPIO_STEPPER_1_CS_PIN, 0); //Pull CS LOW

    uint32_t ui32Data = 0b1000000000000000 | (address << 8) | data;

    SSIDataPut(SSI2_BASE, ui32Data);

    while(SSIBusy(SSI2_BASE))
    {
    }

    GPIOPinWrite(GPIO_STEPPER_1_CS_PORT, GPIO_STEPPER_1_CS_PIN, 255); //Pull CS HIGH
}

uint32_t SPIReadByte(uint8_t address){
    GPIOPinWrite(GPIO_STEPPER_1_CS_PORT, GPIO_STEPPER_1_CS_PIN, 0); //Pull CS LOW

    uint32_t readRequest = (address << 8);
    uint32_t dataIn =0;

    SSIDataPut(SSI2_BASE, readRequest);

    while(SSIBusy(SSI2_BASE))
    {
    }

    while(SSIDataGetNonBlocking(SSI2_BASE, &dataIn) != 0);

    GPIOPinWrite(GPIO_STEPPER_1_CS_PORT, GPIO_STEPPER_1_CS_PIN, 255); //Pull CS HIGH

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

    stepper_status_msg.direction_forward = forward;
    stepper_status.publish(&stepper_status_msg);

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

    stepper_status_msg.enabled = true;
    TimerEnable(TIMER0_BASE, TIMER_B);
}
void StepperDisable(void){
    CR2_reg = 0;
    SPIWriteByte(CR2, CR2_reg);

    stepper_status_msg.enabled = false;
    stepper_status_msg.speed_steps_per_second = 0;
    TimerDisable(TIMER0_BASE, TIMER_B);
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

void ScheduleStepPinReset(){

    TimerEnable(TIMER0_BASE, TIMER_A); //Enable reset
}



#define STEPPER_ACCEL_INTERVAL 2

void UpdateSteppers(void){

    TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    if(stepper_status_msg.speed_steps_per_second != Stepper1_target_speed)
    {
        int original_speed = stepper_status_msg.speed_steps_per_second;

        if(stepper_status_msg.speed_steps_per_second < Stepper1_target_speed)
        {
            stepper_status_msg.speed_steps_per_second += Stepper1_accel;

        }
        else{
            stepper_status_msg.speed_steps_per_second -= Stepper1_accel;
        }

        if(!SameSign(original_speed, stepper_status_msg.speed_steps_per_second)){
            if(stepper_status_msg.speed_steps_per_second>=0){
                SetStepperDirection(true);
            }else{
                SetStepperDirection(false);
            }
        }

        TimerLoadSet(TIMER1_BASE, TIMER_A, std::min((SysCtlClockGet() / abs(stepper_status_msg.speed_steps_per_second)) -1, SysCtlClockGet()/STEPPER_ACCEL_INTERVAL));
    }

    stepper_status.publish(&stepper_status_msg);
}

void Stepper1StepPinSet(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    GPIOPinWrite(GPIO_STEPPER_1_STEP_PORT, GPIO_STEPPER_1_STEP_PIN, 255);

    if(stepper_status_msg.direction_forward)
        stepper_status_msg.position_steps++;
    else
        stepper_status_msg.position_steps--;


    ScheduleStepPinReset();
}

void StepPinReset(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    GPIOPinWrite(GPIO_STEPPER_1_STEP_PORT, GPIO_STEPPER_1_STEP_PIN, 0);
}

void SW1_SW2_pressed(void){
    if (GPIOIntStatus(GPIO_PORTF_BASE, false) & GPIO_PIN_0) {

        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);  // Clear interrupt flag
    }
    else if(GPIOIntStatus(GPIO_PORTF_BASE, false) & GPIO_PIN_4) {

        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);  // Clear interrupt flag

        ADCProcessorTrigger(ADC0_BASE, 1);

        while(ADCBusy(ADC0_BASE));
         
        uint32_t adc_values[8] = {0,0,0,0,0,0,0,0};

        ADCSequenceDataGet(ADC0_BASE, 1, adc_values);

        js_msg.x_axis_zero = adc_values[0];
        js_msg.y_axis_zero = adc_values[2];

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

    setupJoystick();

    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor
    GPIOIntRegister(GPIO_PORTF_BASE, SW1_SW2_pressed);     // Register our handler function for port A
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_RISING_EDGE);         // Configure PF4 for falling edge trigger
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);     // Enable interrupt for PF4

    GPIOPinTypeGPIOOutput(GPIO_STEPPER_CLRALL_PORT, GPIO_STEPPER_CLRALL_PIN);
    GPIOPadConfigSet(GPIO_STEPPER_CLRALL_PORT, GPIO_STEPPER_CLRALL_PIN,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOOutput(GPIO_STEPPER_1_STEP_PORT, GPIO_STEPPER_1_STEP_PIN);
    GPIOPadConfigSet(GPIO_STEPPER_1_STEP_PORT, GPIO_STEPPER_1_STEP_PIN, GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOOutput(GPIO_STEPPER_1_CS_PORT, GPIO_STEPPER_1_CS_PIN);
    GPIOPadConfigSet(GPIO_STEPPER_1_CS_PORT, GPIO_STEPPER_1_CS_PIN,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);

    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinTypeSSI(GPIO_PORTB_BASE,GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7);

    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 100000, 16);
    SSIEnable(SSI2_BASE);

    GPIOPinWrite(GPIO_STEPPER_1_CS_PORT, GPIO_STEPPER_1_CS_PIN, 255); //Pull CS HIGH

    GPIOPinWrite(GPIO_STEPPER_CLRALL_PORT, GPIO_STEPPER_CLRALL_PIN, 255); //Pull CLR HIGH
    SysCtlDelay(1000);
    GPIOPinWrite(GPIO_STEPPER_CLRALL_PORT, GPIO_STEPPER_CLRALL_PIN, 0); //Pull CLR LOW

    uint32_t buf;
    while(SSIDataGetNonBlocking(SSI2_BASE, &buf) != 0); //clear spi fifo buffer


    nh.initNode();
     
    nh.advertise(adc_joystick);
    nh.advertise(stepper_status);

    ClearStepperRegisters();
    SetStepperCurrent(280);
    SetStepperStepMode(STEPMODE_MICRO_16);
    SetStepperDirection(true);


    stepper_status_msg.position_steps = 0;
    stepper_status_msg.speed_steps_per_second = 1;
    stepper_status_msg.enabled = false;
    Stepper1_min_speed     = 1;
    Stepper1_max_speed     = 8000;
    Stepper1_accel         = 10;
    Stepper1_target_speed   = 2000;


    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    TimerDisable(TIMER0_BASE, TIMER_A|TIMER_B);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR| TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_PERIODIC);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

    //stepper_status_msg.speed_steps_per_second = 3200;
    TimerLoadSet(TIMER1_BASE, TIMER_A, (SysCtlClockGet() / stepper_status_msg.speed_steps_per_second) -1);
    TimerUpdateMode(TIMER1_BASE, TIMER_A, TIMER_UP_LOAD_TIMEOUT);

    uint32_t StepPinResetDelay_us = 2; //microseconds
    TimerLoadSet(TIMER0_BASE, TIMER_A, (StepPinResetDelay_us*(SysCtlClockGet()/1000000))-1);

    TimerLoadSet(TIMER0_BASE, TIMER_B, (SysCtlClockGet() / STEPPER_ACCEL_INTERVAL)-1);

    TimerIntRegister(TIMER1_BASE, TIMER_A, Stepper1StepPinSet);
    TimerIntRegister(TIMER0_BASE, TIMER_A, StepPinReset);
    TimerIntRegister(TIMER0_BASE, TIMER_B, UpdateSteppers);

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    IntEnable(INT_TIMER0B);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    IntMasterEnable();

    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_A);



    while(1)
    {
        // uint32_t sr0_stat = SPIReadByte(SR0);
        // uint32_t sr1_stat = SPIReadByte(SR1);
        // uint32_t sr2_stat = SPIReadByte(SR2);

        // if(((sr0_stat | sr1_stat | sr2_stat ) & 0b1111111) > 0){
        //     StepperDisable();
        // }

        ADCProcessorTrigger(ADC0_BASE, 1);

        nh.spinOnce();
        nh.getHardware()->delay(50);
    }
}