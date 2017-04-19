#include <stdbool.h>
#include <stdint.h>
#include <algorithm>
#include <string>

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

    #include "inc/hw_types.h"
  #include "inc/hw_gpio.h"
  #include "inc/hw_memmap.h"
}

// ROS includes
#include <ros.h>

//message includes
#include "adc_joystick_msg/ADC_Joystick.h"
#include "stepper_msg/Stepper_Control.h"
#include "stepper_msg/Stepper_Status.h"

// ROS nodehandle
ros::NodeHandle nh;

//Pin definitions
#define GPIO_JOYSTICK_CLICK_PORT        GPIO_PORTA_BASE
#define GPIO_JOYSTICK_CLICK_PIN         GPIO_PIN_5

#define GPIO_JOYSTICK_X_AXIS_PORT       GPIO_PORTE_BASE
#define GPIO_JOYSTICK_X_AXIS_PIN        GPIO_PIN_2
#define ADC_CH_JOYSTICK_X_AXIS          ADC_CTL_CH1

#define GPIO_JOYSTICK_Y_AXIS_PORT       GPIO_PORTE_BASE
#define GPIO_JOYSTICK_Y_AXIS_PIN        GPIO_PIN_1
#define ADC_CH_JOYSTICK_Y_AXIS          ADC_CTL_CH2


#define GPIO_STEPPER_1_CS_PORT          GPIO_PORTD_BASE
#define GPIO_STEPPER_1_CS_PIN           GPIO_PIN_7
#define GPIO_STEPPER_1_STEP_PORT        GPIO_PORTD_BASE
#define GPIO_STEPPER_1_STEP_PIN         GPIO_PIN_6
#define GPIO_STEPPER_1_LIMIT_SW_PORT    GPIO_PORTA_BASE
#define GPIO_STEPPER_1_LIMIT_SW_PIN     GPIO_PIN_7

#define GPIO_STEPPER_2_CS_PORT          GPIO_PORTC_BASE
#define GPIO_STEPPER_2_CS_PIN           GPIO_PIN_7
#define GPIO_STEPPER_2_STEP_PORT        GPIO_PORTC_BASE
#define GPIO_STEPPER_2_STEP_PIN         GPIO_PIN_6
#define GPIO_STEPPER_2_LIMIT_SW_PORT    GPIO_PORTA_BASE
#define GPIO_STEPPER_2_LIMIT_SW_PIN     GPIO_PIN_6

#define GPIO_STEPPER_3_CS_PORT          GPIO_PORTA_BASE
#define GPIO_STEPPER_3_CS_PIN           GPIO_PIN_3
#define GPIO_STEPPER_3_STEP_PORT        GPIO_PORTA_BASE
#define GPIO_STEPPER_3_STEP_PIN         GPIO_PIN_4
#define GPIO_STEPPER_3_LIMIT_SW_PORT    GPIO_PORTA_BASE
#define GPIO_STEPPER_3_LIMIT_SW_PIN     GPIO_PIN_2

#define GPIO_STEPPER_4_CS_PORT          GPIO_PORTC_BASE
#define GPIO_STEPPER_4_CS_PIN           GPIO_PIN_5
#define GPIO_STEPPER_4_STEP_PORT        GPIO_PORTC_BASE
#define GPIO_STEPPER_4_STEP_PIN         GPIO_PIN_4
#define GPIO_STEPPER_4_LIMIT_SW_PORT    GPIO_PORTB_BASE
#define GPIO_STEPPER_4_LIMIT_SW_PIN     GPIO_PIN_3

#define GPIO_STEPPER_ALL_CLR_PORT       GPIO_PORTB_BASE
#define GPIO_STEPPER_ALL_CLR_PIN        GPIO_PIN_2

#define GPIO_STEPPER_ALL_ERR_PORT       GPIO_PORTE_BASE
#define GPIO_STEPPER_ALL_ERR_PIN        GPIO_PIN_0


#define DEFAULT_STEPPER_MAX_SPEED       2400
#define DEFAULT_STEPPER_ACCEL           100
#define DEFAULT_STEPPER_TRAVEL_LIMIT    400
#define DEFAULT_STEPPER_STEPMODE        2
#define DEFAULT_STEPPER_PH_CURRENT      800

#define PERIODIC_UPDATE_RATE_HZ 32


#include "include/tuhand_stepper.h"


//
// function declarations
//

void JoystickClicked(void);
void JoystickReleased(void);
void ReadADC(void);

void enableSysPeripherals(void);
void setupSharedStepperPins(void);
void setupJoystick(void);


void StepperGetParamsFromROS(Stepper &stepper);


//globals

int x_axis_zero;
int y_axis_zero;

Stepper Tendon1Stepper;
Stepper Tendon2Stepper;
Stepper WristStepper;

#define TENDON1_STEPPER 1
#define TENDON2_STEPPER 2
#define WRIST_STEPPER 3

void StepperControlHandler(const stepper_msg::Stepper_Control &msg){

  switch(msg.stepper_index){
    case TENDON1_STEPPER:
      StepperControlMode(Tendon1Stepper, msg);
      break;
    case TENDON2_STEPPER:
      StepperControlMode(Tendon2Stepper, msg);
      break;
    case WRIST_STEPPER:
      StepperControlMode(WristStepper, msg);
      break;
  }
}

ros::Publisher tendon1_status("TUhand/Tendon1Stepper/status", &Tendon1Stepper.status);
ros::Publisher tendon2_status("TUhand/Tendon2Stepper/status", &Tendon2Stepper.status);
ros::Publisher wrist_status("TUhand/WristStepper/status", &WristStepper.status);

ros::Subscriber<stepper_msg::Stepper_Control> stepper_control("TUhand/StepperControl", &StepperControlHandler);

adc_joystick_msg::ADC_Joystick js_msg;
ros::Publisher adc_joystick("adc_joystick", &js_msg);


void JoystickClicked(void) {
    if (GPIOIntStatus(GPIO_JOYSTICK_CLICK_PORT, false) & GPIO_JOYSTICK_CLICK_PIN) {
        // PA5 was interrupt cause
        js_msg.select = true;
        //adc_joystick.publish(&js_msg);
        GPIOIntRegister(GPIO_JOYSTICK_CLICK_PORT, JoystickReleased);   // Register our handler function for port A
        GPIOIntTypeSet(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN,
            GPIO_RISING_EDGE);          // Configure PA5 for rising edge trigger
        GPIOIntClear(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN);  // Clear interrupt flag
    }
}

void JoystickReleased(void) {
    if (GPIOIntStatus(GPIO_JOYSTICK_CLICK_PORT, false) & GPIO_JOYSTICK_CLICK_PIN) {
        // PA5 was interrupt cause
        js_msg.select = false;
        //adc_joystick.publish(&js_msg);
        GPIOIntRegister(GPIO_JOYSTICK_CLICK_PORT, JoystickClicked); // Register our handler function for port F
        GPIOIntTypeSet(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN,
            GPIO_FALLING_EDGE);         // Configure PF4 for falling edge trigger
        GPIOIntClear(GPIO_JOYSTICK_CLICK_PORT, GPIO_JOYSTICK_CLICK_PIN);  // Clear interrupt flag
    }
}


void SW1_SW2_pressed(void){
    if (GPIOIntStatus(GPIO_PORTF_BASE, false) & GPIO_PIN_0) {

        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);  // Clear interrupt flag

        // StepperEnable(Tendon1Stepper);
        // StepperEnable(Tendon2Stepper);
        // StepperEnable(WristStepper);
    }
    else if(GPIOIntStatus(GPIO_PORTF_BASE, false) & GPIO_PIN_4) {

        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);  // Clear interrupt flag

        // StepperDisable(Tendon1Stepper);
        // StepperDisable(Tendon2Stepper);
        // StepperDisable(WristStepper);

    }
}


void StepperError(void){
    if (GPIOIntStatus(GPIO_STEPPER_ALL_ERR_PORT, false) & GPIO_STEPPER_ALL_ERR_PIN) {

        GPIOIntClear(GPIO_STEPPER_ALL_ERR_PORT, GPIO_STEPPER_ALL_ERR_PIN);  // Clear interrupt flag
        
        StepperDisable(Tendon1Stepper);
        StepperDisable(Tendon2Stepper);
        StepperDisable(WristStepper);

        StepperReadErrors(Tendon1Stepper);
        StepperReadErrors(Tendon2Stepper);
        StepperReadErrors(WristStepper);

        if(nh.connected())
        {
          tendon1_status.publish(&Tendon1Stepper.status);
          tendon2_status.publish(&Tendon2Stepper.status);
          wrist_status.publish(&WristStepper.status);
        }

    }
}


void ReadADC(void){

  ADCIntClear(ADC0_BASE,1);

  uint32_t adc_values[8] = {0,0,0,0,0,0,0,0};

  ADCSequenceDataGet(ADC0_BASE, 1, adc_values);

  int x = ((((int)(adc_values[0]+adc_values[1])/2)-x_axis_zero)*1000)/2048;
  int y = ((((int)(adc_values[2]+adc_values[3])/2)-y_axis_zero)*1000)/2048;

  if( abs(x - js_msg.x_axis_1000) > 0 || abs(y - js_msg.y_axis_1000) > 0)
  {
      js_msg.x_axis_1000 = x;
      js_msg.y_axis_1000 = y;

      if(nh.connected())
        {
          adc_joystick.publish(&js_msg);
        }
      

      // Tendon1Stepper.target_speed = (Tendon1Stepper.max_speed_steps_per_second*(js_msg.x_axis_1000-x_axis_zero))/2048;
      // Tendon2Stepper.target_speed = (Tendon2Stepper.max_speed_steps_per_second*(js_msg.y_axis_1000-y_axis_zero))/2048;

      // WristStepper.target_speed = (WristStepper.max_speed_steps_per_second*(js_msg.x_axis_1000-x_axis_zero))/2048;

  }
}


void PeriodicUpdate(void){

  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

  StepperControlSpeed(Tendon1Stepper, js_msg.x_axis_1000, js_msg.y_axis_1000);
  StepperControlSpeed(Tendon2Stepper, js_msg.x_axis_1000, js_msg.y_axis_1000);
  StepperControlSpeed(WristStepper, js_msg.x_axis_1000, js_msg.y_axis_1000);

  StepperUpdate(Tendon1Stepper);
  StepperUpdate(Tendon2Stepper);
  StepperUpdate(WristStepper);

  if(nh.connected())
  {
    tendon1_status.publish(&Tendon1Stepper.status);
    tendon2_status.publish(&Tendon2Stepper.status);
    wrist_status.publish(&WristStepper.status);
  }

  nh.spinOnce();

  ADCProcessorTrigger(ADC0_BASE, 1);
}

void Tendon1StepperStepHandler(void)
{
  StepperStepPinSet(Tendon1Stepper);
}

void Tendon2StepperStepHandler(void)
{
  StepperStepPinSet(Tendon2Stepper);
}

void WristStepperStepHandler(void)
{
  StepperStepPinSet(WristStepper);
}


int main(void)
{
    // TivaC application specific code
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    // TivaC system clock configuration. Set to 80MHz.
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    IntMasterDisable();

    enableSysPeripherals();

    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor
    GPIOIntRegister(GPIO_PORTF_BASE, SW1_SW2_pressed);     // Register our handler function for port A
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_RISING_EDGE);         // Configure PF4 for falling edge trigger
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);     // Enable interrupt for PF4

    setupJoystick();
    setupSharedStepperPins();

    nh.initNode();

    nh.advertise(adc_joystick);
    nh.spinOnce();

    nh.advertise(tendon1_status);
    nh.advertise(tendon2_status);
    nh.advertise(wrist_status);

    nh.subscribe(stepper_control);
    //nh.subscribe(tendon2_control);
    //nh.subscribe(wrist_control);

    while(!nh.connected()) {nh.spinOnce();}

    // nh.getParam("/ADC_Joystick/x_axis_zero", &x_axis_zero, 1);
    // nh.getParam("/ADC_Joystick/y_axis_zero", &y_axis_zero, 1);

    Tendon1Stepper.name = std::string("Tendon1Stepper");
    Tendon1Stepper.ChipSelectPin.PIN = GPIO_STEPPER_1_CS_PIN;
    Tendon1Stepper.ChipSelectPin.PORT = GPIO_STEPPER_1_CS_PORT;
    Tendon1Stepper.StepPin.PIN = GPIO_STEPPER_1_STEP_PIN;
    Tendon1Stepper.StepPin.PORT = GPIO_STEPPER_1_STEP_PORT;
    Tendon1Stepper.LimitSwitchPin.PIN = GPIO_STEPPER_1_LIMIT_SW_PIN;
    Tendon1Stepper.LimitSwitchPin.PORT = GPIO_STEPPER_1_LIMIT_SW_PORT;
    Tendon1Stepper.TIMER_BASE = TIMER2_BASE;
    Tendon1Stepper.status.position_steps = 0;
    Tendon1Stepper.status.speed_steps_per_second = 1;
    Tendon1Stepper.status.enabled = false;
    Tendon1Stepper.control.control_mode = 0;
    Tendon1Stepper.target_speed   = 2000;
    StepperGetParamsFromROS(Tendon1Stepper);
    StepperInitGPIO(Tendon1Stepper);
    StepperInitSPI(Tendon1Stepper);    
    StepperInitTimer(Tendon1StepperStepHandler, Tendon1Stepper);


    Tendon2Stepper.name = std::string("Tendon2Stepper");
    Tendon2Stepper.ChipSelectPin.PIN = GPIO_STEPPER_2_CS_PIN;
    Tendon2Stepper.ChipSelectPin.PORT = GPIO_STEPPER_2_CS_PORT;
    Tendon2Stepper.StepPin.PIN = GPIO_STEPPER_2_STEP_PIN;
    Tendon2Stepper.StepPin.PORT = GPIO_STEPPER_2_STEP_PORT;
    Tendon2Stepper.LimitSwitchPin.PIN = GPIO_STEPPER_2_LIMIT_SW_PIN;
    Tendon2Stepper.LimitSwitchPin.PORT = GPIO_STEPPER_2_LIMIT_SW_PORT;
    Tendon2Stepper.TIMER_BASE = TIMER3_BASE;
    Tendon2Stepper.status.position_steps = 0;
    Tendon2Stepper.status.speed_steps_per_second = 1;
    Tendon2Stepper.status.enabled = false;
    Tendon2Stepper.control.control_mode = 0;
    Tendon2Stepper.target_speed   = 2000;
    StepperGetParamsFromROS(Tendon2Stepper);
    StepperInitGPIO(Tendon2Stepper);
    StepperInitSPI(Tendon2Stepper);    
    StepperInitTimer(Tendon2StepperStepHandler, Tendon2Stepper);


    WristStepper.name = std::string("WristStepper");
    WristStepper.ChipSelectPin.PIN = GPIO_STEPPER_3_CS_PIN;
    WristStepper.ChipSelectPin.PORT = GPIO_STEPPER_3_CS_PORT;
    WristStepper.StepPin.PIN = GPIO_STEPPER_3_STEP_PIN;
    WristStepper.StepPin.PORT = GPIO_STEPPER_3_STEP_PORT;
    WristStepper.LimitSwitchPin.PIN = GPIO_STEPPER_3_LIMIT_SW_PIN;
    WristStepper.LimitSwitchPin.PORT = GPIO_STEPPER_3_LIMIT_SW_PORT;
    WristStepper.TIMER_BASE = TIMER4_BASE;
    WristStepper.status.position_steps = 0;
    WristStepper.status.speed_steps_per_second = 1;
    WristStepper.status.enabled = false;
    WristStepper.control.control_mode = 0;
    WristStepper.target_speed   = 2000;
    StepperGetParamsFromROS(WristStepper);
    StepperInitGPIO(WristStepper);
    StepperInitSPI(WristStepper);    
    StepperInitTimer(WristStepperStepHandler, WristStepper);


    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerDisable(TIMER1_BASE, TIMER_A);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); //Periodic tasks
    TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT); //stepper pin reset

    TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet() / PERIODIC_UPDATE_RATE_HZ)-1);

    uint32_t StepPinResetDelay_us = 2; //microseconds
    TimerLoadSet(TIMER1_BASE, TIMER_A, (StepPinResetDelay_us*(SysCtlClockGet()/1000000))-1);

    TimerIntRegister(TIMER0_BASE, TIMER_A, PeriodicUpdate);
    TimerIntRegister(TIMER1_BASE, TIMER_A, StepPinReset);
    
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);



    IntPrioritySet(INT_TIMER1A, 0b01000000); //Motor reset
    IntPrioritySet(INT_TIMER0A, 0b00100000); //update

    IntPrioritySet(INT_TIMER2A, 0b00000000); //motor set
    IntPrioritySet(INT_TIMER3A, 0b00000000);
    IntPrioritySet(INT_TIMER4A, 0b00000000);
    IntPrioritySet(INT_TIMER5A, 0b00000000);


    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_A);

    IntMasterEnable();

    StepperDisable(Tendon1Stepper);
    StepperDisable(Tendon2Stepper);
    StepperDisable(WristStepper);

    while(1)
    {
    }
}

//
// Initialization functions
//

void enableSysPeripherals(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
  {
  }

  HWREG(GPIO_PORTC_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;  //unlock pins C 0-3 for use
  HWREG(GPIO_PORTC_BASE+GPIO_O_CR) |= (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
  
  HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY; //Unlock pin D7 for use
  HWREG(GPIO_PORTD_BASE+GPIO_O_CR) |= GPIO_PIN_7;

  HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY; //unlock pin F0 for use
  HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= GPIO_PIN_0;
}

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


  x_axis_zero = 2085;
  y_axis_zero = 2066;
}

void setupSharedStepperPins(void)
{
    GPIOPinTypeGPIOInput(GPIO_STEPPER_ALL_ERR_PORT, GPIO_STEPPER_ALL_ERR_PIN);
    GPIOPadConfigSet(GPIO_STEPPER_ALL_ERR_PORT, GPIO_STEPPER_ALL_ERR_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor
    GPIOIntRegister(GPIO_STEPPER_ALL_ERR_PORT, StepperError);     // Register our handler function for port A
    GPIOIntTypeSet(GPIO_STEPPER_ALL_ERR_PORT, GPIO_STEPPER_ALL_ERR_PIN, GPIO_FALLING_EDGE);         // Configure PF4 for falling edge trigger
    GPIOIntEnable(GPIO_STEPPER_ALL_ERR_PORT, GPIO_STEPPER_ALL_ERR_PIN);     // Enable interrupt for PF4

    GPIOPinTypeGPIOOutput(GPIO_STEPPER_ALL_CLR_PORT, GPIO_STEPPER_ALL_CLR_PIN);
    GPIOPadConfigSet(GPIO_STEPPER_ALL_CLR_PORT, GPIO_STEPPER_ALL_CLR_PIN,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);

    SetupStepperSPIMaster();

    GPIOPinWrite(GPIO_STEPPER_ALL_CLR_PORT, GPIO_STEPPER_ALL_CLR_PIN, 255); //Pull CLR HIGH to reset all motor drivers
    SysCtlDelay(1000);
    GPIOPinWrite(GPIO_STEPPER_ALL_CLR_PORT, GPIO_STEPPER_ALL_CLR_PIN, 0); //Pull CLR LOW
}


void StepperGetParamsFromROS(Stepper &stepper){
  std::string paramname = std::string("/TUhand/");
  paramname.append(stepper.name);

  int prefix = paramname.length();

  paramname.append("/max_speed_steps_per_second");

  if(!nh.getParam(paramname.c_str(), &stepper.max_speed_steps_per_second, 1))
    stepper.max_speed_steps_per_second = DEFAULT_STEPPER_MAX_SPEED;

  paramname.erase(prefix, std::string::npos);
  paramname.append("/travel_limit_steps");

  if(!nh.getParam(paramname.c_str(), &stepper.travel_limit_steps, 1))
    stepper.travel_limit_steps = DEFAULT_STEPPER_TRAVEL_LIMIT;

  paramname.erase(prefix, std::string::npos);
  paramname.append("/acceleration");

  if(!nh.getParam(paramname.c_str(), &stepper.acceleration, 1))
    stepper.acceleration = DEFAULT_STEPPER_ACCEL;
  
  paramname.erase(prefix, std::string::npos);
  paramname.append("/microstep_mode");

  if(!nh.getParam(paramname.c_str(), &stepper.microstep_mode, 1))
    stepper.microstep_mode = DEFAULT_STEPPER_STEPMODE;

  paramname.erase(prefix, std::string::npos);
  paramname.append("/phase_current_ma");

  if(!nh.getParam(paramname.c_str(), &stepper.phase_current_ma, 1))
    stepper.phase_current_ma = DEFAULT_STEPPER_PH_CURRENT;
}
