extern "C"
{
  #include <driverlib/interrupt.h>
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>

  #include "driverlib/pin_map.h"
  #include "driverlib/timer.h"
  
  #include "inc/hw_ints.h"
  #include "inc/hw_types.h"
  #include "inc/hw_gpio.h"
  #include "inc/hw_memmap.h"
}

#include <queue>

// ROS includes
#include <ros.h>

//message includes
#include "stepper_msg/Stepper_Target.h"
#include "stepper_msg/Stepper_Status.h"


#include "spi_AMIS_30543_stepper.h"
#include "tivac_pin.h"



struct Stepper {
  std::string name;

  int max_speed_steps_per_second;
  int travel_limit_steps;
  int acceleration;
  int microstep_mode;
  int phase_current_ma;

  stepper_msg::Stepper_Status status;
  int target_speed;

  uint32_t TIMER_BASE;
  TivaC_Pin ChipSelectPin;
  TivaC_Pin StepPin;
};


bool SameSign(int x, int y)
{
    return (x >= 0) ^ (y < 0);
}



//
// Stepper control functions
//

void StepperEnable(Stepper &stepper){
    SPIStepperEnable(stepper.ChipSelectPin.PORT, stepper.ChipSelectPin.PIN);

    stepper.status.enabled = true;

    TimerEnable(stepper.TIMER_BASE, TIMER_A);
}
void StepperDisable(Stepper &stepper){
    SPIStepperDisable(stepper.ChipSelectPin.PORT, stepper.ChipSelectPin.PIN);

    stepper.status.enabled = false;
    stepper.status.speed_steps_per_second = 0;

    TimerDisable(stepper.TIMER_BASE, TIMER_A);
}


void StepperUpdate(Stepper &stepper)
{
  if(stepper.status.enabled){

    if(stepper.status.speed_steps_per_second != stepper.target_speed)
    {
        int original_speed = stepper.status.speed_steps_per_second;

        if(stepper.status.speed_steps_per_second < stepper.target_speed)
        {
          if(stepper.target_speed - stepper.status.speed_steps_per_second  < stepper.acceleration)
            stepper.status.speed_steps_per_second = stepper.target_speed;
          else
            stepper.status.speed_steps_per_second += stepper.acceleration;

        }
        else{
          if(stepper.status.speed_steps_per_second - stepper.target_speed < stepper.acceleration)
            stepper.status.speed_steps_per_second = stepper.target_speed;
          else
            stepper.status.speed_steps_per_second -= stepper.acceleration;
        }

        if(!SameSign(original_speed, stepper.status.speed_steps_per_second) || original_speed == 0){
            if(stepper.status.speed_steps_per_second>=0){
                SetStepperDirection(stepper.ChipSelectPin.PORT, stepper.ChipSelectPin.PIN, true);
                stepper.status.direction_forward = true;
            }else{
                SetStepperDirection(stepper.ChipSelectPin.PORT, stepper.ChipSelectPin.PIN, false);
                stepper.status.direction_forward = false;
            }
        }

        TimerLoadSet(stepper.TIMER_BASE, TIMER_A, std::min((SysCtlClockGet() / abs(stepper.status.speed_steps_per_second)) -1, SysCtlClockGet()/PERIODIC_UPDATE_RATE_HZ));
    }


  }

}

std::queue<TivaC_Pin> resetQueue;

void ScheduleStepPinReset(TivaC_Pin StepPin){
    resetQueue.push(StepPin);
    TimerEnable(TIMER1_BASE, TIMER_A); //Enable reset
}

void StepperStepPinSet(Stepper &stepper)
{
    // Clear the timer interrupt
    TimerIntClear(stepper.TIMER_BASE, TIMER_TIMA_TIMEOUT);

    if(stepper.status.direction_forward)
    {
      if(stepper.status.position_steps < stepper.travel_limit_steps)
      {
        stepper.status.position_steps++;
      }
      else
      {
        stepper.status.speed_steps_per_second = 0;
        return;
      }
    }
    else
    {
      if(stepper.status.position_steps > 0)
      {
        stepper.status.position_steps--;
      }
      else
      {
        stepper.status.speed_steps_per_second = 0;
        return;
      }
    }

    GPIOPinWrite(stepper.StepPin.PORT, stepper.StepPin.PIN, 255);


    ScheduleStepPinReset(stepper.StepPin);
}

void StepPinReset(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    if(resetQueue.size()){
      GPIOPinWrite(resetQueue.front().PORT, resetQueue.front().PIN, 0);
      resetQueue.pop();

      if(resetQueue.size()){
        TimerEnable(TIMER1_BASE, TIMER_A); //Enable reset
      }
    }
}



//
// Stepper initialization
//

void StepperInitGPIO(Stepper &stepper)
{
    GPIOPinTypeGPIOOutput(stepper.StepPin.PORT, stepper.StepPin.PIN);
    GPIOPadConfigSet(stepper.StepPin.PORT, stepper.StepPin.PIN, GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOOutput(stepper.ChipSelectPin.PORT, stepper.ChipSelectPin.PIN);
    GPIOPadConfigSet(stepper.ChipSelectPin.PORT, stepper.ChipSelectPin.PIN,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);

    GPIOPinWrite(stepper.ChipSelectPin.PORT, stepper.ChipSelectPin.PIN, 255); //Pull CS HIGH
}

void StepperInitSPI(Stepper &stepper){
  // !! stepper GPIO must be initialized first!

    ClearStepperRegisters(stepper.ChipSelectPin.PORT, stepper.ChipSelectPin.PIN);
    SetStepperCurrent(stepper.ChipSelectPin.PORT, stepper.ChipSelectPin.PIN, stepper.phase_current_ma);
    SetStepperStepMode(stepper.ChipSelectPin.PORT, stepper.ChipSelectPin.PIN, stepper.microstep_mode);
    SetStepperDirection(stepper.ChipSelectPin.PORT, stepper.ChipSelectPin.PIN, true);
}

void StepperInitTimer(void (*pfnHandler)(void), Stepper &stepper){
    TimerDisable(stepper.TIMER_BASE, TIMER_A);
    TimerConfigure(stepper.TIMER_BASE, TIMER_CFG_PERIODIC); //stepper1 set

    TimerLoadSet(stepper.TIMER_BASE, TIMER_A, (SysCtlClockGet() / stepper.status.speed_steps_per_second) -1);
    TimerUpdateMode(stepper.TIMER_BASE, TIMER_A, TIMER_UP_LOAD_TIMEOUT);

    TimerIntRegister(stepper.TIMER_BASE, TIMER_A, pfnHandler);

    switch(stepper.TIMER_BASE){
     case TIMER0_BASE:
       IntEnable(INT_TIMER0A);
       break;
     case TIMER1_BASE:
       IntEnable(INT_TIMER1A);
       break;
     case TIMER2_BASE:
       IntEnable(INT_TIMER2A);
       break;
     case TIMER3_BASE:
       IntEnable(INT_TIMER3A);
       break;
     case TIMER4_BASE:
       IntEnable(INT_TIMER4A);
       break;
     case TIMER5_BASE:
       IntEnable(INT_TIMER5A);
       break;
     }
    
    TimerIntEnable(stepper.TIMER_BASE, TIMER_TIMA_TIMEOUT);

    //TimerEnable(stepper.TIMER_BASE, TIMER_A);
}

