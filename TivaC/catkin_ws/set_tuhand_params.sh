#!/bin/bash
source devel/setup.bash

rosparam set /TUhand/Tendon1Stepper/max_speed_steps_per_second 24000
rosparam set /TUhand/Tendon1Stepper/travel_limit_steps 64000
rosparam set /TUhand/Tendon1Stepper/acceleration 10
rosparam set /TUhand/Tendon1Stepper/microstep_mode 16
rosparam set /TUhand/Tendon1Stepper/phase_current_ma 2800

rosparam set /TUhand/Tendon2Stepper/max_speed_steps_per_second 24000
rosparam set /TUhand/Tendon2Stepper/travel_limit_steps 64000
rosparam set /TUhand/Tendon2Stepper/acceleration 800
rosparam set /TUhand/Tendon2Stepper/microstep_mode 16
rosparam set /TUhand/Tendon2Stepper/phase_current_ma 2800

rosparam set /TUhand/WristStepper/max_speed_steps_per_second 24000
rosparam set /TUhand/WristStepper/travel_limit_steps 160000
rosparam set /TUhand/WristStepper/acceleration 800
rosparam set /TUhand/WristStepper/microstep_mode 16
rosparam set /TUhand/WristStepper/phase_current_ma 1000

rosparam list /TUhand

