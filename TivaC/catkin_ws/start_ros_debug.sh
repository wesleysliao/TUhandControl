#!/bin/bash
source devel/setup.bash

gnome-terminal --command 'roscore'

gnome-terminal --title='/adc_joystick' --command 'rostopic echo adc_joystick'
gnome-terminal --title='/stepper_status' --command 'rostopic echo stepper_status'

