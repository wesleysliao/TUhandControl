#ifndef _ROS_adc_joystick_msg_ADC_Joystick_h
#define _ROS_adc_joystick_msg_ADC_Joystick_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace adc_joystick_msg
{

  class ADC_Joystick : public ros::Msg
  {
    public:
      typedef bool _select_type;
      _select_type select;
      typedef uint16_t _x_axis_raw_type;
      _x_axis_raw_type x_axis_raw;
      typedef uint16_t _y_axis_raw_type;
      _y_axis_raw_type y_axis_raw;

    ADC_Joystick():
      select(0),
      x_axis_raw(0),
      y_axis_raw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_select;
      u_select.real = this->select;
      *(outbuffer + offset + 0) = (u_select.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->select);
      *(outbuffer + offset + 0) = (this->x_axis_raw >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->x_axis_raw >> (8 * 1)) & 0xFF;
      offset += sizeof(this->x_axis_raw);
      *(outbuffer + offset + 0) = (this->y_axis_raw >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->y_axis_raw >> (8 * 1)) & 0xFF;
      offset += sizeof(this->y_axis_raw);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_select;
      u_select.base = 0;
      u_select.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->select = u_select.real;
      offset += sizeof(this->select);
      this->x_axis_raw =  ((uint16_t) (*(inbuffer + offset)));
      this->x_axis_raw |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->x_axis_raw);
      this->y_axis_raw =  ((uint16_t) (*(inbuffer + offset)));
      this->y_axis_raw |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->y_axis_raw);
     return offset;
    }

    const char * getType(){ return "adc_joystick_msg/ADC_Joystick"; };
    const char * getMD5(){ return "832227e0bdded7eb16fd560e3102fbf2"; };

  };

}
#endif