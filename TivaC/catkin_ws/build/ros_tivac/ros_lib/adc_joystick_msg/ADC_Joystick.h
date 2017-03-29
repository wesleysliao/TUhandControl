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
      typedef int16_t _x_axis_raw_type;
      _x_axis_raw_type x_axis_raw;
      typedef int16_t _x_axis_zero_type;
      _x_axis_zero_type x_axis_zero;
      typedef int16_t _y_axis_raw_type;
      _y_axis_raw_type y_axis_raw;
      typedef int16_t _y_axis_zero_type;
      _y_axis_zero_type y_axis_zero;

    ADC_Joystick():
      select(0),
      x_axis_raw(0),
      x_axis_zero(0),
      y_axis_raw(0),
      y_axis_zero(0)
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
      union {
        int16_t real;
        uint16_t base;
      } u_x_axis_raw;
      u_x_axis_raw.real = this->x_axis_raw;
      *(outbuffer + offset + 0) = (u_x_axis_raw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_axis_raw.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->x_axis_raw);
      union {
        int16_t real;
        uint16_t base;
      } u_x_axis_zero;
      u_x_axis_zero.real = this->x_axis_zero;
      *(outbuffer + offset + 0) = (u_x_axis_zero.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_axis_zero.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->x_axis_zero);
      union {
        int16_t real;
        uint16_t base;
      } u_y_axis_raw;
      u_y_axis_raw.real = this->y_axis_raw;
      *(outbuffer + offset + 0) = (u_y_axis_raw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_axis_raw.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->y_axis_raw);
      union {
        int16_t real;
        uint16_t base;
      } u_y_axis_zero;
      u_y_axis_zero.real = this->y_axis_zero;
      *(outbuffer + offset + 0) = (u_y_axis_zero.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_axis_zero.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->y_axis_zero);
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
      union {
        int16_t real;
        uint16_t base;
      } u_x_axis_raw;
      u_x_axis_raw.base = 0;
      u_x_axis_raw.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_axis_raw.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->x_axis_raw = u_x_axis_raw.real;
      offset += sizeof(this->x_axis_raw);
      union {
        int16_t real;
        uint16_t base;
      } u_x_axis_zero;
      u_x_axis_zero.base = 0;
      u_x_axis_zero.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_axis_zero.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->x_axis_zero = u_x_axis_zero.real;
      offset += sizeof(this->x_axis_zero);
      union {
        int16_t real;
        uint16_t base;
      } u_y_axis_raw;
      u_y_axis_raw.base = 0;
      u_y_axis_raw.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_axis_raw.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->y_axis_raw = u_y_axis_raw.real;
      offset += sizeof(this->y_axis_raw);
      union {
        int16_t real;
        uint16_t base;
      } u_y_axis_zero;
      u_y_axis_zero.base = 0;
      u_y_axis_zero.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_axis_zero.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->y_axis_zero = u_y_axis_zero.real;
      offset += sizeof(this->y_axis_zero);
     return offset;
    }

    const char * getType(){ return "adc_joystick_msg/ADC_Joystick"; };
    const char * getMD5(){ return "10957cbdb8481676f0a00f626dbd9899"; };

  };

}
#endif