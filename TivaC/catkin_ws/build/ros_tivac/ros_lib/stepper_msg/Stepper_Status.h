#ifndef _ROS_stepper_msg_Stepper_Status_h
#define _ROS_stepper_msg_Stepper_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace stepper_msg
{

  class Stepper_Status : public ros::Msg
  {
    public:
      typedef int64_t _position_steps_type;
      _position_steps_type position_steps;
      typedef int32_t _speed_steps_per_second_type;
      _speed_steps_per_second_type speed_steps_per_second;
      typedef bool _direction_forward_type;
      _direction_forward_type direction_forward;
      typedef bool _enabled_type;
      _enabled_type enabled;

    Stepper_Status():
      position_steps(0),
      speed_steps_per_second(0),
      direction_forward(0),
      enabled(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_position_steps;
      u_position_steps.real = this->position_steps;
      *(outbuffer + offset + 0) = (u_position_steps.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_steps.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_steps.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_steps.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_position_steps.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_position_steps.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_position_steps.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_position_steps.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->position_steps);
      union {
        int32_t real;
        uint32_t base;
      } u_speed_steps_per_second;
      u_speed_steps_per_second.real = this->speed_steps_per_second;
      *(outbuffer + offset + 0) = (u_speed_steps_per_second.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_steps_per_second.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_steps_per_second.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_steps_per_second.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_steps_per_second);
      union {
        bool real;
        uint8_t base;
      } u_direction_forward;
      u_direction_forward.real = this->direction_forward;
      *(outbuffer + offset + 0) = (u_direction_forward.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->direction_forward);
      union {
        bool real;
        uint8_t base;
      } u_enabled;
      u_enabled.real = this->enabled;
      *(outbuffer + offset + 0) = (u_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enabled);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_position_steps;
      u_position_steps.base = 0;
      u_position_steps.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_steps.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_steps.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_steps.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_position_steps.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_position_steps.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_position_steps.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_position_steps.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->position_steps = u_position_steps.real;
      offset += sizeof(this->position_steps);
      union {
        int32_t real;
        uint32_t base;
      } u_speed_steps_per_second;
      u_speed_steps_per_second.base = 0;
      u_speed_steps_per_second.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_steps_per_second.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_steps_per_second.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_steps_per_second.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_steps_per_second = u_speed_steps_per_second.real;
      offset += sizeof(this->speed_steps_per_second);
      union {
        bool real;
        uint8_t base;
      } u_direction_forward;
      u_direction_forward.base = 0;
      u_direction_forward.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->direction_forward = u_direction_forward.real;
      offset += sizeof(this->direction_forward);
      union {
        bool real;
        uint8_t base;
      } u_enabled;
      u_enabled.base = 0;
      u_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enabled = u_enabled.real;
      offset += sizeof(this->enabled);
     return offset;
    }

    const char * getType(){ return "stepper_msg/Stepper_Status"; };
    const char * getMD5(){ return "b8e41235ddba5043cc88e78b8401e13b"; };

  };

}
#endif