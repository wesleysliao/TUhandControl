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
      typedef uint32_t _position_steps_type;
      _position_steps_type position_steps;
      typedef uint32_t _speed_steps_per_second_type;
      _speed_steps_per_second_type speed_steps_per_second;
      typedef bool _direction_forward_type;
      _direction_forward_type direction_forward;

    Stepper_Status():
      position_steps(0),
      speed_steps_per_second(0),
      direction_forward(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->position_steps >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_steps >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position_steps >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position_steps >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_steps);
      *(outbuffer + offset + 0) = (this->speed_steps_per_second >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->speed_steps_per_second >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->speed_steps_per_second >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->speed_steps_per_second >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_steps_per_second);
      union {
        bool real;
        uint8_t base;
      } u_direction_forward;
      u_direction_forward.real = this->direction_forward;
      *(outbuffer + offset + 0) = (u_direction_forward.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->direction_forward);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->position_steps =  ((uint32_t) (*(inbuffer + offset)));
      this->position_steps |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->position_steps |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->position_steps |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->position_steps);
      this->speed_steps_per_second =  ((uint32_t) (*(inbuffer + offset)));
      this->speed_steps_per_second |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed_steps_per_second |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->speed_steps_per_second |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->speed_steps_per_second);
      union {
        bool real;
        uint8_t base;
      } u_direction_forward;
      u_direction_forward.base = 0;
      u_direction_forward.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->direction_forward = u_direction_forward.real;
      offset += sizeof(this->direction_forward);
     return offset;
    }

    const char * getType(){ return "stepper_msg/Stepper_Status"; };
    const char * getMD5(){ return "3a08b832803b195f0f005fead32aedac"; };

  };

}
#endif