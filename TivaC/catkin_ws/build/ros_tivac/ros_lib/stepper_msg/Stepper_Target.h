#ifndef _ROS_stepper_msg_Stepper_Target_h
#define _ROS_stepper_msg_Stepper_Target_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace stepper_msg
{

  class Stepper_Target : public ros::Msg
  {
    public:
      typedef uint32_t _position_steps_type;
      _position_steps_type position_steps;
      typedef uint32_t _top_speed_steps_per_second_type;
      _top_speed_steps_per_second_type top_speed_steps_per_second;

    Stepper_Target():
      position_steps(0),
      top_speed_steps_per_second(0)
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
      *(outbuffer + offset + 0) = (this->top_speed_steps_per_second >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->top_speed_steps_per_second >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->top_speed_steps_per_second >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->top_speed_steps_per_second >> (8 * 3)) & 0xFF;
      offset += sizeof(this->top_speed_steps_per_second);
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
      this->top_speed_steps_per_second =  ((uint32_t) (*(inbuffer + offset)));
      this->top_speed_steps_per_second |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->top_speed_steps_per_second |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->top_speed_steps_per_second |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->top_speed_steps_per_second);
     return offset;
    }

    const char * getType(){ return "stepper_msg/Stepper_Target"; };
    const char * getMD5(){ return "737f4dc6071f1d903de2da2901e5c3ca"; };

  };

}
#endif