#ifndef _ROS_tigerbot7_servoInfo_h
#define _ROS_tigerbot7_servoInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tigerbot7
{

  class servoInfo : public ros::Msg
  {
    public:
      uint32_t joints_length;
      typedef int64_t _joints_type;
      _joints_type st_joints;
      _joints_type * joints;
      uint32_t deg_length;
      typedef float _deg_type;
      _deg_type st_deg;
      _deg_type * deg;

    servoInfo():
      joints_length(0), joints(NULL),
      deg_length(0), deg(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joints_length);
      for( uint32_t i = 0; i < joints_length; i++){
      union {
        int64_t real;
        uint64_t base;
      } u_jointsi;
      u_jointsi.real = this->joints[i];
      *(outbuffer + offset + 0) = (u_jointsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_jointsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_jointsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_jointsi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_jointsi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_jointsi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_jointsi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_jointsi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joints[i]);
      }
      *(outbuffer + offset + 0) = (this->deg_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->deg_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->deg_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->deg_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->deg_length);
      for( uint32_t i = 0; i < deg_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->deg[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t joints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joints_length);
      if(joints_lengthT > joints_length)
        this->joints = (int64_t*)realloc(this->joints, joints_lengthT * sizeof(int64_t));
      joints_length = joints_lengthT;
      for( uint32_t i = 0; i < joints_length; i++){
      union {
        int64_t real;
        uint64_t base;
      } u_st_joints;
      u_st_joints.base = 0;
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_joints = u_st_joints.real;
      offset += sizeof(this->st_joints);
        memcpy( &(this->joints[i]), &(this->st_joints), sizeof(int64_t));
      }
      uint32_t deg_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      deg_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      deg_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      deg_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->deg_length);
      if(deg_lengthT > deg_length)
        this->deg = (float*)realloc(this->deg, deg_lengthT * sizeof(float));
      deg_length = deg_lengthT;
      for( uint32_t i = 0; i < deg_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_deg));
        memcpy( &(this->deg[i]), &(this->st_deg), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "tigerbot7/servoInfo"; };
    const char * getMD5(){ return "c160e49536a986333ad7e738ae8a5d79"; };

  };

}
#endif