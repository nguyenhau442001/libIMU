#ifndef _ROS_droidspeak_DroidChat_h
#define _ROS_droidspeak_DroidChat_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace droidspeak
{

  class DroidChat : public ros::Msg
  {
    public:
      typedef const char* _data_type;
      _data_type data;
      uint32_t spaces_length;
      typedef float _spaces_type;
      _spaces_type st_spaces;
      _spaces_type * spaces;

    DroidChat():
      data(""),
      spaces_length(0), st_spaces(), spaces(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_data = strlen(this->data);
      varToArr(outbuffer + offset, length_data);
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
      *(outbuffer + offset + 0) = (this->spaces_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->spaces_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->spaces_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->spaces_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->spaces_length);
      for( uint32_t i = 0; i < spaces_length; i++){
      union {
        float real;
        uint32_t base;
      } u_spacesi;
      u_spacesi.real = this->spaces[i];
      *(outbuffer + offset + 0) = (u_spacesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_spacesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_spacesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_spacesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->spaces[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_data;
      arrToVar(length_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
      uint32_t spaces_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      spaces_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      spaces_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      spaces_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->spaces_length);
      if(spaces_lengthT > spaces_length)
        this->spaces = (float*)realloc(this->spaces, spaces_lengthT * sizeof(float));
      spaces_length = spaces_lengthT;
      for( uint32_t i = 0; i < spaces_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_spaces;
      u_st_spaces.base = 0;
      u_st_spaces.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_spaces.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_spaces.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_spaces.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_spaces = u_st_spaces.real;
      offset += sizeof(this->st_spaces);
        memcpy( &(this->spaces[i]), &(this->st_spaces), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "droidspeak/DroidChat"; };
    virtual const char * getMD5() override { return "14b3c1f13ca46764581966919c8bdd15"; };

  };

}
#endif
