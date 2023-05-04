#ifndef _ROS_SERVICE_DroidEmotion_h
#define _ROS_SERVICE_DroidEmotion_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/duration.h"

namespace droidspeak
{

static const char DROIDEMOTION[] = "droidspeak/DroidEmotion";

  class DroidEmotionRequest : public ros::Msg
  {
    public:
      typedef uint8_t _emotion_type;
      _emotion_type emotion;
      typedef ros::Duration _duration_type;
      _duration_type duration;
      enum { NORMAL = 0 };
      enum { HAPPY = 1 };
      enum { SORRY = 2 };
      enum { ANGRY = 3 };
      enum { JADED = 4 };
      enum { LOOK_LEFT = 5 };
      enum { LOOK_RIGHT = 6 };
      enum { LOOK_UP = 7 };
      enum { LOOK_DOWN = 8 };
      enum { BLINK = 9 };

    DroidEmotionRequest():
      emotion(0),
      duration()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->emotion >> (8 * 0)) & 0xFF;
      offset += sizeof(this->emotion);
      *(outbuffer + offset + 0) = (this->duration.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->duration.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->duration.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->duration.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->duration.sec);
      *(outbuffer + offset + 0) = (this->duration.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->duration.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->duration.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->duration.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->duration.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->emotion =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->emotion);
      this->duration.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->duration.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->duration.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->duration.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->duration.sec);
      this->duration.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->duration.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->duration.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->duration.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->duration.nsec);
     return offset;
    }

    virtual const char * getType() override { return DROIDEMOTION; };
    virtual const char * getMD5() override { return "3121217addb14943825726ec606b461f"; };

  };

  class DroidEmotionResponse : public ros::Msg
  {
    public:

    DroidEmotionResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return DROIDEMOTION; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class DroidEmotion {
    public:
    typedef DroidEmotionRequest Request;
    typedef DroidEmotionResponse Response;
  };

}
#endif
