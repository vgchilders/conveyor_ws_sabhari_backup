// Generated by gencpp from file arm/stepper_srv.msg
// DO NOT EDIT!


#ifndef ARM_MESSAGE_STEPPER_SRV_H
#define ARM_MESSAGE_STEPPER_SRV_H

#include <ros/service_traits.h>


#include <arm/stepper_srvRequest.h>
#include <arm/stepper_srvResponse.h>


namespace arm
{

struct stepper_srv
{

typedef stepper_srvRequest Request;
typedef stepper_srvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct stepper_srv
} // namespace arm


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::arm::stepper_srv > {
  static const char* value()
  {
    return "1a060548b1bc718fe8b44339268f4d29";
  }

  static const char* value(const ::arm::stepper_srv&) { return value(); }
};

template<>
struct DataType< ::arm::stepper_srv > {
  static const char* value()
  {
    return "arm/stepper_srv";
  }

  static const char* value(const ::arm::stepper_srv&) { return value(); }
};


// service_traits::MD5Sum< ::arm::stepper_srvRequest> should match
// service_traits::MD5Sum< ::arm::stepper_srv >
template<>
struct MD5Sum< ::arm::stepper_srvRequest>
{
  static const char* value()
  {
    return MD5Sum< ::arm::stepper_srv >::value();
  }
  static const char* value(const ::arm::stepper_srvRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::arm::stepper_srvRequest> should match
// service_traits::DataType< ::arm::stepper_srv >
template<>
struct DataType< ::arm::stepper_srvRequest>
{
  static const char* value()
  {
    return DataType< ::arm::stepper_srv >::value();
  }
  static const char* value(const ::arm::stepper_srvRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::arm::stepper_srvResponse> should match
// service_traits::MD5Sum< ::arm::stepper_srv >
template<>
struct MD5Sum< ::arm::stepper_srvResponse>
{
  static const char* value()
  {
    return MD5Sum< ::arm::stepper_srv >::value();
  }
  static const char* value(const ::arm::stepper_srvResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::arm::stepper_srvResponse> should match
// service_traits::DataType< ::arm::stepper_srv >
template<>
struct DataType< ::arm::stepper_srvResponse>
{
  static const char* value()
  {
    return DataType< ::arm::stepper_srv >::value();
  }
  static const char* value(const ::arm::stepper_srvResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ARM_MESSAGE_STEPPER_SRV_H