// Generated by gencpp from file arm/dynamixel_srvResponse.msg
// DO NOT EDIT!


#ifndef ARM_MESSAGE_DYNAMIXEL_SRVRESPONSE_H
#define ARM_MESSAGE_DYNAMIXEL_SRVRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace arm
{
template <class ContainerAllocator>
struct dynamixel_srvResponse_
{
  typedef dynamixel_srvResponse_<ContainerAllocator> Type;

  dynamixel_srvResponse_()
    : result(false)  {
    }
  dynamixel_srvResponse_(const ContainerAllocator& _alloc)
    : result(false)  {
  (void)_alloc;
    }



   typedef uint8_t _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::arm::dynamixel_srvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arm::dynamixel_srvResponse_<ContainerAllocator> const> ConstPtr;

}; // struct dynamixel_srvResponse_

typedef ::arm::dynamixel_srvResponse_<std::allocator<void> > dynamixel_srvResponse;

typedef boost::shared_ptr< ::arm::dynamixel_srvResponse > dynamixel_srvResponsePtr;
typedef boost::shared_ptr< ::arm::dynamixel_srvResponse const> dynamixel_srvResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::arm::dynamixel_srvResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::arm::dynamixel_srvResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::arm::dynamixel_srvResponse_<ContainerAllocator1> & lhs, const ::arm::dynamixel_srvResponse_<ContainerAllocator2> & rhs)
{
  return lhs.result == rhs.result;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::arm::dynamixel_srvResponse_<ContainerAllocator1> & lhs, const ::arm::dynamixel_srvResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace arm

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::arm::dynamixel_srvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::arm::dynamixel_srvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::arm::dynamixel_srvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::arm::dynamixel_srvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::arm::dynamixel_srvResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::arm::dynamixel_srvResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::arm::dynamixel_srvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eb13ac1f1354ccecb7941ee8fa2192e8";
  }

  static const char* value(const ::arm::dynamixel_srvResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xeb13ac1f1354ccecULL;
  static const uint64_t static_value2 = 0xb7941ee8fa2192e8ULL;
};

template<class ContainerAllocator>
struct DataType< ::arm::dynamixel_srvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "arm/dynamixel_srvResponse";
  }

  static const char* value(const ::arm::dynamixel_srvResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::arm::dynamixel_srvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool result\n"
;
  }

  static const char* value(const ::arm::dynamixel_srvResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::arm::dynamixel_srvResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct dynamixel_srvResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::arm::dynamixel_srvResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::arm::dynamixel_srvResponse_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARM_MESSAGE_DYNAMIXEL_SRVRESPONSE_H