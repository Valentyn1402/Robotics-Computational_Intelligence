// Generated by gencpp from file motor/MotorPWM.msg
// DO NOT EDIT!


#ifndef MOTOR_MESSAGE_MOTORPWM_H
#define MOTOR_MESSAGE_MOTORPWM_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace motor
{
template <class ContainerAllocator>
struct MotorPWM_
{
  typedef MotorPWM_<ContainerAllocator> Type;

  MotorPWM_()
    : pwm_left(0.0)
    , pwm_right(0.0)  {
    }
  MotorPWM_(const ContainerAllocator& _alloc)
    : pwm_left(0.0)
    , pwm_right(0.0)  {
  (void)_alloc;
    }



   typedef float _pwm_left_type;
  _pwm_left_type pwm_left;

   typedef float _pwm_right_type;
  _pwm_right_type pwm_right;





  typedef boost::shared_ptr< ::motor::MotorPWM_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motor::MotorPWM_<ContainerAllocator> const> ConstPtr;

}; // struct MotorPWM_

typedef ::motor::MotorPWM_<std::allocator<void> > MotorPWM;

typedef boost::shared_ptr< ::motor::MotorPWM > MotorPWMPtr;
typedef boost::shared_ptr< ::motor::MotorPWM const> MotorPWMConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motor::MotorPWM_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motor::MotorPWM_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::motor::MotorPWM_<ContainerAllocator1> & lhs, const ::motor::MotorPWM_<ContainerAllocator2> & rhs)
{
  return lhs.pwm_left == rhs.pwm_left &&
    lhs.pwm_right == rhs.pwm_right;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::motor::MotorPWM_<ContainerAllocator1> & lhs, const ::motor::MotorPWM_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace motor

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::motor::MotorPWM_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motor::MotorPWM_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motor::MotorPWM_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motor::MotorPWM_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motor::MotorPWM_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motor::MotorPWM_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motor::MotorPWM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d7a5f6b78fec2b5366e39d282d33bb64";
  }

  static const char* value(const ::motor::MotorPWM_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd7a5f6b78fec2b53ULL;
  static const uint64_t static_value2 = 0x66e39d282d33bb64ULL;
};

template<class ContainerAllocator>
struct DataType< ::motor::MotorPWM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motor/MotorPWM";
  }

  static const char* value(const ::motor::MotorPWM_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motor::MotorPWM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Represents the PWM value of the left and right motors 0...1\n"
"float32 pwm_left\n"
"float32 pwm_right\n"
;
  }

  static const char* value(const ::motor::MotorPWM_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motor::MotorPWM_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pwm_left);
      stream.next(m.pwm_right);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotorPWM_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motor::MotorPWM_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motor::MotorPWM_<ContainerAllocator>& v)
  {
    s << indent << "pwm_left: ";
    Printer<float>::stream(s, indent + "  ", v.pwm_left);
    s << indent << "pwm_right: ";
    Printer<float>::stream(s, indent + "  ", v.pwm_right);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTOR_MESSAGE_MOTORPWM_H
