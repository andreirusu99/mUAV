// Generated by gencpp from file drone/ControlAxes.msg
// DO NOT EDIT!


#ifndef DRONE_MESSAGE_CONTROLAXES_H
#define DRONE_MESSAGE_CONTROLAXES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace drone
{
template <class ContainerAllocator>
struct ControlAxes_
{
  typedef ControlAxes_<ContainerAllocator> Type;

  ControlAxes_()
    : data()  {
      data.assign(0);
  }
  ControlAxes_(const ContainerAllocator& _alloc)
    : data()  {
  (void)_alloc;
      data.assign(0);
  }



   typedef boost::array<uint16_t, 4>  _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::drone::ControlAxes_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::drone::ControlAxes_<ContainerAllocator> const> ConstPtr;

}; // struct ControlAxes_

typedef ::drone::ControlAxes_<std::allocator<void> > ControlAxes;

typedef boost::shared_ptr< ::drone::ControlAxes > ControlAxesPtr;
typedef boost::shared_ptr< ::drone::ControlAxes const> ControlAxesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::drone::ControlAxes_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::drone::ControlAxes_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::drone::ControlAxes_<ContainerAllocator1> & lhs, const ::drone::ControlAxes_<ContainerAllocator2> & rhs)
{
  return lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::drone::ControlAxes_<ContainerAllocator1> & lhs, const ::drone::ControlAxes_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace drone

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::drone::ControlAxes_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::drone::ControlAxes_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::drone::ControlAxes_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::drone::ControlAxes_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::drone::ControlAxes_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::drone::ControlAxes_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::drone::ControlAxes_<ContainerAllocator> >
{
  static const char* value()
  {
    return "073fbf0245def3a5b07ad0a112777420";
  }

  static const char* value(const ::drone::ControlAxes_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x073fbf0245def3a5ULL;
  static const uint64_t static_value2 = 0xb07ad0a112777420ULL;
};

template<class ContainerAllocator>
struct DataType< ::drone::ControlAxes_<ContainerAllocator> >
{
  static const char* value()
  {
    return "drone/ControlAxes";
  }

  static const char* value(const ::drone::ControlAxes_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::drone::ControlAxes_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# represents the channels that the FC accepts\n"
"# [Roll, Pitch, Throttle, Yaw]\n"
"uint16[4] data\n"
;
  }

  static const char* value(const ::drone::ControlAxes_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::drone::ControlAxes_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ControlAxes_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::drone::ControlAxes_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::drone::ControlAxes_<ContainerAllocator>& v)
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<uint16_t>::stream(s, indent + "  ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DRONE_MESSAGE_CONTROLAXES_H