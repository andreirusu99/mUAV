// Generated by gencpp from file drone/Channels.msg
// DO NOT EDIT!


#ifndef DRONE_MESSAGE_CHANNELS_H
#define DRONE_MESSAGE_CHANNELS_H


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
struct Channels_
{
  typedef Channels_<ContainerAllocator> Type;

  Channels_()
    : channel()  {
      channel.assign(0);
  }
  Channels_(const ContainerAllocator& _alloc)
    : channel()  {
  (void)_alloc;
      channel.assign(0);
  }



   typedef boost::array<uint16_t, 6>  _channel_type;
  _channel_type channel;





  typedef boost::shared_ptr< ::drone::Channels_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::drone::Channels_<ContainerAllocator> const> ConstPtr;

}; // struct Channels_

typedef ::drone::Channels_<std::allocator<void> > Channels;

typedef boost::shared_ptr< ::drone::Channels > ChannelsPtr;
typedef boost::shared_ptr< ::drone::Channels const> ChannelsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::drone::Channels_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::drone::Channels_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::drone::Channels_<ContainerAllocator1> & lhs, const ::drone::Channels_<ContainerAllocator2> & rhs)
{
  return lhs.channel == rhs.channel;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::drone::Channels_<ContainerAllocator1> & lhs, const ::drone::Channels_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace drone

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::drone::Channels_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::drone::Channels_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::drone::Channels_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::drone::Channels_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::drone::Channels_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::drone::Channels_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::drone::Channels_<ContainerAllocator> >
{
  static const char* value()
  {
    return "31b508c75ec65d5aaa1d5c71970654be";
  }

  static const char* value(const ::drone::Channels_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x31b508c75ec65d5aULL;
  static const uint64_t static_value2 = 0xaa1d5c71970654beULL;
};

template<class ContainerAllocator>
struct DataType< ::drone::Channels_<ContainerAllocator> >
{
  static const char* value()
  {
    return "drone/Channels";
  }

  static const char* value(const ::drone::Channels_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::drone::Channels_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# represents the channels that the FC accepts\n"
"# [Roll, Pitch, Throttle, Yaw, AUX1, AUX2]\n"
"uint16[6] channel\n"
;
  }

  static const char* value(const ::drone::Channels_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::drone::Channels_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.channel);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Channels_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::drone::Channels_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::drone::Channels_<ContainerAllocator>& v)
  {
    s << indent << "channel[]" << std::endl;
    for (size_t i = 0; i < v.channel.size(); ++i)
    {
      s << indent << "  channel[" << i << "]: ";
      Printer<uint16_t>::stream(s, indent + "  ", v.channel[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DRONE_MESSAGE_CHANNELS_H
