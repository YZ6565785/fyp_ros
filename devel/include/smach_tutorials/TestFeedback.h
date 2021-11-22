// Generated by gencpp from file smach_tutorials/TestFeedback.msg
// DO NOT EDIT!


#ifndef SMACH_TUTORIALS_MESSAGE_TESTFEEDBACK_H
#define SMACH_TUTORIALS_MESSAGE_TESTFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace smach_tutorials
{
template <class ContainerAllocator>
struct TestFeedback_
{
  typedef TestFeedback_<ContainerAllocator> Type;

  TestFeedback_()
    {
    }
  TestFeedback_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::smach_tutorials::TestFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::smach_tutorials::TestFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct TestFeedback_

typedef ::smach_tutorials::TestFeedback_<std::allocator<void> > TestFeedback;

typedef boost::shared_ptr< ::smach_tutorials::TestFeedback > TestFeedbackPtr;
typedef boost::shared_ptr< ::smach_tutorials::TestFeedback const> TestFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::smach_tutorials::TestFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::smach_tutorials::TestFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace smach_tutorials

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'smach_tutorials': ['/home/yuhang/fyp_ws/devel/share/smach_tutorials/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::smach_tutorials::TestFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::smach_tutorials::TestFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::smach_tutorials::TestFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::smach_tutorials::TestFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::smach_tutorials::TestFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::smach_tutorials::TestFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::smach_tutorials::TestFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::smach_tutorials::TestFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::smach_tutorials::TestFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "smach_tutorials/TestFeedback";
  }

  static const char* value(const ::smach_tutorials::TestFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::smach_tutorials::TestFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
";
  }

  static const char* value(const ::smach_tutorials::TestFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::smach_tutorials::TestFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TestFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::smach_tutorials::TestFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::smach_tutorials::TestFeedback_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // SMACH_TUTORIALS_MESSAGE_TESTFEEDBACK_H
