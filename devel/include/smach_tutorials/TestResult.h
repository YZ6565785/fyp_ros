// Generated by gencpp from file smach_tutorials/TestResult.msg
// DO NOT EDIT!


#ifndef SMACH_TUTORIALS_MESSAGE_TESTRESULT_H
#define SMACH_TUTORIALS_MESSAGE_TESTRESULT_H


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
struct TestResult_
{
  typedef TestResult_<ContainerAllocator> Type;

  TestResult_()
    {
    }
  TestResult_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::smach_tutorials::TestResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::smach_tutorials::TestResult_<ContainerAllocator> const> ConstPtr;

}; // struct TestResult_

typedef ::smach_tutorials::TestResult_<std::allocator<void> > TestResult;

typedef boost::shared_ptr< ::smach_tutorials::TestResult > TestResultPtr;
typedef boost::shared_ptr< ::smach_tutorials::TestResult const> TestResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::smach_tutorials::TestResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::smach_tutorials::TestResult_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::smach_tutorials::TestResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::smach_tutorials::TestResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::smach_tutorials::TestResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::smach_tutorials::TestResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::smach_tutorials::TestResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::smach_tutorials::TestResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::smach_tutorials::TestResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::smach_tutorials::TestResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::smach_tutorials::TestResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "smach_tutorials/TestResult";
  }

  static const char* value(const ::smach_tutorials::TestResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::smach_tutorials::TestResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
";
  }

  static const char* value(const ::smach_tutorials::TestResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::smach_tutorials::TestResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TestResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::smach_tutorials::TestResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::smach_tutorials::TestResult_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // SMACH_TUTORIALS_MESSAGE_TESTRESULT_H
