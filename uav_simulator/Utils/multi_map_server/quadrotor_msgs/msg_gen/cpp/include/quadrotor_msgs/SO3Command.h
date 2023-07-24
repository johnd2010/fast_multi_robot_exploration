/* Auto-generated by genmsg_cpp for file /home/jchen/workspace/src/quadrotor_msgs/msg/SO3Command.msg */
#ifndef QUADROTOR_MSGS_MESSAGE_SO3COMMAND_H
#define QUADROTOR_MSGS_MESSAGE_SO3COMMAND_H
#include <map>
#include <ostream>
#include <string>
#include <vector>

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "quadrotor_msgs/AuxCommand.h"
#include "ros/assert.h"
#include "ros/builtin_message_traits.h"
#include "ros/macros.h"
#include "ros/message_operations.h"
#include "ros/serialization.h"
#include "ros/time.h"
#include "std_msgs/Header.h"

namespace quadrotor_msgs {
template <class ContainerAllocator>
struct SO3Command_ {
  typedef SO3Command_<ContainerAllocator> Type;

  SO3Command_() : header(), force(), orientation(), kR(), kOm(), aux() {
    kR.assign(0.0);
    kOm.assign(0.0);
  }

  SO3Command_(const ContainerAllocator& _alloc)
      : header(_alloc),
        force(_alloc),
        orientation(_alloc),
        kR(),
        kOm(),
        aux(_alloc) {
    kR.assign(0.0);
    kOm.assign(0.0);
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  ::std_msgs::Header_<ContainerAllocator> header;

  typedef ::geometry_msgs::Vector3_<ContainerAllocator> _force_type;
  ::geometry_msgs::Vector3_<ContainerAllocator> force;

  typedef ::geometry_msgs::Quaternion_<ContainerAllocator> _orientation_type;
  ::geometry_msgs::Quaternion_<ContainerAllocator> orientation;

  typedef boost::array<double, 3> _kR_type;
  boost::array<double, 3> kR;

  typedef boost::array<double, 3> _kOm_type;
  boost::array<double, 3> kOm;

  typedef ::quadrotor_msgs::AuxCommand_<ContainerAllocator> _aux_type;
  ::quadrotor_msgs::AuxCommand_<ContainerAllocator> aux;

  typedef boost::shared_ptr< ::quadrotor_msgs::SO3Command_<ContainerAllocator> >
      Ptr;
  typedef boost::shared_ptr<
      ::quadrotor_msgs::SO3Command_<ContainerAllocator> const>
      ConstPtr;
};  // struct SO3Command
typedef ::quadrotor_msgs::SO3Command_<std::allocator<void> > SO3Command;

typedef boost::shared_ptr< ::quadrotor_msgs::SO3Command> SO3CommandPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::SO3Command const>
    SO3CommandConstPtr;

template <typename ContainerAllocator>
std::ostream& operator<<(
    std::ostream& s,
    const ::quadrotor_msgs::SO3Command_<ContainerAllocator>& v) {
  ros::message_operations::Printer<
      ::quadrotor_msgs::SO3Command_<ContainerAllocator> >::stream(s, "", v);
  return s;
}

}  // namespace quadrotor_msgs

namespace ros {
namespace message_traits {
template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::SO3Command_<ContainerAllocator> >
    : public TrueType {};
template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::SO3Command_<ContainerAllocator> const>
    : public TrueType {};
template <class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::SO3Command_<ContainerAllocator> > {
  static const char* value() {
    return "a466650b2633e768513aa3bf62383c86";
  }

  static const char* value(
      const ::quadrotor_msgs::SO3Command_<ContainerAllocator>&) {
    return value();
  }
  static const uint64_t static_value1 = 0xa466650b2633e768ULL;
  static const uint64_t static_value2 = 0x513aa3bf62383c86ULL;
};

template <class ContainerAllocator>
struct DataType< ::quadrotor_msgs::SO3Command_<ContainerAllocator> > {
  static const char* value() {
    return "quadrotor_msgs/SO3Command";
  }

  static const char* value(
      const ::quadrotor_msgs::SO3Command_<ContainerAllocator>&) {
    return value();
  }
};

template <class ContainerAllocator>
struct Definition< ::quadrotor_msgs::SO3Command_<ContainerAllocator> > {
  static const char* value() {
    return "Header header\n\
geometry_msgs/Vector3 force\n\
geometry_msgs/Quaternion orientation\n\
float64[3] kR\n\
float64[3] kOm\n\
quadrotor_msgs/AuxCommand aux\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: quadrotor_msgs/AuxCommand\n\
float64 current_yaw\n\
float64 kf_correction\n\
float64[2] angle_corrections# Trims for roll, pitch\n\
bool enable_motors\n\
bool use_external_yaw\n\
\n\
";
  }

  static const char* value(
      const ::quadrotor_msgs::SO3Command_<ContainerAllocator>&) {
    return value();
  }
};

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::SO3Command_<ContainerAllocator> >
    : public TrueType {};
template <class ContainerAllocator>
struct HasHeader<const ::quadrotor_msgs::SO3Command_<ContainerAllocator> >
    : public TrueType {};
}  // namespace message_traits
}  // namespace ros

namespace ros {
namespace serialization {

template <class ContainerAllocator>
struct Serializer< ::quadrotor_msgs::SO3Command_<ContainerAllocator> > {
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m) {
    stream.next(m.header);
    stream.next(m.force);
    stream.next(m.orientation);
    stream.next(m.kR);
    stream.next(m.kOm);
    stream.next(m.aux);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
};  // struct SO3Command_
}  // namespace serialization
}  // namespace ros

namespace ros {
namespace message_operations {

template <class ContainerAllocator>
struct Printer< ::quadrotor_msgs::SO3Command_<ContainerAllocator> > {
  template <typename Stream>
  static void stream(
      Stream& s, const std::string& indent,
      const ::quadrotor_msgs::SO3Command_<ContainerAllocator>& v) {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ",
                                                               v.header);
    s << indent << "force: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(
        s, indent + "  ", v.force);
    s << indent << "orientation: ";
    s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(
        s, indent + "  ", v.orientation);
    s << indent << "kR[]" << std::endl;
    for (size_t i = 0; i < v.kR.size(); ++i) {
      s << indent << "  kR[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.kR[i]);
    }
    s << indent << "kOm[]" << std::endl;
    for (size_t i = 0; i < v.kOm.size(); ++i) {
      s << indent << "  kOm[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.kOm[i]);
    }
    s << indent << "aux: ";
    s << std::endl;
    Printer< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >::stream(
        s, indent + "  ", v.aux);
  }
};

}  // namespace message_operations
}  // namespace ros

#endif  // QUADROTOR_MSGS_MESSAGE_SO3COMMAND_H
