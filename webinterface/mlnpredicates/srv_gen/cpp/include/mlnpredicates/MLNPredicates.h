/* Auto-generated by genmsg_cpp for file /home/ai/rosbuild_ws/mlnpredicates/srv/MLNPredicates.srv */
#ifndef MLNPREDICATES_SERVICE_MLNPREDICATES_H
#define MLNPREDICATES_SERVICE_MLNPREDICATES_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"



#include "mlnpredicates/MLNPredicate.h"

namespace mlnpredicates
{
template <class ContainerAllocator>
struct MLNPredicatesRequest_ {
  typedef MLNPredicatesRequest_<ContainerAllocator> Type;

  MLNPredicatesRequest_()
  : dummy()
  {
  }

  MLNPredicatesRequest_(const ContainerAllocator& _alloc)
  : dummy(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _dummy_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  dummy;


  typedef boost::shared_ptr< ::mlnpredicates::MLNPredicatesRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mlnpredicates::MLNPredicatesRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MLNPredicatesRequest
typedef  ::mlnpredicates::MLNPredicatesRequest_<std::allocator<void> > MLNPredicatesRequest;

typedef boost::shared_ptr< ::mlnpredicates::MLNPredicatesRequest> MLNPredicatesRequestPtr;
typedef boost::shared_ptr< ::mlnpredicates::MLNPredicatesRequest const> MLNPredicatesRequestConstPtr;



template <class ContainerAllocator>
struct MLNPredicatesResponse_ {
  typedef MLNPredicatesResponse_<ContainerAllocator> Type;

  MLNPredicatesResponse_()
  : predicates()
  {
  }

  MLNPredicatesResponse_(const ContainerAllocator& _alloc)
  : predicates(_alloc)
  {
  }

  typedef std::vector< ::mlnpredicates::MLNPredicate_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::mlnpredicates::MLNPredicate_<ContainerAllocator> >::other >  _predicates_type;
  std::vector< ::mlnpredicates::MLNPredicate_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::mlnpredicates::MLNPredicate_<ContainerAllocator> >::other >  predicates;


  typedef boost::shared_ptr< ::mlnpredicates::MLNPredicatesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mlnpredicates::MLNPredicatesResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MLNPredicatesResponse
typedef  ::mlnpredicates::MLNPredicatesResponse_<std::allocator<void> > MLNPredicatesResponse;

typedef boost::shared_ptr< ::mlnpredicates::MLNPredicatesResponse> MLNPredicatesResponsePtr;
typedef boost::shared_ptr< ::mlnpredicates::MLNPredicatesResponse const> MLNPredicatesResponseConstPtr;


struct MLNPredicates
{

typedef MLNPredicatesRequest Request;
typedef MLNPredicatesResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct MLNPredicates
} // namespace mlnpredicates

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::mlnpredicates::MLNPredicatesRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::mlnpredicates::MLNPredicatesRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::mlnpredicates::MLNPredicatesRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5eab3f8fe195848369929fb790db61c1";
  }

  static const char* value(const  ::mlnpredicates::MLNPredicatesRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5eab3f8fe1958483ULL;
  static const uint64_t static_value2 = 0x69929fb790db61c1ULL;
};

template<class ContainerAllocator>
struct DataType< ::mlnpredicates::MLNPredicatesRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "mlnpredicates/MLNPredicatesRequest";
  }

  static const char* value(const  ::mlnpredicates::MLNPredicatesRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::mlnpredicates::MLNPredicatesRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string dummy\n\
\n\
";
  }

  static const char* value(const  ::mlnpredicates::MLNPredicatesRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::mlnpredicates::MLNPredicatesResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::mlnpredicates::MLNPredicatesResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::mlnpredicates::MLNPredicatesResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "29e7df00c9ca72a3e0d0414a6ef671b8";
  }

  static const char* value(const  ::mlnpredicates::MLNPredicatesResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x29e7df00c9ca72a3ULL;
  static const uint64_t static_value2 = 0xe0d0414a6ef671b8ULL;
};

template<class ContainerAllocator>
struct DataType< ::mlnpredicates::MLNPredicatesResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "mlnpredicates/MLNPredicatesResponse";
  }

  static const char* value(const  ::mlnpredicates::MLNPredicatesResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::mlnpredicates::MLNPredicatesResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "MLNPredicate[] predicates\n\
\n\
\n\
================================================================================\n\
MSG: mlnpredicates/MLNPredicate\n\
string name\n\
string[] domain\n\
\n\
";
  }

  static const char* value(const  ::mlnpredicates::MLNPredicatesResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::mlnpredicates::MLNPredicatesRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.dummy);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MLNPredicatesRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::mlnpredicates::MLNPredicatesResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.predicates);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MLNPredicatesResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<mlnpredicates::MLNPredicates> {
  static const char* value() 
  {
    return "723d9aac36a8f458c6439a8f71c782b1";
  }

  static const char* value(const mlnpredicates::MLNPredicates&) { return value(); } 
};

template<>
struct DataType<mlnpredicates::MLNPredicates> {
  static const char* value() 
  {
    return "mlnpredicates/MLNPredicates";
  }

  static const char* value(const mlnpredicates::MLNPredicates&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<mlnpredicates::MLNPredicatesRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "723d9aac36a8f458c6439a8f71c782b1";
  }

  static const char* value(const mlnpredicates::MLNPredicatesRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<mlnpredicates::MLNPredicatesRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "mlnpredicates/MLNPredicates";
  }

  static const char* value(const mlnpredicates::MLNPredicatesRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<mlnpredicates::MLNPredicatesResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "723d9aac36a8f458c6439a8f71c782b1";
  }

  static const char* value(const mlnpredicates::MLNPredicatesResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<mlnpredicates::MLNPredicatesResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "mlnpredicates/MLNPredicates";
  }

  static const char* value(const mlnpredicates::MLNPredicatesResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // MLNPREDICATES_SERVICE_MLNPREDICATES_H

