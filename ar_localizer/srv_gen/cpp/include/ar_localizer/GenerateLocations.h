/* Auto-generated by genmsg_cpp for file /home/jan/ros/ar_localizer/srv/GenerateLocations.srv */
#ifndef AR_LOCALIZER_SERVICE_GENERATELOCATIONS_H
#define AR_LOCALIZER_SERVICE_GENERATELOCATIONS_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "ros/service_traits.h"




namespace ar_localizer
{
template <class ContainerAllocator>
struct GenerateLocationsRequest_ : public ros::Message
{
  typedef GenerateLocationsRequest_<ContainerAllocator> Type;

  GenerateLocationsRequest_()
  {
  }

  GenerateLocationsRequest_(const ContainerAllocator& _alloc)
  {
  }


private:
  static const char* __s_getDataType_() { return "ar_localizer/GenerateLocationsRequest"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "d41d8cd98f00b204e9800998ecf8427e"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "358e233cde0c8a8bcfea4ce193f8fc15"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    return size;
  }

  typedef boost::shared_ptr< ::ar_localizer::GenerateLocationsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ar_localizer::GenerateLocationsRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct GenerateLocationsRequest
typedef  ::ar_localizer::GenerateLocationsRequest_<std::allocator<void> > GenerateLocationsRequest;

typedef boost::shared_ptr< ::ar_localizer::GenerateLocationsRequest> GenerateLocationsRequestPtr;
typedef boost::shared_ptr< ::ar_localizer::GenerateLocationsRequest const> GenerateLocationsRequestConstPtr;


template <class ContainerAllocator>
struct GenerateLocationsResponse_ : public ros::Message
{
  typedef GenerateLocationsResponse_<ContainerAllocator> Type;

  GenerateLocationsResponse_()
  : success(false)
  {
  }

  GenerateLocationsResponse_(const ContainerAllocator& _alloc)
  : success(false)
  {
  }

  typedef uint8_t _success_type;
  uint8_t success;


private:
  static const char* __s_getDataType_() { return "ar_localizer/GenerateLocationsResponse"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "358e233cde0c8a8bcfea4ce193f8fc15"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "358e233cde0c8a8bcfea4ce193f8fc15"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "bool success\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, success);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, success);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(success);
    return size;
  }

  typedef boost::shared_ptr< ::ar_localizer::GenerateLocationsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ar_localizer::GenerateLocationsResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct GenerateLocationsResponse
typedef  ::ar_localizer::GenerateLocationsResponse_<std::allocator<void> > GenerateLocationsResponse;

typedef boost::shared_ptr< ::ar_localizer::GenerateLocationsResponse> GenerateLocationsResponsePtr;
typedef boost::shared_ptr< ::ar_localizer::GenerateLocationsResponse const> GenerateLocationsResponseConstPtr;

struct GenerateLocations
{

typedef GenerateLocationsRequest Request;
typedef GenerateLocationsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct GenerateLocations
} // namespace ar_localizer

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::ar_localizer::GenerateLocationsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::ar_localizer::GenerateLocationsRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::ar_localizer::GenerateLocationsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ar_localizer/GenerateLocationsRequest";
  }

  static const char* value(const  ::ar_localizer::GenerateLocationsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ar_localizer::GenerateLocationsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::ar_localizer::GenerateLocationsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::ar_localizer::GenerateLocationsRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::ar_localizer::GenerateLocationsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const  ::ar_localizer::GenerateLocationsResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::ar_localizer::GenerateLocationsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ar_localizer/GenerateLocationsResponse";
  }

  static const char* value(const  ::ar_localizer::GenerateLocationsResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ar_localizer::GenerateLocationsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool success\n\
\n\
";
  }

  static const char* value(const  ::ar_localizer::GenerateLocationsResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::ar_localizer::GenerateLocationsResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ar_localizer::GenerateLocationsRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GenerateLocationsRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ar_localizer::GenerateLocationsResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.success);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GenerateLocationsResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<ar_localizer::GenerateLocations> {
  static const char* value() 
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ar_localizer::GenerateLocations&) { return value(); } 
};

template<>
struct DataType<ar_localizer::GenerateLocations> {
  static const char* value() 
  {
    return "ar_localizer/GenerateLocations";
  }

  static const char* value(const ar_localizer::GenerateLocations&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<ar_localizer::GenerateLocationsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ar_localizer::GenerateLocationsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<ar_localizer::GenerateLocationsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ar_localizer/GenerateLocations";
  }

  static const char* value(const ar_localizer::GenerateLocationsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<ar_localizer::GenerateLocationsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ar_localizer::GenerateLocationsResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<ar_localizer::GenerateLocationsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ar_localizer/GenerateLocations";
  }

  static const char* value(const ar_localizer::GenerateLocationsResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // AR_LOCALIZER_SERVICE_GENERATELOCATIONS_H

