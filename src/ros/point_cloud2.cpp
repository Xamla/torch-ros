#include "torch-ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// torch-pcl interop methods to read and write PointCloud2 messages.


ROSIMP(int32_t, pcl, readPointCloud2)(THByteStorage *serialized_message, int32_t offset, pcl::PCLPointCloud2 *cloud) {
  // deserialize to sensor_msgs::PointCloud2 message
  long buffer_length = THByteStorage_size(serialized_message);
  if (offset < 0 || offset > buffer_length)
    throw RosWrapperException("Offset out of range");

  uint8_t *buffer = THByteStorage_data(serialized_message);

  ros::serialization::IStream stream(buffer + offset, static_cast<uint32_t>(buffer_length - offset));
  sensor_msgs::PointCloud2 cloud_msg;
  ros::serialization::deserialize(stream, cloud_msg);

  // convert to pcl::PointCloud2
  pcl_conversions::toPCL(cloud_msg, *cloud);

  return static_cast<int32_t>(stream.getData() - buffer);   // return new offset
}


ROSIMP(int32_t, pcl, writePointCloud2)(THByteStorage *serialized_message, int32_t offset, pcl::PCLPointCloud2 *cloud) {
  // convert to sensor_msgs:PointCloud2
  sensor_msgs::PointCloud2 cloud_msg;
  if (cloud != NULL) {
    pcl_conversions::fromPCL(*cloud, cloud_msg);
  }

  // determine serialization length & resize output buffer
  uint32_t length = ros::serialization::serializationLength(cloud_msg);

  // check if buffer length is sufficient
  if (THByteStorage_size(serialized_message) < offset + length) {
    THByteStorage_resize(serialized_message, offset + length);
  }

  uint8_t *buffer = THByteStorage_data(serialized_message);

  // write message
  ros::serialization::OStream stream(buffer + offset, THByteStorage_size(serialized_message) - offset);
  ros::serialization::Serializer<sensor_msgs::PointCloud2>::write(stream, cloud_msg);

  return static_cast<int32_t>(stream.getData() - buffer);   // return new offset
}
