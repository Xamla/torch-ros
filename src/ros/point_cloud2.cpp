#include "torch-ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// torch-pcl interop methods to read and write PointCloud2 messages.


ROSIMP(void, pcl, readPointCloud2)(THByteStorage *serialized_message, pcl::PCLPointCloud2 *cloud) {
  // deserialize to sensor_msgs::PointCloud2 message
  uint8_t *buffer = THByteStorage_data(serialized_message);
  uint32_t buffer_length = THByteStorage_size(serialized_message);
  ros::serialization::IStream stream(buffer, buffer_length);
  sensor_msgs::PointCloud2 cloud_msg;
  ros::serialization::deserialize(stream, cloud_msg);

  // convert to pcl::PointCloud2
  pcl_conversions::toPCL(cloud_msg, *cloud);
}


ROSIMP(void, pcl, writePointCloud2)(pcl::PCLPointCloud2 *cloud, THByteStorage *serialized_message) {
  // convert to sensor_msgs:PointCloud2
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(*cloud, cloud_msg);

  // determine serialization length & resize output buffer
  uint32_t length = ros::serialization::serializationLength(cloud_msg);
  THByteStorage_resize(serialized_message, length);
  uint8_t *buffer = THByteStorage_data(serialized_message);

  // write message
  ros::serialization::OStream stream(buffer, THByteStorage_size(serialized_message));
  ros::serialization::Serializer<sensor_msgs::PointCloud2>::write(stream, cloud_msg);
}
