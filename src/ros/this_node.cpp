#include "torch-ros.h"
#include "../std/torch-std.h"
#include <ros/this_node.h>

ROSIMP(const char *, ThisNode, getName)() {
  return ros::this_node::getName().c_str();
}

ROSIMP(const char *, ThisNode, getNamespace)() {
  return ros::this_node::getNamespace().c_str();
}

ROSIMP(void, ThisNode, getAdvertisedTopics)(StringVector *topics) {
  ros::this_node::getAdvertisedTopics(*topics);
}

ROSIMP(void, ThisNode, getSubscribedTopics)(StringVector *topics) {
  ros::this_node::getSubscribedTopics(*topics);
}
