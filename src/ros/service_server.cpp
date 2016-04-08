#include "torch-ros.h"
#include <ros/service_server.h>

ROSIMP(void, ServiceServer, delete)(ros::ServiceServer *ptr) {
  delete ptr;
}

ROSIMP(void, ServiceServer, shutdown)(ros::ServiceServer *self) {
  self->shutdown();
}

ROSIMP(void, ServiceServer, getService)(ros::ServiceServer *self, std::string *result) {
  *result = self->getService();
}
