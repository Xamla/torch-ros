#include "torch-ros.h"

ROSIMP(void, _, init)(const char *name, uint32_t options)
{
  int argc = 0;
  ros::init(argc, NULL, name, options);
}

ROSIMP(void, _, shutdown)()
{
  ros::shutdown();
}

ROSIMP(void, _, spinOnce)()
{
  ros::spinOnce();
}

ROSIMP(void, _, requestShutdown)()
{
  ros::requestShutdown();
}

ROSIMP(bool, _, isInitialized)()
{
  return ros::isInitialized();
}

ROSIMP(bool, _, isStarted)()
{
  return ros::isStarted();
}

ROSIMP(bool, _, isShuttingDown)()
{
  return ros::isShuttingDown();
}

ROSIMP(bool, _, ok)()
{
  return ros::ok();
}

ROSIMP(void, _, waitForShutdown)()
{
  return ros::waitForShutdown();
}
