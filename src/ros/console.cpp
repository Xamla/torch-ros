#include "torch-ros.h"
#include <ros/console.h>

ROSIMP(void, Console, initialize)() {
  ros::console::initialize();
}

ROSIMP(void, Console, shutdown)() {
  ros::console::shutdown();
}

ROSIMP(void, Console, set_logger_level)(const char *name, int level) {
  if (!name)
    name = ROSCONSOLE_DEFAULT_NAME;
  if (ros::console::set_logger_level(name, (ros::console::levels::Level)level))
    ros::console::notifyLoggerLevelsChanged();
}

ROSIMP(bool, Console, get_loggers)(std::vector<std::string> *names, THShortTensor *levels) {
  std::map<std::string, ros::console::levels::Level> loggers;
  if (!ros::console::get_loggers(loggers))
    return false;

  THShortTensor *levels_ = THShortTensor_newContiguous(levels);
  THShortTensor_resize1d(levels_, loggers.size());
  names->clear();
  names->reserve(loggers.size());

  std::map<std::string, ros::console::levels::Level>::const_iterator i = loggers.begin();
  short *levels_data = THShortTensor_data(levels_);
  for (; i != loggers.end(); ++i, ++levels_data) {
    names->push_back(i->first);
    *levels_data = i->second;
  }
  THShortTensor_freeCopyTo(levels_, levels);
  return true;
}

ROSIMP(void*, Console, get_logger)(const char *name) {
  ros::console::LogLocation dummy = { false, false, ros::console::levels::Count, 0 };
  ros::console::initializeLogLocation(&dummy, name, ros::console::levels::Info);
  return dummy.logger_;
}

ROSIMP(void, Console, print)(void *logger, int level, const char *text, const char *file, const char *function_name, int line) {
  static void *default_logger = ros_Console_get_logger(ROSCONSOLE_DEFAULT_NAME);

  if (!logger)
    logger = default_logger;

  ros::console::print(
    NULL,
    logger,
    (ros::console::levels::Level)level,
    file,
    line,
    function_name,
    "%s",
    text
  );
}
