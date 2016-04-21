#include "torch-ros.h"
#include <ros/console.h>
#include "log4cxx/logger.h"

ros::console::LogLocation global_locs[ros::console::levels::Count];

ROSIMP(void, Console, initialize)() {
  ros::console::initialize();
  for (int l = ros::console::levels::Debug; l < ros::console::levels::Count; ++l) {
    ros::console::initializeLogLocation(&global_locs[l], ROSCONSOLE_DEFAULT_NAME, static_cast<ros::console::levels::Level>(l));
  }
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

ROSIMP(bool, Console, check_loglevel)(int level) {
  return level >= 0 && level < ros::console::levels::Count && global_locs[level].logger_enabled_;
}

ROSIMP(void*, Console, get_logger)(const char *name) {
  return &(*log4cxx::Logger::getLogger(name));
}

ROSIMP(void, Console, print)(void *logger, int level, const char *text, const char *file, const char *function_name, int line) {
  static void *default_logger = global_locs[ros::console::levels::Info].logger_;

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
