#include "torch-ros.h"
#include <ros/console.h>
#include <boost/unordered_map.hpp>

namespace xamla {

typedef boost::unordered_map<std::string, boost::shared_ptr<std::vector<ros::console::LogLocation> > > NamedLoggerMap;
NamedLoggerMap global_named_locs;

ros::console::LogLocation *getNamedLoc(const std::string& name, int level) {
  if (level < 0 || level >= ::ros::console::levels::Count)
    throw std::out_of_range("level");

  NamedLoggerMap::iterator i = global_named_locs.find(name);
  if (i == global_named_locs.end()) {
    static ros::console::LogLocation __empty = { false, false, ::ros::console::levels::Count, 0 };

    boost::shared_ptr<std::vector<ros::console::LogLocation> > levelLoggers(new std::vector<ros::console::LogLocation>(::ros::console::levels::Count, __empty));
    for (int j = 0; j < ::ros::console::levels::Count; ++j) {
      ros::console::initializeLogLocation(&levelLoggers->operator[](j), name, (ros::console::Level)j);
    }

    i = global_named_locs.insert(std::make_pair(name, levelLoggers)).first;
  }
  return &i->second->operator[](level);
}

inline std::string getLoggerName(const char *name, bool no_default_prefix) {
  if (!name)
    return ROSCONSOLE_DEFAULT_NAME;
  else if (no_default_prefix)
    return name;
  else
    return std::string(ROSCONSOLE_NAME_PREFIX) + "." + name;
}

} // namespace xamla


ROSIMP(const char *, Console, initialize)() {
  ros::console::initialize();
  xamla::getNamedLoc(ROSCONSOLE_DEFAULT_NAME, (int)ros::console::levels::Info);
  return ROSCONSOLE_NAME_PREFIX;
}

ROSIMP(void, Console, shutdown)() {
  ros::console::shutdown();
  xamla::global_named_locs.clear();
}

ROSIMP(void, Console, set_logger_level)(const char *name, int level, bool no_default_prefix) {
  const std::string& name_ = xamla::getLoggerName(name, no_default_prefix);
  ros::console::setLogLocationLevel(xamla::getNamedLoc(name_, level), (ros::console::levels::Level)level);
  if (ros::console::set_logger_level(name_, (ros::console::levels::Level)level))
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

ROSIMP(bool, Console, check_loglevel)(const char *name, int level, bool no_default_prefix) {
  const std::string& name_ = xamla::getLoggerName(name, no_default_prefix);
  ros::console::LogLocation *loc = xamla::getNamedLoc(name_, level);
  return loc != NULL && loc->logger_enabled_;
}

ROSIMP(void*, Console, get_logger)(const char *name, bool no_default_prefix) {
  const std::string& name_ = xamla::getLoggerName(name, no_default_prefix);
  return xamla::getNamedLoc(name_, ros::console::levels::Info)->logger_;
}

ROSIMP(void, Console, print)(void *logger, int level, const char *text, const char *file, const char *function_name, int line) {
  if (!logger) {
    logger = xamla::getNamedLoc(ROSCONSOLE_DEFAULT_NAME, level)->logger_;
  }

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
