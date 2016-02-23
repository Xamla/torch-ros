#ifndef torch_std_h
#define torch_std_h

extern "C" {
#include <TH/TH.h>
}

#include <string>
#include <vector>

#define STDIMP(return_type, class_name, name) extern "C" return_type TH_CONCAT_4(std_, class_name, _, name)

typedef std::vector<std::string> StringVector;

#endif // torch_std_h
