#include "torch-std.h"
#include "variable.h"

using namespace xamla;

STDIMP(Variable *, Variable, new)() {
  return new Variable();
}

STDIMP(Variable *, Variable, clone)(Variable *self) {
  return new Variable(*self);
}

STDIMP(void, Variable, delete)(Variable *ptr) {
  delete ptr;
}

STDIMP(int, Variable, get_Type)(Variable *self) {
  return self->get_Type();
}

STDIMP(void, Variable, clear)(Variable *self) {
  self->clear();
}

STDIMP(void, Variable, assign)(Variable *self, Variable *src) {
  *self = *src;
}

#define declare_getter(_name, _type) \
STDIMP(_type, Variable, get_##_name)(Variable *self) { \
  return self->get_##_name(); \
}

declare_getter(bool, bool)
declare_getter(int8, int8_t)
declare_getter(int16, int16_t)
declare_getter(int32, int32_t)
declare_getter(int64, int64_t)
declare_getter(uint8, uint8_t)
declare_getter(uint16, uint16_t)
declare_getter(uint32, uint32_t)
declare_getter(uint64, uint64_t)
declare_getter(float32, float)
declare_getter(float64, double)

#undef declare_getter

STDIMP(const char *, Variable, get_string)(Variable *self) {
  self->get_string().c_str();
}

#define declare_setter(_name, _type) \
STDIMP(void, Variable, set_##_name)(Variable *self, const _type value) { \
  self->set_##_name(value); \
}

declare_setter(bool, bool)
declare_setter(int8, int8_t)
declare_setter(int16, int16_t)
declare_setter(int32, int32_t)
declare_setter(int64, int64_t)
declare_setter(uint8, uint8_t)
declare_setter(uint16, uint16_t)
declare_setter(uint32, uint32_t)
declare_setter(uint64, uint64_t)
declare_setter(float32, float)
declare_setter(float64, double)

#undef declare_setter

STDIMP(void, Variable, set_string)(Variable *self, const char *value) {
  self->set_string(value);
}
