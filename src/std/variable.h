#ifndef _variable_h
#define _variable_h

#include <stdint.h>
#include "exceptions.h"
#include <vector>
#include <map>

namespace xamla {

struct VariableType {
  enum Enum {
    Void = 0,
    Bool = 1,
    Int8 = 2,
    Int16 = 3,
    Int32 = 4,
    Int64 = 5,
    UInt8 = 6,
    UInt16 = 7,
    UInt32 = 8,
    UInt64 = 9,
    Float32 = 10,
    Float64 = 11,
    String = 12,
    Vector = 13,      // one-dimensional variable array
    Table = 14        // unique string to variable mapping
  };
};

class Variable {
public:
  Variable()
    : type_code(VariableType::Void) {
    this->value.i64 = 0;
  }

  Variable(const Variable &src) {
    if (src.type_code == VariableType::String) {
      this->set_string(*src.value.s);
    } else if (src.type_code == VariableType::Vector) {
      this->set_vector(*src.value.v);
    } else if (src.type_code == VariableType::Table) {
      this->set_table(*src.value.t);
    } else {
      this->type_code = src.type_code;
      this->value = src.value;
    }
  }

#define declare_ctor(_type, _field, _type_code) \
  Variable(const _type &value) \
    : type_code(_type_code) { \
      this->value._field =  value; \
  }

  declare_ctor(bool,      b,    VariableType::Bool)
  declare_ctor(int8_t,    i8,   VariableType::Int8)
  declare_ctor(int16_t,   i16,  VariableType::Int16)
  declare_ctor(int32_t,   i32,  VariableType::Int32)
  declare_ctor(int64_t,   i64,  VariableType::Int64)
  declare_ctor(uint8_t,   ui8,  VariableType::UInt8)
  declare_ctor(uint16_t,  ui16, VariableType::UInt16)
  declare_ctor(uint32_t,  ui32, VariableType::UInt32)
  declare_ctor(uint64_t,  ui64, VariableType::UInt64)
  declare_ctor(float,     f32,  VariableType::Float32)
  declare_ctor(double,    f64,  VariableType::Float64)

#undef declare_ctor

  Variable(const std::string &value)
    : type_code(VariableType::String) {
    this->value.s =  new std::string(value);
  }

  ~Variable() {
    this->clear();
  }

  VariableType::Enum get_type() const {
    return type_code;
  }

  void clear() {
    if (type_code == VariableType::String)
      delete this->value.s;
    else if (type_code == VariableType::Vector)
      delete this->value.v;
    else if (type_code == VariableType::Table)
      delete this->value.t;
    this->type_code = VariableType::Void;
    this->value.i64 = 0;
  }

#define declare_accessor(_name, _type, _field, _type_code) \
  void set_##_name(const _type& value) { \
    clear(); \
    this->value._field = value; \
    this->type_code = _type_code; \
  } \
  _type get_##_name() const { \
    ensureType(_type_code); \
    return this->value._field; \
  }

  declare_accessor(bool, bool, b, VariableType::Bool)
  declare_accessor(int8, int8_t, i8, VariableType::Int8)
  declare_accessor(int16, int16_t, i16, VariableType::Int16)
  declare_accessor(int32, int32_t, i32, VariableType::Int32)
  declare_accessor(int64, int64_t, i64, VariableType::Int64)
  declare_accessor(uint8, uint8_t, ui8, VariableType::UInt8)
  declare_accessor(uint16, uint16_t, ui16, VariableType::UInt16)
  declare_accessor(uint32, uint32_t, ui32, VariableType::UInt32)
  declare_accessor(uint64, uint64_t, ui64, VariableType::UInt64)
  declare_accessor(float32, float, f32, VariableType::Float32)
  declare_accessor(float64, double, f64, VariableType::Float64)

#undef declare_accessor

  void set_string(const std::string &value) {
    clear();
    this->value.s = new std::string(value);
    this->type_code = VariableType::String;
  }
  const std::string &get_string() const {
    ensureType(VariableType::String);
    return *this->value.s;
  }

  typedef std::vector<Variable> vector_t;
  void set_vector(const vector_t &value) {
    clear();
    this->value.v = new vector_t(value);
    this->type_code = VariableType::Vector;
  }
  vector_t &get_vector() const {
    ensureType(VariableType::Vector);
    return *this->value.v;
  }

  typedef std::map<std::string, Variable> table_t;
  void set_table(const table_t &value) {
    clear();
    this->value.t = new table_t(value);
    this->type_code = VariableType::Table;
  }
  table_t &get_table() {
    ensureType(VariableType::Table);
    return *this->value.t;
  }

  Variable &operator=(const Variable &src) {
    if (this == &src)
      return *this;

    this->clear();

    if (src.type_code == VariableType::String) {
      this->set_string(*src.value.s);
    } else if (src.type_code == VariableType::Vector) {
      this->set_vector(*src.value.v);
    } else if (src.type_code == VariableType::Table) {
      this->set_table(*src.value.t);
    } else {
      this->type_code = src.type_code;
      this->value = src.value;
    }

    return *this;
  }

private:
  VariableType::Enum type_code;

  union {
    bool b;
    int8_t i8;
    int16_t i16;
    int32_t i32;
    int64_t i64;
    uint8_t ui8;
    uint16_t ui16;
    uint32_t ui32;
    uint64_t ui64;
    float f32;
    double f64;
    std::string *s;
    std::vector<Variable> *v;
    std::map<std::string, Variable> *t;
  } value;

  void ensureType(VariableType::Enum expected) const {
    if (this->type_code != expected) {
      throw InvalidTypeException("Variable does not hold requested type.");
    }
  }
};

}

#endif