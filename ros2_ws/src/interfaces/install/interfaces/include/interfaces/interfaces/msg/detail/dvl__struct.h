// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/DVL.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__DVL__STRUCT_H_
#define INTERFACES__MSG__DETAIL__DVL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/DVL in the package interfaces.
typedef struct interfaces__msg__DVL
{
  std_msgs__msg__Header header;
  /// Body-frame velocity
  geometry_msgs__msg__Vector3 velocity;
  double covariance[9];
} interfaces__msg__DVL;

// Struct for a sequence of interfaces__msg__DVL.
typedef struct interfaces__msg__DVL__Sequence
{
  interfaces__msg__DVL * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__DVL__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__DVL__STRUCT_H_
