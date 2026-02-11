// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interfaces:msg/DVL.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interfaces/msg/detail/dvl__rosidl_typesupport_introspection_c.h"
#include "interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interfaces/msg/detail/dvl__functions.h"
#include "interfaces/msg/detail/dvl__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `velocity`
#include "geometry_msgs/msg/vector3.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void interfaces__msg__DVL__rosidl_typesupport_introspection_c__DVL_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interfaces__msg__DVL__init(message_memory);
}

void interfaces__msg__DVL__rosidl_typesupport_introspection_c__DVL_fini_function(void * message_memory)
{
  interfaces__msg__DVL__fini(message_memory);
}

size_t interfaces__msg__DVL__rosidl_typesupport_introspection_c__size_function__DVL__covariance(
  const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * interfaces__msg__DVL__rosidl_typesupport_introspection_c__get_const_function__DVL__covariance(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * interfaces__msg__DVL__rosidl_typesupport_introspection_c__get_function__DVL__covariance(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void interfaces__msg__DVL__rosidl_typesupport_introspection_c__fetch_function__DVL__covariance(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    interfaces__msg__DVL__rosidl_typesupport_introspection_c__get_const_function__DVL__covariance(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void interfaces__msg__DVL__rosidl_typesupport_introspection_c__assign_function__DVL__covariance(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    interfaces__msg__DVL__rosidl_typesupport_introspection_c__get_function__DVL__covariance(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember interfaces__msg__DVL__rosidl_typesupport_introspection_c__DVL_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__DVL, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__DVL, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "covariance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__DVL, covariance),  // bytes offset in struct
    NULL,  // default value
    interfaces__msg__DVL__rosidl_typesupport_introspection_c__size_function__DVL__covariance,  // size() function pointer
    interfaces__msg__DVL__rosidl_typesupport_introspection_c__get_const_function__DVL__covariance,  // get_const(index) function pointer
    interfaces__msg__DVL__rosidl_typesupport_introspection_c__get_function__DVL__covariance,  // get(index) function pointer
    interfaces__msg__DVL__rosidl_typesupport_introspection_c__fetch_function__DVL__covariance,  // fetch(index, &value) function pointer
    interfaces__msg__DVL__rosidl_typesupport_introspection_c__assign_function__DVL__covariance,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers interfaces__msg__DVL__rosidl_typesupport_introspection_c__DVL_message_members = {
  "interfaces__msg",  // message namespace
  "DVL",  // message name
  3,  // number of fields
  sizeof(interfaces__msg__DVL),
  interfaces__msg__DVL__rosidl_typesupport_introspection_c__DVL_message_member_array,  // message members
  interfaces__msg__DVL__rosidl_typesupport_introspection_c__DVL_init_function,  // function to initialize message memory (memory has to be allocated)
  interfaces__msg__DVL__rosidl_typesupport_introspection_c__DVL_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t interfaces__msg__DVL__rosidl_typesupport_introspection_c__DVL_message_type_support_handle = {
  0,
  &interfaces__msg__DVL__rosidl_typesupport_introspection_c__DVL_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interfaces, msg, DVL)() {
  interfaces__msg__DVL__rosidl_typesupport_introspection_c__DVL_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  interfaces__msg__DVL__rosidl_typesupport_introspection_c__DVL_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!interfaces__msg__DVL__rosidl_typesupport_introspection_c__DVL_message_type_support_handle.typesupport_identifier) {
    interfaces__msg__DVL__rosidl_typesupport_introspection_c__DVL_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &interfaces__msg__DVL__rosidl_typesupport_introspection_c__DVL_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
