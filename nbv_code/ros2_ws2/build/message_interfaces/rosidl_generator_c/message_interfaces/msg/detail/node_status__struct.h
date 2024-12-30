// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from message_interfaces:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_INTERFACES__MSG__DETAIL__NODE_STATUS__STRUCT_H_
#define MESSAGE_INTERFACES__MSG__DETAIL__NODE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/NodeStatus in the package message_interfaces.
/**
  * will record the ismoving of the camera
 */
typedef struct message_interfaces__msg__NodeStatus
{
  /// <Debug> msg field 要is_moving才行, 不可isMoving
  bool is_moving;
} message_interfaces__msg__NodeStatus;

// Struct for a sequence of message_interfaces__msg__NodeStatus.
typedef struct message_interfaces__msg__NodeStatus__Sequence
{
  message_interfaces__msg__NodeStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} message_interfaces__msg__NodeStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGE_INTERFACES__MSG__DETAIL__NODE_STATUS__STRUCT_H_
