// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from message_interfaces:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_INTERFACES__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
#define MESSAGE_INTERFACES__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/BoundingBox in the package message_interfaces.
/**
  * BoundingBox
 */
typedef struct message_interfaces__msg__BoundingBox
{
  /// Top-left corner x-coordinate
  int32_t lu_x;
  /// Top-left corner y-coordinate
  int32_t lu_y;
  /// Bottom-right corner x-coordinate
  int32_t rd_x;
  /// Bottom-right corner y-coordinate
  int32_t rd_y;
} message_interfaces__msg__BoundingBox;

// Struct for a sequence of message_interfaces__msg__BoundingBox.
typedef struct message_interfaces__msg__BoundingBox__Sequence
{
  message_interfaces__msg__BoundingBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} message_interfaces__msg__BoundingBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGE_INTERFACES__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
