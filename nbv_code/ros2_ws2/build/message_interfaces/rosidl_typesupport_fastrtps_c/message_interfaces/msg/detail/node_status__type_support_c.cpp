// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from message_interfaces:msg/NodeStatus.idl
// generated code does not contain a copyright notice
#include "message_interfaces/msg/detail/node_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "message_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "message_interfaces/msg/detail/node_status__struct.h"
#include "message_interfaces/msg/detail/node_status__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _NodeStatus__ros_msg_type = message_interfaces__msg__NodeStatus;

static bool _NodeStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _NodeStatus__ros_msg_type * ros_message = static_cast<const _NodeStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: is_moving
  {
    cdr << (ros_message->is_moving ? true : false);
  }

  // Field name: iteration
  {
    cdr << ros_message->iteration;
  }

  // Field name: detection_done
  {
    cdr << (ros_message->detection_done ? true : false);
  }

  // Field name: icp_done
  {
    cdr << (ros_message->icp_done ? true : false);
  }

  // Field name: octomap_done
  {
    cdr << (ros_message->octomap_done ? true : false);
  }

  // Field name: nbv_done
  {
    cdr << (ros_message->nbv_done ? true : false);
  }

  // Field name: nbv_point_x
  {
    cdr << ros_message->nbv_point_x;
  }

  // Field name: nbv_point_y
  {
    cdr << ros_message->nbv_point_y;
  }

  // Field name: nbv_point_z
  {
    cdr << ros_message->nbv_point_z;
  }

  // Field name: is_final_result
  {
    cdr << (ros_message->is_final_result ? true : false);
  }

  return true;
}

static bool _NodeStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _NodeStatus__ros_msg_type * ros_message = static_cast<_NodeStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: is_moving
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_moving = tmp ? true : false;
  }

  // Field name: iteration
  {
    cdr >> ros_message->iteration;
  }

  // Field name: detection_done
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->detection_done = tmp ? true : false;
  }

  // Field name: icp_done
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->icp_done = tmp ? true : false;
  }

  // Field name: octomap_done
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->octomap_done = tmp ? true : false;
  }

  // Field name: nbv_done
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->nbv_done = tmp ? true : false;
  }

  // Field name: nbv_point_x
  {
    cdr >> ros_message->nbv_point_x;
  }

  // Field name: nbv_point_y
  {
    cdr >> ros_message->nbv_point_y;
  }

  // Field name: nbv_point_z
  {
    cdr >> ros_message->nbv_point_z;
  }

  // Field name: is_final_result
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_final_result = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_message_interfaces
size_t get_serialized_size_message_interfaces__msg__NodeStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _NodeStatus__ros_msg_type * ros_message = static_cast<const _NodeStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name is_moving
  {
    size_t item_size = sizeof(ros_message->is_moving);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name iteration
  {
    size_t item_size = sizeof(ros_message->iteration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name detection_done
  {
    size_t item_size = sizeof(ros_message->detection_done);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name icp_done
  {
    size_t item_size = sizeof(ros_message->icp_done);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name octomap_done
  {
    size_t item_size = sizeof(ros_message->octomap_done);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name nbv_done
  {
    size_t item_size = sizeof(ros_message->nbv_done);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name nbv_point_x
  {
    size_t item_size = sizeof(ros_message->nbv_point_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name nbv_point_y
  {
    size_t item_size = sizeof(ros_message->nbv_point_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name nbv_point_z
  {
    size_t item_size = sizeof(ros_message->nbv_point_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_final_result
  {
    size_t item_size = sizeof(ros_message->is_final_result);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _NodeStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_message_interfaces__msg__NodeStatus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_message_interfaces
size_t max_serialized_size_message_interfaces__msg__NodeStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: is_moving
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: iteration
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: detection_done
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: icp_done
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: octomap_done
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: nbv_done
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: nbv_point_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: nbv_point_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: nbv_point_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: is_final_result
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = message_interfaces__msg__NodeStatus;
    is_plain =
      (
      offsetof(DataType, is_final_result) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _NodeStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_message_interfaces__msg__NodeStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_NodeStatus = {
  "message_interfaces::msg",
  "NodeStatus",
  _NodeStatus__cdr_serialize,
  _NodeStatus__cdr_deserialize,
  _NodeStatus__get_serialized_size,
  _NodeStatus__max_serialized_size
};

static rosidl_message_type_support_t _NodeStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_NodeStatus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, message_interfaces, msg, NodeStatus)() {
  return &_NodeStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
