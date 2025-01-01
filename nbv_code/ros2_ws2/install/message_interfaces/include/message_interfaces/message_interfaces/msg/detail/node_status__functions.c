// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from message_interfaces:msg/NodeStatus.idl
// generated code does not contain a copyright notice
#include "message_interfaces/msg/detail/node_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
message_interfaces__msg__NodeStatus__init(message_interfaces__msg__NodeStatus * msg)
{
  if (!msg) {
    return false;
  }
  // is_moving
  // iteration
  // detection_done
  // icp_done
  // octomap_done
  // nbv_done
  // nbv_point_x
  // nbv_point_y
  // nbv_point_z
  // is_final_result
  return true;
}

void
message_interfaces__msg__NodeStatus__fini(message_interfaces__msg__NodeStatus * msg)
{
  if (!msg) {
    return;
  }
  // is_moving
  // iteration
  // detection_done
  // icp_done
  // octomap_done
  // nbv_done
  // nbv_point_x
  // nbv_point_y
  // nbv_point_z
  // is_final_result
}

bool
message_interfaces__msg__NodeStatus__are_equal(const message_interfaces__msg__NodeStatus * lhs, const message_interfaces__msg__NodeStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // is_moving
  if (lhs->is_moving != rhs->is_moving) {
    return false;
  }
  // iteration
  if (lhs->iteration != rhs->iteration) {
    return false;
  }
  // detection_done
  if (lhs->detection_done != rhs->detection_done) {
    return false;
  }
  // icp_done
  if (lhs->icp_done != rhs->icp_done) {
    return false;
  }
  // octomap_done
  if (lhs->octomap_done != rhs->octomap_done) {
    return false;
  }
  // nbv_done
  if (lhs->nbv_done != rhs->nbv_done) {
    return false;
  }
  // nbv_point_x
  if (lhs->nbv_point_x != rhs->nbv_point_x) {
    return false;
  }
  // nbv_point_y
  if (lhs->nbv_point_y != rhs->nbv_point_y) {
    return false;
  }
  // nbv_point_z
  if (lhs->nbv_point_z != rhs->nbv_point_z) {
    return false;
  }
  // is_final_result
  if (lhs->is_final_result != rhs->is_final_result) {
    return false;
  }
  return true;
}

bool
message_interfaces__msg__NodeStatus__copy(
  const message_interfaces__msg__NodeStatus * input,
  message_interfaces__msg__NodeStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // is_moving
  output->is_moving = input->is_moving;
  // iteration
  output->iteration = input->iteration;
  // detection_done
  output->detection_done = input->detection_done;
  // icp_done
  output->icp_done = input->icp_done;
  // octomap_done
  output->octomap_done = input->octomap_done;
  // nbv_done
  output->nbv_done = input->nbv_done;
  // nbv_point_x
  output->nbv_point_x = input->nbv_point_x;
  // nbv_point_y
  output->nbv_point_y = input->nbv_point_y;
  // nbv_point_z
  output->nbv_point_z = input->nbv_point_z;
  // is_final_result
  output->is_final_result = input->is_final_result;
  return true;
}

message_interfaces__msg__NodeStatus *
message_interfaces__msg__NodeStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_interfaces__msg__NodeStatus * msg = (message_interfaces__msg__NodeStatus *)allocator.allocate(sizeof(message_interfaces__msg__NodeStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(message_interfaces__msg__NodeStatus));
  bool success = message_interfaces__msg__NodeStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
message_interfaces__msg__NodeStatus__destroy(message_interfaces__msg__NodeStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    message_interfaces__msg__NodeStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
message_interfaces__msg__NodeStatus__Sequence__init(message_interfaces__msg__NodeStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_interfaces__msg__NodeStatus * data = NULL;

  if (size) {
    data = (message_interfaces__msg__NodeStatus *)allocator.zero_allocate(size, sizeof(message_interfaces__msg__NodeStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = message_interfaces__msg__NodeStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        message_interfaces__msg__NodeStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
message_interfaces__msg__NodeStatus__Sequence__fini(message_interfaces__msg__NodeStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      message_interfaces__msg__NodeStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

message_interfaces__msg__NodeStatus__Sequence *
message_interfaces__msg__NodeStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_interfaces__msg__NodeStatus__Sequence * array = (message_interfaces__msg__NodeStatus__Sequence *)allocator.allocate(sizeof(message_interfaces__msg__NodeStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = message_interfaces__msg__NodeStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
message_interfaces__msg__NodeStatus__Sequence__destroy(message_interfaces__msg__NodeStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    message_interfaces__msg__NodeStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
message_interfaces__msg__NodeStatus__Sequence__are_equal(const message_interfaces__msg__NodeStatus__Sequence * lhs, const message_interfaces__msg__NodeStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!message_interfaces__msg__NodeStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
message_interfaces__msg__NodeStatus__Sequence__copy(
  const message_interfaces__msg__NodeStatus__Sequence * input,
  message_interfaces__msg__NodeStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(message_interfaces__msg__NodeStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    message_interfaces__msg__NodeStatus * data =
      (message_interfaces__msg__NodeStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!message_interfaces__msg__NodeStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          message_interfaces__msg__NodeStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!message_interfaces__msg__NodeStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
