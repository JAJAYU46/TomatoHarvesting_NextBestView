// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from message_interfaces:msg/BoundingBox.idl
// generated code does not contain a copyright notice
#include "message_interfaces/msg/detail/bounding_box__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
message_interfaces__msg__BoundingBox__init(message_interfaces__msg__BoundingBox * msg)
{
  if (!msg) {
    return false;
  }
  // lu_x
  // lu_y
  // rd_x
  // rd_y
  return true;
}

void
message_interfaces__msg__BoundingBox__fini(message_interfaces__msg__BoundingBox * msg)
{
  if (!msg) {
    return;
  }
  // lu_x
  // lu_y
  // rd_x
  // rd_y
}

bool
message_interfaces__msg__BoundingBox__are_equal(const message_interfaces__msg__BoundingBox * lhs, const message_interfaces__msg__BoundingBox * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // lu_x
  if (lhs->lu_x != rhs->lu_x) {
    return false;
  }
  // lu_y
  if (lhs->lu_y != rhs->lu_y) {
    return false;
  }
  // rd_x
  if (lhs->rd_x != rhs->rd_x) {
    return false;
  }
  // rd_y
  if (lhs->rd_y != rhs->rd_y) {
    return false;
  }
  return true;
}

bool
message_interfaces__msg__BoundingBox__copy(
  const message_interfaces__msg__BoundingBox * input,
  message_interfaces__msg__BoundingBox * output)
{
  if (!input || !output) {
    return false;
  }
  // lu_x
  output->lu_x = input->lu_x;
  // lu_y
  output->lu_y = input->lu_y;
  // rd_x
  output->rd_x = input->rd_x;
  // rd_y
  output->rd_y = input->rd_y;
  return true;
}

message_interfaces__msg__BoundingBox *
message_interfaces__msg__BoundingBox__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_interfaces__msg__BoundingBox * msg = (message_interfaces__msg__BoundingBox *)allocator.allocate(sizeof(message_interfaces__msg__BoundingBox), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(message_interfaces__msg__BoundingBox));
  bool success = message_interfaces__msg__BoundingBox__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
message_interfaces__msg__BoundingBox__destroy(message_interfaces__msg__BoundingBox * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    message_interfaces__msg__BoundingBox__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
message_interfaces__msg__BoundingBox__Sequence__init(message_interfaces__msg__BoundingBox__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_interfaces__msg__BoundingBox * data = NULL;

  if (size) {
    data = (message_interfaces__msg__BoundingBox *)allocator.zero_allocate(size, sizeof(message_interfaces__msg__BoundingBox), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = message_interfaces__msg__BoundingBox__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        message_interfaces__msg__BoundingBox__fini(&data[i - 1]);
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
message_interfaces__msg__BoundingBox__Sequence__fini(message_interfaces__msg__BoundingBox__Sequence * array)
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
      message_interfaces__msg__BoundingBox__fini(&array->data[i]);
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

message_interfaces__msg__BoundingBox__Sequence *
message_interfaces__msg__BoundingBox__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_interfaces__msg__BoundingBox__Sequence * array = (message_interfaces__msg__BoundingBox__Sequence *)allocator.allocate(sizeof(message_interfaces__msg__BoundingBox__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = message_interfaces__msg__BoundingBox__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
message_interfaces__msg__BoundingBox__Sequence__destroy(message_interfaces__msg__BoundingBox__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    message_interfaces__msg__BoundingBox__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
message_interfaces__msg__BoundingBox__Sequence__are_equal(const message_interfaces__msg__BoundingBox__Sequence * lhs, const message_interfaces__msg__BoundingBox__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!message_interfaces__msg__BoundingBox__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
message_interfaces__msg__BoundingBox__Sequence__copy(
  const message_interfaces__msg__BoundingBox__Sequence * input,
  message_interfaces__msg__BoundingBox__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(message_interfaces__msg__BoundingBox);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    message_interfaces__msg__BoundingBox * data =
      (message_interfaces__msg__BoundingBox *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!message_interfaces__msg__BoundingBox__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          message_interfaces__msg__BoundingBox__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!message_interfaces__msg__BoundingBox__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
