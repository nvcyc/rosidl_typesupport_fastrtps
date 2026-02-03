// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rosidl_typesupport_fastrtps_cpp/buffer_serialization.hpp"

// Explicit template instantiation for commonly used Buffer types
// This ensures the template functions are available for linking

namespace eprosima
{
namespace fastcdr
{

// Explicit instantiation for Buffer<uint8_t> (most common case for image data, etc.)
template void serialize<rcl_buffer::Buffer<uint8_t, std::allocator<uint8_t>>>(
  Cdr & cdr,
  const rcl_buffer::Buffer<uint8_t, std::allocator<uint8_t>> & buffer);

template void deserialize<rcl_buffer::Buffer<uint8_t, std::allocator<uint8_t>>>(
  Cdr & cdr,
  rcl_buffer::Buffer<uint8_t, std::allocator<uint8_t>> & buffer);

template Cdr & operator<<<rcl_buffer::Buffer<uint8_t, std::allocator<uint8_t>>>(
  Cdr & cdr,
  const rcl_buffer::Buffer<uint8_t, std::allocator<uint8_t>> & buffer);

template Cdr & operator>><rcl_buffer::Buffer<uint8_t, std::allocator<uint8_t>>>(
  Cdr & cdr,
  rcl_buffer::Buffer<uint8_t, std::allocator<uint8_t>> & buffer);

}  // namespace fastcdr
}  // namespace eprosima
