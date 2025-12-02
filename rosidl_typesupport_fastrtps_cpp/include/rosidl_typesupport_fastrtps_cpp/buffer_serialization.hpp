// Copyright 2024 NVIDIA Corporation
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

#ifndef ROSIDL_TYPESUPPORT_FASTRTPS_CPP__BUFFER_SERIALIZATION_HPP_
#define ROSIDL_TYPESUPPORT_FASTRTPS_CPP__BUFFER_SERIALIZATION_HPP_

#include <vector>

#include "rosidl_runtime_cpp/buffer.hpp"
#include "fastcdr/Cdr.h"

namespace eprosima
{
namespace fastcdr
{

/// Serialize Buffer<T> using operator<< overload.
/// For CPU backend, uses implicit conversion to std::vector<T>.
/// This is a temporary solution for Phase 1-2. Phase 3 will add full backend support.
template<typename T, typename Allocator>
inline Cdr & operator<<(Cdr & cdr, const rosidl_runtime_cpp::Buffer<T, Allocator> & buffer)
{
  // For CPU backend, use implicit conversion to std::vector
  if (buffer.get_backend_type() == "cpu") {
    const std::vector<T> & vec = buffer;
    cdr << vec;
  } else {
    // For non-CPU backends, this will be handled by backend registry in Phase 3
    throw std::runtime_error(
            "Serialization of non-CPU buffer backends requires Phase 3 implementation");
  }
  return cdr;
}

/// Deserialize into Buffer<T> using operator>> overload.
template<typename T, typename Allocator>
inline Cdr & operator>>(Cdr & cdr, rosidl_runtime_cpp::Buffer<T, Allocator> & buffer)
{
  // Deserialize as std::vector (CPU backend)
  std::vector<T> vec;
  cdr >> vec;
  
  // Copy into buffer (which is CPU backend by default)
  buffer.resize(vec.size());
  for (size_t i = 0; i < vec.size(); ++i) {
    buffer[i] = vec[i];
  }
  return cdr;
}

}  // namespace fastcdr
}  // namespace eprosima

#endif  // ROSIDL_TYPESUPPORT_FASTRTPS_CPP__BUFFER_SERIALIZATION_HPP_

