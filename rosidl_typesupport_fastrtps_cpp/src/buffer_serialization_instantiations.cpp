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

// Explicit template instantiations for common Buffer types
// This ensures the serialize/deserialize symbols are available at link time

#include "rosidl_typesupport_fastrtps_cpp/buffer_serialization.hpp"

namespace eprosima
{
namespace fastcdr
{

// Common numeric types - explicit instantiations
template void serialize(Cdr & cdr, const rosidl_runtime_cpp::Buffer<uint8_t> & buffer);
template void deserialize(Cdr & cdr, rosidl_runtime_cpp::Buffer<uint8_t> & buffer);

template void serialize(Cdr & cdr, const rosidl_runtime_cpp::Buffer<int8_t> & buffer);
template void deserialize(Cdr & cdr, rosidl_runtime_cpp::Buffer<int8_t> & buffer);

template void serialize(Cdr & cdr, const rosidl_runtime_cpp::Buffer<uint16_t> & buffer);
template void deserialize(Cdr & cdr, rosidl_runtime_cpp::Buffer<uint16_t> & buffer);

template void serialize(Cdr & cdr, const rosidl_runtime_cpp::Buffer<int16_t> & buffer);
template void deserialize(Cdr & cdr, rosidl_runtime_cpp::Buffer<int16_t> & buffer);

template void serialize(Cdr & cdr, const rosidl_runtime_cpp::Buffer<uint32_t> & buffer);
template void deserialize(Cdr & cdr, rosidl_runtime_cpp::Buffer<uint32_t> & buffer);

template void serialize(Cdr & cdr, const rosidl_runtime_cpp::Buffer<int32_t> & buffer);
template void deserialize(Cdr & cdr, rosidl_runtime_cpp::Buffer<int32_t> & buffer);

template void serialize(Cdr & cdr, const rosidl_runtime_cpp::Buffer<uint64_t> & buffer);
template void deserialize(Cdr & cdr, rosidl_runtime_cpp::Buffer<uint64_t> & buffer);

template void serialize(Cdr & cdr, const rosidl_runtime_cpp::Buffer<int64_t> & buffer);
template void deserialize(Cdr & cdr, rosidl_runtime_cpp::Buffer<int64_t> & buffer);

template void serialize(Cdr & cdr, const rosidl_runtime_cpp::Buffer<float> & buffer);
template void deserialize(Cdr & cdr, rosidl_runtime_cpp::Buffer<float> & buffer);

template void serialize(Cdr & cdr, const rosidl_runtime_cpp::Buffer<double> & buffer);
template void deserialize(Cdr & cdr, rosidl_runtime_cpp::Buffer<double> & buffer);

}  // namespace fastcdr
}  // namespace eprosima
