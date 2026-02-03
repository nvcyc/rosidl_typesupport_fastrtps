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

// Explicit template instantiations for common Buffer types
// This ensures the serialize/deserialize symbols are available at link time

#include "rosidl_typesupport_fastrtps_cpp/buffer_serialization.hpp"

namespace eprosima
{
namespace fastcdr
{

// Common numeric types - explicit instantiations with explicit allocator
template void serialize(Cdr & cdr, const rcl_buffer::Buffer<uint8_t, std::allocator<uint8_t>> & buffer);
template void deserialize(Cdr & cdr, rcl_buffer::Buffer<uint8_t, std::allocator<uint8_t>> & buffer);

template void serialize(Cdr & cdr, const rcl_buffer::Buffer<int8_t, std::allocator<int8_t>> & buffer);
template void deserialize(Cdr & cdr, rcl_buffer::Buffer<int8_t, std::allocator<int8_t>> & buffer);

template void serialize(Cdr & cdr, const rcl_buffer::Buffer<uint16_t, std::allocator<uint16_t>> & buffer);
template void deserialize(Cdr & cdr, rcl_buffer::Buffer<uint16_t, std::allocator<uint16_t>> & buffer);

template void serialize(Cdr & cdr, const rcl_buffer::Buffer<int16_t, std::allocator<int16_t>> & buffer);
template void deserialize(Cdr & cdr, rcl_buffer::Buffer<int16_t, std::allocator<int16_t>> & buffer);

template void serialize(Cdr & cdr, const rcl_buffer::Buffer<uint32_t, std::allocator<uint32_t>> & buffer);
template void deserialize(Cdr & cdr, rcl_buffer::Buffer<uint32_t, std::allocator<uint32_t>> & buffer);

template void serialize(Cdr & cdr, const rcl_buffer::Buffer<int32_t, std::allocator<int32_t>> & buffer);
template void deserialize(Cdr & cdr, rcl_buffer::Buffer<int32_t, std::allocator<int32_t>> & buffer);

template void serialize(Cdr & cdr, const rcl_buffer::Buffer<uint64_t, std::allocator<uint64_t>> & buffer);
template void deserialize(Cdr & cdr, rcl_buffer::Buffer<uint64_t, std::allocator<uint64_t>> & buffer);

template void serialize(Cdr & cdr, const rcl_buffer::Buffer<int64_t, std::allocator<int64_t>> & buffer);
template void deserialize(Cdr & cdr, rcl_buffer::Buffer<int64_t, std::allocator<int64_t>> & buffer);

template void serialize(Cdr & cdr, const rcl_buffer::Buffer<float, std::allocator<float>> & buffer);
template void deserialize(Cdr & cdr, rcl_buffer::Buffer<float, std::allocator<float>> & buffer);

template void serialize(Cdr & cdr, const rcl_buffer::Buffer<double, std::allocator<double>> & buffer);
template void deserialize(Cdr & cdr, rcl_buffer::Buffer<double, std::allocator<double>> & buffer);

}  // namespace fastcdr
}  // namespace eprosima
