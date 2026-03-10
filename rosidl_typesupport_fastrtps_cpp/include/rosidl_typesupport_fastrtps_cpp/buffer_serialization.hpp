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

#ifndef ROSIDL_TYPESUPPORT_FASTRTPS_CPP__BUFFER_SERIALIZATION_HPP_
#define ROSIDL_TYPESUPPORT_FASTRTPS_CPP__BUFFER_SERIALIZATION_HPP_

#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosidl_buffer/buffer.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/visibility_control.h"
#include "fastcdr/Cdr.h"
#include "rmw/topic_endpoint_info.h"
#include "rcutils/logging_macros.h"

namespace rosidl_typesupport_fastrtps_cpp
{

/// Global storage for buffer backend functionality
/// Populated by RMW layer during initialization - no direct BufferBackendRegistry dependency here
/// This keeps rosidl_typesupport_fastrtps_cpp free of pluginlib/registry dependencies

/// Backend descriptor operations (technology-independent, provided by backend)
struct BackendDescriptorOps
{
  // Create descriptor with endpoint awareness
  std::function<std::shared_ptr<void>(const std::shared_ptr<void> &,
    const rmw_topic_endpoint_info_t &)> create_descriptor_with_endpoint;
  // Create buffer impl from descriptor with endpoint awareness
  std::function<std::shared_ptr<void>(const std::shared_ptr<void> &,
    const rmw_topic_endpoint_info_t &)> from_descriptor_with_endpoint;
};

/// FastCDR-specific descriptor serialization functions (technology-specific)
struct DescriptorSerializers
{
  std::function<void(eprosima::fastcdr::Cdr &, const std::shared_ptr<void> &)> serialize;
  std::function<std::shared_ptr<void>(eprosima::fastcdr::Cdr &)> deserialize;
};

/// Get global map of backend descriptor operations
/// RMW layer populates this during backend initialization
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC
inline std::unordered_map<std::string, BackendDescriptorOps> & get_backend_descriptor_ops()
{
  static std::unordered_map<std::string, BackendDescriptorOps> ops;
  return ops;
}

/// Get global map of FastCDR descriptor serializers
/// RMW layer populates this by calling backend registration functions
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC
inline std::unordered_map<std::string, DescriptorSerializers> & get_descriptor_serializers()
{
  static std::unordered_map<std::string, DescriptorSerializers> serializers;
  return serializers;
}

/// Marker format for descriptor-backed Buffer payloads:
/// - CPU/legacy vector path starts with plain sequence length (high bit clear).
/// - Descriptor path starts with a marker whose high bit is set.
inline constexpr uint32_t kBufferDescriptorMarkerMask = 0x80000000u;
inline constexpr uint32_t kBufferDescriptorMarker = 0x8000B001u;

/// Register FastCDR serialization functions for a buffer descriptor message type.
///
/// This leverages the existing rosidl-generated type support callbacks
/// (cdr_serialize/cdr_deserialize via message_type_support_callbacks_t) so that
/// backend vendors do not need to manually write registration code or build
/// separate registration libraries.
///
/// Backend implementations should call this once during construction:
///   rosidl_typesupport_fastrtps_cpp::register_descriptor_serializers<
///     my_backend_msgs::msg::MyDescriptor>("my_backend");
///
/// @tparam DescriptorMsgT  The rosidl-generated descriptor message type
///                          (e.g., demo_buffer_backend_msgs::msg::DemoBufferDescriptor).
/// @param backend_name      The backend type name used as the registry key
///                          (e.g., "demo", "cuda").
template<typename DescriptorMsgT>
inline void register_descriptor_serializers(const std::string & backend_name)
{
  // Obtain the generated FastRTPS type support handle for the descriptor message.
  // This handle contains type-erased cdr_serialize / cdr_deserialize callbacks.
  const auto * ts_handle =
    rosidl_typesupport_fastrtps_cpp::get_message_type_support_handle<DescriptorMsgT>();
  const auto * callbacks =
    static_cast<const message_type_support_callbacks_t *>(ts_handle->data);

  DescriptorSerializers desc_ser;

  desc_ser.serialize = [callbacks](
    eprosima::fastcdr::Cdr & cdr,
    const std::shared_ptr<void> & desc_ptr)
    {
      callbacks->cdr_serialize(desc_ptr.get(), cdr);
    };

  desc_ser.deserialize = [callbacks](
    eprosima::fastcdr::Cdr & cdr) -> std::shared_ptr<void>
    {
      auto desc = std::make_shared<DescriptorMsgT>();
      callbacks->cdr_deserialize(cdr, desc.get());
      return desc;
    };

  auto & serializers = get_descriptor_serializers();
  serializers[backend_name] = desc_ser;
}

/// Get serialized size of Buffer<T> - for use by generated type support code
template<typename T, typename Allocator>
inline size_t get_buffer_serialized_size(
  const rosidl::Buffer<T, Allocator> & buffer,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;
  const size_t padding = 4;

  const std::string backend_type = buffer.get_backend_type();

  if (backend_type == "cpu") {
    // CPU path is wire-compatible with std::vector<T>:
    // uint32 length + element bytes.
    size_t array_size = buffer.size();

    // Align to 4-byte boundary for the length field
    current_alignment += eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    // Add 4 bytes for the array length
    current_alignment += padding;

    // Add array elements
    if (array_size > 0) {
      size_t item_size = sizeof(T);
      // Elements might need alignment
      current_alignment += eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
      current_alignment += array_size * item_size;
    }
  } else {
    // Descriptor marker prefix.
    current_alignment += eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    current_alignment += padding;

    // backend_type string
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
      backend_type.size() + 1;  // +1 for null terminator

    // Vendor backends: account for element_type_id and descriptor
    // Conservative estimate: buffer data size + overhead for metadata fields
    size_t buffer_data_size = buffer.size() * sizeof(T);
    size_t metadata_overhead = 256;
    current_alignment += buffer_data_size + metadata_overhead;
  }

  return current_alignment - initial_alignment;
}

/// Serialize Buffer<T> with endpoint awareness.
/// Calls endpoint-specific descriptor creation for optimization.
/// If the backend returns nullptr from create_descriptor_with_endpoint(), the buffer
/// is serialized as std::vector<T> (CPU fallback) for legacy wire compatibility.
template<typename T, typename Allocator>
inline void serialize_buffer_with_endpoint(
  eprosima::fastcdr::Cdr & cdr,
  const rosidl::Buffer<T, Allocator> & buffer,
  const rmw_topic_endpoint_info_t & endpoint_info)
{
  const std::string backend_type = buffer.get_backend_type();

  RCUTILS_LOG_INFO_NAMED("serialize_buffer_with_endpoint",
    ("Serializing buffer (backend: " + backend_type + ")").c_str());

  if (backend_type == "cpu") {
    RCUTILS_LOG_INFO_NAMED("serialize_buffer_with_endpoint", "Serializing buffer as std::vector");
    std::vector<T> vec = buffer.to_vector();
    cdr << vec;
    return;
  }

  const auto * impl = buffer.get_impl();
  if (!impl) {
    throw std::runtime_error("Buffer implementation is null");
  }

  auto & backend_ops = get_backend_descriptor_ops();
  auto ops_it = backend_ops.find(backend_type);
  if (ops_it == backend_ops.end()) {
    throw std::runtime_error(
      "No backend registered for type: " + backend_type);
  }

  auto & serializers = get_descriptor_serializers();
  auto ser_it = serializers.find(backend_type);
  if (ser_it == serializers.end()) {
    throw std::runtime_error(
      "FastCDR serializers not registered for backend: " + backend_type);
  }

  auto * non_const_impl = const_cast<rosidl::BufferImplBase<T> *>(impl);
  std::shared_ptr<void> impl_shared(static_cast<void *>(non_const_impl), [](void *){});

  auto descriptor = ops_it->second.create_descriptor_with_endpoint(impl_shared, endpoint_info);

  // nullptr means the backend cannot handle this endpoint — fall back to CPU wire format.
  if (!descriptor) {
    RCUTILS_LOG_INFO_NAMED(
      "serialize_buffer_with_endpoint", "Backend returned null descriptor, falling back to CPU");
    std::vector<T> vec = buffer.to_vector();
    cdr << vec;
    return;
  }

  // Descriptor-backed payload marker in first uint32 (high-bit set).
  cdr << static_cast<uint32_t>(kBufferDescriptorMarker);
  cdr << backend_type;

  const std::string element_type_id = typeid(T).name();
  cdr << element_type_id;

  RCUTILS_LOG_INFO_NAMED("serialize_buffer_with_endpoint",
    ("Serializing descriptor for backend: " + backend_type).c_str());

  ser_it->second.serialize(cdr, descriptor);
}

/// Deserialize Buffer<T> with endpoint awareness.
template<typename T, typename Allocator>
inline void deserialize_buffer_with_endpoint(
  eprosima::fastcdr::Cdr & cdr,
  rosidl::Buffer<T, Allocator> & buffer,
  const rmw_topic_endpoint_info_t & endpoint_info)
{
  RCUTILS_LOG_INFO_NAMED( "deserialize_buffer_with_endpoint", "Starting buffer deserialization");

  // Peek first uint32 to disambiguate legacy vector bytes vs descriptor payload.
  auto original_state = cdr.get_state();
  uint32_t first_word = 0u;
  try {
    cdr >> first_word;
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("deserialize_buffer_with_endpoint",
      ("EXCEPTION peeking first word: " + std::string(e.what())).c_str());
    throw;
  }
  cdr.set_state(original_state);

  // Legacy/vector path: no marker in first word (high-bit clear).
  if ((first_word & kBufferDescriptorMarkerMask) == 0u) {
    RCUTILS_LOG_INFO_NAMED(
      "deserialize_buffer_with_endpoint", "Legacy vector path: deserializing std::vector");
    std::vector<T> vec;
    try {
      cdr >> vec;
    } catch (const std::exception & e) {
      throw std::runtime_error(
        "EXCEPTION deserializing std::vector: " + std::string(e.what()));
    }

    buffer.resize(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
      buffer[i] = vec[i];
    }
    return;
  }

  // Descriptor path: marker is present in first uint32.
  if (first_word != kBufferDescriptorMarker) {
    throw std::runtime_error(
            "Unknown Buffer descriptor marker: " + std::to_string(first_word));
  }

  // Consume marker now that it has been validated.
  cdr >> first_word;

  std::string backend_type;
  std::string element_type_id;

  cdr >> backend_type;
  cdr >> element_type_id;
  RCUTILS_LOG_INFO_NAMED("deserialize_buffer_with_endpoint",
    (backend_type + " backend: deserializing element_type_id: '" + element_type_id + "'").c_str());

  // Validate element type
  if (element_type_id != typeid(T).name()) {
    throw std::runtime_error(
      "Type mismatch during deserialization: expected " +
      std::string(typeid(T).name()) + ", got " + element_type_id);
  }

  // Get backend descriptor operations
  auto & backend_ops = get_backend_descriptor_ops();
  auto ops_it = backend_ops.find(backend_type);
  if (ops_it == backend_ops.end()) {
    throw std::runtime_error(
      "No backend registered for type: " + backend_type);
  }

  // Get FastCDR serializers
  auto & serializers = get_descriptor_serializers();
  auto ser_it = serializers.find(backend_type);
  if (ser_it == serializers.end()) {
    throw std::runtime_error(
      "FastCDR serializers not registered for backend: " + backend_type);
  }

  // Deserialize descriptor
  RCUTILS_LOG_INFO_NAMED( "deserialize_buffer_with_endpoint", "Deserializing descriptor");
  auto descriptor = ser_it->second.deserialize(cdr);

  // Create buffer implementation with endpoint awareness
  RCUTILS_LOG_INFO_NAMED( "deserialize_buffer_with_endpoint", "Creating buffer from descriptor");
  auto impl_shared = ops_it->second.from_descriptor_with_endpoint(descriptor, endpoint_info);

  // Wrap implementation in Buffer
  auto typed_impl_shared =
    std::static_pointer_cast<rosidl::BufferImplBase<T>>(impl_shared);
  std::unique_ptr<rosidl::BufferImplBase<T>> typed_impl_unique =
    typed_impl_shared->clone();
  buffer.set_impl(std::move(typed_impl_unique), backend_type);
}

}  // namespace rosidl_typesupport_fastrtps_cpp

namespace eprosima
{
namespace fastcdr
{

/// FastCDR serialize() function for Buffer<T> (called by FastCDR internally)
template<typename T, typename Allocator>
inline void serialize(Cdr & cdr, const rosidl::Buffer<T, Allocator> & buffer)
{
  cdr << buffer;  // Delegate to our custom operator<<
}

/// FastCDR deserialize() function for Buffer<T> (called by FastCDR internally)
template<typename T, typename Allocator>
inline void deserialize(Cdr & cdr, rosidl::Buffer<T, Allocator> & buffer)
{
  cdr >> buffer;  // Delegate to our custom operator>>
}

/// Serialize Buffer<T>.
/// CPU backend: serializes directly as std::vector<T>
/// Other backends: force-convert to CPU backend and serialize as std::vector<T>
template<typename T, typename Allocator>
inline Cdr & operator<<(Cdr & cdr, const rosidl::Buffer<T, Allocator> & buffer)
{
  const std::string backend_type = buffer.get_backend_type();
  if (backend_type != "cpu") {
    RCUTILS_LOG_INFO_NAMED("Serialize Buffer<T>",
      ("Force-converting to CPU buffer for serialization (backend: " + backend_type + ")").c_str());
  }

  // Serialize as std::vector<T> for legacy wire compatibility.
  const std::vector<T> & vec = buffer.to_vector();
  cdr << vec;
  return cdr;
}

/// Deserialize Buffer<T>.
/// CPU backend: deserializes directly from std::vector<T> (fully backward compatible)
/// Other backends: use descriptor message approach
template<typename T, typename Allocator>
inline Cdr & operator>>(Cdr & cdr, rosidl::Buffer<T, Allocator> & buffer)
{
  // Only supports legacy vector-compatible CPU path.
  auto original_state = cdr.get_state();
  uint32_t first_word = 0u;
  cdr >> first_word;
  cdr.set_state(original_state);
  if ((first_word & rosidl_typesupport_fastrtps_cpp::kBufferDescriptorMarkerMask) != 0u) {
    throw std::runtime_error(
            "Deserializing Buffer<T> with operator>> only supports legacy CPU vector bytes");
  }

  std::vector<T> vec;
  cdr >> vec;

  // Copy into buffer (which defaults to CPU backend)
  buffer.resize(vec.size());
  for (size_t i = 0; i < vec.size(); ++i) {
    buffer[i] = vec[i];
  }
  return cdr;
}

}  // namespace fastcdr
}  // namespace eprosima

#endif  // ROSIDL_TYPESUPPORT_FASTRTPS_CPP__BUFFER_SERIALIZATION_HPP_
