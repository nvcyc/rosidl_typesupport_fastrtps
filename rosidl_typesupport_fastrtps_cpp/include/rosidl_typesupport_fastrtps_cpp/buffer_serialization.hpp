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

#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosidl_runtime_cpp/buffer.hpp"
#include "rosidl_typesupport_fastrtps_cpp/visibility_control.h"
#include "fastcdr/Cdr.h"
#include "rmw/topic_endpoint_info.h"

namespace rosidl_typesupport_fastrtps_cpp
{

/// Global storage for buffer backend functionality
/// Populated by RMW layer during initialization - no direct BufferBackendRegistry dependency here
/// This keeps rosidl_typesupport_fastrtps_cpp free of pluginlib/registry dependencies

/// Backend descriptor operations (technology-independent, provided by backend)
struct BackendDescriptorOps
{
  // Create descriptor from buffer impl
  std::function<std::shared_ptr<void>(const std::shared_ptr<void> &)> create_descriptor;
  // Create buffer impl from descriptor
  std::function<std::shared_ptr<void>(const std::shared_ptr<void> &)> from_descriptor;
  // Create descriptor with endpoint awareness
  std::function<std::shared_ptr<void>(const std::shared_ptr<void> &,
    const rmw_topic_endpoint_info_t &)> create_descriptor_with_endpoint;
  // Create buffer impl from descriptor with endpoint awareness
  std::function<std::shared_ptr<void>(const std::shared_ptr<void> &,
    const rmw_topic_endpoint_info_t &)> from_descriptor_with_endpoint;
  // Descriptor type name (e.g., "cuda_buffer_msgs::msg::CudaBufferDescriptor")
  std::string descriptor_type_name;
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

/// Resolver for endpoint/backend compatibility (optional).
using EndpointCompatibilityResolver =
  std::function<bool(const rmw_topic_endpoint_info_t &, const std::string &)>;

/// Get global endpoint compatibility resolver.
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC
inline EndpointCompatibilityResolver & get_endpoint_compatibility_resolver()
{
  static EndpointCompatibilityResolver resolver;
  return resolver;
}

/// Helper macro to register FastCDR serialization functions for a buffer descriptor message.
/// This is called by descriptor message packages to enable FastCDR serialization.
/// Example: REGISTER_BUFFER_DESCRIPTOR_TYPE(cuda_buffer_msgs, CudaBufferDescriptor, "cuda")
/// The macro expects pkg_name::msg::MessageName and will call the generated cdr_serialize/cdr_deserialize functions
#define REGISTER_BUFFER_DESCRIPTOR_TYPE(pkg_name, msg_name, backend_name) \
  do { \
    using MessageType = pkg_name::msg::msg_name; \
    auto & serializers = rosidl_typesupport_fastrtps_cpp::get_descriptor_serializers(); \
    rosidl_typesupport_fastrtps_cpp::DescriptorSerializers desc_ser; \
    desc_ser.serialize = [](eprosima::fastcdr::Cdr & cdr, \
      const std::shared_ptr<void> & desc_ptr) { \
        auto desc = std::static_pointer_cast<MessageType>(desc_ptr); \
        pkg_name::msg::typesupport_fastrtps_cpp::cdr_serialize(*desc, cdr); \
      }; \
    desc_ser.deserialize = [](eprosima::fastcdr::Cdr & cdr) -> std::shared_ptr<void> { \
        auto desc = std::make_shared<MessageType>(); \
        pkg_name::msg::typesupport_fastrtps_cpp::cdr_deserialize(cdr, *desc); \
        return desc; \
      }; \
    serializers[backend_name] = desc_ser; \
  } while (0)

/// Get serialized size of Buffer<T> - for use by generated type support code
template<typename T, typename Allocator>
inline size_t get_buffer_serialized_size(
  const rosidl_runtime_cpp::Buffer<T, Allocator> & buffer,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;
  const size_t padding = 4;

  const std::string backend_type = buffer.get_backend_type();

  // Size of backend_type string
  // FastCDR format: uint32_t length (4 bytes, 4-byte aligned) + string data (byte-aligned)
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    backend_type.size() + 1;  // +1 for null terminator

  if (backend_type == "cpu") {
    // Size of vector: array size (4 bytes) + elements
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
    // Vendor backends: account for element_type_id, descriptor_type_name, and descriptor
    // The descriptor includes metadata PLUS the serialized buffer data
    // Conservative estimate: buffer data size + overhead for metadata fields
    size_t buffer_data_size = buffer.size() * sizeof(T);
    size_t metadata_overhead = 512;  // Strings, integers, bool, etc.
    current_alignment += buffer_data_size + metadata_overhead;
  }

  return current_alignment - initial_alignment;
}

/// Serialize Buffer<T> with endpoint awareness.
/// Calls endpoint-specific descriptor creation for optimization.
template<typename T, typename Allocator>
inline void serialize_buffer_with_endpoint(
  eprosima::fastcdr::Cdr & cdr,
  const rosidl_runtime_cpp::Buffer<T, Allocator> & buffer,
  const rmw_topic_endpoint_info_t & endpoint_info)
{
  const std::string backend_type = buffer.get_backend_type();

  std::cerr << "[serialize_buffer_with_endpoint] Backend: " << backend_type
            << ", buffer size: " << buffer.size() << " elements\n";

  bool force_cpu = false;
  auto & compat_resolver = get_endpoint_compatibility_resolver();
  if (compat_resolver && backend_type != "cpu") {
    force_cpu = !compat_resolver(endpoint_info, backend_type);
  }

  // Serialize backend type first (for all backends)
  if (force_cpu) {
    cdr << std::string("cpu");
    std::cerr << "[serialize_buffer_with_endpoint] Forcing CPU serialization for backend '"
              << backend_type << "'\n";
  } else {
    cdr << backend_type;
  }
  std::cerr << "[serialize_buffer_with_endpoint] Wrote backend_type string\n";

  // CPU backend (or forced CPU): serialize directly as std::vector
  if (backend_type == "cpu" || force_cpu) {
    std::vector<T> vec = buffer.to_vector();
    std::cerr << "[serialize_buffer_with_endpoint] Writing vector of " << vec.size() <<
      " elements\n";
    cdr << vec;
    std::cerr << "[serialize_buffer_with_endpoint] Vector written successfully\n";
    return;
  }

  // Vendor backends: use endpoint-aware descriptor approach
  const std::string element_type_id = typeid(T).name();
  cdr << element_type_id;

  const auto * impl = buffer.get_impl();
  if (!impl) {
    throw std::runtime_error("Buffer implementation is null");
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

  // Serialize descriptor type name
  cdr << ops_it->second.descriptor_type_name;

  // Create descriptor with endpoint awareness
  auto * non_const_impl = const_cast<rosidl_runtime_cpp::BufferImplBase<T> *>(impl);
  std::shared_ptr<void> impl_shared(static_cast<void *>(non_const_impl), [](void *){});

  std::shared_ptr<void> descriptor;
  if (ops_it->second.create_descriptor_with_endpoint) {
    descriptor = ops_it->second.create_descriptor_with_endpoint(impl_shared, endpoint_info);
  } else {
    descriptor = ops_it->second.create_descriptor(impl_shared);
  }

  // Serialize descriptor
  ser_it->second.serialize(cdr, descriptor);
}

/// Deserialize Buffer<T> with endpoint awareness.
template<typename T, typename Allocator>
inline void deserialize_buffer_with_endpoint(
  eprosima::fastcdr::Cdr & cdr,
  rosidl_runtime_cpp::Buffer<T, Allocator> & buffer,
  const rmw_topic_endpoint_info_t & endpoint_info)
{
  std::cerr << "[deserialize_buffer_with_endpoint] Starting deserialization...\n";

  // Deserialize backend type first
  std::string backend_type;
  try {
    cdr >> backend_type;
    std::cerr << "[deserialize_buffer_with_endpoint] Read backend_type: '" << backend_type << "'\n";
  } catch (const std::exception & e) {
    std::cerr << "[deserialize_buffer_with_endpoint] EXCEPTION reading backend_type: " <<
      e.what() << "\n";
    throw;
  }

  // CPU backend: deserialize directly from std::vector
  if (backend_type == "cpu") {
    std::cerr << "[deserialize_buffer_with_endpoint] CPU backend, reading vector...\n";
    std::vector<T> vec;
    try {
      cdr >> vec;
      std::cerr << "[deserialize_buffer_with_endpoint] Read vector of " << vec.size() <<
        " elements\n";
    } catch (const std::exception & e) {
      std::cerr << "[deserialize_buffer_with_endpoint] EXCEPTION reading vector: " << e.what() <<
        "\n";
      throw;
    }

    buffer.resize(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
      buffer[i] = vec[i];
    }
    std::cerr << "[deserialize_buffer_with_endpoint] Buffer populated successfully\n";
    return;
  } else {
    // Vendor backends: use endpoint-aware descriptor approach
    std::cerr << "[deserialize_buffer_with_endpoint] Vendor backend: " << backend_type << "\n";
    std::string element_type_id;
    std::string descriptor_type_name;

    cdr >> element_type_id;
    cdr >> descriptor_type_name;

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
    auto descriptor = ser_it->second.deserialize(cdr);

    // Create buffer implementation with endpoint awareness
    std::shared_ptr<void> impl_shared;
    if (ops_it->second.from_descriptor_with_endpoint) {
      impl_shared = ops_it->second.from_descriptor_with_endpoint(descriptor, endpoint_info);
    } else {
      impl_shared = ops_it->second.from_descriptor(descriptor);
    }

    // Wrap implementation in Buffer
    auto typed_impl_shared =
      std::static_pointer_cast<rosidl_runtime_cpp::BufferImplBase<T>>(impl_shared);
    std::unique_ptr<rosidl_runtime_cpp::BufferImplBase<T>> typed_impl_unique =
      typed_impl_shared->clone();

    buffer.set_impl(std::move(typed_impl_unique), backend_type);
  }
}

}  // namespace rosidl_typesupport_fastrtps_cpp

namespace eprosima
{
namespace fastcdr
{

/// FastCDR serialize() function for Buffer<T> (called by FastCDR internally)
template<typename T, typename Allocator>
inline void serialize(Cdr & cdr, const rosidl_runtime_cpp::Buffer<T, Allocator> & buffer)
{
  cdr << buffer;  // Delegate to our custom operator<<
}

/// FastCDR deserialize() function for Buffer<T> (called by FastCDR internally)
template<typename T, typename Allocator>
inline void deserialize(Cdr & cdr, rosidl_runtime_cpp::Buffer<T, Allocator> & buffer)
{
  cdr >> buffer;  // Delegate to our custom operator>>
}

/// Serialize Buffer<T>.
/// CPU backend: serializes directly as std::vector<T> (fully backward compatible)
/// Other backends: use descriptor message approach
template<typename T, typename Allocator>
inline Cdr & operator<<(Cdr & cdr, const rosidl_runtime_cpp::Buffer<T, Allocator> & buffer)
{
  const std::string backend_type = buffer.get_backend_type();

  // Serialize backend type first
  cdr << backend_type;

  // CPU backend: serialize directly as std::vector (backward compatible, no descriptor needed)
  if (backend_type == "cpu") {
    const std::vector<T> & vec = buffer;
    cdr << vec;
    return cdr;
  }

  // Vendor backends: use descriptor approach
  const std::string element_type_id = typeid(T).name();
  cdr << element_type_id;

  const auto * impl = buffer.get_impl();  // Returns const BufferImplBase<T>*
  std::cerr << "[Buffer Serialization] Serializing Buffer with backend: " << backend_type << "\n";

  if (!impl) {
    throw std::runtime_error("Buffer implementation is null");
  }

  // Get backend descriptor operations (populated by RMW layer)
  auto & backend_ops = rosidl_typesupport_fastrtps_cpp::get_backend_descriptor_ops();
  auto ops_it = backend_ops.find(backend_type);
  if (ops_it == backend_ops.end()) {
    std::cerr << "[Buffer Serialization] ERROR: Backend '" << backend_type << "' not registered!\n";
    throw std::runtime_error(
      "No backend registered for type: " + backend_type +
      ". RMW layer may not have initialized buffer backends.");
  }

  std::cerr << "[Buffer Serialization] Backend ops found\n";

  // Get FastCDR serializers for this backend
  auto & serializers = rosidl_typesupport_fastrtps_cpp::get_descriptor_serializers();
  auto ser_it = serializers.find(backend_type);
  if (ser_it == serializers.end()) {
    std::cerr << "[Buffer Serialization] ERROR: FastCDR serializers not registered for backend '" <<
      backend_type << "'!\n";
    throw std::runtime_error(
      "FastCDR serializers not registered for backend: " + backend_type);
  }

  std::cerr << "[Buffer Serialization] FastCDR serializers found\n";

  // Serialize descriptor type name
  std::cerr << "[Buffer Serialization] About to serialize descriptor_type_name: " <<
    ops_it->second.descriptor_type_name << "\n";
  cdr << ops_it->second.descriptor_type_name;
  std::cerr << "[Buffer Serialization] Descriptor type name serialized\n";

  // Create descriptor from buffer implementation
  // Wrap raw pointer in shared_ptr with no-op deleter since Buffer owns the impl via unique_ptr
  std::cerr << "[Buffer Serialization] Creating descriptor from impl\n";
  // Remove const and wrap in shared_ptr with no-op deleter
  auto * non_const_impl = const_cast<rosidl_runtime_cpp::BufferImplBase<T> *>(impl);
  std::shared_ptr<void> impl_shared(static_cast<void *>(non_const_impl), [](void *){/* no-op deleter */
    });
  auto descriptor = ops_it->second.create_descriptor(impl_shared);
  std::cerr << "[Buffer Serialization] Descriptor created\n";

  // Serialize descriptor using registered FastCDR function
  std::cerr << "[Buffer Serialization] About to serialize descriptor message\n";
  ser_it->second.serialize(cdr, descriptor);
  std::cerr << "[Buffer Serialization] Descriptor message serialized\n";

  return cdr;
}

/// Deserialize Buffer<T>.
/// CPU backend: deserializes directly from std::vector<T> (fully backward compatible)
/// Other backends: use descriptor message approach
template<typename T, typename Allocator>
inline Cdr & operator>>(Cdr & cdr, rosidl_runtime_cpp::Buffer<T, Allocator> & buffer)
{
  // Deserialize backend type first
  std::string backend_type;
  cdr >> backend_type;

  std::cerr << "[Buffer Deserialization] Deserializing Buffer with backend: " << backend_type <<
    "\n";

  // CPU backend: deserialize directly from std::vector (backward compatible)
  if (backend_type == "cpu") {
    std::cerr << "[Buffer Deserialization] CPU backend - deserializing as std::vector\n";
    std::vector<T> vec;
    cdr >> vec;

    std::cerr << "[Buffer Deserialization] Deserialized " << vec.size() << " elements\n";

    // Copy into buffer (which defaults to CPU backend)
    buffer.resize(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
      buffer[i] = vec[i];
    }
    std::cerr << "[Buffer Deserialization] CPU buffer reconstruction complete\n";
    return cdr;
  }

  // Vendor backends: use descriptor approach
  std::string element_type_id;
  std::string descriptor_type_name;

  cdr >> element_type_id;
  std::cerr << "[Buffer Deserialization] Element type: " << element_type_id << "\n";

  cdr >> descriptor_type_name;
  std::cerr << "[Buffer Deserialization] Descriptor type name: " << descriptor_type_name << "\n";

  // Validate element type
  if (element_type_id != typeid(T).name()) {
    std::cerr << "[Buffer Deserialization] ERROR: Type mismatch!\n";
    throw std::runtime_error(
      "Type mismatch during deserialization: expected " +
      std::string(typeid(T).name()) + ", got " + element_type_id);
  }

  // Get backend descriptor operations (populated by RMW layer)
  auto & backend_ops = rosidl_typesupport_fastrtps_cpp::get_backend_descriptor_ops();
  auto ops_it = backend_ops.find(backend_type);
  if (ops_it == backend_ops.end()) {
    std::cerr << "[Buffer Deserialization] ERROR: Backend '" << backend_type <<
      "' not registered!\n";
    throw std::runtime_error(
      "No backend registered for type: " + backend_type +
      ". RMW layer may not have initialized buffer backends.");
  }

  std::cerr << "[Buffer Deserialization] Backend ops found\n";

  // Get FastCDR serializers for this backend
  auto & serializers = rosidl_typesupport_fastrtps_cpp::get_descriptor_serializers();
  auto ser_it = serializers.find(backend_type);
  if (ser_it == serializers.end()) {
    std::cerr <<
      "[Buffer Deserialization] ERROR: FastCDR deserializers not registered for backend '" <<
      backend_type << "'!\n";
    throw std::runtime_error(
      "FastCDR serializers not registered for backend: " + backend_type);
  }

  std::cerr << "[Buffer Deserialization] FastCDR deserializers found\n";

  // Deserialize descriptor using registered FastCDR function
  std::cerr << "[Buffer Deserialization] About to deserialize descriptor message\n";
  auto descriptor = ser_it->second.deserialize(cdr);
  std::cerr << "[Buffer Deserialization] Descriptor message deserialized\n";

  // Create buffer implementation from descriptor (returns shared_ptr)
  std::cerr << "[Buffer Deserialization] Creating buffer impl from descriptor\n";
  auto impl_shared = ops_it->second.from_descriptor(descriptor);
  std::cerr << "[Buffer Deserialization] Buffer impl created from descriptor\n";

  // Cast to correct type
  auto typed_impl_shared =
    std::static_pointer_cast<rosidl_runtime_cpp::BufferImplBase<T>>(impl_shared);

  // Transfer ownership: create unique_ptr from the shared_ptr by cloning
  // This ensures proper value semantics and unique ownership in the Buffer
  std::cerr << "[Buffer Deserialization] Cloning impl for unique ownership\n";
  std::unique_ptr<rosidl_runtime_cpp::BufferImplBase<T>> typed_impl_unique =
    typed_impl_shared->clone();

  std::cerr << "[Buffer Deserialization] Setting impl on buffer\n";
  buffer.set_impl(std::move(typed_impl_unique), backend_type);
  std::cerr << "[Buffer Deserialization] Deserialization complete\n";

  return cdr;
}

}  // namespace fastcdr
}  // namespace eprosima

#endif  // ROSIDL_TYPESUPPORT_FASTRTPS_CPP__BUFFER_SERIALIZATION_HPP_
