#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <flight_forge_connector/data_types.h>

namespace flightforge
{

/* Reply //{ */

// a received Response, owning its bytes
struct Reply : Status
{
  std::vector<uint8_t>  frame;
  const wire::Response* response = nullptr;

  template <typename T>
  const T* body() const {
    return response ? response->body_as<T>() : nullptr;
  }
};

/* //} */

/* frame header //{ */

// {u32 length, u8 flags, u8[3] reserved}, little-endian
constexpr size_t kFrameHeaderBytes = 8;

inline void EncodeFrameHeader(const uint32_t length, uint8_t* out) {
  out[0] = static_cast<uint8_t>(length);
  out[1] = static_cast<uint8_t>(length >> 8);
  out[2] = static_cast<uint8_t>(length >> 16);
  out[3] = static_cast<uint8_t>(length >> 24);
  out[4] = out[5] = out[6] = out[7] = 0;
}

inline uint32_t DecodeFrameLength(const uint8_t* in) {
  return uint32_t(in[0]) | (uint32_t(in[1]) << 8) | (uint32_t(in[2]) << 16) | (uint32_t(in[3]) << 24);
}

/* //} */

// calls take turns; replies match requests by seq, so one that arrives after its caller gave up is skipped
class Connection {
public:
  static constexpr uint16_t kDefaultPort = 8600;

  Connection(std::string address = "127.0.0.1", uint16_t port = kDefaultPort);
  ~Connection();

  Connection(const Connection&)            = delete;
  Connection& operator=(const Connection&) = delete;

  // tries attempts times, retry_delay apart
  bool Connect(int attempts = 1, std::chrono::milliseconds retry_delay = std::chrono::milliseconds(500));
  void Disconnect();
  bool Connected() const;

  const std::string& Address() const {
    return address_;
  }
  uint16_t Port() const {
    return port_;
  }

  void SetReceiveTimeout(std::chrono::milliseconds timeout);
  std::chrono::milliseconds ReceiveTimeout() const;

  // finishes a Request around body in builder, sends it and waits for its Response
  template <typename T>
  Reply Call(flatbuffers::FlatBufferBuilder& builder, flatbuffers::Offset<T> body, std::optional<int32_t> robot_id = std::nullopt,
             std::optional<int32_t> sensor_id = std::nullopt, std::optional<std::chrono::milliseconds> timeout = std::nullopt) {
    return Send(builder, wire::RequestBodyTraits<T>::enum_value, body.Union(), robot_id, sensor_id, timeout);
  }

  Reply Send(flatbuffers::FlatBufferBuilder& builder, wire::RequestBody type, flatbuffers::Offset<void> body, std::optional<int32_t> robot_id,
             std::optional<int32_t> sensor_id, std::optional<std::chrono::milliseconds> timeout);

  // an already finished frame body, for tests that send what a client never would
  Reply SendRaw(const uint8_t* data, size_t size, uint32_t expect_seq, std::optional<std::chrono::milliseconds> timeout = std::nullopt);

private:
  bool OpenLocked();
  void CloseLocked();
  bool WriteAll(const uint8_t* data, size_t size);
  // reads one whole frame; a timeout keeps what arrived so the stream stays in step
  bool ReadFrame(std::vector<uint8_t>& out, std::chrono::steady_clock::time_point deadline, std::string& error);
  Reply Exchange(const uint8_t* data, size_t size, uint32_t seq, std::chrono::milliseconds timeout);

  std::string address_;
  uint16_t    port_;

  mutable std::mutex mutex_;
  intptr_t           socket_ = -1;
  uint32_t           next_seq_ = 1;

  std::chrono::milliseconds timeout_{10000};

  std::vector<uint8_t> rx_;
  size_t               rx_start_ = 0;
};

}  // namespace flightforge
