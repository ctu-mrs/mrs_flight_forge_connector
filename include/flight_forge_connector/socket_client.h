#pragma once

#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <typeinfo>

#include <cereal/archives/binary.hpp>
#include <kissnet/kissnet.hpp>
#include <flight_forge_connector/serialization/serializable_extended.h>
#include <flight_forge_connector/serialization/serializable_schema.h>

#define LOCALHOST "127.0.0.1"
#define DEFAULT_PORT 8080
// One lidar sweep is a few hundred kilobytes. At 1 KB a chunk that was 256 recv() calls,
// each with its own select(), two heap allocations and two copies, to move data that
// loopback delivers in one burst.
#define BUFFER_SIZE 65536

#define END_OF_MESSAGE '$'

namespace flightforge
{

class SocketClient {
public:
  SocketClient();
  SocketClient(const std::string& address, uint16_t port);
  ~SocketClient();

  kissnet::socket_status::values Connect();
  bool                           ConnectSimple();
  bool                           Disconnect();

  std::tuple<uint32_t, kissnet::socket_status> ReceiveMessage();


  // how long a reply may take; a lockstep step legitimately runs for many seconds

  void SetReceiveTimeoutMs(int ms) {

    receive_timeout_ms_ = ms;

  }

  int GetReceiveTimeoutMs() const {

    return receive_timeout_ms_;

  }

  bool Ping();

  /* The server's op catalogue: per-op schema hashes plus the same as JSON. Both the game mode and every drone answer it. */
  std::pair<bool, Serializable::Common::GetMeta::Response> GetMeta();

  /**
   * Ops this connector was compiled against that the server lacks or serializes differently.
   * Pass the names a client actually uses to tolerate everything else; an empty list means compatible.
   */
  std::pair<bool, std::vector<std::string>> CheckCompatibility(const std::vector<std::string>& op_names = {});

  /* SendMessage() //{ */

  template <typename TRequest>
  std::tuple<uint32_t, kissnet::socket_status> SendMessage(TRequest& message) {
    std::stringstream outputStream;

    try {
      cereal::BinaryOutputArchive oa(outputStream);
      oa(message);
    }
    catch (cereal::Exception& exception) {
      std::cerr << "Serialization error: " << exception.what() << std::endl;
      return std::make_tuple(0, kissnet::socket_status::errored);
    }

    const std::string str = outputStream.str();

    // A single send() may write only part of a large payload, which would truncate the
    // request. Keep writing until the whole message is out.
    const auto* buffer = reinterpret_cast<const std::byte*>(str.data());
    uint32_t    sent   = 0;

    while (sent < str.size()) {
      const auto [chunk_size, chunk_status] = SendMessage_(buffer + sent, str.size() - sent);

      if (chunk_status != kissnet::socket_status::valid || chunk_size == 0) {
        return std::make_tuple(sent, chunk_status);
      }

      sent += chunk_size;
    }

    return std::make_tuple(sent, kissnet::socket_status::valid);
  }

  //}

  /* Request() //{ */

  template <typename TRequest, typename TResponse>
  bool Request(TRequest& message, TResponse& response) {
    DiscardStaleReplies_();

    message.seq                         = ++seq_;
    const auto [send_size, send_status] = SendMessage<TRequest>(message);

    if (send_status != kissnet::socket_status::valid || send_size == 0) {
      return false;
    }

    // a reply to an earlier request that timed out can still arrive, alone or glued in front of
    // ours; step past every stale one (the registry knows each reply's shape) and keep waiting
    for (int attempt = 0; attempt < 8; ++attempt) {
      std::string response_str;
      if (!GetMessage(response_str)) {
        return false;
      }

      try {
        std::stringstream ss2(response_str);
        while (ss2.peek() != std::char_traits<char>::eof()) {
          const auto start = ss2.tellg();

          Serializable::Common::NetworkResponse header{};
          {
            cereal::BinaryInputArchive ia(ss2);
            ia(header);
          }

          ss2.seekg(start);
          cereal::BinaryInputArchive body(ss2);
          if (header.seq == message.seq && header.type == message.type) {
            body(response);
            return true;
          }

          if (!Serializable::Registry::LoadResponse(body, scope_, header.type)) {
            break;
          }
          ss2.ignore(3);  // the END_OF_MESSAGE terminator behind the stale reply
        }
      }
      catch (cereal::Exception& exception) {
        std::cout << "SOCKET-CLIENT serialization crashed!!!" << exception.what() << std::endl;
        return false;
      }
    }
    return false;
  }

  //}

  uint16_t getPort() const {
    return port_;
  }

  std::string getAddress() const {
    return address_;
  }

private:
  uint16_t                             port_    = DEFAULT_PORT;
  std::string                          address_ = LOCALHOST;
  std::unique_ptr<kissnet::tcp_socket> socket_  = nullptr;

  std::queue<std::unique_ptr<std::vector<std::byte>>> in_queue_;

  uint32_t seq_ = 0;

  // bytes already waiting before a request is sent can only be late replies
  void DiscardStaleReplies_();

protected:
  // which server this client talks to; drone and game-mode ids overlap
  Serializable::Registry::Scope scope_ = Serializable::Registry::Scope::Common;

  void                                                       PushToInQueue_(std::unique_ptr<std::vector<std::byte>> item);
  std::unique_ptr<std::vector<std::byte>>                    PopFromInQueue_();
  void                                                       ClearInQueue_();
  [[nodiscard]] bool                                         InQueueEmpty_() const;
  [[nodiscard]] bool                                         IsSocketValid_() const;
  [[nodiscard]] bool                                         IsReadyToSend_() const;
  int receive_timeout_ms_ = 1000;

  [[nodiscard]] std::tuple<uint32_t, kissnet::socket_status> SendMessage_(const std::byte* buffer, uint32_t size) const;

  bool GetMessage(std::string& message);
};

}  // namespace flightforge
