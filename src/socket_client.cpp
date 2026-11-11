#include <flight_forge_connector/socket_client.h>

#include <chrono>
#include <thread>

using kissnet::socket_status;
using flightforge::SocketClient;

namespace {
// Matches the server's own threshold for treating a payload as possibly segmented.
constexpr std::size_t kSegmentedReplyThreshold = 1024;
}  // namespace

SocketClient::SocketClient() = default;

/* SocketClient() //{ */

SocketClient::SocketClient(const std::string& address, uint16_t port) {
  address_ = address;
  port_    = port;
}

//}

/* ~SocketClient() //{ */

SocketClient::~SocketClient() {
  ClearInQueue_();
  Disconnect();
}

//}

/* IsSocketValid_() //{ */

bool SocketClient::IsSocketValid_() const {
  return socket_ != nullptr && socket_->is_valid();
}

//}

/* IsReadyToSend_() //{ */

bool SocketClient::IsReadyToSend_() const {
  return InQueueEmpty_();
}

//}

/* PushToInQueue_() //{ */

void SocketClient::PushToInQueue_(std::unique_ptr<std::vector<std::byte>> item) {
  in_queue_.emplace(std::move(item));
}

//}

/* PopFromInQueue_() //{ */

std::unique_ptr<std::vector<std::byte>> SocketClient::PopFromInQueue_() {

  if (!InQueueEmpty_()) {
    auto item = std::move(in_queue_.front());
    in_queue_.pop();
    return item;
  }

  return nullptr;
}

//}

/*  ClearInQueue_() //{ */

void SocketClient::ClearInQueue_() {

  while (!in_queue_.empty()) {
    in_queue_.pop();
  }
}

//}

/* InQueueEmpty_() //{ */

bool SocketClient::InQueueEmpty_() const {
  return in_queue_.empty();
}

//}

/* connect() //{ */

socket_status::values SocketClient::Connect() {

  socket_ = std::make_unique<kissnet::tcp_socket>(kissnet::endpoint(address_ + ":" + std::to_string(port_)));

  try {
    const auto connect_result = socket_->connect();

    if (connect_result.value == socket_status::valid) {
      // request/response with the caller blocked; coalescing only adds latency here
      socket_->set_tcp_no_delay(true);
    }

    return connect_result.value;
  }

  catch (const std::runtime_error& err) {
    return socket_status::errored;
  }
}

//}

/* connectSimple() //{ */

bool SocketClient::ConnectSimple() {
  return Connect() == socket_status::values::valid;
}

//}

/* disconnect() //{ */

bool SocketClient::Disconnect() {

  if (IsSocketValid_()) {

    socket_->close();

    socket_.reset(nullptr);

    return true;
  }

  return false;
}

//}

/* sendMessage() //{ */

std::tuple<uint32_t, socket_status> SocketClient::SendMessage_(const std::byte* buffer, uint32_t size) const {

  if (IsSocketValid_() && IsReadyToSend_()) {
    const auto [res_size, res_status] = socket_->send(buffer, size);
    return std::make_tuple(res_size, res_status);
  }

  return std::make_tuple(0, socket_status::errored);
}

//}

/* receiveMessage() //{ */

std::tuple<uint32_t, socket_status> SocketClient::ReceiveMessage() {

  if (IsSocketValid_()) {

    auto select_status = socket_->select(kissnet::fds_read, receive_timeout_ms_);

    if (select_status.get_value() == socket_status::timed_out) {
      return std::make_tuple(0, select_status);
    }

    auto buffer               = std::make_unique<kissnet::buffer<BUFFER_SIZE>>();
    const auto [size, status] = socket_->recv(*buffer);

    // TODO any faster solution?
    auto cropped_buffer = std::make_unique<std::vector<std::byte>>(buffer->data(), buffer->data() + size);

    if (status == socket_status::valid) {
      PushToInQueue_(std::move(cropped_buffer));
    }

    return std::make_tuple(size, status);
  }

  return std::make_tuple(0, socket_status::errored);
}

//}

/* discardStaleReplies_() //{ */

void SocketClient::DiscardStaleReplies_() {

  if (!IsSocketValid_()) {
    return;
  }

  while (socket_->bytes_available() > 0) {
    kissnet::buffer<BUFFER_SIZE> buffer;
    const auto [size, status] = socket_->recv(buffer);
    if (size == 0 || status != socket_status::valid) {
      break;
    }
  }
}

//}

/* getMessage() //{ */

bool SocketClient::GetMessage(std::string& message) {

  while (true) {

    const auto [receive_size, receive_status] = ReceiveMessage();

    if (!receive_size || receive_status == 0) {
      ClearInQueue_();
      return false;
    }

    const auto  data_pointer = PopFromInQueue_();
    const auto  chunk        = reinterpret_cast<const char*>(data_pointer->data());
    std::string chunk_str(chunk, receive_size);
    message += chunk_str;

    if (message[message.size() - 1] == END_OF_MESSAGE && message[message.size() - 2] == END_OF_MESSAGE && message[message.size() - 3] == END_OF_MESSAGE) {
      // Let in-flight segments land before declaring the message complete: through a proxy
      // chain bytes_available() reads 0 between them, and binary payloads do contain '$$$'.
      // Small replies cannot span segments, and paying 2 ms on every pose read put a
      // 2.1 ms floor under the whole API.
      if (message.size() >= kSegmentedReplyThreshold) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }

      if (socket_->bytes_available() == 0) {
        break;
      }
    }
  }

  ClearInQueue_();
  return !message.empty();
}

//}

/* ping() //{ */

bool SocketClient::Ping() {

  Serializable::Common::Ping::Request request{};

  Serializable::Common::Ping::Response response{};
  const auto                           status = Request(request, response);

  return status;
}

//}

/* GetMeta() //{ */

std::pair<bool, Serializable::Common::GetMeta::Response> SocketClient::GetMeta() {

  Serializable::Common::GetMeta::Request request{};

  Serializable::Common::GetMeta::Response response{};
  const auto                              status = Request(request, response);

  return std::make_pair(status && response.status, response);
}

//}

/* CheckCompatibility() //{ */

std::pair<bool, std::vector<std::string>> SocketClient::CheckCompatibility(const std::vector<std::string>& op_names) {

  const auto [ok, meta] = GetMeta();
  if (!ok) {
    return std::make_pair(false, std::vector<std::string>{"get_meta: the server does not answer it (older than API 0.19)"});
  }

  return std::make_pair(true, Serializable::Schema::Incompatibilities(meta.ops_, op_names));
}

//}
