#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <openssl/bio.h>
#include <openssl/buffer.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <openssl/ssl.h>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{

std::string trim_copy(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

bool looks_like_json_object(const std::string & value)
{
  const auto trimmed = trim_copy(value);
  return trimmed.size() >= 2 && trimmed.front() == '{' && trimmed.back() == '}';
}

std::string json_escape(const std::string & value)
{
  std::ostringstream out;
  for (const unsigned char c : value) {
    switch (c) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\b':
        out << "\\b";
        break;
      case '\f':
        out << "\\f";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (c < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(c) << std::dec << std::setfill(' ');
        } else {
          out << static_cast<char>(c);
        }
        break;
    }
  }
  return out.str();
}

std::int64_t now_millis()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string lowercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

std::string url_encode(const std::string & value)
{
  static constexpr char hex[] = "0123456789ABCDEF";
  std::string encoded;
  encoded.reserve(value.size() * 3);
  for (const unsigned char c : value) {
    if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encoded.push_back(static_cast<char>(c));
    } else {
      encoded.push_back('%');
      encoded.push_back(hex[c >> 4]);
      encoded.push_back(hex[c & 0x0F]);
    }
  }
  return encoded;
}

std::vector<unsigned char> base64_decode(const std::string & value)
{
  BIO * mem = BIO_new_mem_buf(value.data(), static_cast<int>(value.size()));
  BIO * b64 = BIO_new(BIO_f_base64());
  if (mem == nullptr || b64 == nullptr) {
    if (mem != nullptr) {
      BIO_free(mem);
    }
    if (b64 != nullptr) {
      BIO_free(b64);
    }
    throw std::runtime_error("failed to allocate OpenSSL BIO for base64 decode");
  }

  BIO_set_flags(b64, BIO_FLAGS_BASE64_NO_NL);
  BIO * bio = BIO_push(b64, mem);
  std::vector<unsigned char> decoded(value.size());
  const int len = BIO_read(bio, decoded.data(), static_cast<int>(decoded.size()));
  BIO_free_all(bio);

  if (len < 0) {
    throw std::runtime_error("failed to decode base64 access key");
  }
  decoded.resize(static_cast<std::size_t>(len));
  return decoded;
}

std::string base64_encode(const unsigned char * data, std::size_t size)
{
  BIO * mem = BIO_new(BIO_s_mem());
  BIO * b64 = BIO_new(BIO_f_base64());
  if (mem == nullptr || b64 == nullptr) {
    if (mem != nullptr) {
      BIO_free(mem);
    }
    if (b64 != nullptr) {
      BIO_free(b64);
    }
    throw std::runtime_error("failed to allocate OpenSSL BIO for base64 encode");
  }

  BIO_set_flags(b64, BIO_FLAGS_BASE64_NO_NL);
  BIO * bio = BIO_push(b64, mem);
  if (BIO_write(bio, data, static_cast<int>(size)) <= 0 || BIO_flush(bio) != 1) {
    BIO_free_all(bio);
    throw std::runtime_error("failed to encode base64 data");
  }

  BUF_MEM * buffer = nullptr;
  BIO_get_mem_ptr(mem, &buffer);
  std::string encoded(buffer->data, buffer->length);
  BIO_free_all(bio);
  return encoded;
}

void append_mqtt_string(std::vector<unsigned char> & out, const std::string & value)
{
  if (value.size() > 65535) {
    throw std::runtime_error("MQTT string is longer than 65535 bytes");
  }
  out.push_back(static_cast<unsigned char>((value.size() >> 8) & 0xFF));
  out.push_back(static_cast<unsigned char>(value.size() & 0xFF));
  out.insert(out.end(), value.begin(), value.end());
}

void append_remaining_length(std::vector<unsigned char> & out, std::size_t length)
{
  do {
    unsigned char byte = static_cast<unsigned char>(length % 128);
    length /= 128;
    if (length > 0) {
      byte |= 0x80;
    }
    out.push_back(byte);
  } while (length > 0);
}

std::string openssl_error_string()
{
  const unsigned long error = ERR_get_error();
  if (error == 0) {
    return "unknown OpenSSL error";
  }
  char buffer[256];
  ERR_error_string_n(error, buffer, sizeof(buffer));
  return buffer;
}

class MqttClient
{
public:
  struct PublishMessage
  {
    std::string topic;
    std::string payload;
  };

  MqttClient(
    std::string host, int port, bool use_tls, bool verify_tls, int keepalive_seconds,
    int timeout_seconds)
  : host_(std::move(host)),
    port_(port),
    use_tls_(use_tls),
    verify_tls_(verify_tls),
    keepalive_seconds_(keepalive_seconds),
    timeout_seconds_(timeout_seconds)
  {
  }

  ~MqttClient()
  {
    try {
      disconnect();
    } catch (...) {
    }
  }

  void connect_to_broker(
    const std::string & client_id,
    const std::string & username,
    const std::string & password)
  {
    open_socket();
    if (use_tls_) {
      open_tls();
    }

    std::vector<unsigned char> body;
    append_mqtt_string(body, "MQTT");
    body.push_back(4);
    body.push_back(0x80 | 0x40 | 0x02);
    body.push_back(static_cast<unsigned char>((keepalive_seconds_ >> 8) & 0xFF));
    body.push_back(static_cast<unsigned char>(keepalive_seconds_ & 0xFF));
    append_mqtt_string(body, client_id);
    append_mqtt_string(body, username);
    append_mqtt_string(body, password);

    std::vector<unsigned char> packet;
    packet.push_back(0x10);
    append_remaining_length(packet, body.size());
    packet.insert(packet.end(), body.begin(), body.end());
    write_all(packet);

    unsigned char packet_type = 0;
    const auto connack = read_packet(packet_type);
    if (packet_type != 0x20 || connack.size() < 2) {
      throw std::runtime_error("MQTT broker did not return a valid CONNACK");
    }
    if (connack[1] != 0) {
      std::ostringstream out;
      out << "MQTT CONNACK failed with code " << static_cast<int>(connack[1]);
      throw std::runtime_error(out.str());
    }
  }

  void publish(const std::string & topic, const std::string & payload)
  {
    std::vector<unsigned char> body;
    append_mqtt_string(body, topic);
    body.insert(body.end(), payload.begin(), payload.end());

    std::vector<unsigned char> packet;
    packet.push_back(0x30);
    append_remaining_length(packet, body.size());
    packet.insert(packet.end(), body.begin(), body.end());
    write_all(packet);
  }

  void subscribe(const std::string & topic)
  {
    const std::uint16_t packet_identifier = next_packet_identifier();

    std::vector<unsigned char> body;
    body.push_back(static_cast<unsigned char>((packet_identifier >> 8) & 0xFF));
    body.push_back(static_cast<unsigned char>(packet_identifier & 0xFF));
    append_mqtt_string(body, topic);
    body.push_back(0x00);

    std::vector<unsigned char> packet;
    packet.push_back(0x82);
    append_remaining_length(packet, body.size());
    packet.insert(packet.end(), body.begin(), body.end());
    write_all(packet);

    unsigned char packet_type = 0;
    const auto suback = read_packet(packet_type);
    if (packet_type != 0x90 || suback.size() < 3) {
      throw std::runtime_error("MQTT broker did not return a valid SUBACK");
    }
    const std::uint16_t response_identifier =
      (static_cast<std::uint16_t>(suback[0]) << 8) | suback[1];
    if (response_identifier != packet_identifier || suback[2] == 0x80) {
      throw std::runtime_error("MQTT broker rejected subscription to " + topic);
    }
  }

  std::vector<PublishMessage> read_available(int timeout_ms)
  {
    std::vector<PublishMessage> messages;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (true) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        break;
      }
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
      if (!wait_readable(static_cast<int>(remaining.count()))) {
        break;
      }

      unsigned char packet_type = 0;
      unsigned char flags = 0;
      const auto packet = read_packet(packet_type, flags);
      if (packet_type == 0x30) {
        messages.push_back(parse_publish(packet, flags));
      }
    }
    return messages;
  }

  void ping()
  {
    const unsigned char packet[] = {0xC0, 0x00};
    write_all(packet, sizeof(packet));

    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(timeout_seconds_);
    while (std::chrono::steady_clock::now() < deadline) {
      if (!wait_readable(500)) {
        continue;
      }
      unsigned char packet_type = 0;
      const auto response = read_packet(packet_type);
      if (packet_type == 0xD0 && response.empty()) {
        return;
      }
    }
    throw std::runtime_error("MQTT broker did not return a valid PINGRESP");
  }

  bool connected() const
  {
    return fd_ >= 0;
  }

  void disconnect()
  {
    if (fd_ < 0) {
      return;
    }

    const unsigned char packet[] = {0xE0, 0x00};
    (void)write_all_no_throw(packet, sizeof(packet));

    if (ssl_ != nullptr) {
      SSL_shutdown(ssl_);
      SSL_free(ssl_);
      ssl_ = nullptr;
    }
    if (ssl_ctx_ != nullptr) {
      SSL_CTX_free(ssl_ctx_);
      ssl_ctx_ = nullptr;
    }
    ::close(fd_);
    fd_ = -1;
  }

private:
  std::uint16_t next_packet_identifier()
  {
    ++packet_identifier_;
    if (packet_identifier_ == 0) {
      ++packet_identifier_;
    }
    return packet_identifier_;
  }

  void open_socket()
  {
    struct addrinfo hints {};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo * results = nullptr;
    const std::string port_text = std::to_string(port_);
    const int rc = getaddrinfo(host_.c_str(), port_text.c_str(), &hints, &results);
    if (rc != 0) {
      throw std::runtime_error(std::string("getaddrinfo failed: ") + gai_strerror(rc));
    }

    int connected_fd = -1;
    std::string last_error;
    for (auto * item = results; item != nullptr; item = item->ai_next) {
      const int candidate = ::socket(item->ai_family, item->ai_socktype, item->ai_protocol);
      if (candidate < 0) {
        last_error = std::strerror(errno);
        continue;
      }

      const int original_flags = fcntl(candidate, F_GETFL, 0);
      if (original_flags >= 0) {
        (void)fcntl(candidate, F_SETFL, original_flags | O_NONBLOCK);
      }

      bool connected = false;
      if (::connect(candidate, item->ai_addr, item->ai_addrlen) == 0) {
        connected = true;
      } else if (errno == EINPROGRESS) {
        fd_set write_set;
        FD_ZERO(&write_set);
        FD_SET(candidate, &write_set);
        struct timeval timeout {};
        timeout.tv_sec = timeout_seconds_;
        timeout.tv_usec = 0;
        const int ready = select(candidate + 1, nullptr, &write_set, nullptr, &timeout);
        if (ready > 0 && FD_ISSET(candidate, &write_set)) {
          int socket_error = 0;
          socklen_t len = sizeof(socket_error);
          if (
            getsockopt(candidate, SOL_SOCKET, SO_ERROR, &socket_error, &len) == 0 &&
            socket_error == 0)
          {
            connected = true;
          } else {
            last_error = std::strerror(socket_error);
          }
        } else if (ready == 0) {
          last_error = "connect timed out";
        } else {
          last_error = std::strerror(errno);
        }
      } else {
        last_error = std::strerror(errno);
      }

      if (original_flags >= 0) {
        (void)fcntl(candidate, F_SETFL, original_flags);
      }

      if (connected) {
        set_socket_timeouts(candidate);
        connected_fd = candidate;
        break;
      }
      ::close(candidate);
    }

    freeaddrinfo(results);
    if (connected_fd < 0) {
      throw std::runtime_error("failed to connect TCP socket to MQTT broker: " + last_error);
    }
    fd_ = connected_fd;
  }

  void set_socket_timeouts(int fd) const
  {
    struct timeval timeout {};
    timeout.tv_sec = timeout_seconds_;
    timeout.tv_usec = 0;
    (void)setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    (void)setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  }

  void open_tls()
  {
    SSL_library_init();
    SSL_load_error_strings();

    ssl_ctx_ = SSL_CTX_new(TLS_client_method());
    if (ssl_ctx_ == nullptr) {
      throw std::runtime_error("SSL_CTX_new failed: " + openssl_error_string());
    }

    if (verify_tls_) {
      SSL_CTX_set_verify(ssl_ctx_, SSL_VERIFY_PEER, nullptr);
      if (SSL_CTX_set_default_verify_paths(ssl_ctx_) != 1) {
        throw std::runtime_error("failed to load default TLS trust store");
      }
    } else {
      SSL_CTX_set_verify(ssl_ctx_, SSL_VERIFY_NONE, nullptr);
    }

    ssl_ = SSL_new(ssl_ctx_);
    if (ssl_ == nullptr) {
      throw std::runtime_error("SSL_new failed: " + openssl_error_string());
    }
    SSL_set_tlsext_host_name(ssl_, host_.c_str());
    SSL_set_fd(ssl_, fd_);
    if (SSL_connect(ssl_) != 1) {
      throw std::runtime_error("SSL_connect failed: " + openssl_error_string());
    }
  }

  bool write_all_no_throw(const unsigned char * data, std::size_t size)
  {
    try {
      write_all(data, size);
      return true;
    } catch (...) {
      return false;
    }
  }

  void write_all(const std::vector<unsigned char> & data)
  {
    write_all(data.data(), data.size());
  }

  void write_all(const unsigned char * data, std::size_t size)
  {
    std::size_t offset = 0;
    while (offset < size) {
      int written = 0;
      if (ssl_ != nullptr) {
        written = SSL_write(ssl_, data + offset, static_cast<int>(size - offset));
      } else {
        written = static_cast<int>(::send(fd_, data + offset, size - offset, 0));
      }
      if (written <= 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          throw std::runtime_error("timed out writing MQTT packet");
        }
        throw std::runtime_error(std::string("failed to write MQTT packet: ") + std::strerror(errno));
      }
      offset += static_cast<std::size_t>(written);
    }
  }

  std::vector<unsigned char> read_packet(unsigned char & packet_type)
  {
    unsigned char flags = 0;
    return read_packet(packet_type, flags);
  }

  std::vector<unsigned char> read_packet(unsigned char & packet_type, unsigned char & flags)
  {
    unsigned char fixed_header = 0;
    read_exact(&fixed_header, 1);
    packet_type = fixed_header & 0xF0;
    flags = fixed_header & 0x0F;

    std::size_t remaining_length = 0;
    std::size_t multiplier = 1;
    unsigned char encoded = 0;
    do {
      read_exact(&encoded, 1);
      remaining_length += (encoded & 127) * multiplier;
      multiplier *= 128;
      if (multiplier > 128 * 128 * 128 * 128) {
        throw std::runtime_error("invalid MQTT remaining length");
      }
    } while ((encoded & 128) != 0);

    std::vector<unsigned char> payload(remaining_length);
    if (!payload.empty()) {
      read_exact(payload.data(), payload.size());
    }
    return payload;
  }

  bool wait_readable(int timeout_ms) const
  {
    if (ssl_ != nullptr && SSL_pending(ssl_) > 0) {
      return true;
    }
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(fd_, &read_set);
    struct timeval timeout {};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    const int ready = select(fd_ + 1, &read_set, nullptr, nullptr, &timeout);
    if (ready < 0) {
      throw std::runtime_error(std::string("failed waiting for MQTT packet: ") + std::strerror(errno));
    }
    return ready > 0 && FD_ISSET(fd_, &read_set);
  }

  PublishMessage parse_publish(const std::vector<unsigned char> & packet, unsigned char flags) const
  {
    if (packet.size() < 2) {
      throw std::runtime_error("received invalid MQTT PUBLISH packet");
    }
    const std::size_t topic_size =
      (static_cast<std::size_t>(packet[0]) << 8) | static_cast<std::size_t>(packet[1]);
    std::size_t offset = 2;
    if (packet.size() < offset + topic_size) {
      throw std::runtime_error("received MQTT PUBLISH packet with invalid topic length");
    }

    PublishMessage message;
    message.topic = std::string(packet.begin() + offset, packet.begin() + offset + topic_size);
    offset += topic_size;

    const unsigned char qos = (flags >> 1) & 0x03;
    if (qos > 0) {
      if (packet.size() < offset + 2) {
        throw std::runtime_error("received MQTT PUBLISH packet without packet identifier");
      }
      offset += 2;
    }
    message.payload = std::string(packet.begin() + offset, packet.end());
    return message;
  }

  void read_exact(unsigned char * data, std::size_t size)
  {
    std::size_t offset = 0;
    while (offset < size) {
      int received = 0;
      if (ssl_ != nullptr) {
        received = SSL_read(ssl_, data + offset, static_cast<int>(size - offset));
      } else {
        received = static_cast<int>(::recv(fd_, data + offset, size - offset, 0));
      }
      if (received <= 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          throw std::runtime_error("timed out reading MQTT packet");
        }
        throw std::runtime_error(std::string("failed to read MQTT packet: ") + std::strerror(errno));
      }
      offset += static_cast<std::size_t>(received);
    }
  }

  std::string host_;
  int port_{};
  bool use_tls_{};
  bool verify_tls_{};
  int keepalive_seconds_{};
  int timeout_seconds_{};
  int fd_{-1};
  std::uint16_t packet_identifier_{0};
  SSL_CTX * ssl_ctx_{nullptr};
  SSL * ssl_{nullptr};
};

}  // namespace

class OneNetBridgeNode : public rclcpp::Node
{
public:
  OneNetBridgeNode() : Node("onenet_bridge_node")
  {
    product_id_ = declare_parameter<std::string>("product_id", "");
    device_name_ = declare_parameter<std::string>("device_name", "");
    access_key_ = declare_parameter<std::string>("access_key", "");
    access_key_is_base64_ = declare_parameter<bool>("access_key_is_base64", true);
    token_version_ = declare_parameter<std::string>("token_version", "2018-10-31");
    token_method_ = declare_parameter<std::string>("token_method", "sha1");
    token_expire_seconds_ = declare_parameter<int>("token_expire_seconds", 86400);
    token_resource_ = declare_parameter<std::string>("token_resource", "");
    mqtt_host_ = declare_parameter<std::string>("mqtt_host", "mqtts.heclouds.com");
    mqtt_port_ = declare_parameter<int>("mqtt_port", 1883);
    mqtt_client_id_ = declare_parameter<std::string>("mqtt_client_id", "");
    mqtt_keepalive_seconds_ = declare_parameter<int>("mqtt_keepalive_seconds", 60);
    mqtt_timeout_seconds_ = declare_parameter<int>("mqtt_timeout_seconds", 6);
    cloud_maintain_period_ms_ = declare_parameter<int>("cloud_maintain_period_ms", 30000);
    use_tls_ = declare_parameter<bool>("use_tls", false);
    verify_tls_ = declare_parameter<bool>("verify_tls", true);
    report_period_ms_ = declare_parameter<int>("report_period_ms", 2000);
    enable_status_report_ = declare_parameter<bool>("enable_status_report", false);
    enable_cloud_ = declare_parameter<bool>("enable_cloud", false);
    delivery_record_topic_ =
      declare_parameter<std::string>("delivery_record_topic", "/medicine_delivery_record");
    delivery_event_identifier_ =
      declare_parameter<std::string>("delivery_event_identifier", "medicine_delivery");
    delivery_report_topic_ = declare_parameter<std::string>("delivery_report_topic", "");

    voltage_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/PowerVoltage", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) { voltage_ = msg->data; });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom_combined", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        linear_x_ = msg->twist.twist.linear.x;
        linear_y_ = msg->twist.twist.linear.y;
        angular_z_ = msg->twist.twist.angular.z;
      });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw", 10,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_angular_z_ = msg->angular_velocity.z;
        imu_accel_x_ = msg->linear_acceleration.x;
        imu_accel_y_ = msg->linear_acceleration.y;
      });

    delivery_record_sub_ = create_subscription<std_msgs::msg::String>(
      delivery_record_topic_, 10,
      std::bind(&OneNetBridgeNode::handle_delivery_record, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    report_timer_ = create_wall_timer(
      std::chrono::milliseconds(report_period_ms_),
      std::bind(&OneNetBridgeNode::report_status, this));
    cloud_maintain_timer_ = create_wall_timer(
      std::chrono::milliseconds(cloud_maintain_period_ms_),
      std::bind(&OneNetBridgeNode::maintain_cloud_connection, this));

    RCLCPP_INFO(
      get_logger(),
      "OneNET bridge initialized for device '%s' at %s:%d, cloud=%s, delivery_topic=%s",
      device_name_.c_str(), mqtt_host_.c_str(), mqtt_port_, enable_cloud_ ? "enabled" : "disabled",
      delivery_record_topic_.c_str());

    if (enable_cloud_) {
      maintain_cloud_connection();
    }
  }

private:
  std::string status_payload() const
  {
    const auto timestamp = now_millis();
    const double speed = std::hypot(linear_x_, linear_y_);
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "{"
        << "\"id\":\"" << timestamp << "\","
        << "\"version\":\"1.0\","
        << "\"params\":{"
        << "\"voltage\":{\"value\":" << voltage_ << ",\"time\":" << timestamp << "},"
        << "\"speed\":{\"value\":" << speed << ",\"time\":" << timestamp << "},"
        << "\"linear_x\":{\"value\":" << linear_x_ << ",\"time\":" << timestamp << "},"
        << "\"linear_y\":{\"value\":" << linear_y_ << ",\"time\":" << timestamp << "},"
        << "\"angular_z\":{\"value\":" << angular_z_ << ",\"time\":" << timestamp << "},"
        << "\"imu_angular_z\":{\"value\":" << imu_angular_z_ << ",\"time\":" << timestamp << "},"
        << "\"imu_accel_x\":{\"value\":" << imu_accel_x_ << ",\"time\":" << timestamp << "},"
        << "\"imu_accel_y\":{\"value\":" << imu_accel_y_ << ",\"time\":" << timestamp << "}"
        << "}"
        << "}";
    return out.str();
  }

  void report_status()
  {
    if (!enable_status_report_) {
      return;
    }

    const auto payload = status_payload();

    if (!enable_cloud_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "OneNET cloud disabled, status payload preview: %s", payload.c_str());
      return;
    }

    if (!cloud_config_ready()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "OneNET cloud is enabled but product_id/device_name/access_key are incomplete");
      return;
    }

    try {
      const auto replies = publish_to_onenet(property_topic(), payload);
      log_cloud_replies(replies, "status property report");
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000, "Failed to upload OneNET status property: %s",
        error.what());
    }
  }

  void maintain_cloud_connection()
  {
    if (!enable_cloud_) {
      return;
    }

    if (!cloud_config_ready()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "OneNET cloud is enabled but product_id/device_name/access_key are incomplete");
      return;
    }

    std::lock_guard<std::mutex> lock(mqtt_mutex_);
    try {
      if (!mqtt_client_ || !mqtt_client_->connected()) {
        connect_cloud_locked();
        return;
      }

      mqtt_client_->ping();
      RCLCPP_DEBUG(get_logger(), "OneNET MQTT heartbeat acknowledged");
    } catch (const std::exception & error) {
      reset_cloud_locked();
      RCLCPP_WARN(
        get_logger(), "OneNET MQTT connection lost, will reconnect later: %s", error.what());
    }
  }

  void handle_delivery_record(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto payload = delivery_payload(msg->data);
    const auto topic = delivery_topic();

    if (!enable_cloud_) {
      RCLCPP_INFO(
        get_logger(), "OneNET cloud disabled, delivery record preview topic=%s payload=%s",
        topic.c_str(), payload.c_str());
      return;
    }

    if (!cloud_config_ready()) {
      RCLCPP_ERROR(
        get_logger(),
        "OneNET cloud is enabled but product_id/device_name/access_key are incomplete");
      return;
    }

    try {
      RCLCPP_INFO(
        get_logger(), "Uploading delivery record to OneNET topic=%s payload=%s", topic.c_str(),
        payload.c_str());
      const auto replies = publish_to_onenet(topic, payload);
      RCLCPP_INFO(get_logger(), "Uploaded delivery record to OneNET topic=%s", topic.c_str());
      if (replies.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "OneNET did not return a delivery reply in the current read window; check platform logs");
      }
      log_cloud_replies(replies, "delivery event report");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to upload delivery record to OneNET: %s", error.what());
    }
  }

  std::string delivery_payload(const std::string & record) const
  {
    const auto timestamp = now_millis();
    const auto trimmed = trim_copy(record);
    const std::string value_json = looks_like_json_object(trimmed)
      ? trimmed
      : "{\"text\":\"" + json_escape(trimmed) + "\"}";

    std::ostringstream out;
    out << "{"
        << "\"id\":\"" << timestamp << "\","
        << "\"version\":\"1.0\","
        << "\"params\":{"
        << "\"" << json_escape(delivery_event_identifier_) << "\":{"
        << "\"value\":" << value_json << ","
        << "\"time\":" << timestamp
        << "}"
        << "}"
        << "}";
    return out.str();
  }

  std::string delivery_topic() const
  {
    if (!delivery_report_topic_.empty()) {
      return delivery_report_topic_;
    }
    return "$sys/" + product_id_ + "/" + device_name_ + "/thing/event/post";
  }

  std::string delivery_reply_topic() const
  {
    return delivery_topic() + "/reply";
  }

  std::string property_topic() const
  {
    return "$sys/" + product_id_ + "/" + device_name_ + "/thing/property/post";
  }

  std::string property_reply_topic() const
  {
    return property_topic() + "/reply";
  }

  bool cloud_config_ready() const
  {
    return !product_id_.empty() && !device_name_.empty() && !access_key_.empty();
  }

  std::string token_resource() const
  {
    if (!token_resource_.empty()) {
      return token_resource_;
    }
    return "products/" + product_id_ + "/devices/" + device_name_;
  }

  std::string build_token() const
  {
    const auto method = lowercase(token_method_);
    const EVP_MD * digest = nullptr;
    if (method == "sha1") {
      digest = EVP_sha1();
    } else if (method == "sha256") {
      digest = EVP_sha256();
    } else {
      throw std::runtime_error("unsupported OneNET token method: " + token_method_);
    }

    std::vector<unsigned char> key;
    if (access_key_is_base64_) {
      key = base64_decode(access_key_);
    } else {
      key.assign(access_key_.begin(), access_key_.end());
    }
    if (key.empty()) {
      throw std::runtime_error("OneNET access_key decoded to empty data");
    }

    const auto expire_at =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) +
      token_expire_seconds_;
    const auto expire_text = std::to_string(expire_at);
    const auto resource = token_resource();
    const auto signature_text =
      expire_text + "\n" + method + "\n" + resource + "\n" + token_version_;

    unsigned char digest_buffer[EVP_MAX_MD_SIZE];
    unsigned int digest_length = 0;
    if (HMAC(
        digest, key.data(), static_cast<int>(key.size()),
        reinterpret_cast<const unsigned char *>(signature_text.data()), signature_text.size(),
        digest_buffer, &digest_length) == nullptr)
    {
      throw std::runtime_error("failed to create OneNET token signature");
    }

    const auto sign = base64_encode(digest_buffer, digest_length);
    std::ostringstream token;
    token << "version=" << url_encode(token_version_)
          << "&res=" << url_encode(resource)
          << "&et=" << expire_text
          << "&method=" << url_encode(method)
          << "&sign=" << url_encode(sign);
    return token.str();
  }

  std::vector<MqttClient::PublishMessage> publish_to_onenet(
    const std::string & topic, const std::string & payload)
  {
    std::lock_guard<std::mutex> lock(mqtt_mutex_);
    try {
      if (!mqtt_client_ || !mqtt_client_->connected()) {
        connect_cloud_locked();
      }
      mqtt_client_->publish(topic, payload);
      return mqtt_client_->read_available(1500);
    } catch (...) {
      reset_cloud_locked();
      throw;
    }
  }

  void log_cloud_replies(
    const std::vector<MqttClient::PublishMessage> & replies, const std::string & context) const
  {
    for (const auto & reply : replies) {
      RCLCPP_INFO(
        get_logger(), "OneNET reply for %s topic=%s payload=%s", context.c_str(),
        reply.topic.c_str(), reply.payload.c_str());
    }
  }

  void connect_cloud_locked()
  {
    const auto client_id = mqtt_client_id_.empty() ? device_name_ : mqtt_client_id_;
    const auto password = build_token();
    auto client = std::make_unique<MqttClient>(
      mqtt_host_, mqtt_port_, use_tls_, verify_tls_, mqtt_keepalive_seconds_, mqtt_timeout_seconds_);
    client->connect_to_broker(client_id, product_id_, password);
    client->subscribe(delivery_reply_topic());
    client->subscribe(property_reply_topic());
    mqtt_client_ = std::move(client);
    RCLCPP_INFO(
      get_logger(), "OneNET MQTT connected and subscribed to %s and %s",
      delivery_reply_topic().c_str(), property_reply_topic().c_str());
  }

  void reset_cloud_locked()
  {
    if (mqtt_client_) {
      mqtt_client_->disconnect();
      mqtt_client_.reset();
    }
  }

  std::string product_id_;
  std::string device_name_;
  std::string access_key_;
  bool access_key_is_base64_{};
  std::string token_version_;
  std::string token_method_;
  int token_expire_seconds_{};
  std::string token_resource_;
  std::string mqtt_host_;
  int mqtt_port_{};
  std::string mqtt_client_id_;
  int mqtt_keepalive_seconds_{};
  int mqtt_timeout_seconds_{};
  int cloud_maintain_period_ms_{};
  bool use_tls_{};
  bool verify_tls_{};
  int report_period_ms_{};
  bool enable_status_report_{};
  bool enable_cloud_{};
  std::string delivery_record_topic_;
  std::string delivery_event_identifier_;
  std::string delivery_report_topic_;

  float voltage_{};
  double linear_x_{};
  double linear_y_{};
  double angular_z_{};
  double imu_angular_z_{};
  double imu_accel_x_{};
  double imu_accel_y_{};

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr voltage_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr delivery_record_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  rclcpp::TimerBase::SharedPtr cloud_maintain_timer_;
  std::mutex mqtt_mutex_;
  std::unique_ptr<MqttClient> mqtt_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OneNetBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
