#include <mpc-rbt-solution/Sender.hpp>

void Sender::Node::run()
{
  while (errno != EINTR) {
    if ((std::chrono::steady_clock::now() - timer_tick) < timer_period) continue;
    timer_tick = std::chrono::steady_clock::now();

    callback();
  }
}

void Sender::Node::onDataTimerTick()
{
  //UNIMPLEMENTED(__PRETTY_FUNCTION__);

  data.timestamp =
    static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());
  data.x = data.x + 1;
  data.y = data.y + 1;
  data.z = data.z + 1;

  Socket::IPFrame frame{
    .port = config.remotePort,
    .address = config.remoteAddress,
  };

  if (!Utils::Message::serialize(frame, data)) {
    RCLCPP_INFO(logger, "Failed to serialize message!");
    return;
  }

  if (!this->send(frame)) {
    RCLCPP_INFO(logger, "Failed to send message!");
    return;
  }

  //RCLCPP_INFO(logger, "Sending data to host: '%s:%d'", frame.address.c_str(), frame.port);
  RCLCPP_INFO(logger, "Sent: x=%.2f, y=%.2f, z=%.2f to %s:%d", data.x, data.y, data.z, frame.address.c_str(), frame.port);
  RCLCPP_INFO(logger, "\n\tstamp: %ld", data.timestamp);
}
