#include "example/remote_control.h"
#include "example/joystick_link_monitor.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"

namespace
{
using namespace std::chrono_literals;
using Twist = geometry_msgs::msg::Twist;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

bool zero(const Twist & command)
{
  return command.linear.x == 0.0 && command.linear.y == 0.0 && command.linear.z == 0.0 &&
         command.angular.x == 0.0 && command.angular.y == 0.0 && command.angular.z == 0.0;
}

bool moving(const Twist & command)
{
  return command.linear.x > 0.0 && command.linear.y > 0.0 && command.angular.z > 0.0;
}

class RemoteFixture
{
public:
  RemoteFixture(
    bool connected, bool waiting, double timeout_s = 0.0, bool arm_controls = false,
    std::vector<rclcpp::Parameter> extra_parameters = {},
    std::shared_ptr<chassis::RemoteControlSession> shared_session = nullptr)
  {
    char pattern[] = "/tmp/chassis-remote-test-XXXXXX";
    const char * directory = ::mkdtemp(pattern);
    require(directory != nullptr, "cannot create joystick test directory");
    directory_ = directory;
    device_ = directory_ / "joystick";
    if (connected) {connect();}

    auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
    std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("device_path", device_.string()),
      rclcpp::Parameter("cmd_vel_topic", "/remote_disconnect_test/cmd_vel"),
      rclcpp::Parameter("arm_position_topic", "/remote_disconnect_test/arm_jog"),
      rclcpp::Parameter("arm_pose_topic", "/remote_disconnect_test/arm_pose"),
      rclcpp::Parameter("arm_node", "/remote_disconnect_test_arm"),
      rclcpp::Parameter("invert_linear_axis", false),
      rclcpp::Parameter("invert_lateral_axis", false),
      rclcpp::Parameter("invert_angular_axis", false),
      rclcpp::Parameter("max_accel", 0.2),
      rclcpp::Parameter("max_decel", 0.01)};
    if (timeout_s > 0.0) {parameters.emplace_back("timeout_sec", timeout_s);}
    parameters.insert(parameters.end(), extra_parameters.begin(), extra_parameters.end());
    options.parameter_overrides(parameters);
    if (shared_session) {
      session_ = std::move(shared_session);
      session_->on_owner_changed = [this] {++owner_changes;};
      session_->remote_velocity = [this](std::uint64_t generation, double x, double y, double yaw) {
          check_remote_lease(generation);
          Twist command;
          command.linear.x = x;
          command.linear.y = y;
          command.angular.z = yaw;
          commands.push_back(command);
        };
      session_->remote_arm_step = [this](std::uint64_t generation, unsigned coordinate, double delta) {
          check_remote_lease(generation);
          steps.push_back({coordinate, delta});
          return true;
        };
      session_->remote_arm_jog = [this](std::uint64_t generation, int direction) {
          check_remote_lease(generation);
          jog_commands.push_back(direction);
        };
      session_->remote_tracking = [this](std::uint64_t generation, bool requested) {
          check_remote_lease(generation);
          tracking_requests.push_back(requested);
        };
    } else if (waiting) {
      session_ = std::make_shared<chassis::RemoteControlSession>();
    }
    if (session_) {
      session_->remote_relay_toggle = [this](std::uint64_t generation) {
          if (session_->shared_control) {check_remote_lease(generation);}
          ++relay_toggles;
        };
    }
    link_available_ = std::make_shared<std::atomic<bool>>(true);
    link_monitor_ = std::make_shared<chassis::JoystickLinkMonitor>(
      [available = link_available_] {return available->load();});
    remote_ = chassis::make_remote_control(options, session_, arm_controls, "", link_monitor_);
    observer_ = std::make_shared<rclcpp::Node>(
      "remote_disconnect_test", rclcpp::NodeOptions().use_intra_process_comms(true));
    subscription_ = observer_->create_subscription<Twist>(
      "/remote_disconnect_test/cmd_vel", 10,
      [this](Twist::ConstSharedPtr command) {
        ++public_motion_messages;
        commands.push_back(*command);
      });
    if (arm_controls) {
      tracking_subscription_ = observer_->create_subscription<std_msgs::msg::Bool>(
        "/arm_tracking_requested", 10,
        [this](std_msgs::msg::Bool::ConstSharedPtr message) {
          ++public_motion_messages;
          tracking_requests.push_back(message->data);
        });
      get_service_ = observer_->create_service<rcl_interfaces::srv::GetParameters>(
        "/remote_disconnect_test_arm/get_parameters",
        [this](const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request>,
        std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response) {
          ++manual_gets;
          response->values.push_back(
            rclcpp::Parameter("yiyou_arm_target", std::vector<double>{}).to_parameter_msg().value);
        });
      set_service_ = observer_->create_service<rcl_interfaces::srv::SetParametersAtomically>(
        "/remote_disconnect_test_arm/set_parameters_atomically",
        [this](const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request>,
        std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> response) {
          ++manual_sets;
          response->result.successful = true;
        });
      arm_subscription_ = observer_->create_subscription<std_msgs::msg::Int8>(
        "/remote_disconnect_test/arm_jog", 10,
        [this](std_msgs::msg::Int8::ConstSharedPtr command) {
          ++public_motion_messages;
          jog_commands.push_back(command->data);
        });
      pose_publisher_ = observer_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/remote_disconnect_test/arm_pose", 1);
      pose_timer_ = observer_->create_wall_timer(50ms, [this] {
          if (!publish_pose_) {return;}
          geometry_msgs::msg::PoseStamped message;
          message.header.frame_id = "arm_shoulder";
          message.pose.orientation.w = 1.0;
          pose_publisher_->publish(message);
        });
    }
    executor_.add_node(remote_);
    executor_.add_node(observer_);
  }

  ~RemoteFixture()
  {
    disconnect();
    executor_.remove_node(remote_);
    executor_.remove_node(observer_);
    remote_.reset();
    if (session_ && session_->shared_control) {
      session_->on_owner_changed = {};
      session_->remote_velocity = {};
      session_->remote_arm_step = {};
      session_->remote_arm_jog = {};
      session_->remote_tracking = {};
    }
    if (session_) {session_->remote_relay_toggle = {};}
    subscription_.reset();
    observer_.reset();
    std::error_code error;
    std::filesystem::remove_all(directory_, error);
  }

  void connect()
  {
    if (!std::filesystem::exists(device_)) {
      require(::mkfifo(device_.c_str(), 0600) == 0, "cannot create joystick FIFO");
    }
    require(writer_ < 0, "joystick writer is already connected");
    // Holding the write end prevents EOF; closing it simulates device removal.
    writer_ = ::open(device_.c_str(), O_RDWR | O_NONBLOCK);
    require(writer_ >= 0, "cannot open joystick FIFO writer");
  }

  void disconnect()
  {
    if (writer_ >= 0) {
      ::close(writer_);
      writer_ = -1;
    }
  }

  void link_available(bool available) {link_available_->store(available);}

  void axes(std::int16_t value, bool initialization = false)
  {
    require(writer_ >= 0, "cannot send events to a disconnected joystick");
    for (const unsigned char axis : {0, 3, 6}) {
      js_event event{};
      event.type = JS_EVENT_AXIS | (initialization ? JS_EVENT_INIT : 0);
      event.number = axis;
      event.value = value;
      require(
        ::write(writer_, &event, sizeof(event)) == static_cast<ssize_t>(sizeof(event)),
        "cannot write joystick event");
    }
  }

  void clear()
  {
    // Drain a command already queued by the timer before changing connection state.
    executor_.spin_some();
    executor_.spin_some();
    commands.clear();
    jog_commands.clear();
    tracking_requests.clear();
    steps.clear();
    relay_toggles = 0;
  }

  void button(unsigned char number, bool pressed, bool initialization = false)
  {
    js_event event{};
    event.type = JS_EVENT_BUTTON | (initialization ? JS_EVENT_INIT : 0);
    event.number = number;
    event.value = pressed ? 1 : 0;
    require(::write(writer_, &event, sizeof(event)) == static_cast<ssize_t>(sizeof(event)),
      "cannot write joystick button event");
  }

  void arm_axis(std::int16_t value, bool initialization = false)
  {
    js_event event{};
    event.type = JS_EVENT_AXIS | (initialization ? JS_EVENT_INIT : 0);
    event.number = 7;
    event.value = value;
    require(::write(writer_, &event, sizeof(event)) == static_cast<ssize_t>(sizeof(event)),
      "cannot write arm axis event");
  }

  void initialize_arm(bool positive_held = false, bool tracking_held = false)
  {
    axes(0, true);
    arm_axis(0, true);
    for (const unsigned char number : {0, 1, 2, 3, 4, 6, 7}) {
      button(number, (number == 0 && positive_held) || (number == 6 && tracking_held), true);
    }
  }

  void stage(chassis::RemoteControlSession::Stage value)
  {
    require(session_ != nullptr, "fixture has no startup session");
    session_->stage = value;
  }

  bool tracking_publisher_exists() const
  {
    for (const auto & endpoint : remote_->get_publishers_info_by_topic("/arm_tracking_requested")) {
      if (endpoint.node_name() == remote_->get_name()) {return true;}
    }
    return false;
  }

  void require_tracking(bool expected, const char * message, std::chrono::milliseconds duration = 150ms)
  {
    clear();
    observe_for(duration);
    require(tracking_requests.size() >= 2, "tracking request must refresh its heartbeat");
    require(std::all_of(tracking_requests.begin(), tracking_requests.end(),
      [expected](bool requested) {return requested == expected;}), message);
  }

  void wait_for_tracking(bool expected, const char * message,
    std::chrono::milliseconds duration = 300ms)
  {
    clear();
    wait_for([this, expected] {
        return !tracking_requests.empty() && tracking_requests.back() == expected;
      }, message, duration);
  }

  void publish_pose(bool enabled) {publish_pose_ = enabled;}

  void require_jog_zeros(const char * message, std::chrono::milliseconds duration = 200ms)
  {
    clear();
    observe_for(duration);
    require(jog_commands.size() >= 2, "arm neutral must refresh zero commands");
    require(std::all_of(jog_commands.begin(), jog_commands.end(), [](int jog) {return jog == 0;}),
      message);
  }

  void wait_for_jog(int expected, const char * message, std::chrono::milliseconds limit = 300ms)
  {
    wait_for([this, expected] {return !jog_commands.empty() && jog_commands.back() == expected;},
      message, limit);
  }

  template<typename Predicate>
  void wait_for(Predicate predicate, const char * message, std::chrono::milliseconds limit = 1500ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {return;}
      std::this_thread::sleep_for(2ms);
    }
    throw std::runtime_error(message);
  }

  void observe_for(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(2ms);
    }
    executor_.spin_some();
  }

  void require_zeros(const char * message, std::chrono::milliseconds duration = 250ms)
  {
    clear();
    observe_for(duration);
    require(commands.size() >= 3, "remote must continuously publish commands");
    require(std::all_of(commands.begin(), commands.end(), zero), message);
  }

  void arm_and_move()
  {
    axes(0, true);
    require_zeros("neutral initialization must publish zero");
    axes(32767);
    clear();
    wait_for(
      [this] {
        return !commands.empty() && moving(commands.back()) &&
        commands.back().linear.x >= 0.04;
      },
      "joystick must produce movement after neutral initialization");
  }

  void require_no_shared_output(const char * message)
  {
    require(commands.empty() && jog_commands.empty() && tracking_requests.empty() &&
      steps.empty() && relay_toggles == 0 && manual_gets == 0 && manual_sets == 0 &&
      public_motion_messages == 0,
      message);
  }

  void check_remote_lease(std::uint64_t generation)
  {
    require(session_->owner == chassis::RemoteControlSession::Source::remote &&
      session_->stage == chassis::RemoteControlSession::Stage::running &&
      session_->generation == generation, "shared command must carry the current remote lease");
  }

  struct Step {unsigned coordinate; double delta;};
  std::vector<Step> steps;
  unsigned relay_toggles{0};
  unsigned owner_changes{0};
  unsigned public_motion_messages{0};

  std::vector<Twist> commands;
  std::vector<int> jog_commands;
  std::vector<bool> tracking_requests;
  unsigned manual_gets{0};
  unsigned manual_sets{0};

private:
  std::filesystem::path directory_;
  std::filesystem::path device_;
  int writer_{-1};
  std::shared_ptr<chassis::RemoteControlSession> session_;
  std::shared_ptr<std::atomic<bool>> link_available_;
  std::shared_ptr<chassis::JoystickLinkMonitor> link_monitor_;
  std::shared_ptr<rclcpp::Node> remote_;
  std::shared_ptr<rclcpp::Node> observer_;
  rclcpp::Subscription<Twist>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr arm_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tracking_subscription_;
  rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_service_;
  rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_service_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  bool publish_pose_{true};
  rclcpp::executors::SingleThreadedExecutor executor_;
};

void missing_joystick_publishes_zero(bool waiting)
{
  RemoteFixture fixture(false, waiting);
  fixture.require_zeros("missing joystick must publish zero while waiting or running");
}

void unplug_stops_immediately_and_reconnect_requires_neutral()
{
  RemoteFixture fixture(true, false);
  fixture.arm_and_move();
  fixture.clear();
  fixture.disconnect();
  fixture.wait_for(
    [&fixture] {return !fixture.commands.empty();}, "no command after disconnect", 250ms);
  require(zero(fixture.commands.front()), "first command after disconnect must be exactly zero");
  fixture.require_zeros("disconnected joystick must keep publishing zero");

  fixture.connect();
  fixture.axes(32767, true);
  fixture.require_zeros("reconnecting with held axes must not move", 1200ms);
  fixture.axes(0);
  fixture.require_zeros("reconnected joystick must remain stopped while neutral");
  fixture.axes(32767);
  fixture.wait_for(
    [&fixture] {return !fixture.commands.empty() && moving(fixture.commands.back());},
    "joystick must recover after reporting neutral");

  fixture.clear();
  fixture.observe_for(800ms);
  require(fixture.commands.size() >= 10, "held joystick must keep publishing commands");
  require(
    std::all_of(fixture.commands.begin(), fixture.commands.end(), moving),
    "default timeout must not stop a held joystick merely because no new events arrive");
}

void configured_timeout_stops_immediately_and_requires_neutral()
{
  RemoteFixture fixture(true, false, 0.4);
  fixture.arm_and_move();
  fixture.clear();
  fixture.wait_for(
    [&fixture] {
      return std::any_of(
        fixture.commands.begin(), fixture.commands.end(),
        [](const Twist & command) {return command.angular.z == 0.0;});
    },
    "configured joystick timeout must stop motion", 1000ms);
  const auto stopped = std::find_if(
    fixture.commands.begin(), fixture.commands.end(),
    [](const Twist & command) {return command.angular.z == 0.0;});
  require(zero(*stopped), "first command at timeout must bypass the speed ramp and be all zero");

  fixture.axes(32767);
  fixture.require_zeros("fresh held-axis events after timeout must wait for neutral");
  fixture.axes(0);
  fixture.require_zeros("returning to neutral after timeout must remain stopped");
  fixture.axes(32767);
  fixture.wait_for(
    [&fixture] {return !fixture.commands.empty() && moving(fixture.commands.back());},
    "joystick must recover from timeout after returning to neutral");
}

void radio_loss_with_live_device_stops_and_requires_fresh_neutral()
{
  RemoteFixture fixture(true, false);
  fixture.arm_and_move();
  fixture.clear();
  // Keep the joystick FIFO open: wireless failure precedes Linux device removal.
  fixture.link_available(false);
  fixture.wait_for(
    [&fixture] {return !fixture.commands.empty() && zero(fixture.commands.back());},
    "radio loss must stop even while the joystick device is present", 450ms);
  fixture.require_zeros("failed link must continuously publish exact zero");

  fixture.link_available(true);
  fixture.axes(32767, true);
  fixture.require_zeros("link recovery with held axes must not restore old motion", 500ms);
  fixture.axes(0);
  fixture.require_zeros("neutral after link recovery must remain stopped");
  fixture.axes(32767);
  fixture.wait_for(
    [&fixture] {return !fixture.commands.empty() && moving(fixture.commands.back());},
    "verified link must allow motion only after fresh neutral");
}

void unverified_link_never_allows_motion()
{
  RemoteFixture fixture(true, false);
  fixture.link_available(false);
  fixture.observe_for(200ms);
  fixture.axes(0, true);
  fixture.require_zeros("unverified link must not arm from neutral");
  fixture.axes(32767);
  fixture.require_zeros("unverified link must not send nonzero commands");
}

void recovered_link_cannot_hide_failure_between_ros_ticks()
{
  RemoteFixture fixture(true, false);
  fixture.arm_and_move();
  fixture.clear();
  // Do not spin ROS: the background monitor observes a failure and recovery in between ticks.
  fixture.link_available(false);
  std::this_thread::sleep_for(180ms);
  fixture.link_available(true);
  std::this_thread::sleep_for(180ms);
  fixture.axes(32767, true);
  fixture.require_zeros("recovered generation must invalidate held input after a delayed tick");
}
void position_jog_requires_release_and_stops_on_conflicting_buttons()
{
  RemoteFixture fixture(true, false, 0.0, true);
  fixture.initialize_arm(true);
  fixture.require_jog_zeros("held fourth-axis button at initialization must not move", 300ms);
  fixture.button(0, false);
  fixture.require_jog_zeros("released fourth-axis buttons must stay stopped");
  fixture.button(0, true);
  fixture.wait_for_jog(1, "button0 must request positive position jog");
  fixture.clear();
  fixture.observe_for(150ms);
  require(fixture.jog_commands.size() >= 2, "held position jog must refresh its lease");
  require(std::all_of(fixture.jog_commands.begin(), fixture.jog_commands.end(),
    [](int direction) {return direction == 1;}), "held button0 must maintain positive jog");

  fixture.clear();
  fixture.button(0, false);
  fixture.wait_for_jog(0, "releasing button0 must publish halt");
  require(fixture.jog_commands.front() == 0, "first fourth-axis command after release must halt");
  fixture.button(4, true);
  fixture.wait_for_jog(-1, "button4 must request negative position jog");
  fixture.button(0, true);
  fixture.wait_for_jog(0, "simultaneous fourth-axis buttons must halt");
  fixture.button(4, false);
  fixture.require_jog_zeros("one held button after a conflict must not rearm position jog");
  fixture.button(0, false);
  fixture.require_jog_zeros("both buttons must be released before rearming");
  fixture.button(0, true);
  fixture.wait_for_jog(1, "position jog must resume after both buttons were released");
}

void position_jog_stale_feedback_and_disconnect_require_release()
{
  RemoteFixture fixture(true, false, 0.0, true);
  fixture.initialize_arm();
  fixture.require_jog_zeros("position jog starts neutral", 300ms);
  fixture.button(0, true);
  fixture.wait_for_jog(1, "positive position jog must start");
  fixture.publish_pose(false);
  fixture.clear();
  fixture.wait_for_jog(0, "stale arm feedback must halt position jog", 800ms);
  fixture.publish_pose(true);
  fixture.require_jog_zeros("fresh feedback must not restart a still-held button");
  fixture.button(0, false);
  fixture.require_jog_zeros("release must rearm after stale feedback");
  fixture.button(4, true);
  fixture.wait_for_jog(-1, "negative position jog must start after release");

  fixture.clear();
  fixture.disconnect();
  fixture.wait_for_jog(0, "joystick disconnect must halt position jog");
  require(fixture.jog_commands.front() == 0,
    "first fourth-axis command after disconnect must halt");
  fixture.connect();
  fixture.initialize_arm(true);
  fixture.require_jog_zeros("reconnected held position button must not move", 1200ms);
  fixture.button(0, false);
  fixture.require_jog_zeros("reconnected position controls require release");
  fixture.button(0, true);
  fixture.wait_for_jog(1, "position jog must recover after reconnection and release");

  fixture.clear();
  fixture.link_available(false);
  fixture.wait_for_jog(0, "radio loss with live device must halt position jog", 450ms);
}

void tracking_button_requires_release_and_only_toggles_on_edges()
{
  RemoteFixture fixture(true, false, 0.0, true);
  fixture.initialize_arm(false, true);
  fixture.require_tracking(false, "held tracking button at initialization must not start", 250ms);
  fixture.button(6, true);
  fixture.require_tracking(false, "repeated held initialization button must not start");
  fixture.button(6, false);
  fixture.button(6, true);
  fixture.wait_for_tracking(true, "released tracking button must enter on the next rising edge");
  fixture.button(6, true);
  fixture.button(6, true);
  fixture.require_tracking(true, "held tracking button must not repeatedly toggle");
  fixture.button(6, false);
  fixture.require_tracking(true, "release alone must not cancel tracking");
  fixture.button(6, true);
  fixture.wait_for_tracking(false, "second tracking press must cancel");
  fixture.require_tracking(false, "cancelled tracking must remain off");

  fixture.button(2, true);
  fixture.button(2, false);
  fixture.require_tracking(false, "button2 must no longer request tracking");

  fixture.button(6, false);
  // Queue both presses before spinning: one timer read drains this entire batch.
  for (int press = 0; press < 2; ++press) {
    fixture.button(6, true);
    fixture.button(6, false);
  }
  fixture.require_tracking(false, "two tracking presses in one batch must remain off");
  fixture.button(6, true);
  fixture.button(6, false, true);
  fixture.require_tracking(false, "initialization must clear an earlier queued tracking edge");

  fixture.button(6, true);
  fixture.wait_for_tracking(true, "fresh press after initialization must enter tracking");
  fixture.button(6, false);
  for (int press = 0; press < 2; ++press) {
    fixture.button(6, true);
    fixture.button(6, false);
  }
  fixture.require_tracking(true, "two tracking presses in one batch must preserve the on state");
}

void tracking_waits_for_startup_and_neutral_without_queuing_presses()
{
  {
    RemoteFixture fixture(true, true, 0.0, true);
    fixture.initialize_arm();
    fixture.require_tracking(false, "waiting for startup must keep tracking off", 200ms);
    fixture.button(6, true);
    fixture.require_tracking(false, "tracking press while waiting must be discarded");
    fixture.stage(chassis::RemoteControlSession::Stage::starting);
    fixture.button(6, false);
    fixture.button(6, true);
    fixture.require_tracking(false, "tracking press during calibration must be discarded");
    fixture.stage(chassis::RemoteControlSession::Stage::running);
    fixture.require_tracking(false, "finishing calibration must not replay a tracking press");
    fixture.button(6, false);
    fixture.button(6, true);
    fixture.wait_for_tracking(true, "fresh press after calibration must enter tracking");
  }
  {
    RemoteFixture fixture(true, false, 0.0, true);
    fixture.initialize_arm();
    fixture.axes(32767, true);
    fixture.button(6, true);
    fixture.require_tracking(false, "uncentered joystick must reject tracking", 250ms);
    fixture.axes(0);
    fixture.require_tracking(false, "centering later must not replay rejected tracking press");
    fixture.button(6, false);
    fixture.button(6, true);
    fixture.wait_for_tracking(true, "fresh press after centering must enter tracking");
  }
}

void tracking_stop_and_enable_buttons_cancel_without_resuming()
{
  RemoteFixture fixture(true, false, 0.0, true, {
      rclcpp::Parameter("stop_button", 5), rclcpp::Parameter("enable_button", 7)});
  fixture.initialize_arm();
  fixture.button(7, true);
  fixture.require_tracking(false, "tracking starts off when joystick is enabled", 200ms);
  fixture.button(6, true);
  fixture.wait_for_tracking(true, "tracking must enter before stop test");
  fixture.button(5, true);
  fixture.wait_for_tracking(false, "stop button must cancel tracking");
  fixture.button(5, false);
  fixture.button(6, true);
  fixture.require_tracking(false, "stop release must not resume held tracking request");
  fixture.button(6, false);
  fixture.button(6, true);
  fixture.wait_for_tracking(true, "fresh press after stop release must enter tracking");
  fixture.button(7, false);
  fixture.wait_for_tracking(false, "releasing enable must cancel tracking");
  fixture.button(7, true);
  fixture.button(6, true);
  fixture.require_tracking(false, "enable recovery must not resume held tracking request");
}

void tracking_disconnect_and_timeout_require_a_fresh_press()
{
  {
    RemoteFixture fixture(true, false, 0.0, true);
    fixture.initialize_arm();
    fixture.require_tracking(false, "tracking starts off", 200ms);
    fixture.button(6, true);
    fixture.wait_for_tracking(true, "tracking must enter before disconnect test");
    fixture.disconnect();
    fixture.wait_for_tracking(false, "device disconnection must cancel tracking");
    fixture.connect();
    fixture.initialize_arm(false, true);
    fixture.require_tracking(false, "reconnected held tracking button must not resume", 1200ms);
    fixture.button(6, false);
    fixture.button(6, true);
    fixture.wait_for_tracking(true, "fresh press after reconnect must enter tracking");
    fixture.link_available(false);
    fixture.wait_for_tracking(false, "radio loss with live device must cancel tracking", 450ms);
    fixture.link_available(true);
    fixture.initialize_arm(false, true);
    fixture.require_tracking(false, "radio recovery must not restore tracking", 500ms);
    fixture.button(6, false);
    fixture.button(6, true);
    fixture.wait_for_tracking(true, "fresh press after radio recovery must enter tracking");
  }
  {
    RemoteFixture fixture(true, false, 0.35, true);
    fixture.initialize_arm();
    fixture.require_tracking(false, "tracking starts off before timeout test", 150ms);
    fixture.button(6, true);
    fixture.wait_for_tracking(true, "tracking must enter before timeout test");
    fixture.wait_for_tracking(false, "event timeout must cancel tracking", 700ms);
    fixture.axes(0);
    fixture.require_tracking(false, "fresh neutral alone must not resume tracking");
    fixture.button(6, false);
    fixture.button(6, true);
    fixture.wait_for_tracking(true, "fresh press after timeout must enter tracking");
  }
}

void tracking_reserves_chassis_but_preserves_fourth_axis()
{
  RemoteFixture fixture(true, false, 0.0, true);
  fixture.initialize_arm();
  fixture.require_tracking(false, "tracking starts off before coexistence test", 200ms);
  fixture.button(6, true);
  fixture.wait_for_tracking(true, "tracking must enter before manual suppression test");
  fixture.require_zeros("visual tracking must publish exact neutral chassis heartbeats");
  fixture.arm_axis(-32767);
  fixture.observe_for(200ms);
  require(fixture.manual_gets == 0 && fixture.manual_sets == 0,
    "tracking must suppress planar manual parameter requests");
  fixture.button(0, true);
  fixture.wait_for_jog(1, "tracking must preserve fourth-axis jog");
  fixture.axes(32767);
  fixture.wait_for_tracking(false, "chassis stick intervention must cancel visual tracking");
  fixture.wait_for([&fixture] {return !fixture.commands.empty() && moving(fixture.commands.back());},
    "manual intervention must reach the controller's ownership gate");
  fixture.button(6, false);
  fixture.button(6, true);
  fixture.require_tracking(false, "held chassis sticks must prevent tracking re-entry");
  fixture.observe_for(150ms);
  require(fixture.manual_gets == 0 && fixture.manual_sets == 0,
    "held planar stick must not replay after tracking exit");
  fixture.arm_axis(0);
  fixture.observe_for(100ms);
  fixture.arm_axis(-32767);
  fixture.wait_for([&fixture] {return fixture.manual_sets > 0;},
    "planar manual control must recover after centering and a fresh step");
}

void tracking_entry_clears_the_previous_chassis_ramp()
{
  RemoteFixture fixture(true, false, 0.0, true);
  fixture.initialize_arm();
  fixture.require_tracking(false, "tracking starts off before ramp test", 200ms);
  fixture.axes(32767);
  fixture.wait_for([&fixture] {
      return !fixture.commands.empty() && moving(fixture.commands.back()) &&
             fixture.commands.back().linear.x >= 0.04;
    }, "manual ramp must be moving before tracking entry");
  fixture.button(6, true);
  fixture.require_tracking(false, "tracking must not start with a moving chassis stick");
  fixture.axes(0);
  fixture.require_tracking(false, "neutral alone must not replay a rejected tracking press");
  fixture.button(6, false);
  fixture.button(6, true);
  fixture.wait_for_tracking(true, "fresh press with centered chassis sticks must enable tracking");
  fixture.require_zeros("tracking entry must clear the slow manual deceleration ramp");
}

void tracking_button_configuration_is_validated()
{
  const auto rejected = [](std::vector<rclcpp::Parameter> parameters) {
      bool invalid = false;
      try {
        chassis::make_remote_control(
          rclcpp::NodeOptions().parameter_overrides(parameters), nullptr, true, "");
      } catch (const std::invalid_argument & error) {
        invalid = std::string(error.what()).find("arm_tracking_button") != std::string::npos;
      }
      require(invalid, "invalid or conflicting tracking button must be rejected");
    };
  for (const int button : {-1, 256, 0, 1, 3, 4, 11, 14}) {
    rejected({rclcpp::Parameter("arm_tracking_button", button)});
  }
  for (const auto * name : {"enable_button", "stop_button", "emergency_stop_button", "start_button"}) {
    rejected({rclcpp::Parameter(name, 6)});
  }

  char pattern[] = "/tmp/chassis-tracking-button-test-XXXXXX";
  const auto directory = ::mkdtemp(pattern);
  require(directory != nullptr, "cannot create tracking button test directory");
  const auto device = std::filesystem::path(directory) / "joystick";
  require(::mkfifo(device.c_str(), 0600) == 0, "cannot create tracking button joystick FIFO");
  try {
    // FIFO ioctl falls back to 16 buttons; index16 must fail before any device events.
    rejected({rclcpp::Parameter("device_path", device.string()),
      rclcpp::Parameter("arm_tracking_button", 16)});
  } catch (...) {
    std::filesystem::remove_all(directory);
    throw;
  }
  std::filesystem::remove_all(directory);
}

void chassis_only_has_no_tracking_publisher()
{
  RemoteFixture fixture(true, false, 0.0, false, {
      rclcpp::Parameter("arm_tracking_button", 0)});
  fixture.observe_for(150ms);
  require(!fixture.tracking_publisher_exists(),
    "chassis-only remote must not create a tracking request publisher");
}

void stale_current_override_is_rejected()
{
  auto options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("arm_current_permille", 500)});
  bool rejected = false;
  try {
    chassis::make_remote_control(options, nullptr, true, "");
  } catch (const std::invalid_argument & error) {
    rejected = std::string(error.what()).find("arm_current_permille is retired") != std::string::npos;
  }
  require(rejected, "old current amplitude must not silently configure position jog");
}

using Session = chassis::RemoteControlSession;

std::shared_ptr<Session> shared_session(Session::Source owner = Session::Source::app)
{
  auto session = std::make_shared<Session>();
  session->shared_control = true;
  if (owner != Session::Source::none) {
    require(session->request_start(owner), "test startup request must be accepted");
    session->mark_running();
  }
  return session;
}

void relay_button_edges_and_debounce_match_both_control_modes()
{
  for (const bool shared : {false, true}) {
    auto session = shared ? shared_session() : std::make_shared<Session>();
    RemoteFixture fixture(true, false, 0.0, false, {
        rclcpp::Parameter("relay_enabled", true), rclcpp::Parameter("start_chassis", shared)},
      session);
    fixture.initialize_arm();
    fixture.button(7, true, true);
    fixture.observe_for(180ms);
    fixture.button(7, true);
    fixture.observe_for(60ms);
    require(fixture.relay_toggles == 0, "held initialization RB must not toggle the relay");
    if (shared) {
      require(session->owner == Session::Source::app,
        "held initialization RB must preserve App ownership");
    }
    // All events are read in one timer tick: only the first new press survives debounce.
    fixture.button(7, false);
    fixture.button(7, true);
    fixture.button(7, false);
    fixture.button(7, true);
    fixture.button(7, true);
    fixture.wait_for([&] {return fixture.relay_toggles == 1;},
      "a fresh released RB press must invoke the relay callback exactly once");
    if (shared) {
      require(session->owner == Session::Source::remote,
        "fresh RB must claim remote control and execute with its new lease");
    }
    fixture.observe_for(220ms);
    require(fixture.relay_toggles == 1,
      "button bounce and a long RB hold must not generate repeated relay toggles");
    fixture.button(7, false);
    fixture.observe_for(60ms);
    require(fixture.relay_toggles == 1, "RB release alone must retain relay state");
    fixture.button(7, true);
    fixture.wait_for([&] {return fixture.relay_toggles == 2;},
      "a second released RB press after debounce must toggle again");
  }
}

void relay_waits_for_neutral_and_discards_startup_presses()
{
  for (const bool shared : {false, true}) {
    auto session = shared ? shared_session() : std::make_shared<Session>();
    RemoteFixture fixture(true, false, 0.0, false, {
        rclcpp::Parameter("relay_enabled", true), rclcpp::Parameter("start_chassis", shared)},
      session);
    fixture.initialize_arm();
    fixture.axes(32767, true);
    fixture.button(7, true);
    fixture.observe_for(180ms);
    require(fixture.relay_toggles == 0, "uncentered initialization must reject RB");
    fixture.axes(0);
    fixture.observe_for(60ms);
    require(fixture.relay_toggles == 0, "centering later must not replay a rejected RB press");
    fixture.button(7, false);
    fixture.button(7, true);
    fixture.wait_for([&] {return fixture.relay_toggles == 1;},
      "fresh RB after neutral must toggle the relay");
    fixture.stage(Session::Stage::waiting);
    fixture.button(7, false);
    fixture.button(7, true);
    fixture.observe_for(80ms);
    fixture.stage(Session::Stage::starting);
    fixture.button(7, false);
    fixture.button(7, true);
    fixture.observe_for(80ms);
    fixture.stage(Session::Stage::running);
    fixture.observe_for(80ms);
    require(fixture.relay_toggles == 1,
      "waiting/calibration RB presses must not replay when startup finishes");
    fixture.button(7, false);
    fixture.button(7, true);
    fixture.wait_for([&] {return fixture.relay_toggles == 2;},
      "startup completion requires a fresh released RB press");
  }
}

void relay_reconnect_requires_release_without_changing_the_output()
{
  auto session = shared_session();
  RemoteFixture fixture(true, false, 0.0, false, {rclcpp::Parameter("relay_enabled", true)}, session);
  fixture.initialize_arm();
  fixture.observe_for(180ms);
  fixture.button(7, true);
  fixture.wait_for([&] {return fixture.relay_toggles == 1;}, "RB must toggle before link loss");
  fixture.disconnect();
  fixture.wait_for([&] {return session->owner == Session::Source::none;},
    "input removal must release the shared remote lease", 350ms);
  require(fixture.relay_toggles == 1, "input removal must retain the relay output");
  fixture.connect();
  fixture.initialize_arm();
  fixture.button(7, true, true);
  fixture.observe_for(1200ms);
  fixture.button(7, true);
  fixture.observe_for(60ms);
  require(fixture.relay_toggles == 1 && session->owner == Session::Source::none,
    "held RB after reconnect must neither toggle nor reclaim control");
  fixture.button(7, false);
  fixture.button(7, true);
  fixture.wait_for([&] {return fixture.relay_toggles == 2;},
    "released RB after reconnect must toggle with a fresh remote lease");
}

void relay_stop_enable_and_emergency_events_have_priority()
{
  auto session = shared_session();
  RemoteFixture fixture(true, false, 0.0, false, {
      rclcpp::Parameter("relay_enabled", true), rclcpp::Parameter("stop_button", 5),
      rclcpp::Parameter("enable_button", 9)}, session);
  fixture.initialize_arm();
  fixture.button(9, true, true);
  fixture.observe_for(180ms);
  fixture.button(7, true);
  fixture.button(5, true);
  fixture.button(5, false);
  fixture.observe_for(220ms);
  require(fixture.relay_toggles == 0 && session->owner == Session::Source::app,
    "even a short stop press in the same tick must discard a pending RB toggle");
  fixture.button(7, false);
  fixture.button(7, true);
  fixture.wait_for([&] {return fixture.relay_toggles == 1;},
    "fresh RB after the stop gesture must toggle");
  fixture.button(7, false);
  fixture.observe_for(220ms);
  fixture.button(7, true);
  fixture.button(9, false);
  fixture.button(9, true);
  fixture.observe_for(220ms);
  require(fixture.relay_toggles == 1,
    "even a short enable release in the same tick must discard a pending RB toggle");
  fixture.button(9, false);
  fixture.button(7, false);
  fixture.button(7, true);
  fixture.observe_for(60ms);
  require(fixture.relay_toggles == 1, "disabled joystick must discard RB presses");
  fixture.button(9, true);
  fixture.observe_for(220ms);
  require(fixture.relay_toggles == 1, "enable recovery must not replay a held RB press");
  fixture.button(7, false);
  fixture.button(7, true);
  fixture.button(11, true);
  fixture.wait_for([&] {return session->stage == Session::Stage::stopping;},
    "emergency stop must supersede an RB press in the same input tick");
  require(fixture.relay_toggles == 1 && rclcpp::ok(),
    "shared emergency stop must retain relay state and keep the resident runtime alive");
}

void relay_button_configuration_and_disabled_controls_are_validated()
{
  const auto rejected = [](std::vector<rclcpp::Parameter> parameters, bool with_session = true) {
      parameters.emplace_back("relay_enabled", true);
      bool invalid = false;
      try {
        chassis::make_remote_control(rclcpp::NodeOptions().parameter_overrides(parameters),
          with_session ? std::make_shared<Session>() : nullptr, true, "");
      } catch (const std::invalid_argument &) {invalid = true;}
      require(invalid, "relay control must reject missing sessions and conflicting button7 bindings");
    };
  rejected({}, false);
  for (const auto * name : {"enable_button", "stop_button", "emergency_stop_button",
      "start_button", "arm_tracking_button"})
  {
    rejected({rclcpp::Parameter(name, 7)});
  }
  auto session = std::make_shared<Session>();
  RemoteFixture fixture(true, false, 0.0, false, {
      rclcpp::Parameter("relay_enabled", false), rclcpp::Parameter("start_chassis", false)}, session);
  fixture.initialize_arm();
  fixture.observe_for(180ms);
  fixture.button(7, true);
  fixture.button(7, false);
  fixture.observe_for(60ms);
  require(fixture.relay_toggles == 0, "disabled relay controls must ignore button7");
}

void shared_app_owner_ignores_missing_or_timed_out_remote()
{
  {
    auto session = shared_session();
    RemoteFixture fixture(false, false, 0.0, true, {}, session);
    fixture.observe_for(250ms);
    fixture.link_available(false);
    fixture.observe_for(250ms);
    fixture.require_no_shared_output("missing or disconnected remote must not interrupt App");
    require(session->owner == Session::Source::app && fixture.owner_changes == 0,
      "inactive remote cannot release App ownership");
  }
  {
    auto session = shared_session();
    RemoteFixture fixture(true, false, 0.15, false, {}, session);
    fixture.axes(0, true);
    fixture.observe_for(400ms);
    fixture.require_no_shared_output("non-owner input timeout must not send zero to App");
    require(session->owner == Session::Source::app, "timeout must preserve App ownership");
  }
}

void shared_remote_takes_over_only_on_fresh_motion()
{
  auto session = shared_session();
  RemoteFixture fixture(true, false, 0.0, false, {}, session);
  fixture.axes(32767, true);
  fixture.observe_for(200ms);
  fixture.axes(32767);
  fixture.observe_for(100ms);
  fixture.require_no_shared_output("initialization and held sticks cannot take control");
  fixture.axes(0);
  fixture.observe_for(120ms);
  fixture.require_no_shared_output("neutral heartbeats cannot take control");
  fixture.axes(32767);
  fixture.wait_for([&] {
      return session->owner == Session::Source::remote && !fixture.commands.empty() &&
             moving(fixture.commands.back());
    }, "first fresh motion must take over and execute in the same input gesture");
  require(fixture.public_motion_messages == 0, "shared velocity must not use the public ROS topic");
  require(fixture.jog_commands.empty() && fixture.tracking_requests.empty() && fixture.steps.empty(),
    "chassis-only shared remote must not invoke arm command callbacks");

  require(session->claim(Session::Source::app), "App explicit reclaim must succeed");
  fixture.clear();
  fixture.observe_for(200ms);
  fixture.axes(32767);
  fixture.observe_for(100ms);
  fixture.require_no_shared_output("cached or repeatedly held sticks cannot steal App control");
  fixture.axes(0);
  fixture.observe_for(100ms);
  fixture.axes(32767);
  fixture.wait_for([&] {return session->owner == Session::Source::remote;},
    "new motion after centering must take over again");

  fixture.link_available(false);
  fixture.wait_for([&] {return session->owner == Session::Source::none;},
    "remote link loss must release its lease and trigger the shared stop", 450ms);
  fixture.clear();
  fixture.observe_for(150ms);
  fixture.require_no_shared_output("released remote must not keep sending zero heartbeats");
  require(session->owner == Session::Source::none, "App must not automatically resume on radio loss");
  require(session->claim(Session::Source::app), "App can explicitly reclaim after radio loss");
  fixture.observe_for(150ms);
  fixture.require_no_shared_output("continued radio loss must not interrupt the new App owner");
}

void shared_arm_steps_are_synchronous_and_expire_on_reclaim()
{
  auto session = shared_session();
  RemoteFixture fixture(true, false, 0.0, true, {}, session);
  fixture.initialize_arm();
  fixture.observe_for(250ms);
  fixture.require_no_shared_output("idle arm remote must not cancel App jog or tracking");
  require(!fixture.tracking_publisher_exists(), "shared mode must not create a tracking publisher");
  fixture.arm_axis(-32767);
  fixture.arm_axis(0, true);
  fixture.observe_for(150ms);
  fixture.require_no_shared_output("initialization must discard a queued arm step without taking control");
  require(session->owner == Session::Source::app, "cancelled arm input must preserve App ownership");
  fixture.arm_axis(-32767);
  fixture.wait_for([&] {return fixture.steps.size() == 1;},
    "first arm step must take over and execute without an async parameter request");
  require(session->owner == Session::Source::remote && fixture.steps.front().coordinate == 0 &&
    fixture.steps.front().delta == 0.001, "shared arm step must use the default one-millimetre magnitude");
  fixture.observe_for(150ms);
  require(fixture.steps.size() >= 2, "current owner may continue held step repeats");
  require(fixture.manual_gets == 0 && fixture.manual_sets == 0 && fixture.public_motion_messages == 0,
    "shared arm controls must bypass public topics and parameter services");

  require(session->claim(Session::Source::app), "App reclaim must succeed during held arm repeat");
  fixture.clear();
  fixture.observe_for(250ms);
  fixture.arm_axis(-32767);
  fixture.observe_for(100ms);
  fixture.require_no_shared_output("old arm step and held repeat must not survive App reclaim");
  fixture.arm_axis(0);
  fixture.observe_for(100ms);
  fixture.arm_axis(-32767);
  fixture.wait_for([&] {return !fixture.steps.empty();},
    "fresh arm step after neutral must regain control without losing the first step");
}

void shared_jog_and_tracking_require_new_released_button_presses()
{
  auto session = shared_session();
  RemoteFixture fixture(true, false, 0.0, true, {}, session);
  fixture.initialize_arm(true, true);
  fixture.observe_for(250ms);
  fixture.require_no_shared_output("held initialization jog/tracking buttons cannot take control");
  fixture.button(0, true);
  fixture.button(6, true);
  fixture.observe_for(100ms);
  fixture.require_no_shared_output("repeated held buttons cannot take control");
  fixture.button(0, false);
  fixture.button(6, false);
  fixture.observe_for(100ms);
  fixture.button(0, true);
  fixture.wait_for([&] {
      return session->owner == Session::Source::remote && !fixture.jog_commands.empty() &&
             fixture.jog_commands.back() == 1;
    }, "new released jog press must take over and preserve its first command");

  session->claim(Session::Source::app);
  fixture.clear();
  fixture.observe_for(150ms);
  fixture.require_no_shared_output("held jog must not survive App reclaim");
  fixture.button(0, false);
  fixture.observe_for(100ms);
  fixture.button(6, true);
  fixture.wait_for([&] {
      return session->owner == Session::Source::remote && !fixture.tracking_requests.empty() &&
             fixture.tracking_requests.back();
    }, "new tracking press must take over through the shared callback");
  require(fixture.public_motion_messages == 0, "shared jog/tracking must not publish public commands");
}

void shared_startup_cancellation_tracks_the_requesting_source()
{
  for (const auto source : {Session::Source::app, Session::Source::remote}) {
    auto session = shared_session(Session::Source::none);
    require(session->request_start(source), "startup request must succeed");
    session->stage = Session::Stage::starting;
    RemoteFixture fixture(false, false, 0.0, false, {}, session);
    fixture.observe_for(200ms);
    require(session->stage == (source == Session::Source::app ?
      Session::Stage::starting : Session::Stage::stopping),
      "missing remote may only cancel startup requested by that remote");
    require(rclcpp::ok(), "shared startup cancellation must not shut down the resident process");
  }
}

void shared_emergency_stop_is_global_and_allows_a_new_start_request()
{
  auto session = shared_session();
  RemoteFixture fixture(true, false, 0.0, false, {}, session);
  fixture.axes(0, true);
  fixture.button(11, false, true);
  fixture.button(14, false, true);
  fixture.observe_for(150ms);
  fixture.button(11, true);
  fixture.wait_for([&] {return session->stage == Session::Stage::stopping;},
    "remote emergency stop must stop even while App owns control");
  require(session->owner == Session::Source::none && rclcpp::ok(),
    "emergency stop must revoke ownership without killing the resident process");
  session->finish_stop();
  fixture.button(11, false);
  fixture.button(14, false);
  fixture.observe_for(120ms);
  fixture.button(14, true);
  fixture.wait_for([&] {return session->stage == Session::Stage::requested;},
    "a fresh start button press must work after a shared emergency stop");
  require(session->startup_source == Session::Source::remote,
    "remote start button must record the startup request source");
  fixture.link_available(false);
  fixture.wait_for([&] {return session->stage == Session::Stage::stopping;},
    "radio loss must cancel the remote's pending startup", 450ms);
  fixture.require_no_shared_output("startup and global stop must not leak legacy command messages");
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    relay_button_edges_and_debounce_match_both_control_modes();
    relay_waits_for_neutral_and_discards_startup_presses();
    relay_reconnect_requires_release_without_changing_the_output();
    relay_stop_enable_and_emergency_events_have_priority();
    relay_button_configuration_and_disabled_controls_are_validated();
    missing_joystick_publishes_zero(true);
    missing_joystick_publishes_zero(false);
    unplug_stops_immediately_and_reconnect_requires_neutral();
    configured_timeout_stops_immediately_and_requires_neutral();
    radio_loss_with_live_device_stops_and_requires_fresh_neutral();
    unverified_link_never_allows_motion();
    recovered_link_cannot_hide_failure_between_ros_ticks();
    position_jog_requires_release_and_stops_on_conflicting_buttons();
    position_jog_stale_feedback_and_disconnect_require_release();
    tracking_button_requires_release_and_only_toggles_on_edges();
    tracking_waits_for_startup_and_neutral_without_queuing_presses();
    tracking_stop_and_enable_buttons_cancel_without_resuming();
    tracking_disconnect_and_timeout_require_a_fresh_press();
    tracking_reserves_chassis_but_preserves_fourth_axis();
    tracking_entry_clears_the_previous_chassis_ramp();
    tracking_button_configuration_is_validated();
    chassis_only_has_no_tracking_publisher();
    stale_current_override_is_rejected();
    shared_app_owner_ignores_missing_or_timed_out_remote();
    shared_remote_takes_over_only_on_fresh_motion();
    shared_arm_steps_are_synchronous_and_expire_on_reclaim();
    shared_jog_and_tracking_require_new_released_button_presses();
    shared_startup_cancellation_tracks_the_requesting_source();
    shared_emergency_stop_is_global_and_allows_a_new_start_request();
    std::cout << "remote disconnect protection tests passed\n";
  } catch (const std::exception & error) {
    std::cerr << "remote disconnect protection: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
