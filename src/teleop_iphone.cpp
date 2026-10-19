#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

// Publish task-space goals to the native OCS2 target interface and, when
// present, go2_x5_planning. OCS2 owns joint commands.
class TeleopIphone : public rclcpp::Node {
 public:
  TeleopIphone() : Node("teleop_iphone") {
    iphone_source_frame_ = declare_parameter<std::string>("iphone_source_frame", "iphone_odom");
    iphone_target_frame_ = declare_parameter<std::string>("iphone_target_frame", "iphone");
    goal_frame_ = declare_parameter<std::string>("goal_frame", "world");
    const auto goal_topic = declare_parameter<std::string>("goal_topic", "/go2_x5/ee/goal");
    const auto mpc_target_topic = declare_parameter<std::string>(
        "mpc_target_topic", "/mobile_manipulator_mpc_target");
    mpc_input_dim_ = declare_parameter<int>("mpc_input_dim", 12);
    if (mpc_input_dim_ <= 0) throw std::invalid_argument("mpc_input_dim must be positive");
    const auto ee_topic = declare_parameter<std::string>("ee_state_topic", "/go2_x5/ee/state");
    const auto mode_topic = declare_parameter<std::string>("mode_topic", "/go2_x5/arm/mode/state");
    const auto mode_request_topic = declare_parameter<std::string>(
        "mode_request_topic", "/go2_x5/arm/mode/request");
    home_mode_ = declare_parameter<std::string>("home_mode", "HOME");
    if (home_mode_.empty()) throw std::invalid_argument("home_mode must not be empty");
    const auto gripper_topic = declare_parameter<std::string>(
        "gripper_command_topic", "/go2_x5/arm/gripper/command");
    gripper_open_width_ = declare_parameter<double>("gripper_open_width", 0.088);
    gripper_closed_width_ = declare_parameter<double>("gripper_closed_width", 0.0);
    if (!std::isfinite(gripper_open_width_) || !std::isfinite(gripper_closed_width_) ||
        gripper_closed_width_ < 0.0 || gripper_open_width_ <= gripper_closed_width_) {
      throw std::invalid_argument("Gripper widths must be finite with 0 <= closed < open");
    }
    gripper_filter_alpha_ = declare_parameter<double>("gripper_filter_alpha", 0.3);
    gripper_deadband_ = declare_parameter<double>("gripper_deadband", 0.002);
    if (!std::isfinite(gripper_filter_alpha_) || gripper_filter_alpha_ <= 0.0 || gripper_filter_alpha_ > 1.0 ||
        !std::isfinite(gripper_deadband_) || gripper_deadband_ < 0.0) {
      throw std::invalid_argument("gripper_filter_alpha must be in (0, 1]; gripper_deadband must be nonnegative");
    }
    const auto touch_topic = declare_parameter<std::string>("touch_topic", "/iphone/touch");
    const auto buttons_topic = declare_parameter<std::string>("buttons_topic", "/iphone/buttons");
    const auto tracking_topic = declare_parameter<std::string>("tracking_ok_topic", "/iphone/tracking_ok");
    const auto opening_topic = declare_parameter<std::string>("gripper_opening_topic", "/iphone/gripper/opening");
    const auto status_topic = declare_parameter<std::string>("status_topic", "/teleop_iphone/status");
    const auto marker_topic = declare_parameter<std::string>("status_marker_topic", "/teleop_iphone/status_marker");
    touch_timeout_ = positive_parameter("touch_timeout", 0.2);
    pose_timeout_ = positive_parameter("pose_timeout", 0.2);
    ee_timeout_ = positive_parameter("ee_state_timeout", 0.5);
    require_touch_clutch_ = declare_parameter<bool>("require_touch_clutch", false);
    clutch_delay_ = declare_parameter<double>("clutch_engage_delay", 0.15);
    home_hold_time_ = positive_parameter("home_hold_time", 2.0);
    position_scale_ = declare_parameter<double>("position_scale", 1.0);
    rotation_scale_ = declare_parameter<double>("rotation_scale", 0.6);
    scale_range_ = declare_parameter<double>("scale_slider_range", 3.0);
    precision_factor_ = declare_parameter<double>("precision_factor", 0.3);
    align_hold_time_ = positive_parameter("align_hold_time", 0.6);
    align_reference_frame_ = declare_parameter<std::string>("align_reference_frame", "");
    // Buttons are 1-based to match the phone labels f1..f8.
    position_scale_button_ = button_parameter("position_scale_button", 1);
    home_button_ = button_parameter("home_button", 2);
    axis_lock_button_ = button_parameter("axis_lock_button", 3);
    rotation_scale_button_ = button_parameter("rotation_scale_button", 4);
    precision_button_ = button_parameter("precision_button", 5);
    align_button_ = button_parameter("align_button", 6);
    ocs2_button_ = button_parameter("ocs2_button", 7);
    wbc_button_ = button_parameter("wbc_button", 8);
    phone_yaw_ = declare_parameter<double>("phone_to_world_yaw", 1.5707963267948966);
    const double rate = positive_parameter("control_rate", 50.0);
    if (!std::isfinite(clutch_delay_) || clutch_delay_ < 0.0 ||
        !std::isfinite(position_scale_) || position_scale_ < 0.0 ||
        !std::isfinite(rotation_scale_) || rotation_scale_ < 0.0 || !std::isfinite(phone_yaw_) ||
        !std::isfinite(scale_range_) || scale_range_ < 1.0 ||
        !std::isfinite(precision_factor_) || precision_factor_ <= 0.0 || precision_factor_ > 1.0) {
      throw std::invalid_argument(
          "Clutch delay and scales must be finite and nonnegative, scale_slider_range >= 1, "
          "precision_factor in (0, 1], yaw finite");
    }
    phone_to_world_.setRPY(0.0, 0.0, phone_yaw_);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, rclcpp::QoS(1));
    mpc_target_pub_ = create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
        mpc_target_topic, rclcpp::QoS(1).transient_local());
    mode_request_pub_ = create_publisher<std_msgs::msg::String>(mode_request_topic, rclcpp::QoS(10));
    gripper_pub_ = create_publisher<std_msgs::msg::Float64>(gripper_topic, rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, rclcpp::QoS(1));
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic, rclcpp::QoS(1));
    touch_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
        touch_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
          touch_count_ = msg->poses.size();
          last_touch_ = Clock::now();
          have_touch_ = true;
          RCLCPP_INFO_ONCE(get_logger(), "Received iPhone touch input");
        });
    buttons_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        buttons_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) { on_buttons(*msg); });
    tracking_sub_ = create_subscription<std_msgs::msg::Bool>(
        tracking_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          // Stop at once on stream end or lost ARKit tracking instead of
          // waiting for the pose timeout.
          if (!msg->data) disengage("iPhone tracking lost or stream inactive");
          tracking_ok_ = msg->data;
        });
    opening_sub_ = create_subscription<std_msgs::msg::Float64>(
        opening_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](std_msgs::msg::Float64::ConstSharedPtr msg) { on_gripper_opening(msg->data); });
    ee_sub_ = create_subscription<ocs2_msgs::msg::MpcObservation>(
        ee_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
          const auto& value = msg->state.value;
          if (value.size() != 7 || msg->mode == 0 || !std::isfinite(msg->time)) {
            have_ee_ = false;
            disengage("invalid MPC observation");
            return;
          }
          tf2::Vector3 position(value[0], value[1], value[2]);
          tf2::Quaternion orientation(value[3], value[4], value[5], value[6]);
          if (!valid_pose(position, orientation)) {
            have_ee_ = false;
            disengage("invalid MPC EE pose");
            return;
          }
          if (have_ee_ && msg->time < last_mpc_time_) disengage("MPC time went backwards");
          ee_position_ = position;
          ee_orientation_ = orientation.normalized();
          if (!msg->input.value.empty()) {
            mpc_input_dim_ = static_cast<int>(msg->input.value.size());
          }
          last_mpc_time_ = msg->time;
          last_ee_ = Clock::now();
          have_ee_ = true;
          RCLCPP_INFO_ONCE(get_logger(), "Received valid MPC EE feedback");
        });
    mode_sub_ = create_subscription<std_msgs::msg::String>(
        mode_topic, rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          // A transient-local publisher normally delivers the current mode
          // immediately on startup. Do not treat that initial snapshot as a
          // mode transition, otherwise the first active touch is blocked
          // until the phone happens to emit an empty release frame.
          if (!mode_.empty() && msg->data != mode_) {
            disengage("arm mode changed");
            have_ee_ = false;  // Require feedback from the new controller mode.
            gripper_sent_ = NAN;  // HOME/HOLD may move the gripper.
            home_hold_until_ = Clock::time_point{};
          }
          if (msg->data != mode_) RCLCPP_INFO(get_logger(), "Arm mode is %s", msg->data.c_str());
          mode_ = msg->data;
        });
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
        [this]() { control(); });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() { publish_status(); });

    // Gamepad mode: left stick drives forward/lateral velocity, right stick
    // x the yaw rate (Unitree remote layout). rl_sar ignores commands older
    // than its own timeout; this node also sends one zero when the phone
    // stream stops. ABXY and the D-pad are not mapped.
    const auto gamepad_topic = declare_parameter<std::string>("gamepad_topic", "/iphone/gamepad");
    const auto base_velocity_topic = declare_parameter<std::string>("base_velocity_topic", "/go2_x5/base/cmd_vel");
    gamepad_timeout_ = positive_parameter("gamepad_timeout", 0.2);
    max_linear_x_ = positive_parameter("gamepad_max_linear_x", 0.8);
    max_linear_y_ = positive_parameter("gamepad_max_linear_y", 0.4);
    max_yaw_rate_ = positive_parameter("gamepad_max_yaw_rate", 1.0);
    base_velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>(base_velocity_topic, rclcpp::QoS(1));
    gamepad_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        gamepad_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) { on_gamepad(*msg); });
    gamepad_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      if (gamepad_driving_ && std::chrono::duration<double>(Clock::now() - last_gamepad_).count() > gamepad_timeout_) {
        base_velocity_pub_->publish(geometry_msgs::msg::Twist());
        gamepad_driving_ = false;
        RCLCPP_WARN(get_logger(), "Gamepad stream stopped; base velocity zeroed");
      }
    });
    RCLCPP_INFO(get_logger(),
                "EE teleop: %s -> %s and %s (%s). Waiting for OCS2/WBC, fresh EE feedback and touch input.",
                ee_topic.c_str(), goal_topic.c_str(), mpc_target_topic.c_str(), goal_frame_.c_str());
  }

 private:
  using Clock = std::chrono::steady_clock;
  static constexpr size_t kMaxButtons = 8;

  double positive_parameter(const std::string& name, double default_value) {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument(name + " must be positive and finite");
    return value;
  }

  size_t button_parameter(const std::string& name, int default_value) {
    const auto value = declare_parameter<int>(name, default_value);
    if (value < 1 || value > static_cast<int>(kMaxButtons)) {
      throw std::invalid_argument(name + " must be in [1, 8]");
    }
    return static_cast<size_t>(value - 1);
  }

  static bool valid_pose(const tf2::Vector3& p, const tf2::Quaternion& q) {
    return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z()) &&
           std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()) &&
           std::isfinite(q.w()) && std::isfinite(q.length2()) && q.length2() > 1e-12;
  }

  // Any disengage while fingers are down requires a full release. Log the
  // reason once per interrupted clutch so dropouts can be diagnosed.
  void disengage(const char* reason) {
    if (pressing_) {
      if (require_touch_clutch_) {
        RCLCPP_WARN(get_logger(), "Tracking interrupted: %s (fingers=%zu); release to re-clutch",
                    reason, clutch_fingers());
      } else {
        RCLCPP_WARN(get_logger(), "Tracking interrupted: %s", reason);
      }
    }
    pressing_ = false;
    release_required_ = true;
  }

  // Joy layout from osc_node: buttons[i] pressed, buttons[N+i] latched,
  // buttons[2N+i] press counter, axes[i] slider value in [0, 1].
  void on_buttons(const sensor_msgs::msg::Joy& msg) {
    const size_t n = std::min(msg.axes.size(), kMaxButtons);
    if (msg.buttons.size() < 3 * msg.axes.size()) return;
    const auto wall = Clock::now();
    const size_t stride = msg.axes.size();
    for (size_t i = 0; i < n; ++i) {
      pressed_[i] = msg.buttons[i] != 0;
      latched_[i] = msg.buttons[stride + i] != 0;
      const double value =
          std::isfinite(msg.axes[i]) ? std::clamp(static_cast<double>(msg.axes[i]), 0.0, 1.0) : 0.5;
      // A finger dragging a latched slider is not a clutch finger.
      if (latched_[i] && have_buttons_ && std::abs(value - values_[i]) > 1e-3) {
        slider_active_until_ = wall + std::chrono::milliseconds(300);
      }
      values_[i] = value;
      const int presses = msg.buttons[2 * stride + i];
      // Counters restart with each phone stream; only an increase is a tap.
      const bool tapped = have_buttons_ && presses > presses_[i];
      presses_[i] = presses;
      if (tapped && i == home_button_) request_home();
      if (tapped && i == ocs2_button_) request_mode(ocs2_button_, "OCS2");
      if (tapped && i == wbc_button_) request_mode(wbc_button_, "WBC");
    }
    for (size_t i = n; i < kMaxButtons; ++i) pressed_[i] = latched_[i] = false;
    have_buttons_ = true;

    if (pressed_[align_button_]) {
      if (!align_holding_) align_since_ = wall;
      align_holding_ = true;
      if (!align_done_ && std::chrono::duration<double>(wall - align_since_).count() >= align_hold_time_) {
        align_pending_ = true;
        align_done_ = true;
        std::cout<<"heading align done"<<std::endl;
      }
    } else {
      align_holding_ = align_done_ = false;
    }
  }

  void on_gamepad(const sensor_msgs::msg::Joy& msg) {
    if (msg.axes.size() < 4) return;
    for (size_t i = 0; i < 4; ++i) {
      if (!std::isfinite(msg.axes[i])) return;
    }
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = std::clamp(static_cast<double>(msg.axes[1]), -1.0, 1.0) * max_linear_x_;
    velocity.linear.y = -std::clamp(static_cast<double>(msg.axes[0]), -1.0, 1.0) * max_linear_y_;
    velocity.angular.z = -std::clamp(static_cast<double>(msg.axes[2]), -1.0, 1.0) * max_yaw_rate_;
    base_velocity_pub_->publish(velocity);
    last_gamepad_ = Clock::now();
    if (!gamepad_driving_) RCLCPP_INFO(get_logger(), "Gamepad driving the base on velocity topic");
    gamepad_driving_ = true;
  }

  void request_home() {
    disengage("HOME button");
    // Without a touch clutch, keep teleop from re-engaging before the arm
    // reports the mode switch.
    home_hold_until_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                          std::chrono::duration<double>(home_hold_time_));
    std_msgs::msg::String request;
    request.data = home_mode_;
    mode_request_pub_->publish(request);
    RCLCPP_WARN(get_logger(), "f%zu: requesting arm mode %s", home_button_ + 1, home_mode_.c_str());
  }

  // The mode-state callback disengages once the switch is reported, so
  // tracking re-anchors at the measured EE under the new controller.
  void request_mode(size_t button, const char* mode) {
    if (mode_ == mode) {
      RCLCPP_INFO(get_logger(), "f%zu: arm mode is already %s", button + 1, mode);
      return;
    }
    std_msgs::msg::String request;
    request.data = mode;
    mode_request_pub_->publish(request);
    RCLCPP_WARN(get_logger(), "f%zu: requesting arm mode %s", button + 1, mode);
  }

  // Fingers resting on unlatched buttons are part of touch.count but are not
  // clutch fingers; a latched button stays pressed without a finger.
  size_t clutch_fingers() const {
    size_t on_buttons = 0;
    for (size_t i = 0; i < kMaxButtons; ++i) on_buttons += pressed_[i] && !latched_[i];
    if (Clock::now() < slider_active_until_) ++on_buttons;
    return touch_count_ > on_buttons ? touch_count_ - on_buttons : 0;
  }

  // Latched slider: 50% keeps the base scale, 0%/100% divide/multiply it by
  // scale_slider_range.
  double slider_factor(size_t button) const {
    return latched_[button] ? std::pow(scale_range_, 2.0 * values_[button] - 1.0) : 1.0;
  }

  enum class AxisLock { None, TranslationOnly, RotationOnly };
  AxisLock axis_lock() const {
    if (!latched_[axis_lock_button_]) return AxisLock::None;
    return values_[axis_lock_button_] > 0.5 ? AxisLock::RotationOnly : AxisLock::TranslationOnly;
  }
  bool precision() const { return pressed_[precision_button_]; }

  void effective_scales(double& position_scale, double& rotation_scale) const {
    const double precise = precision() ? precision_factor_ : 1.0;
    const auto lock = axis_lock();
    position_scale = lock == AxisLock::RotationOnly ? 0.0
                                                    : position_scale_ * slider_factor(position_scale_button_) * precise;
    rotation_scale = lock == AxisLock::TranslationOnly
                         ? 0.0
                         : std::min(1.0, rotation_scale_ * slider_factor(rotation_scale_button_) * precise);
  }

  void on_gripper_opening(double opening) {
    if (!std::isfinite(opening)) return;
    opening = std::clamp(opening, 0.0, 1.0);
    // The marker estimate is ~14 Hz and noisy; while it is invalid osc_node
    // publishes nothing, so the last width is held.
    gripper_filtered_ = std::isfinite(gripper_filtered_)
                            ? gripper_filter_alpha_ * opening + (1.0 - gripper_filter_alpha_) * gripper_filtered_
                            : opening;
    if (mode_ != "OCS2" && mode_ != "WBC") return;
    if (!tracking_ok_) return;
    const double width = gripper_closed_width_ + gripper_filtered_ * (gripper_open_width_ - gripper_closed_width_);
    if (std::isfinite(gripper_sent_) && std::abs(width - gripper_sent_) < gripper_deadband_) return;
    std_msgs::msg::Float64 command;
    command.data = width;
    gripper_pub_->publish(command);
    gripper_sent_ = width;
  }

  // Map the phone heading (camera axis, or the top edge when held flat) onto
  // the reference heading: world +x, or align_reference_frame's +x.
  void align_heading(const tf2::Quaternion& phone_orientation) {
    tf2::Vector3 heading = tf2::quatRotate(phone_orientation, tf2::Vector3(0.0, 0.0, -1.0));
    if (std::hypot(heading.x(), heading.y()) < 0.3) {
      heading = tf2::quatRotate(phone_orientation, tf2::Vector3(0.0, 1.0, 0.0));
    }
    double reference_yaw = 0.0;
    if (!align_reference_frame_.empty()) {
      try {
        const auto t = tf_buffer_->lookupTransform(goal_frame_, align_reference_frame_, tf2::TimePointZero);
        const auto& q = t.transform.rotation;
        double roll, pitch;
        tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, reference_yaw);
      } catch (const tf2::TransformException& error) {
        RCLCPP_WARN(get_logger(), "Align: %s unavailable (%s); using %s +x", align_reference_frame_.c_str(),
                    error.what(), goal_frame_.c_str());
      }
    }
    phone_yaw_ = std::remainder(reference_yaw - std::atan2(heading.y(), heading.x()), 2.0 * M_PI);
    phone_to_world_.setRPY(0.0, 0.0, phone_yaw_);
    RCLCPP_WARN(get_logger(), "f%zu: phone_to_world_yaw set to %.3f rad (%.1f deg)", align_button_ + 1, phone_yaw_,
                phone_yaw_ * 180.0 / M_PI);
  }

  void control() {
    const auto wall = Clock::now();
    if (mode_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Not publishing EE target: waiting for arm mode state");
      return;
    }
    if (mode_ != "OCS2" && mode_ != "WBC") {
      disengage("arm mode is not OCS2/WBC");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Not publishing EE target: arm mode is '%s'", mode_.c_str());
      return;
    }
    if (!have_touch_ || std::chrono::duration<double>(wall - last_touch_).count() > touch_timeout_) {
      disengage("iPhone stream timed out");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Not publishing EE target: waiting for fresh iPhone stream");
      return;
    }

    tf2::Vector3 phone_position;
    tf2::Quaternion phone_orientation;
    bool have_phone = false;
    try {
      const auto transform = tf_buffer_->lookupTransform(iphone_source_frame_, iphone_target_frame_, tf2::TimePointZero);
      const rclcpp::Time stamp(transform.header.stamp, get_clock()->get_clock_type());
      const double age = (now() - stamp).seconds();
      const auto& p = transform.transform.translation;
      const auto& q = transform.transform.rotation;
      phone_position = tf2::Vector3(p.x, p.y, p.z);
      phone_orientation = tf2::Quaternion(q.x, q.y, q.z, q.w);
      // A cached TF must not masquerade as a new phone sample.
      if (age < 0.0 || age > pose_timeout_) {
        disengage(age < 0.0 ? "iPhone TF stamp is in the future" : "iPhone TF is stale");
      } else if (!valid_pose(phone_position, phone_orientation)) {
        disengage("invalid iPhone pose");
      } else {
        phone_orientation.normalize();
        have_phone = true;
      }
    } catch (const tf2::TransformException& error) {
      disengage("iPhone TF lookup failed");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "iPhone pose unavailable: %s", error.what());
    }
    if (align_pending_ && have_phone) {
      align_pending_ = false;
      align_heading(phone_orientation);
      if (pressing_) reanchor_ = true;
    }

    // With a touch clutch, the required release already covers HOME.
    if (!require_touch_clutch_ && wall < home_hold_until_) {
      pressing_ = false;
      return;
    }
    if (require_touch_clutch_) {
      const size_t fingers = clutch_fingers();
      if (fingers == 0) {
        // Observe an explicit release before enabling/re-enabling the clutch.
        pressing_ = false;
        release_required_ = false;
        clutch_pending_ = false;
        return;
      }
      // Button taps briefly show up as a clutch finger before the button
      // state arrives; require the touch to persist.
      if (!clutch_pending_) {
        clutch_pending_ = true;
        clutch_since_ = wall;
      }
      if (std::chrono::duration<double>(wall - clutch_since_).count() < clutch_delay_) return;
      if (release_required_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Not publishing EE target: release touch before re-clutching");
        return;
      }
    }
    // Without a touch clutch the phone's transmit switch is the clutch:
    // healthy stream engages (re-anchored), inactive/lost stream stops.
    if (!tracking_ok_) {
      disengage("iPhone tracking lost or stream inactive");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Not publishing EE target: iPhone stream inactive or tracking lost");
      return;
    }
    if (!have_ee_ || std::chrono::duration<double>(wall - last_ee_).count() > ee_timeout_) {
      disengage("MPC EE feedback timed out");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for fresh MPC EE feedback; release to re-clutch");
      return;
    }
    if (!have_phone) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Not publishing EE target: no fresh iPhone pose");
      return;
    }

    double position_scale, rotation_scale;
    effective_scales(position_scale, rotation_scale);
    // Changing a scale or mode mid-clutch restarts the relative motion from
    // the last commanded target, so the goal never jumps.
    if (pressing_ && (reanchor_ || position_scale != active_position_scale_ ||
                      rotation_scale != active_rotation_scale_)) {
      start_phone_position_ = phone_position;
      start_phone_orientation_ = phone_orientation;
      start_ee_position_ = target_position_;
      start_ee_orientation_ = target_orientation_;
    }
    reanchor_ = false;
    if (!pressing_) {
      start_phone_position_ = phone_position;
      start_phone_orientation_ = phone_orientation;
      start_ee_position_ = ee_position_;
      start_ee_orientation_ = ee_orientation_;
      pressing_ = true;
      RCLCPP_INFO(get_logger(), "Tracking engaged at measured EE [%.3f, %.3f, %.3f]",
                  ee_position_.x(), ee_position_.y(), ee_position_.z());
    }
    active_position_scale_ = position_scale;
    active_rotation_scale_ = rotation_scale;
    const auto position = start_ee_position_ +
        tf2::quatRotate(phone_to_world_, phone_position - start_phone_position_) * position_scale;
    auto delta = phone_orientation * start_phone_orientation_.inverse();
    delta = phone_to_world_ * delta * phone_to_world_.inverse();
    delta.normalize();
    // Use the short rotation, including when the phone quaternion changes sign.
    if (delta.w() < 0.0) delta = tf2::Quaternion(-delta.x(), -delta.y(), -delta.z(), -delta.w());
    delta = tf2::Quaternion(0, 0, 0, 1).slerp(delta, rotation_scale);
    const auto orientation = (delta * start_ee_orientation_).normalized();
    if (!valid_pose(position, orientation)) {
      disengage("invalid EE target");
      return;
    }
    target_position_ = position;
    target_orientation_ = orientation;
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = goal_frame_;
    goal.pose.position.x = position.x();
    goal.pose.position.y = position.y();
    goal.pose.position.z = position.z();
    goal.pose.orientation.x = orientation.x();
    goal.pose.orientation.y = orientation.y();
    goal.pose.orientation.z = orientation.z();
    goal.pose.orientation.w = orientation.w();
    goal_pub_->publish(goal);

    // Publish the native OCS2 target as well. This keeps teleop functional
    // when the optional EE reference manager is not running, while the
    // PoseStamped path remains available for the existing planner.
    ocs2_msgs::msg::MpcTargetTrajectories target;
    target.time_trajectory = {last_mpc_time_};
    ocs2_msgs::msg::MpcState state;
    state.value = {
        static_cast<float>(position.x()), static_cast<float>(position.y()), static_cast<float>(position.z()),
        static_cast<float>(orientation.x()), static_cast<float>(orientation.y()),
        static_cast<float>(orientation.z()), static_cast<float>(orientation.w())};
    target.state_trajectory = {state};
    ocs2_msgs::msg::MpcInput input;
    input.value.assign(static_cast<size_t>(mpc_input_dim_), 0.0f);
    target.input_trajectory = {input};
    mpc_target_pub_->publish(target);
  }

  // The phone has no return channel, so expose the tuned state as a topic and
  // an RViz text marker above the measured EE.
  void publish_status() {
    double position_scale, rotation_scale;
    effective_scales(position_scale, rotation_scale);
    const auto lock = axis_lock();
    char text[256];
    std::snprintf(text, sizeof(text),
                  "mode=%s clutch=%s tracking=%s pos_scale=%.2f rot_scale=%.2f lock=%s precision=%s "
                  "yaw=%.1fdeg gripper=%s",
                  mode_.empty() ? "?" : mode_.c_str(), pressing_ ? "on" : "off", tracking_ok_ ? "ok" : "lost",
                  position_scale, rotation_scale,
                  lock == AxisLock::None ? "none" : lock == AxisLock::RotationOnly ? "rotation" : "translation",
                  precision() ? "on" : "off", phone_yaw_ * 180.0 / M_PI,
                  std::isfinite(gripper_sent_) ? (std::to_string(gripper_sent_).substr(0, 5) + "m").c_str() : "-");
    std_msgs::msg::String status;
    status.data = text;
    status_pub_->publish(status);
    if (!have_ee_) return;
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = goal_frame_;
    marker.ns = "teleop_iphone";
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = ee_position_.x();
    marker.pose.position.y = ee_position_.y();
    marker.pose.position.z = ee_position_.z() + 0.25;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.05;
    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0f;
    if (pressing_) marker.color.b = 0.2f;
    char label[160];
    std::snprintf(label, sizeof(label), "pos x%.2f  rot x%.2f\nlock %s%s",
                  position_scale, rotation_scale,
                  lock == AxisLock::None ? "none" : lock == AxisLock::RotationOnly ? "rotation" : "translation",
                  precision() ? "  PRECISION" : "");
    marker.text = label;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker_pub_->publish(marker);
  }

  std::string iphone_source_frame_, iphone_target_frame_, goal_frame_, mode_, home_mode_, align_reference_frame_;
  double touch_timeout_, pose_timeout_, ee_timeout_, clutch_delay_, home_hold_time_, position_scale_, rotation_scale_;
  double scale_range_, precision_factor_, align_hold_time_, phone_yaw_;
  double gamepad_timeout_, max_linear_x_, max_linear_y_, max_yaw_rate_;
  bool gamepad_driving_ = false;
  Clock::time_point last_gamepad_;
  double gripper_open_width_, gripper_closed_width_, gripper_filter_alpha_, gripper_deadband_;
  double gripper_filtered_ = NAN, gripper_sent_ = NAN;
  double active_position_scale_ = 0.0, active_rotation_scale_ = 0.0;
  size_t position_scale_button_, home_button_, axis_lock_button_, rotation_scale_button_;
  size_t precision_button_, align_button_, ocs2_button_, wbc_button_;
  std::array<bool, kMaxButtons> pressed_{}, latched_{};
  std::array<double, kMaxButtons> values_{};
  std::array<int, kMaxButtons> presses_{};
  double last_mpc_time_ = 0.0;
  int mpc_input_dim_ = 12;
  bool have_touch_ = false, have_ee_ = false, have_buttons_ = false, tracking_ok_ = false;
  bool require_touch_clutch_ = false, pressing_ = false, release_required_ = false, clutch_pending_ = false, reanchor_ = false;
  bool align_holding_ = false, align_done_ = false, align_pending_ = false;
  size_t touch_count_ = 0;
  Clock::time_point last_touch_, last_ee_, clutch_since_, align_since_, slider_active_until_, home_hold_until_;
  tf2::Vector3 ee_position_, start_phone_position_, start_ee_position_, target_position_;
  tf2::Quaternion ee_orientation_, start_phone_orientation_, start_ee_orientation_, target_orientation_;
  tf2::Quaternion phone_to_world_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr mpc_target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_request_pub_, status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_velocity_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr touch_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr buttons_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tracking_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr opening_sub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr ee_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_, gamepad_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TeleopIphone>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("teleop_iphone"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
