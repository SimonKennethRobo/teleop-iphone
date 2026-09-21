#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
    const auto touch_topic = declare_parameter<std::string>("touch_topic", "/iphone/touch");
    touch_threshold_ = declare_parameter<double>("touch_threshold", 0.3);
    touch_timeout_ = positive_parameter("touch_timeout", 0.2);
    pose_timeout_ = positive_parameter("pose_timeout", 0.2);
    ee_timeout_ = positive_parameter("ee_state_timeout", 0.5);
    position_scale_ = declare_parameter<double>("position_scale", 1.0);
    rotation_scale_ = declare_parameter<double>("rotation_scale", 0.6);
    const double yaw = declare_parameter<double>("phone_to_world_yaw", 1.5707963267948966);
    const double rate = positive_parameter("control_rate", 50.0);
    if (!std::isfinite(touch_threshold_) || touch_threshold_ < 0.0 ||
        !std::isfinite(position_scale_) || position_scale_ < 0.0 ||
        !std::isfinite(rotation_scale_) || rotation_scale_ < 0.0 || !std::isfinite(yaw)) {
      throw std::invalid_argument("Touch threshold and scales must be finite and nonnegative; yaw must be finite");
    }
    phone_to_world_.setRPY(0.0, 0.0, yaw);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, rclcpp::QoS(1));
    mpc_target_pub_ = create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
        mpc_target_topic, rclcpp::QoS(1).transient_local());
    touch_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
        touch_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
          touch_count_ = 0;
          for (const auto& pose : msg->poses) {
            if (std::isfinite(pose.orientation.w) && pose.orientation.w > touch_threshold_) ++touch_count_;
          }
          last_touch_ = Clock::now();
          have_touch_ = true;
          RCLCPP_INFO_ONCE(get_logger(), "Received iPhone touch input");
          // Observe an explicit release before enabling/re-enabling the clutch.
          if (touch_count_ == 0) {
            pressing_ = false;
            release_required_ = false;
          }
        });
    ee_sub_ = create_subscription<ocs2_msgs::msg::MpcObservation>(
        ee_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
          const auto& value = msg->state.value;
          if (value.size() != 7 || msg->mode == 0 || !std::isfinite(msg->time)) {
            have_ee_ = false;
            disengage();
            return;
          }
          tf2::Vector3 position(value[0], value[1], value[2]);
          tf2::Quaternion orientation(value[3], value[4], value[5], value[6]);
          if (!valid_pose(position, orientation)) {
            have_ee_ = false;
            disengage();
            return;
          }
          if (have_ee_ && msg->time < last_mpc_time_) disengage();
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
            disengage();
            have_ee_ = false;  // Require feedback from the new controller mode.
          }
          if (msg->data != mode_) RCLCPP_INFO(get_logger(), "Arm mode is %s", msg->data.c_str());
          mode_ = msg->data;
        });
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
        [this]() { control(); });
    RCLCPP_INFO(get_logger(),
                "EE teleop: %s -> %s and %s (%s). Waiting for OCS2/WBC, fresh EE feedback and touch input.",
                ee_topic.c_str(), goal_topic.c_str(), mpc_target_topic.c_str(), goal_frame_.c_str());
  }

 private:
  using Clock = std::chrono::steady_clock;

  double positive_parameter(const std::string& name, double default_value) {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument(name + " must be positive and finite");
    return value;
  }

  static bool valid_pose(const tf2::Vector3& p, const tf2::Quaternion& q) {
    return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z()) &&
           std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()) &&
           std::isfinite(q.w()) && std::isfinite(q.length2()) && q.length2() > 1e-12;
  }

  void disengage() {
    pressing_ = false;
    release_required_ = true;
  }

  void control() {
    const auto wall = Clock::now();
    if (mode_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Not publishing EE target: waiting for arm mode state");
      return;
    }
    if (mode_ != "OCS2" && mode_ != "WBC") {
      disengage();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Not publishing EE target: arm mode is '%s'", mode_.c_str());
      return;
    }
    if (!have_touch_ || std::chrono::duration<double>(wall - last_touch_).count() > touch_timeout_) {
      disengage();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Not publishing EE target: waiting for fresh iPhone touch input");
      return;
    }
    if (touch_count_ == 0) return;
    // Three or more fingers disengage until all fingers are lifted. No home or
    // gripper commands bypass the MPC through the legacy arm controller.
    if (touch_count_ >= 3) disengage();
    if (release_required_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Not publishing EE target: release touch before re-clutching");
      return;
    }
    if (!have_ee_ || std::chrono::duration<double>(wall - last_ee_).count() > ee_timeout_) {
      disengage();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for fresh MPC EE feedback; release to re-clutch");
      return;
    }

    tf2::Vector3 phone_position;
    tf2::Quaternion phone_orientation;
    try {
      const auto transform = tf_buffer_->lookupTransform(iphone_source_frame_, iphone_target_frame_, tf2::TimePointZero);
      const rclcpp::Time stamp(transform.header.stamp, get_clock()->get_clock_type());
      const double age = (now() - stamp).seconds();
      // A cached TF must not masquerade as a new phone sample.
      if (age < 0.0 || age > pose_timeout_) {
        disengage();
        return;
      }
      const auto& p = transform.transform.translation;
      const auto& q = transform.transform.rotation;
      phone_position = tf2::Vector3(p.x, p.y, p.z);
      phone_orientation = tf2::Quaternion(q.x, q.y, q.z, q.w);
      if (!valid_pose(phone_position, phone_orientation)) {
        disengage();
        return;
      }
      phone_orientation.normalize();
    } catch (const tf2::TransformException& error) {
      disengage();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "iPhone pose unavailable: %s", error.what());
      return;
    }
    if (!pressing_) {
      start_phone_position_ = phone_position;
      start_phone_orientation_ = phone_orientation;
      start_ee_position_ = ee_position_;
      start_ee_orientation_ = ee_orientation_;
      pressing_ = true;
      RCLCPP_INFO(get_logger(), "Touch engaged at measured EE [%.3f, %.3f, %.3f]",
                  ee_position_.x(), ee_position_.y(), ee_position_.z());
    }
    const auto position = start_ee_position_ +
        tf2::quatRotate(phone_to_world_, phone_position - start_phone_position_) * position_scale_;
    auto delta = phone_orientation * start_phone_orientation_.inverse();
    delta = phone_to_world_ * delta * phone_to_world_.inverse();
    delta.normalize();
    // Use the short rotation, including when the phone quaternion changes sign.
    if (delta.w() < 0.0) delta = tf2::Quaternion(-delta.x(), -delta.y(), -delta.z(), -delta.w());
    delta = tf2::Quaternion(0, 0, 0, 1).slerp(delta, rotation_scale_);
    const auto orientation = (delta * start_ee_orientation_).normalized();
    if (!valid_pose(position, orientation)) {
      disengage();
      return;
    }
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

  std::string iphone_source_frame_, iphone_target_frame_, goal_frame_, mode_;
  double touch_threshold_, touch_timeout_, pose_timeout_, ee_timeout_, position_scale_, rotation_scale_;
  double last_mpc_time_ = 0.0;
  int mpc_input_dim_ = 12;
  bool have_touch_ = false, have_ee_ = false, pressing_ = false, release_required_ = false;
  size_t touch_count_ = 0;
  Clock::time_point last_touch_, last_ee_;
  tf2::Vector3 ee_position_, start_phone_position_, start_ee_position_;
  tf2::Quaternion ee_orientation_, start_phone_orientation_, start_ee_orientation_, phone_to_world_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr mpc_target_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr touch_sub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr ee_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
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
