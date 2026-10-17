#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include "dwb_core/trajectory_critic.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "mobile_openarm_shared_control/msg/human_intent.hpp"
#include "mobile_openarm_shared_control/preference.hpp"

namespace mobile_openarm_shared_control
{
class HumanInfluenceCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override
  {
    auto node = node_.lock();
    if (!node) {throw std::runtime_error("HumanInfluence: controller node expired");}
    const auto prefix = dwb_plugin_name_ + "." + name_ + ".";
    auto parameter = [&](const std::string & key, double value) {
        if (!node->has_parameter(prefix + key)) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.read_only = true;
          node->declare_parameter(prefix + key, value, descriptor);
        }
        return node->get_parameter(prefix + key).as_double();
      };
    sv_ = parameter("linear_weight", 200.0);
    sw_ = parameter("angular_weight", 400.0);
    const auto timeout = parameter("input_timeout", 0.5);
    const auto delay = parameter("release_delay", 2.0);
    if (!std::isfinite(sv_) || !std::isfinite(sw_) || sv_ < 0 || sw_ < 0 ||
      !std::isfinite(timeout) || timeout <= 0 || !std::isfinite(delay) || delay < 0)
    {
      throw std::runtime_error("HumanInfluence: invalid weights or time limits");
    }
    preference_ = std::make_unique<Preference>(timeout, delay);
    subscription_ = node->create_subscription<msg::HumanIntent>(
      "/shared_control/intent", rclcpp::QoS(1),
      [this](msg::HumanIntent::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> guard(mutex_);
        preference_->receive(msg->epoch, msg->permitted, msg->active,
          msg->linear, msg->angular, steady_now());
      });
    state_pub_ = node->create_publisher<std_msgs::msg::String>("/shared_control/critic_state", 1);
    state_pub_->on_activate();
  }

  bool prepare(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override
  {
    std::lock_guard<std::mutex> guard(mutex_);
    // Freeze one preference for the entire candidate evaluation cycle.
    snapshot_ = preference_->snapshot(steady_now());
    std_msgs::msg::String state;
    const char * labels[] = {"AUTONOMOUS", "ASSIST", "TRANSITION"};
    state.data = labels[snapshot_.phase];
    state_pub_->publish(state);
    return true;
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    return snapshot_.cost(trajectory.velocity.x, trajectory.velocity.theta, sv_, sw_);
  }

  void debrief(const nav_2d_msgs::msg::Twist2D & command) override
  {
    std::lock_guard<std::mutex> guard(mutex_);
    preference_->selected(command.x, steady_now());
  }

  // DWB calls reset() on EVERY global replan. The external goal epoch, not this
  // hook, resets human intent so a normal 1 Hz replan cannot erase the transition.
  void reset() override {}

private:
  static double steady_now()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  double sv_ = 0.0, sw_ = 0.0;
  std::mutex mutex_;
  std::unique_ptr<Preference> preference_;
  PreferenceSnapshot snapshot_;
  rclcpp::Subscription<msg::HumanIntent>::SharedPtr subscription_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
};
}  // namespace mobile_openarm_shared_control

PLUGINLIB_EXPORT_CLASS(mobile_openarm_shared_control::HumanInfluenceCritic, dwb_core::TrajectoryCritic)
