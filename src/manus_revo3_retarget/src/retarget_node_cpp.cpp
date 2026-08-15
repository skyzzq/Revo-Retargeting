#include <array>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "manus_ros2_msgs/msg/manus_glove.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "revo3_mit_controller_msgs/msg/revo3_mit_command.hpp"
#include "std_msgs/msg/string.hpp"

#include <dlfcn.h>

#include "manus_revo3_retarget/four_finger_retarget.hpp"
#include "manus_revo3_retarget/spread_retarget.hpp"
#include "manus_revo3_retarget/thumb_retarget.hpp"

namespace manus_revo3_retarget
{

using ManusGlove = manus_ros2_msgs::msg::ManusGlove;
using Revo3MITCommand = revo3_mit_controller_msgs::msg::Revo3MITCommand;

class ThumbPlugin
{
public:
  explicit ThumbPlugin(const std::string & library_path)
  {
    library_ = dlopen(library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (library_ == nullptr) {
      throw std::runtime_error("dlopen failed for " + library_path + ": " + dlerror_string());
    }
    create_ = load_symbol<CreateFn>("manus_revo3_thumb_create");
    destroy_ = load_symbol<DestroyFn>("manus_revo3_thumb_destroy");
    initialize_ = load_symbol<InitializeFn>("manus_revo3_thumb_initialize");
    set_config_ = load_symbol<SetConfigFn>("manus_revo3_thumb_set_config");
    apply_ = load_symbol<ApplyFn>("manus_revo3_thumb_apply");
    last_iteration_count_ = load_symbol<LastIterationCountFn>("manus_revo3_thumb_last_iteration_count");
    handle_ = create_();
    if (handle_ == nullptr) {
      throw std::runtime_error("thumb plugin create returned null");
    }
  }

  ~ThumbPlugin()
  {
    if (handle_ != nullptr && destroy_ != nullptr) {
      destroy_(handle_);
    }
    if (library_ != nullptr) {
      dlclose(library_);
    }
  }

  ThumbPlugin(const ThumbPlugin &) = delete;
  ThumbPlugin & operator=(const ThumbPlugin &) = delete;

  bool initialize(const std::string & model_base, const std::string & side, std::string * error)
  {
    return initialize_(handle_, model_base.c_str(), side.c_str(), error);
  }

  void set_config(const ThumbConfig & config)
  {
    set_config_(handle_, &config);
  }

  void apply(const Ergonomics & ergonomics, const ManusKeypoints & keypoints, JointArray & q)
  {
    apply_(handle_, &ergonomics, &keypoints, &q);
  }

  int last_iteration_count() const
  {
    return last_iteration_count_(handle_);
  }

private:
  using CreateFn = void * (*)();
  using DestroyFn = void (*)(void *);
  using InitializeFn = bool (*)(void *, const char *, const char *, std::string *);
  using SetConfigFn = void (*)(void *, const ThumbConfig *);
  using ApplyFn = void (*)(void *, const Ergonomics *, const ManusKeypoints *, JointArray *);
  using LastIterationCountFn = int (*)(void *);

  static std::string dlerror_string()
  {
    const char * error = dlerror();
    return error != nullptr ? std::string(error) : std::string("unknown dlopen/dlsym error");
  }

  template<typename T>
  T load_symbol(const char * name)
  {
    dlerror();
    void * symbol = dlsym(library_, name);
    const char * error = dlerror();
    if (error != nullptr || symbol == nullptr) {
      throw std::runtime_error(std::string("dlsym failed for ") + name + ": " + dlerror_string());
    }
    return reinterpret_cast<T>(symbol);
  }

  void * library_{nullptr};
  void * handle_{nullptr};
  CreateFn create_{nullptr};
  DestroyFn destroy_{nullptr};
  InitializeFn initialize_{nullptr};
  SetConfigFn set_config_{nullptr};
  ApplyFn apply_{nullptr};
  LastIterationCountFn last_iteration_count_{nullptr};
};

struct SideState
{
  std::string side;
  std::vector<std::string> names;
  rclcpp::Publisher<Revo3MITCommand>::SharedPtr command_pub;
  rclcpp::Publisher<Revo3MITCommand>::SharedPtr target_pub;
  std::mutex mutex;
  std::optional<Revo3MITCommand> latest_target;
  std::vector<double> start_position;
  std::vector<double> target_position;
  std::vector<double> target_velocity;
  rclcpp::Time start_time;
  rclcpp::Time end_time;
  rclcpp::Time last_target_time;
  bool has_segment{false};
  bool has_last_target{false};
  bool action_override{false};
  bool resume_smooth{false};
  std::string active_action;
  FourFingerRetarget four_finger;
  SpreadRetarget spread;
  std::unique_ptr<ThumbPlugin> thumb;
  std::vector<double> output_scale;
  std::vector<double> output_offset;
  std::vector<double> kp;
  std::vector<double> kd;
  std::vector<double> filtered_position;
};

class RetargetNodeCpp : public rclcpp::Node
{
public:
  explicit RetargetNodeCpp(const rclcpp::NodeOptions & options)
  : Node("manus_revo3_retarget", options)
  {
    hand_mode_ = string_param("hand_mode", "both");
    use_revo3_namespace_ = bool_param("use_revo3_namespace", true);
    command_topic_suffix_ = string_param("command_topic_suffix", "joint_forward_mit_controller/commands");
    target_topic_suffix_ = string_param("retarget_target_topic_suffix", "joint_forward_mit_controller/retarget_targets");
    mit_command_publish_hz_ = double_param("mit_command_publish_hz", 200.0);
    mit_velocity_feedforward_enabled_ = bool_param("mit_velocity_feedforward_enabled", false);
    mit_velocity_feedforward_scale_ = double_param("mit_velocity_feedforward_scale", 0.0);
    mit_interpolation_horizon_s_ = double_param("mit_interpolation_horizon_s", 0.008);
    command_ema_alpha_ = double_param("command_ema_alpha", 0.55);
    command_max_delta_rad_ = deg_to_rad(double_param("command_max_delta_deg", 12.0));
    mit_default_kp_ = double_param("mit_default_kp", 0.4);
    mit_default_kd_ = double_param("mit_default_kd", 0.05);
    enable_keyboard_actions_ = bool_param("enable_keyboard_actions", false);
    action_command_topic_ = string_param("action_command_topic", "/manus_revo3_retarget/action_command");
    action_interpolation_duration_s_ = double_param("action_interpolation_duration_s", 0.6);

    if (hand_mode_ != "left" && hand_mode_ != "right" && hand_mode_ != "both") {
      throw std::runtime_error("hand_mode must be left, right, or both");
    }

    if (hand_mode_ == "left" || hand_mode_ == "both") {
      left_ = create_side("left");
    }
    if (hand_mode_ == "right" || hand_mode_ == "both") {
      right_ = create_side("right");
    }

    glove_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions glove_opt;
    glove_opt.callback_group = glove_cb_group_;

    sub_0_ = create_subscription<ManusGlove>(
      "/manus_glove_0", 10, [this](ManusGlove::SharedPtr msg) { on_glove(*msg); }, glove_opt);
    sub_1_ = create_subscription<ManusGlove>(
      "/manus_glove_1", 10, [this](ManusGlove::SharedPtr msg) { on_glove(*msg); }, glove_opt);
    if (enable_keyboard_actions_) {
      action_sub_ = create_subscription<std_msgs::msg::String>(
        action_command_topic_, 10,
        [this](std_msgs::msg::String::SharedPtr msg) { on_action_command(msg->data); },
        glove_opt);
    }

    const double period_s = 1.0 / std::max(1.0, mit_command_publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s)),
      [this]() { publish_latest(); },
      timer_cb_group_);

    param_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &) {
        cache_dirty_.store(true);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });
    cache_dirty_.store(true);
    refresh_cached_params();

    RCLCPP_INFO(
      get_logger(),
      "C++ retarget node ready hand_mode=%s command_hz=%.1f horizon=%.3fs ema=%.2f keyboard_actions=%s",
      hand_mode_.c_str(), mit_command_publish_hz_, mit_interpolation_horizon_s_, command_ema_alpha_,
      enable_keyboard_actions_ ? "on" : "off");
  }

private:
  std::shared_ptr<SideState> create_side(const std::string & side)
  {
    auto state = std::make_shared<SideState>();
    state->side = side;
    state->names = joint_names(side);
    state->command_pub = create_publisher<Revo3MITCommand>(command_topic(side), 10);
    state->target_pub = create_publisher<Revo3MITCommand>(target_topic(side), 10);

    state->four_finger.set_config(load_four_finger_config(side));
    state->spread.set_config(load_spread_config(side));
    state->thumb = std::make_unique<ThumbPlugin>(thumb_plugin_path());
    state->thumb->set_config(load_thumb_config(side));
    const std::string model_base = model_base_path();
    RCLCPP_INFO(get_logger(), "C++ %s thumb Pinocchio description base: %s", side.c_str(), model_base.c_str());
    std::string thumb_error;
    if (!state->thumb->initialize(model_base, side, &thumb_error)) {
      throw std::runtime_error("failed to initialize thumb Pinocchio IK for " + side + ": " + thumb_error);
    }
    RCLCPP_INFO(get_logger(), "C++ %s thumb Pinocchio IK initialized", side.c_str());

    RCLCPP_INFO(get_logger(), "C++ %s retarget -> %s", side.c_str(), command_topic(side).c_str());
    return state;
  }

  FourFingerConfig load_four_finger_config(const std::string & side)
  {
    const std::string p = "legacy_" + side + "_physical_";
    FourFingerConfig cfg;
    cfg.index_angle_scale = double_param(p + "index_angle_scale", 1.0);
    cfg.four_finger_mcp_scale = double_param(p + "four_finger_mcp_scale", 1.0);
    cfg.middle_ring_dip_scale = double_param(p + "middle_ring_dip_scale", 1.0);
    cfg.pinky_angle_scale = double_param(p + "pinky_angle_scale", 1.0);
    cfg.pinky_dip_pip_scale = double_param(p + "pinky_dip_pip_scale", 1.0);
    cfg.pinky_mcp_scale = double_param(p + "pinky_mcp_scale", 1.0);
    cfg.all_finger_angle_scale = double_param(p + "all_finger_angle_scale", 1.0);
    return cfg;
  }

  SpreadConfig load_spread_config(const std::string & side)
  {
    const std::string p = "legacy_" + side + "_physical_";
    SpreadConfig cfg;
    cfg.index_offset_deg = double_param(p + "index_spread_offset_deg", 0.0);
    cfg.middle_offset_deg = double_param(p + "middle_spread_offset_deg", 0.0);
    cfg.ring_offset_deg = double_param(p + "ring_spread_offset_deg", 0.0);
    cfg.pinky_offset_deg = double_param(p + "pinky_spread_offset_deg", 0.0);
    cfg.index_scale = double_param(p + "index_spread_scale", 1.0);
    cfg.middle_scale = double_param(p + "middle_spread_scale", 1.0);
    cfg.ring_scale = double_param(p + "ring_spread_scale", 1.0);
    cfg.pinky_scale = double_param(p + "pinky_spread_scale", 1.0);
    cfg.middle_dynamic = bool_param(p + "middle_spread_dynamic", false);
    cfg.ring_forward_scale = double_param(p + "ring_spread_forward_scale", 1.0);
    cfg.ring_backward_scale = double_param(p + "ring_spread_backward_scale", 1.0);
    cfg.finger_spread_sign = -1.0;
    return cfg;
  }

  ThumbConfig load_thumb_config(const std::string & side)
  {
    const std::string p = "legacy_" + side + "_physical_";
    ThumbConfig cfg;
    cfg.joint_offset_deg = double_param(p + "thumb_joint_offset_deg", 0.0);
    cfg.cmp_offset_deg = double_param(side + "_thumb_cmp_offset_deg_physical", 0.0);
    cfg.cmp_scale = double_param(side + "_thumb_cmp_scale_physical", 1.0);
    cfg.cmr_offset_deg = double_param(p + "thumb_cmr_offset_deg", 0.0);
    cfg.mcp_offset_deg = double_param(p + "thumb_mcp_offset_deg", 0.0);
    cfg.mcp_scale = double_param(p + "thumb_mcp_scale", 1.0);
    cfg.pip_scale = double_param(p + "thumb_pip_scale", 1.0);
    cfg.dip_scale = double_param(p + "thumb_dip_scale", 1.0);
    cfg.spread_sign = side == "left" ? 1.0 : -1.0;
    cfg.manus_out_y_sign = -1.0;
    cfg.reach_scale = double_param(p + "thumb_reach_scale", 1.0);
    cfg.ik_position_scale = double_param(p + "thumb_ik_position_scale", 1.0);
    cfg.pip_ik_scale = double_param(p + "thumb_pip_ik_scale", 1.0);
    cfg.dip_ik_scale = double_param(p + "thumb_dip_ik_scale", 1.0);
    cfg.ema_prev = double_param(p + "thumb_ema_prev", side == "left" ? 0.9 : 0.4);
    cfg.ema_cur = double_param(p + "thumb_ema_cur", side == "left" ? 0.1 : 0.6);
    cfg.ik_posture_weight = double_param("thumb_ik_posture_weight", 0.1);
    cfg.ik_smooth_weight = double_param("thumb_ik_smooth_weight", 0.1);
    cfg.ik_max_iterations = int_param("thumb_ik_max_iterations", 15);
    cfg.ik_max_step_rad = deg_to_rad(double_param("thumb_ik_max_step_deg", 3.0));
    cfg.ik_max_frame_delta_rad = deg_to_rad(double_param("thumb_ik_max_frame_delta_deg", 6.0));
    cfg.ik_damping = double_param("thumb_ik_damping", 0.02);
    cfg.ik_step_size = double_param("thumb_ik_step_size", 0.30);
    cfg.ik_tolerance = double_param("thumb_ik_tolerance", 5e-4);
    return cfg;
  }

  void on_glove(const ManusGlove & msg)
  {
    std::string side = msg.side;
    for (auto & ch : side) {
      ch = static_cast<char>(std::tolower(ch));
    }

    std::shared_ptr<SideState> state;
    if ((side == "left" || side == "l") && left_) {
      state = left_;
    } else if ((side == "right" || side == "r") && right_) {
      state = right_;
    } else {
      return;
    }

    refresh_cached_params();
    if (enable_keyboard_actions_) {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->action_override) {
        return;
      }
    }

    Ergonomics ergonomics;
    ergonomics.reserve(msg.ergonomics.size());
    for (const auto & item : msg.ergonomics) {
      ergonomics[item.type] = static_cast<double>(item.value);
    }

    ManusKeypoints keypoints;
    for (const auto & raw_node : msg.raw_nodes) {
      const int node_id = raw_node.node_id;
      if (node_id < 0 || node_id >= static_cast<int>(keypoints.size())) {
        continue;
      }
      const auto & pos = raw_node.pose.position;
      keypoints[static_cast<std::size_t>(node_id)] = Eigen::Vector3d(
        static_cast<double>(pos.x), static_cast<double>(pos.y), static_cast<double>(pos.z));
    }

    JointArray q{};
    q.fill(0.0);
    state->four_finger.apply(ergonomics, q);
    state->spread.apply(ergonomics, q);
    state->thumb->apply(ergonomics, keypoints, q);

    Revo3MITCommand out;
    out.header.stamp = now();
    out.joint_names = state->names;
    out.position.assign(q.begin(), q.end());
    apply_output_calibration(*state, out.position);
    filter_command(*state, out.position);
    out.velocity.assign(state->names.size(), 0.0);
    out.effort.assign(state->names.size(), 0.0);
    out.kp = state->kp;
    out.kd = state->kd;

    update_interpolation_target(state, out);
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->latest_target) {
        out = *state->latest_target;
      }
    }
    state->target_pub->publish(out);
  }

  void publish_latest()
  {
    publish_latest(left_);
    publish_latest(right_);
  }

  void publish_latest(const std::shared_ptr<SideState> & state)
  {
    if (!state) {
      return;
    }
    std::optional<Revo3MITCommand> msg;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      msg = sample_command_locked(*state, now());
    }
    if (!msg) {
      return;
    }
    state->command_pub->publish(*msg);
  }

  void update_interpolation_target(
    const std::shared_ptr<SideState> & state,
    Revo3MITCommand & target,
    std::optional<double> duration_override = std::nullopt)
  {
    const rclcpp::Time target_time(target.header.stamp);
    std::lock_guard<std::mutex> lock(state->mutex);

    const std::vector<double> current_position = sample_position_locked(*state, target_time);
    const bool compatible =
      state->has_segment &&
      state->latest_target &&
      state->latest_target->joint_names == target.joint_names &&
      state->target_position.size() == target.position.size();

    double duration_s = mit_interpolation_horizon_s_ > 0.0 ?
      mit_interpolation_horizon_s_ : default_interpolation_duration_s();
    std::vector<double> target_velocity(target.position.size(), 0.0);
    const bool use_action_duration =
      duration_override.has_value() || state->resume_smooth;
    if (use_action_duration) {
      duration_s = duration_override.value_or(action_interpolation_duration_s_);
      state->resume_smooth = false;
    }
    if (compatible && velocity_feedforward_enabled()) {
      const double scale = std::max(0.0, mit_velocity_feedforward_scale_);
      const double denom = std::max(duration_s, min_duration_s());
      for (std::size_t i = 0; i < target.position.size(); ++i) {
        target_velocity[i] = scale * (target.position[i] - current_position[i]) / denom;
      }
    }

    state->start_position = compatible ? current_position : target.position;
    state->target_position = target.position;
    state->target_velocity = target_velocity;
    state->start_time = target_time;
    state->end_time = target_time + rclcpp::Duration::from_seconds(std::max(duration_s, min_duration_s()));
    state->last_target_time = target_time;
    state->has_last_target = true;
    state->has_segment = true;

    target.velocity = velocity_feedforward_enabled() ?
      target_velocity : std::vector<double>(target.position.size(), 0.0);
    state->latest_target = target;
  }

  std::optional<Revo3MITCommand> sample_command_locked(SideState & state, const rclcpp::Time & sample_time)
  {
    if (!state.has_segment || !state.latest_target) {
      return std::nullopt;
    }
    Revo3MITCommand out = *state.latest_target;
    out.header.stamp = sample_time;
    out.position = sample_position_locked(state, sample_time);
    out.velocity = sample_velocity_locked(state);
    return out;
  }

  std::vector<double> sample_position_locked(const SideState & state, const rclcpp::Time & sample_time) const
  {
    if (!state.has_segment || state.start_position.size() != state.target_position.size()) {
      return state.target_position;
    }
    const double duration_s = (state.end_time - state.start_time).seconds();
    if (duration_s <= min_duration_s() || sample_time >= state.end_time) {
      return state.target_position;
    }
    const double alpha = std::clamp((sample_time - state.start_time).seconds() / duration_s, 0.0, 1.0);
    std::vector<double> position(state.target_position.size(), 0.0);
    for (std::size_t i = 0; i < position.size(); ++i) {
      position[i] = state.start_position[i] + alpha * (state.target_position[i] - state.start_position[i]);
    }
    return position;
  }

  std::vector<double> sample_velocity_locked(const SideState & state) const
  {
    if (!state.has_segment) {
      return std::vector<double>(state.target_position.size(), 0.0);
    }
    if (!velocity_feedforward_enabled()) {
      return std::vector<double>(state.target_position.size(), 0.0);
    }
    if (state.target_velocity.size() == state.target_position.size()) {
      return state.target_velocity;
    }
    return std::vector<double>(state.target_position.size(), 0.0);
  }

  static double min_duration_s()
  {
    return 1e-6;
  }

  static double default_interpolation_duration_s()
  {
    return 1.0 / 60.0;
  }

  void on_action_command(std::string data)
  {
    data = trim_lower(std::move(data));
    std::string side_filter;
    std::string action = data;
    const auto colon = data.find(':');
    if (colon != std::string::npos) {
      side_filter = data.substr(0, colon);
      action = trim_lower(data.substr(colon + 1));
    }
    if (action.empty()) {
      return;
    }
    if (action == "glove" || action == "none" || action == "clear" || action == "resume") {
      resume_glove(left_, side_filter);
      resume_glove(right_, side_filter);
      return;
    }
    if (!is_known_action(action)) {
      RCLCPP_WARN(get_logger(), "Unknown keyboard action '%s'", action.c_str());
      return;
    }
    apply_named_action(left_, side_filter, action);
    apply_named_action(right_, side_filter, action);
  }

  void resume_glove(const std::shared_ptr<SideState> & state, const std::string & side_filter)
  {
    if (!side_matches(state, side_filter)) {
      return;
    }
    bool was_override = false;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      was_override = state->action_override;
      state->action_override = false;
      state->resume_smooth = true;
      state->active_action.clear();
    }
    if (was_override) {
      RCLCPP_INFO(get_logger(), "Resume glove teleop on %s", state->side.c_str());
    }
  }

  void apply_named_action(
    const std::shared_ptr<SideState> & state,
    const std::string & side_filter,
    const std::string & action)
  {
    if (!side_matches(state, side_filter)) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->action_override = true;
      state->active_action = action;
    }

    refresh_cached_params();
    Revo3MITCommand out;
    out.header.stamp = now();
    out.joint_names = state->names;
    out.position = pose_from_action(state->side, action);
    out.velocity.assign(state->names.size(), 0.0);
    out.effort.assign(state->names.size(), 0.0);
    out.kp = state->kp;
    out.kd = state->kd;

    update_interpolation_target(state, out, action_interpolation_duration_s_);
    state->target_pub->publish(out);
    RCLCPP_INFO(get_logger(), "Apply keyboard action '%s' on %s", action.c_str(), state->side.c_str());
  }

  static bool side_matches(const std::shared_ptr<SideState> & state, const std::string & side_filter)
  {
    if (!state) {
      return false;
    }
    return side_filter.empty() || side_filter == "both" || side_filter == state->side;
  }

  bool is_known_action(const std::string & action)
  {
    static const std::array<const char *, 5> kBuiltin = {"open", "fist", "pinch", "point", "ok"};
    for (const char * name : kBuiltin) {
      if (action == name) {
        return true;
      }
    }
    const auto listed = list_parameters({"action_" + action + "_"}, 32);
    return !listed.names.empty();
  }

  std::vector<double> pose_from_action(const std::string & side, const std::string & action)
  {
    const std::string prefix = side + "_";
    std::vector<double> positions;
    positions.reserve(kJointCount);
    for (const auto & name : joint_names(side)) {
      std::string suffix = name;
      if (suffix.rfind(prefix, 0) == 0) {
        suffix = suffix.substr(prefix.size());
      }
      positions.push_back(deg_to_rad(action_joint_deg(action, side, suffix)));
    }
    return positions;
  }

  double action_joint_deg(const std::string & action, const std::string & side, const std::string & suffix)
  {
    const std::string specific = "action_" + action + "_" + side + "_" + suffix + "_deg";
    if (has_parameter(specific)) {
      double value = 0.0;
      get_parameter(specific, value);
      if (std::isfinite(value)) {
        return value;
      }
    }
    return double_param("action_" + action + "_" + suffix + "_deg", 0.0);
  }

  static std::string trim_lower(std::string value)
  {
    const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    for (auto & ch : value) {
      ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
  }

  bool velocity_feedforward_enabled() const
  {
    return mit_velocity_feedforward_enabled_ && mit_velocity_feedforward_scale_ > 0.0;
  }

  void refresh_cached_params()
  {
    if (!cache_dirty_.exchange(false)) {
      return;
    }
    mit_velocity_feedforward_enabled_ = bool_param(
      "mit_velocity_feedforward_enabled", mit_velocity_feedforward_enabled_);
    mit_velocity_feedforward_scale_ = double_param(
      "mit_velocity_feedforward_scale", mit_velocity_feedforward_scale_);
    mit_interpolation_horizon_s_ = double_param(
      "mit_interpolation_horizon_s", mit_interpolation_horizon_s_);
    command_ema_alpha_ = std::clamp(double_param("command_ema_alpha", command_ema_alpha_), 0.0, 1.0);
    command_max_delta_rad_ = deg_to_rad(double_param("command_max_delta_deg", 12.0));
    mit_default_kp_ = double_param("mit_default_kp", mit_default_kp_);
    mit_default_kd_ = double_param("mit_default_kd", mit_default_kd_);
    action_interpolation_duration_s_ = std::max(
      double_param("action_interpolation_duration_s", action_interpolation_duration_s_), min_duration_s());
    refresh_side_cache(left_);
    refresh_side_cache(right_);
  }

  void refresh_side_cache(const std::shared_ptr<SideState> & state)
  {
    if (!state) {
      return;
    }
    state->four_finger.set_config(load_four_finger_config(state->side));
    state->spread.set_config(load_spread_config(state->side));
    state->thumb->set_config(load_thumb_config(state->side));

    const std::string prefix = state->side + "_";
    state->output_scale.assign(state->names.size(), 1.0);
    state->output_offset.assign(state->names.size(), 0.0);
    for (std::size_t i = 0; i < state->names.size(); ++i) {
      std::string suffix = state->names[i];
      if (suffix.rfind(prefix, 0) == 0) {
        suffix = suffix.substr(prefix.size());
      }
      state->output_scale[i] = double_param("physical_" + state->side + "_" + suffix + "_scale", 1.0);
      state->output_offset[i] = deg_to_rad(
        double_param("physical_" + state->side + "_" + suffix + "_offset_deg", 0.0));
    }
    state->kp = mit_gains_for_joints(state->names, "kp", mit_default_kp_);
    state->kd = mit_gains_for_joints(state->names, "kd", mit_default_kd_);
  }

  static void apply_output_calibration(const SideState & state, std::vector<double> & positions)
  {
    for (std::size_t i = 0; i < positions.size() && i < state.output_scale.size(); ++i) {
      positions[i] = positions[i] * state.output_scale[i] + state.output_offset[i];
    }
  }

  void filter_command(SideState & state, std::vector<double> & position) const
  {
    if (state.filtered_position.size() != position.size()) {
      state.filtered_position = position;
      return;
    }
    const double alpha = std::clamp(command_ema_alpha_, 0.0, 1.0);
    const double max_delta = std::max(0.0, command_max_delta_rad_);
    for (std::size_t i = 0; i < position.size(); ++i) {
      const double previous = state.filtered_position[i];
      const double limited = previous + std::clamp(position[i] - previous, -max_delta, max_delta);
      const double filtered = alpha * limited + (1.0 - alpha) * previous;
      state.filtered_position[i] = filtered;
      position[i] = filtered;
    }
  }

  std::vector<double> mit_gains_for_joints(
    const std::vector<std::string> & names,
    const std::string & field,
    double default_value)
  {
    std::vector<double> values;
    values.reserve(names.size());
    for (const auto & name : names) {
      const double value = double_param("mit_" + name + "_" + field, -1.0);
      values.push_back(std::isfinite(value) && value >= 0.0 ? value : default_value);
    }
    return values;
  }

  std::string command_topic(const std::string & side) const
  {
    return topic_for(side, command_topic_suffix_, "joint_forward_mit_controller/commands");
  }

  std::string target_topic(const std::string & side) const
  {
    return topic_for(side, target_topic_suffix_, "joint_forward_mit_controller/retarget_targets");
  }

  std::string topic_for(const std::string & side, std::string suffix, const std::string & fallback) const
  {
    if (suffix.empty()) {
      suffix = fallback;
    }
    while (!suffix.empty() && suffix.front() == '/') {
      suffix.erase(suffix.begin());
    }
    if (use_revo3_namespace_) {
      return "/revo3_" + side + "/" + suffix;
    }
    return "/" + suffix;
  }

  double double_param(const std::string & name, double fallback)
  {
    if (!has_parameter(name)) {
      declare_parameter<double>(name, fallback);
    }
    double value = fallback;
    get_parameter(name, value);
    return finite_or(value, fallback);
  }

  bool bool_param(const std::string & name, bool fallback)
  {
    if (!has_parameter(name)) {
      declare_parameter<bool>(name, fallback);
    }
    bool value = fallback;
    get_parameter(name, value);
    return value;
  }

  int int_param(const std::string & name, int fallback)
  {
    if (!has_parameter(name)) {
      declare_parameter<int>(name, fallback);
    }
    int value = fallback;
    get_parameter(name, value);
    return value;
  }

  std::string string_param(const std::string & name, const std::string & fallback)
  {
    if (!has_parameter(name)) {
      declare_parameter<std::string>(name, fallback);
    }
    std::string value = fallback;
    get_parameter(name, value);
    return value.empty() ? fallback : value;
  }

  std::string model_base_path()
  {
    const char * env_path = std::getenv("REVO3_MODEL_PATH");
    if (env_path != nullptr && std::string(env_path).size() > 0) {
      return std::string(env_path);
    }
    return ament_index_cpp::get_package_share_directory("revo3_description");
  }

  std::string thumb_plugin_path()
  {
    return (std::filesystem::path(ament_index_cpp::get_package_prefix("manus_revo3_retarget")) / "lib" /
      "libmanus_revo3_retarget_thumb_pinocchio.so").string();
  }

  std::string hand_mode_;
  bool use_revo3_namespace_{true};
  std::string command_topic_suffix_;
  std::string target_topic_suffix_;
  double mit_command_publish_hz_{200.0};
  bool mit_velocity_feedforward_enabled_{false};
  double mit_velocity_feedforward_scale_{0.0};
  double mit_interpolation_horizon_s_{0.008};
  double command_ema_alpha_{0.55};
  double command_max_delta_rad_{deg_to_rad(12.0)};
  double mit_default_kp_{0.4};
  double mit_default_kd_{0.05};
  bool enable_keyboard_actions_{false};
  std::string action_command_topic_;
  double action_interpolation_duration_s_{0.6};
  std::atomic<bool> cache_dirty_{true};
  std::shared_ptr<SideState> left_;
  std::shared_ptr<SideState> right_;
  rclcpp::CallbackGroup::SharedPtr glove_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::Subscription<ManusGlove>::SharedPtr sub_0_;
  rclcpp::Subscription<ManusGlove>::SharedPtr sub_1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace manus_revo3_retarget

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  options.enable_rosout(false);
  options.start_parameter_services(true);
  options.start_parameter_event_publisher(false);
  auto node = std::make_shared<manus_revo3_retarget::RetargetNodeCpp>(options);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
