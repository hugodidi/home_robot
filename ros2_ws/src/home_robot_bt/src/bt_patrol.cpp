#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

struct Waypoint
{
  std::string name;
  double x;
  double y;
  double theta;
};

struct PatrolContext
{
  rclcpp::Node::SharedPtr node;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  std::vector<Waypoint> queue;
  std::optional<Waypoint> current;
  bool skip_errors{false};
};

static geometry_msgs::msg::PoseStamped makePose(
  const rclcpp::Node::SharedPtr & node, const Waypoint & wp)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node->now();
  pose.pose.position.x = wp.x;
  pose.pose.position.y = wp.y;
  pose.pose.orientation.z = std::sin(wp.theta / 2.0);
  pose.pose.orientation.w = std::cos(wp.theta / 2.0);
  return pose;
}

static std::string waypointsPath()
{
  try {
    return ament_index_cpp::get_package_share_directory("home_robot") + "/config/waypoints.yaml";
  } catch (...) {
    const char * root = std::getenv("PROJECT_ROOT");
    const std::string project_root = root ? root : "/home/ubuntu/home_robot";
    return project_root + "/ros2_ws/src/home_robot/config/waypoints.yaml";
  }
}

static std::vector<Waypoint> loadWaypoints()
{
  YAML::Node doc = YAML::LoadFile(waypointsPath());
  std::vector<Waypoint> waypoints;
  for (const auto & item : doc["waypoints"]) {
    const auto name = item.first.as<std::string>();
    const auto wp = item.second;
    waypoints.push_back(Waypoint{
      name,
      wp["x"].as<double>(),
      wp["y"].as<double>(),
      wp["theta"].as<double>()});
  }
  return waypoints;
}

class SelectWaypoint : public BT::SyncActionNode
{
public:
  SelectWaypoint(const std::string & name, const BT::NodeConfig & config,
                 std::shared_ptr<PatrolContext> context)
  : BT::SyncActionNode(name, config), context_(std::move(context)) {}

  BT::NodeStatus tick() override
  {
    if (context_->queue.empty()) {
      RCLCPP_INFO(context_->node->get_logger(), "BT patrol complete: no waypoints left");
      return BT::NodeStatus::FAILURE;
    }
    context_->current = context_->queue.front();
    context_->queue.erase(context_->queue.begin());
    RCLCPP_INFO(
      context_->node->get_logger(), "BT selected waypoint: %s", context_->current->name.c_str());
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<PatrolContext> context_;
};

class NavigateToWaypoint : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToWaypoint(const std::string & name, const BT::NodeConfig & config,
                     std::shared_ptr<PatrolContext> context)
  : BT::StatefulActionNode(name, config), context_(std::move(context)) {}

  BT::NodeStatus onStart() override
  {
    if (!context_->current) {
      return BT::NodeStatus::FAILURE;
    }
    NavigateToPose::Goal goal;
    goal.pose = makePose(context_->node, *context_->current);
    RCLCPP_INFO(
      context_->node->get_logger(), "%s: sending goal to %s", name().c_str(),
      context_->current->name.c_str());
    goal_future_ = context_->nav_client->async_send_goal(goal);
    result_requested_ = false;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!goal_handle_) {
      if (goal_future_.valid() && goal_future_.wait_for(0ms) == std::future_status::ready) {
        goal_handle_ = goal_future_.get();
        if (!goal_handle_) {
          RCLCPP_WARN(context_->node->get_logger(), "%s: goal rejected", name().c_str());
          return BT::NodeStatus::FAILURE;
        }
      } else {
        return BT::NodeStatus::RUNNING;
      }
    }

    if (!result_requested_) {
      result_future_ = context_->nav_client->async_get_result(goal_handle_);
      result_requested_ = true;
      return BT::NodeStatus::RUNNING;
    }

    if (result_future_.valid() && result_future_.wait_for(0ms) == std::future_status::ready) {
      const auto result = result_future_.get();
      goal_handle_.reset();
      result_requested_ = false;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(context_->node->get_logger(), "%s: navigation succeeded", name().c_str());
        return BT::NodeStatus::SUCCESS;
      }
      RCLCPP_WARN(context_->node->get_logger(), "%s: navigation failed", name().c_str());
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (goal_handle_) {
      context_->nav_client->async_cancel_goal(goal_handle_);
    }
    goal_handle_.reset();
    result_requested_ = false;
  }

private:
  std::shared_ptr<PatrolContext> context_;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
  bool result_requested_{false};
};

class BackUpRecovery : public BT::StatefulActionNode
{
public:
  BackUpRecovery(const std::string & name, const BT::NodeConfig & config,
                 std::shared_ptr<PatrolContext> context)
  : BT::StatefulActionNode(name, config), context_(std::move(context)) {}

  BT::NodeStatus onStart() override
  {
    start_ = context_->node->now();
    RCLCPP_WARN(context_->node->get_logger(), "BT recovery: moving backwards");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    geometry_msgs::msg::Twist twist;
    if ((context_->node->now() - start_).seconds() < 1.2) {
      twist.linear.x = -0.25;
      context_->cmd_vel_pub->publish(twist);
      return BT::NodeStatus::RUNNING;
    }
    context_->cmd_vel_pub->publish(twist);
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
    context_->cmd_vel_pub->publish(geometry_msgs::msg::Twist());
  }

private:
  std::shared_ptr<PatrolContext> context_;
  rclcpp::Time start_;
};

class PublishArrival : public BT::SyncActionNode
{
public:
  PublishArrival(const std::string & name, const BT::NodeConfig & config,
                 std::shared_ptr<PatrolContext> context)
  : BT::SyncActionNode(name, config), context_(std::move(context)) {}

  BT::NodeStatus tick() override
  {
    if (!context_->current) {
      return BT::NodeStatus::FAILURE;
    }
    std_msgs::msg::String msg;
    msg.data = "llegada:" + context_->current->name;
    context_->event_pub->publish(msg);
    RCLCPP_INFO(context_->node->get_logger(), "BT event published: %s", msg.data.c_str());
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<PatrolContext> context_;
};

class HandleFailure : public BT::SyncActionNode
{
public:
  HandleFailure(const std::string & name, const BT::NodeConfig & config,
                std::shared_ptr<PatrolContext> context)
  : BT::SyncActionNode(name, config), context_(std::move(context)) {}

  BT::NodeStatus tick() override
  {
    const auto wp = context_->current ? context_->current->name : "unknown";
    if (context_->skip_errors) {
      RCLCPP_WARN(context_->node->get_logger(), "BT skipping failed waypoint: %s", wp.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(context_->node->get_logger(), "BT stopping on failed waypoint: %s", wp.c_str());
    return BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<PatrolContext> context_;
};

class PatrolLoop : public BT::DecoratorNode
{
public:
  PatrolLoop(const std::string & name, const BT::NodeConfig & config,
             std::shared_ptr<PatrolContext> context)
  : BT::DecoratorNode(name, config), context_(std::move(context)) {}

  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);
    const auto child_status = child_node_->executeTick();
    if (child_status == BT::NodeStatus::SUCCESS) {
      haltChild();
      return BT::NodeStatus::RUNNING;
    }
    if (child_status == BT::NodeStatus::FAILURE && context_->queue.empty()) {
      haltChild();
      return BT::NodeStatus::SUCCESS;
    }
    return child_status;
  }

private:
  std::shared_ptr<PatrolContext> context_;
};

static bool hasArg(int argc, char ** argv, const std::string & arg)
{
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == arg) {
      return true;
    }
  }
  return false;
}

static std::string valueArg(int argc, char ** argv, const std::string & prefix, const std::string & fallback)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg.rfind(prefix, 0) == 0) {
      return arg.substr(prefix.size());
    }
  }
  return fallback;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const bool random_order = hasArg(argc, argv, "--random");
  const bool skip_errors = hasArg(argc, argv, "--skip-errors");
  const auto default_xml = ament_index_cpp::get_package_share_directory("home_robot_bt") +
    "/config/bt_patrol.xml";
  auto bt_xml = valueArg(argc, argv, "--bt-xml=", default_xml);
  if (bt_xml.empty()) {
    bt_xml = default_xml;
  }
  const int groot_port = std::stoi(valueArg(argc, argv, "--groot-port=", "1667"));

  auto context = std::make_shared<PatrolContext>();
  context->node = rclcpp::Node::make_shared("bt_patrol");
  context->nav_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    context->node, "navigate_to_pose");
  context->event_pub = context->node->create_publisher<std_msgs::msg::String>("patrol_events", 10);
  context->cmd_vel_pub = context->node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  context->skip_errors = skip_errors;
  context->queue = loadWaypoints();

  if (random_order) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(context->queue.begin(), context->queue.end(), gen);
  }

  RCLCPP_INFO(context->node->get_logger(), "Custom BehaviorTree.CPP patrol starting");
  RCLCPP_INFO(context->node->get_logger(), "BT XML: %s", bt_xml.c_str());
  RCLCPP_INFO(context->node->get_logger(), "Groot2 live publisher port: %d", groot_port);

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<SelectWaypoint>(
    "SelectWaypoint", [context](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<SelectWaypoint>(name, config, context);
    });
  factory.registerBuilder<NavigateToWaypoint>(
    "NavigateToWaypoint", [context](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<NavigateToWaypoint>(name, config, context);
    });
  factory.registerBuilder<BackUpRecovery>(
    "BackUpRecovery", [context](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<BackUpRecovery>(name, config, context);
    });
  factory.registerBuilder<PublishArrival>(
    "PublishArrival", [context](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<PublishArrival>(name, config, context);
    });
  factory.registerBuilder<HandleFailure>(
    "HandleFailure", [context](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<HandleFailure>(name, config, context);
    });
  factory.registerBuilder<PatrolLoop>(
    "PatrolLoop", [context](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<PatrolLoop>(name, config, context);
    });

  auto tree = factory.createTreeFromFile(bt_xml);
  std::unique_ptr<BT::Groot2Publisher> groot_publisher;
  try {
    groot_publisher = std::make_unique<BT::Groot2Publisher>(
      tree, static_cast<unsigned>(groot_port));
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      context->node->get_logger(), "Could not start Groot2 publisher on port %d: %s",
      groot_port, ex.what());
  }

  if (!context->nav_client->wait_for_action_server(60s)) {
    RCLCPP_ERROR(context->node->get_logger(), "Timeout waiting for navigate_to_pose action server");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::Rate rate(5.0);
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  while (rclcpp::ok()) {
    rclcpp::spin_some(context->node);
    status = tree.tickOnce();
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      break;
    }
    rate.sleep();
  }

  tree.haltTree();
  context->cmd_vel_pub->publish(geometry_msgs::msg::Twist());
  RCLCPP_INFO(context->node->get_logger(), "BT finished with status: %s", BT::toStr(status).c_str());
  rclcpp::shutdown();
  return status == BT::NodeStatus::FAILURE ? 1 : 0;
}
