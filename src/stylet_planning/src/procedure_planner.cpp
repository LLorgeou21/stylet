#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/allowed_collision_matrix.hpp"
#include "moveit_msgs/msg/allowed_collision_entry.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "stylet_msgs/action/execute_procedure.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "stylet_planning/geometry_utils.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <atomic>

using ExecuteProcedure = stylet_msgs::action::ExecuteProcedure;
using GoalHandleExecuteProcedure = rclcpp_action::ServerGoalHandle<ExecuteProcedure>;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
using stylet_planning::quaternionFromZAxis;
using stylet_planning::setAllowedCollisionPair;

class ProcedurePlanner : public rclcpp::Node
{
public:
  ProcedurePlanner() : rclcpp::Node("procedure_planner")
  {
    // Fixed target point (ADR-031): mean of the mesh vertices at scale 0.005,
    // expressed in target_frame (the mesh's own local frame, whose origin sits
    // at a bounding-box corner - NOT the centroid, despite the original
    // assumption). A parameter rather than a hardcoded value so it can be
    // tuned without recompiling. x/y offset by +5% (0.1429->0.150045,
    // 0.125->0.13125) to sit more deeply inside the target's volume.
    this->declare_parameter("target_point_target_frame",
      std::vector<double>{0.150045, 0.13125, 0.1071});
    // Retreat distance for the approach pose (3.5), along the entry->target axis.
    this->declare_parameter("approach_retreat_distance_m", 0.05);
    // Insertion (3.6)
    this->declare_parameter("force_stop_threshold_n", 5.0);
    this->declare_parameter("replanning_max_attempts", 3);
    // Fixed needle_base->needle_tcp length (robot.urdf.xacro, "length"
    // property) - duplicated here since this project has no shared
    // xacro/C++ source of truth (same pattern as the mesh scale, duplicated
    // elsewhere too).
    this->declare_parameter("needle_length_m", 0.15);
    // needle_joint advance speed during insertion (3.6) - a conservative,
    // arbitrary value, not derived from any real medical-device spec.
    this->declare_parameter("insertion_speed_m_s", 0.01);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // The target is fixed and published only once (as soon as the transform
    // succeeds): transient_local QoS so a late subscriber (e.g. "ros2 topic
    // echo" started afterward) still receives that last message.
    auto static_qos = rclcpp::QoS(1).transient_local();

    target_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/procedure/target_point", static_qos);
    entry_pose_robot_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/procedure/entry_pose_robot", 10);
    target_pose_robot_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/procedure/target_pose_robot", static_qos);
    progress_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/procedure/progress", 10);
    system_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/stylet/system/state", rclcpp::QoS(1).transient_local());

    entry_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/procedure/entry_point", 10,
      std::bind(&ProcedurePlanner::entryPointCallback, this, std::placeholders::_1));

    // Force monitoring during insertion (3.6): continuous reading, cheap
    // (just a topic subscription) - unlike planning/executing, which only
    // happen at start or after a force-triggered stop.
    haptics_wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/stylet/haptics/wrench", 10,
      std::bind(&ProcedurePlanner::wrenchCallback, this, std::placeholders::_1));

    ik_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

    // target_frame/base_link may not exist yet on the very first attempt (TF
    // not ready) - retry at a regular interval until it succeeds, same
    // pattern as the one already encountered in point_cloud_merger (2.4).
    target_point_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ProcedurePlanner::publishFixedTargetPoint, this));

    action_server_ = rclcpp_action::create_server<ExecuteProcedure>(
      this,
      "execute_procedure",
      std::bind(&ProcedurePlanner::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ProcedurePlanner::handleCancel, this, std::placeholders::_1),
      std::bind(&ProcedurePlanner::handleAccepted, this, std::placeholders::_1));

    // Direct action client to the controller (3.6) rather than
    // MoveGroupInterface::asyncExecute()/stop(), which has a thread-safety
    // issue when called concurrently from another thread (MoveGroupInterface
    // runs its own internal executor on a separate thread to handle that same
    // action, and a concurrent stop() call from elsewhere raced with it,
    // producing a segfault). Talking to the controller directly gives clear,
    // well-defined control for cancelling an in-flight trajectory
    // (async_cancel_goal).
    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/arm_controller/follow_joint_trajectory");

    // Action client to the insertion actuator (3.6, needle_joint) - a
    // controller separate from arm_controller (see robot.urdf.xacro/
    // ros2_controllers.yaml for why: the arm no longer moves during
    // insertion, only this joint advances).
    insertion_trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/needle_insertion_controller/follow_joint_trajectory");
  }

private:
  void publishFixedTargetPoint()
  {
    std::vector<double> coords =
      this->get_parameter("target_point_target_frame").as_double_array();

    geometry_msgs::msg::PointStamped target_point;
    target_point.header.frame_id = "target_frame";
    target_point.header.stamp = rclcpp::Time(0);
    target_point.point.x = coords[0];
    target_point.point.y = coords[1];
    target_point.point.z = coords[2];
    target_point_pub_->publish(target_point);

    if (processPoint(target_point, "target"))
    {
      publishTargetCollisionBox();
      target_point_timer_->cancel();
    }
  }

  // MoveIt has no idea the target physically exists (it only lives in Gazebo,
  // never in the planning scene): RRTConnect therefore plans trajectories
  // straight through it, blocked only by Gazebo's real physics - producing
  // executions that report "success" while the arm is actually stuck.
  //
  // The real collision mesh (target_collision.stl, already used by Gazebo for
  // physics) is used here rather than a bounding box: a box would cover a lot
  // of genuinely empty space around this asymmetric shape (ADR-006), wrongly
  // marking many approach positions as "unreachable" even though they are
  // physically clear.
  void publishTargetCollisionBox()
  {
    geometry_msgs::msg::TransformStamped target_frame_in_base_link;
    try
    {
      target_frame_in_base_link = tf_buffer_->lookupTransform("base_link", "target_frame", tf2::TimePointZero);
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(),
        "procedure_planner: could not add the target to the planning scene: %s", ex.what());
      return;
    }

    std::string mesh_path = "file://" +
      ament_index_cpp::get_package_share_directory("stylet_description") +
      "/meshes/target_collision.stl";
    // Same scale as model.sdf/generate_reference_pcd.py (0.005, ADR-022)
    shapes::Mesh * raw_mesh = shapes::createMeshFromResource(mesh_path, Eigen::Vector3d(0.005, 0.005, 0.005));
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(raw_mesh, mesh_msg);
    shape_msgs::msg::Mesh mesh_shape = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
    delete raw_mesh;

    geometry_msgs::msg::Pose mesh_pose;
    mesh_pose.position.x = target_frame_in_base_link.transform.translation.x;
    mesh_pose.position.y = target_frame_in_base_link.transform.translation.y;
    mesh_pose.position.z = target_frame_in_base_link.transform.translation.z;
    mesh_pose.orientation = target_frame_in_base_link.transform.rotation;

    moveit_msgs::msg::CollisionObject target_object;
    target_object.header.frame_id = "base_link";
    target_object.id = "target";
    target_object.meshes.push_back(mesh_shape);
    target_object.mesh_poses.push_back(mesh_pose);
    target_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface_->applyCollisionObject(target_object);
    RCLCPP_INFO(this->get_logger(),
      "procedure_planner: target added to the MoveIt planning scene (mesh target_collision.stl)");

    allowNeedleTargetCollision();
  }

  // Explicitly allows needle_base <-> target in the collision matrix without
  // losing existing entries (a first attempt - a PlanningScene diff with only
  // 3 names - REPLACED the entire matrix instead of extending it: MoveIt does
  // not merge an ACM diff, it overwrites it wholesale. Consequence: the arm's
  // own SRDF-derived self-collision exemptions disappeared, and the arm
  // reported colliding with itself at rest. This version instead fetches the
  // COMPLETE current matrix via /get_planning_scene, adds only the
  // needle_base/target pair to it, and sends the whole updated matrix back -
  // nothing pre-existing is lost. needle_tcp has no collision geometry (empty
  // link, robot.urdf.xacro) so it doesn't need an entry.
  void allowNeedleTargetCollision()
  {
    // Local, throwaway node/executor, only for this synchronous call - avoids
    // any conflict with the node's main executor (spin() on the main
    // thread), which could be in the middle of running the very callback
    // this function is called from (deadlock otherwise).
    auto helper_node = std::make_shared<rclcpp::Node>("procedure_planner_acm_helper");
    auto client = helper_node->create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene");
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(this->get_logger(),
        "procedure_planner: /get_planning_scene unavailable, needle<->target collision not allowed");
      return;
    }

    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(helper_node, future, std::chrono::seconds(2)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(),
        "procedure_planner: failed to read the current collision matrix");
      return;
    }

    moveit_msgs::msg::AllowedCollisionMatrix acm = future.get()->scene.allowed_collision_matrix;
    setAllowedCollisionPair(acm, "needle_base", "target");

    moveit_msgs::msg::PlanningScene scene_diff;
    scene_diff.is_diff = true;
    scene_diff.allowed_collision_matrix = acm;
    planning_scene_interface_->applyPlanningScene(scene_diff);
    RCLCPP_INFO(this->get_logger(),
      "procedure_planner: needle_base<->target collision allowed (existing matrix preserved)");
  }

  void entryPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    processPoint(*msg, "entry");
  }

  // Axial force (Z, sensor-local frame on needle_joint, Phase 1.8) - read
  // continuously, used by executeInsertion() to decide on a stop.
  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    latest_axial_force_ = msg->wrench.force.z;
  }

  // Transforms the point (target_frame) into base_link, publishes the
  // corresponding robot pose, keeps it in memory (used by the 3.5 action),
  // and triggers the reachability check. Returns true if the transform
  // succeeded.
  bool processPoint(const geometry_msgs::msg::PointStamped & point_target_frame,
    const std::string & label)
  {
    geometry_msgs::msg::PointStamped point_base_link;
    try
    {
      point_base_link = tf_buffer_->transform(point_target_frame, "base_link");
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(),
        "procedure_planner: could not transform the %s point into base_link: %s",
        label.c_str(), ex.what());
      return false;
    }

    geometry_msgs::msg::PoseStamped pose_base_link;
    pose_base_link.header = point_base_link.header;
    pose_base_link.pose.position = point_base_link.point;
    pose_base_link.pose.orientation.w = 1.0;  // orientation determined later (3.5)

    if (label == "entry")
    {
      entry_pose_robot_pub_->publish(pose_base_link);
      last_entry_pose_ = pose_base_link;
      has_entry_pose_ = true;
    }
    else
    {
      target_pose_robot_pub_->publish(pose_base_link);
      last_target_pose_ = pose_base_link;
      has_target_pose_ = true;
    }

    checkReachability(pose_base_link, label);
    return true;
  }

  // Checks reachability via the IK solver MoveIt already has loaded
  // (/compute_ik) rather than re-deriving the arm's analytical workspace
  // envelope by hand - the solver already encodes the joint limits and arm
  // geometry. Asynchronous: never blocks the pose publishing above.
  //
  // avoid_collisions=false HERE is intentional: the entry point is by
  // definition ON the target's surface, and the target point is INSIDE it -
  // a collision-aware check would ALWAYS fail for these two points, by
  // construction (not a real kinematic-reachability problem). The real
  // collision check that matters happens elsewhere, in executeApproach()
  // (planning to the approach pose, which must stay clear of the target).
  void checkReachability(const geometry_msgs::msg::PoseStamped & pose, const std::string & label)
  {
    if (!ik_client_->service_is_ready())
    {
      RCLCPP_WARN(this->get_logger(),
        "procedure_planner: /compute_ik unavailable, reachability check skipped for %s",
        label.c_str());
      return;
    }

    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = "arm";
    request->ik_request.pose_stamped = pose;
    request->ik_request.timeout = rclcpp::Duration::from_seconds(0.5);
    request->ik_request.avoid_collisions = false;

    ik_client_->async_send_request(request,
      [this, label](rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future)
      {
        auto response = future.get();
        if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          RCLCPP_INFO(this->get_logger(),
            "procedure_planner: %s point reachable (IK found)", label.c_str());
        }
        else
        {
          RCLCPP_WARN(this->get_logger(),
            "procedure_planner: %s point NOT reachable (IK failed, code %d)",
            label.c_str(), response->error_code.val);
        }
      });
  }

  // --- ExecuteProcedure action ---

  // goal_in_progress_: the RViz panel's "Launch operation" button makes a
  // double-click (or a click while a procedure is already running) easy to
  // trigger by accident, which used to spawn two concurrent threads sharing
  // the SAME MoveGroupInterface - not thread-safe (same category of issue as
  // the segfault already root-caused elsewhere in this file). Observed
  // concretely: two overlapping executeApproach() runs corrupting needle
  // retraction (an otherwise-unexplained timeout). Only one goal at a time
  // now.
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteProcedure::Goal>)
  {
    if (goal_in_progress_)
    {
      RCLCPP_WARN(this->get_logger(),
        "procedure_planner: goal rejected, a procedure is already in progress");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!has_entry_pose_ || !has_target_pose_)
    {
      RCLCPP_WARN(this->get_logger(),
        "procedure_planner: goal rejected, entry and/or target point not yet available");
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_in_progress_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleExecuteProcedure>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleExecuteProcedure> goal_handle)
  {
    std::thread{[this, goal_handle]()
    {
      executeApproach(goal_handle);
      goal_in_progress_ = false;
    }}.detach();
  }

  void publishState(const std::string & state)
  {
    std_msgs::msg::String msg;
    msg.data = state;
    system_state_pub_->publish(msg);
  }

  void publishProgress(float progress)
  {
    std_msgs::msg::Float32 msg;
    msg.data = progress;
    progress_pub_->publish(msg);
  }

  void executeApproach(const std::shared_ptr<GoalHandleExecuteProcedure> goal_handle)
  {
    auto feedback = std::make_shared<ExecuteProcedure::Feedback>();
    auto result = std::make_shared<ExecuteProcedure::Result>();

    publishState("APPROACHING");
    feedback->progress = 0.0f;
    feedback->current_step = "Computing the approach pose";
    goal_handle->publish_feedback(feedback);
    publishProgress(0.0f);

    // Entry->target axis (base_link): needle_tcp's Z must align with it.
    Eigen::Vector3d entry(
      last_entry_pose_.pose.position.x, last_entry_pose_.pose.position.y,
      last_entry_pose_.pose.position.z);
    Eigen::Vector3d target(
      last_target_pose_.pose.position.x, last_target_pose_.pose.position.y,
      last_target_pose_.pose.position.z);
    Eigen::Vector3d direction = (target - entry).normalized();

    double retreat = this->get_parameter("approach_retreat_distance_m").as_double();
    Eigen::Vector3d approach_position = entry - retreat * direction;
    Eigen::Quaterniond orientation = quaternionFromZAxis(direction);

    // The MoveIt "arm" group now targets tool0, not needle_tcp (SRDF, 3.6 -
    // needle_joint is prismatic, excluding it from the chain avoids a
    // redundant IK degree of freedom). approach_position above remains the
    // TARGETED position for the needle tip (needle_tcp); the planning target
    // must therefore be pulled back by the needle's fixed length along the
    // same axis to get the corresponding tool0 pose (needle_joint stays at
    // 0, retracted, throughout the approach).
    double needle_length = this->get_parameter("needle_length_m").as_double();
    Eigen::Vector3d tool0_target_position = approach_position - needle_length * direction;

    RCLCPP_INFO_STREAM(this->get_logger(),
      "entry (base_link): " << entry.transpose()
      << " | target (base_link): " << target.transpose()
      << " | direction: " << direction.transpose()
      << " | retreat: " << retreat
      << " | approach_position (needle_tcp target): " << approach_position.transpose()
      << " | tool0_target_position: " << tool0_target_position.transpose());

    geometry_msgs::msg::PoseStamped approach_pose;
    approach_pose.header.frame_id = "base_link";
    approach_pose.pose.position.x = tool0_target_position.x();
    approach_pose.pose.position.y = tool0_target_position.y();
    approach_pose.pose.position.z = tool0_target_position.z();
    approach_pose.pose.orientation.x = orientation.x();
    approach_pose.pose.orientation.y = orientation.y();
    approach_pose.pose.orientation.z = orientation.z();
    approach_pose.pose.orientation.w = orientation.w();

    // Created on first call (needs shared_from_this(), so not possible in
    // the constructor - the node isn't owned by a shared_ptr yet at that
    // point).
    if (!move_group_)
    {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm");
    }

    // Retract the needle BEFORE moving the arm: a previous insertion may
    // have left the needle extended - retracting it before any arm motion
    // (including the "up" pass below) avoids swinging the arm around with
    // the needle out.
    feedback->progress = 0.05f;
    feedback->current_step = "Retracting the needle (rest position)";
    goal_handle->publish_feedback(feedback);
    publishProgress(0.05f);

    if (!retractNeedle())
    {
      result->success = false;
      result->message = "Failed to retract the needle before the 'up' pass";
      goal_handle->abort(result);
      publishState("ERROR");
      return;
    }

    // Intermediate step: pass through the named state "up" (SRDF, far from
    // the target) before the approach. This doesn't fix the underlying
    // Gazebo/ros2_control/MoveIt clock-synchronization issue (occasional
    // false successes), but limits its impact: planning restarts from a
    // known, target-far configuration rather than from a current state that
    // current_state_monitor might read incorrectly.
    feedback->progress = 0.1f;
    feedback->current_step = "Moving through the 'up' position";
    goal_handle->publish_feedback(feedback);
    publishProgress(0.1f);

    move_group_->setNamedTarget("up");
    moveit::planning_interface::MoveGroupInterface::Plan up_plan;
    bool up_plan_ok = (move_group_->plan(up_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool up_exec_ok = up_plan_ok &&
      (move_group_->execute(up_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!up_exec_ok)
    {
      result->success = false;
      result->message = "Failed to move through the 'up' position";
      goal_handle->abort(result);
      publishState("ERROR");
      return;
    }

    feedback->progress = 0.3f;
    feedback->current_step = "Planning (joint-space)";
    goal_handle->publish_feedback(feedback);
    publishProgress(0.3f);

    move_group_->setPoseTarget(approach_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!plan_ok)
    {
      result->success = false;
      result->message = "Failed to plan to the approach pose";
      goal_handle->abort(result);
      publishState("ERROR");
      return;
    }

    feedback->progress = 0.6f;
    feedback->current_step = "Executing";
    goal_handle->publish_feedback(feedback);
    publishProgress(0.6f);

    bool exec_ok = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!exec_ok)
    {
      result->success = false;
      result->message = "Failed to execute the approach trajectory";
      goal_handle->abort(result);
      publishState("ERROR");
      return;
    }

    // Diagnostic: compares needle_tcp's real pose (after execution) to the
    // targeted approach pose - distance, and alignment of its Z axis with
    // the entry->target axis. Also serves as a guard rail below: never
    // blindly trust MoveIt's own "success" - a clock-synchronization issue
    // can produce a false success (arm stuck against/hitting the target
    // despite an "Execute request success!").
    bool position_ok = false;
    try
    {
      geometry_msgs::msg::TransformStamped actual_tf =
        tf_buffer_->lookupTransform("base_link", "needle_tcp", tf2::TimePointZero);

      Eigen::Vector3d actual_position(
        actual_tf.transform.translation.x, actual_tf.transform.translation.y,
        actual_tf.transform.translation.z);
      double distance_error_mm = (actual_position - approach_position).norm() * 1000.0;

      Eigen::Quaterniond actual_orientation(
        actual_tf.transform.rotation.w, actual_tf.transform.rotation.x,
        actual_tf.transform.rotation.y, actual_tf.transform.rotation.z);
      Eigen::Vector3d actual_z_axis = actual_orientation.toRotationMatrix().col(2);
      double alignment_deg = std::acos(
        std::max(-1.0, std::min(1.0, actual_z_axis.dot(direction)))) * 180.0 / std::acos(-1.0);

      RCLCPP_INFO_STREAM(this->get_logger(),
        "actual needle_tcp (base_link): " << actual_position.transpose()
        << " | position error vs approach_position: " << distance_error_mm << " mm"
        << " | actual Z axis: " << actual_z_axis.transpose()
        << " | alignment error vs entry->target axis: " << alignment_deg << " deg");

      position_ok = (distance_error_mm < kMaxPositionErrorMm) && (alignment_deg < kMaxAlignmentErrorDeg);
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(), "needle_tcp diagnostic unavailable: %s", ex.what());
    }

    if (!position_ok)
    {
      result->success = false;
      result->message = "MoveIt reported success but needle_tcp's actual position "
        "deviates too far from the targeted approach pose (post-execution check)";
      goal_handle->abort(result);
      publishState("ERROR");
      return;
    }

    feedback->progress = 0.7f;
    feedback->current_step = "Approach complete, starting insertion";
    goal_handle->publish_feedback(feedback);
    publishProgress(0.7f);

    executeInsertion(goal_handle, feedback, result, target, direction);
    // No transition to WAITING_FOR_PHASE here (ADR-012, respiratory
    // synchronization) - that belongs to Phase 5, not yet implemented.
  }

  // Retracts the insertion actuator's 3 stages to 0 (rest position) and
  // waits for the controller's confirmation (async_get_result, not just goal
  // acceptance) before continuing - we want to be sure the needle is
  // actually back in before moving the arm (the caller).
  bool retractNeedle()
  {
    if (!insertion_trajectory_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(this->get_logger(),
        "procedure_planner: insertion controller unavailable for retraction");
      return false;
    }

    double insertion_speed = this->get_parameter("insertion_speed_m_s").as_double();

    // Duration proportional to the ACTUAL distance to retract (never assumed
    // to be the maximum - using kMaxNeedleJointValue/insertion_speed always
    // produced 30s, even when the needle was already nearly retracted,
    // because the controller stretches the motion over the whole requested
    // duration rather than finishing early).
    double current_extension = kMaxNeedleJointValue;  // conservative fallback if TF unavailable
    try
    {
      geometry_msgs::msg::TransformStamped tool0_tf =
        tf_buffer_->lookupTransform("base_link", "tool0", tf2::TimePointZero);
      geometry_msgs::msg::TransformStamped tip_tf =
        tf_buffer_->lookupTransform("base_link", "needle_tcp", tf2::TimePointZero);
      Eigen::Vector3d tool0_pos(
        tool0_tf.transform.translation.x, tool0_tf.transform.translation.y,
        tool0_tf.transform.translation.z);
      Eigen::Vector3d tip_pos(
        tip_tf.transform.translation.x, tip_tf.transform.translation.y,
        tip_tf.transform.translation.z);
      double needle_length = this->get_parameter("needle_length_m").as_double();
      current_extension = std::max(0.0, (tip_pos - tool0_pos).norm() - needle_length);
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(),
        "procedure_planner: TF unavailable to measure current extension, "
        "retraction duration conservatively maximized: %s", ex.what());
    }

    double duration_s = std::max(0.1, current_extension / insertion_speed);

    FollowJointTrajectory::Goal traj_goal;
    traj_goal.trajectory.joint_names = {"needle_stage1_joint", "needle_stage2_joint", "needle_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint traj_point;
    traj_point.positions = {0.0, 0.0, 0.0};
    traj_point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
    traj_goal.trajectory.points = {traj_point};

    auto send_goal_future = insertion_trajectory_client_->async_send_goal(traj_goal);
    if (send_goal_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
      RCLCPP_WARN(this->get_logger(),
        "procedure_planner: insertion controller did not respond to the retraction request");
      return false;
    }
    std::shared_ptr<GoalHandleFollowJointTrajectory> retract_goal_handle = send_goal_future.get();
    if (!retract_goal_handle)
    {
      RCLCPP_WARN(this->get_logger(), "procedure_planner: needle retraction rejected by the controller");
      return false;
    }

    auto result_future = insertion_trajectory_client_->async_get_result(retract_goal_handle);
    if (result_future.wait_for(std::chrono::duration<double>(duration_s + 2.0)) != std::future_status::ready)
    {
      RCLCPP_WARN(this->get_logger(), "procedure_planner: needle retraction not confirmed (timeout)");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "procedure_planner: needle retracted (rest position)");
    return true;
  }

  // 3.6: the arm no longer moves during insertion (see robot.urdf.xacro,
  // needle_joint) - the geometry doesn't allow reaching some targets without
  // driving the wrist itself into the target (a needle too short for a
  // retreat+depth that can exceed its own length). Only needle_joint
  // (prismatic) advances, like a real medical insertion device (arm =
  // one-time positioning, separate actuator = insertion). Continuous
  // monitoring of /stylet/haptics/wrench DURING execution: if the axial
  // force exceeds the threshold, stop (cancel the goal) then replan from the
  // REAL position (never assumed, per ADR-035) toward the target, within
  // replanning_max_attempts tries.
  void executeInsertion(
    const std::shared_ptr<GoalHandleExecuteProcedure> & goal_handle,
    const std::shared_ptr<ExecuteProcedure::Feedback> & feedback,
    const std::shared_ptr<ExecuteProcedure::Result> & result,
    const Eigen::Vector3d & target_position,
    const Eigen::Vector3d & direction)
  {
    publishState("INSERTING");

    int max_attempts = static_cast<int>(this->get_parameter("replanning_max_attempts").as_int());
    double force_threshold = this->get_parameter("force_stop_threshold_n").as_double();
    double needle_length = this->get_parameter("needle_length_m").as_double();
    double insertion_speed = this->get_parameter("insertion_speed_m_s").as_double();

    // Metrics reported at the end (success or failure).
    auto insertion_start_time = std::chrono::steady_clock::now();
    double max_force_observed = 0.0;
    double last_target_joint_value = 0.0;

    for (int attempt = 0; attempt <= max_attempts; ++attempt)
    {
      geometry_msgs::msg::TransformStamped current_tf;
      geometry_msgs::msg::TransformStamped tool0_tf;
      try
      {
        current_tf = tf_buffer_->lookupTransform("base_link", "needle_tcp", tf2::TimePointZero);
        tool0_tf = tf_buffer_->lookupTransform("base_link", "tool0", tf2::TimePointZero);
      }
      catch (const tf2::TransformException & ex)
      {
        result->success = false;
        result->message = "Insertion: could not read needle_tcp/tool0's real position";
        goal_handle->abort(result);
        publishState("ERROR");
        return;
      }

      Eigen::Vector3d current_position(
        current_tf.transform.translation.x, current_tf.transform.translation.y,
        current_tf.transform.translation.z);
      Eigen::Vector3d tool0_position(
        tool0_tf.transform.translation.x, tool0_tf.transform.translation.y,
        tool0_tf.transform.translation.z);

      // Guard rail: if the real position has drifted too far from the
      // entry->target axis (projection outside the segment, or too far
      // laterally), refuse to continue rather than replan from a position
      // that isn't understood (collision, slippage, or the known
      // clock-synchronization issue).
      Eigen::Vector3d to_current = current_position - target_position;
      double lateral_distance = (to_current - to_current.dot(direction) * direction).norm();
      if (lateral_distance > kMaxLineDeviationM)
      {
        result->success = false;
        result->message = "Insertion: real position too far from the entry->target axis, aborting";
        goal_handle->abort(result);
        publishState("ERROR");
        return;
      }

      // needle_joint is defined relative to tool0 (robot.urdf.xacro): its
      // target value is the tool0->target distance along the axis, minus the
      // needle's fixed length. tool0 is re-read here (never assumed
      // stationary), even though the arm shouldn't move anymore at this
      // stage.
      double target_joint_value = (target_position - tool0_position).dot(direction) - needle_length;
      last_target_joint_value = target_joint_value;

      if (target_joint_value < 0.0 || target_joint_value > kMaxNeedleJointValue)
      {
        result->success = false;
        result->message = "Insertion: required depth is out of needle_joint's range";
        goal_handle->abort(result);
        publishState("ERROR");
        return;
      }

      double remaining_distance = (target_position - current_position).dot(direction);
      double duration_s = std::max(0.1, remaining_distance / insertion_speed);

      // Same rationale as trajectory_client_ (constructor): talk directly to
      // the controller rather than through MoveGroupInterface - here
      // needle_joint isn't planned by MoveIt anyway (excluded from the "arm"
      // group, SRDF), the trajectory is built by hand.
      if (!insertion_trajectory_client_->wait_for_action_server(std::chrono::seconds(2)))
      {
        result->success = false;
        result->message =
          "Insertion: /needle_insertion_controller/follow_joint_trajectory unavailable";
        goal_handle->abort(result);
        publishState("ERROR");
        return;
      }

      // Purely visual 3-stage telescoping (robot.urdf.xacro,
      // needle_stage1/2_joint) - the 3 joints share the travel equally,
      // needle_joint (the real FT sensor) included. Always valid by
      // construction: each joint's own limit is stage_max =
      // kMaxNeedleJointValue/3, and target_joint_value is already checked
      // <= kMaxNeedleJointValue above.
      double stage_value = target_joint_value / 3.0;

      FollowJointTrajectory::Goal traj_goal;
      traj_goal.trajectory.joint_names = {"needle_stage1_joint", "needle_stage2_joint", "needle_joint"};
      trajectory_msgs::msg::JointTrajectoryPoint traj_point;
      traj_point.positions = {stage_value, stage_value, stage_value};
      traj_point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
      traj_goal.trajectory.points = {traj_point};

      auto send_goal_future = insertion_trajectory_client_->async_send_goal(traj_goal);
      if (send_goal_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
      {
        result->success = false;
        result->message = "Insertion: the controller did not respond to the trajectory request";
        goal_handle->abort(result);
        publishState("ERROR");
        return;
      }
      std::shared_ptr<GoalHandleFollowJointTrajectory> traj_goal_handle = send_goal_future.get();
      if (!traj_goal_handle)
      {
        result->success = false;
        result->message = "Insertion: trajectory rejected by the controller";
        goal_handle->abort(result);
        publishState("ERROR");
        return;
      }

      RCLCPP_INFO(this->get_logger(),
        "procedure_planner: insertion attempt %d/%d, total depth -> %.4f m "
        "(%.4f m/stage x3, %.2f s)",
        attempt, max_attempts, target_joint_value, stage_value, duration_s);

      auto start_time = std::chrono::steady_clock::now();
      bool force_triggered = false;
      while (true)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start_time).count();

        max_force_observed = std::max(max_force_observed, std::abs(latest_axial_force_));

        if (std::abs(latest_axial_force_) > force_threshold)
        {
          insertion_trajectory_client_->async_cancel_goal(traj_goal_handle);
          force_triggered = true;
          break;
        }

        feedback->progress = 0.7f + 0.3f * static_cast<float>(
          std::min(1.0, elapsed / duration_s));
        feedback->current_step = "Insertion in progress";
        goal_handle->publish_feedback(feedback);
        publishProgress(feedback->progress);

        if (elapsed > duration_s + 1.0)  // safety margin
        {
          break;
        }
      }

      if (force_triggered)
      {
        RCLCPP_WARN(this->get_logger(),
          "procedure_planner: axial force %.2f N > threshold %.2f N, stopping and replanning (attempt %d/%d)",
          latest_axial_force_, force_threshold, attempt, max_attempts);
        continue;  // loop back: re-reads the real position, replans
      }

      // No force-triggered stop: verify we actually arrived (same guard rail
      // as 3.5/ADR-035, never blindly trust the execution).
      try
      {
        geometry_msgs::msg::TransformStamped final_tf =
          tf_buffer_->lookupTransform("base_link", "needle_tcp", tf2::TimePointZero);
        Eigen::Vector3d final_position(
          final_tf.transform.translation.x, final_tf.transform.translation.y,
          final_tf.transform.translation.z);
        double final_error_mm = (final_position - target_position).norm() * 1000.0;

        if (final_error_mm < kMaxPositionErrorMm)
        {
          feedback->progress = 1.0f;
          feedback->current_step = "Insertion complete";
          goal_handle->publish_feedback(feedback);
          publishProgress(1.0f);

          // Metrics: included in the result message AND logged, for an easy
          // read without having to dig through the per-attempt logs.
          Eigen::Quaterniond final_orientation(
            final_tf.transform.rotation.w, final_tf.transform.rotation.x,
            final_tf.transform.rotation.y, final_tf.transform.rotation.z);
          Eigen::Vector3d final_z_axis = final_orientation.toRotationMatrix().col(2);
          double alignment_deg = std::acos(
            std::max(-1.0, std::min(1.0, final_z_axis.dot(direction)))) * 180.0 / std::acos(-1.0);
          double total_duration_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - insertion_start_time).count();

          std::ostringstream summary;
          summary << "Insertion completed successfully | position error: "
            << std::fixed << std::setprecision(2) << final_error_mm << " mm"
            << " | alignment error: " << alignment_deg << " deg"
            << " | insertion depth: " << (last_target_joint_value * 1000.0) << " mm"
            << " | max axial force: " << max_force_observed << " N"
            << " | attempts: " << (attempt + 1) << "/" << (max_attempts + 1)
            << " | total duration: " << total_duration_s << " s";

          RCLCPP_INFO(this->get_logger(), "procedure_planner: %s", summary.str().c_str());

          result->success = true;
          result->message = summary.str();
          goal_handle->succeed(result);
          publishState("COMPLETED");
          return;
        }

        RCLCPP_WARN(this->get_logger(),
          "procedure_planner: insertion finished but deviation from target too large (%.1f mm), replanning",
          final_error_mm);
      }
      catch (const tf2::TransformException & ex)
      {
        RCLCPP_WARN(this->get_logger(), "Insertion diagnostic unavailable: %s", ex.what());
      }
    }

    result->success = false;
    result->message = "Insertion: maximum number of replanning attempts reached";
    goal_handle->abort(result);
    publishState("ERROR");
  }

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr entry_pose_robot_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_robot_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr progress_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr entry_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr haptics_wrench_sub_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::TimerBase::SharedPtr target_point_timer_;
  rclcpp_action::Server<ExecuteProcedure>::SharedPtr action_server_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr insertion_trajectory_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  geometry_msgs::msg::PoseStamped last_entry_pose_;
  geometry_msgs::msg::PoseStamped last_target_pose_;
  bool has_entry_pose_ = false;
  bool has_target_pose_ = false;
  double latest_axial_force_ = 0.0;
  // Written by handleGoal()/handleAccepted() (main executor, single thread)
  // and by executeApproach()'s detached thread at the very end -
  // std::atomic for cross-thread visibility (no mutex needed, independent
  // read/write, only ever one writer at a time).
  std::atomic<bool> goal_in_progress_{false};

  // Post-execution guard-rail thresholds: beyond these, a MoveIt "success"
  // is treated as false (see the diagnostic in executeApproach()).
  static constexpr double kMaxPositionErrorMm = 10.0;
  static constexpr double kMaxAlignmentErrorDeg = 5.0;
  // Max lateral deviation from the entry->target axis tolerated during
  // insertion (3.6) before refusing to continue.
  static constexpr double kMaxLineDeviationM = 0.02;
  // Max total travel (robot.urdf.xacro, stage_max=0.10 x 3 stages) - guard
  // rail before sending an out-of-range target value to the controller.
  // Each individual stage is limited to kMaxNeedleJointValue/3 (see
  // executeInsertion()).
  static constexpr double kMaxNeedleJointValue = 0.30;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProcedurePlanner>());
  rclcpp::shutdown();
  return 0;
}
