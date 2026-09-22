#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;


class PickPlace
{
public:

    PickPlace(
        const rclcpp::Node::SharedPtr& node,
        bool execute,
        bool execute_initial,
        bool execute_pick,
        bool execute_place)
        : node_(node),
          execute_(execute),
          execute_initial_(execute_initial),
          execute_pick_(execute_pick),
          execute_place_(execute_place),
          arm_(node_, "arm"),
          gripper_(node_, "gripper")
    {
        // =====================================================
        // ARM SETTINGS
        // =====================================================

        arm_.setPoseReferenceFrame("base_link");
        arm_.setEndEffectorLink("gripper_base_link");

        arm_.setPlanningTime(5.0);
        arm_.setNumPlanningAttempts(10);

        arm_.setMaxVelocityScalingFactor(0.20);
        arm_.setMaxAccelerationScalingFactor(0.20);


        // =====================================================
        // GRIPPER SETTINGS
        // =====================================================

        gripper_.setPlanningTime(5.0);
        gripper_.setNumPlanningAttempts(5);


        // =====================================================
        // STATUS
        // =====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "Pick and Place node initialized.");

        if (execute_initial_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "INITIAL-ONLY EXECUTION MODE");
        }
        else if (execute_pick_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "PICK + CLOSE + LIFT EXECUTION MODE");
        }
        else if (execute_place_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "PLACE + OPEN EXECUTION MODE");
        }
        else if (execute_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "FULL PICK AND PLACE EXECUTION MODE");
        }
        else
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "PLAN ONLY MODE");
        }
    }


    // =========================================================
    // CREATE CARTESIAN POSE
    // =========================================================

    geometry_msgs::msg::Pose createPose(
        double x,
        double y,
        double z,
        double roll_deg,
        double pitch_deg,
        double yaw_deg)
    {
        geometry_msgs::msg::Pose pose;

        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        const double roll =
            roll_deg * M_PI / 180.0;

        const double pitch =
            pitch_deg * M_PI / 180.0;

        const double yaw =
            yaw_deg * M_PI / 180.0;

        tf2::Quaternion q;

        q.setRPY(
            roll,
            pitch,
            yaw);

        q.normalize();

        pose.orientation =
            tf2::toMsg(q);

        return pose;
    }


    // =========================================================
    // MOVE ARM TO CARTESIAN POSE
    // =========================================================

    bool moveArm(
        const std::string& stage_name,
        const geometry_msgs::msg::Pose& target)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "----------------------------------------");

        RCLCPP_INFO(
            node_->get_logger(),
            "%s",
            stage_name.c_str());

        RCLCPP_INFO(
            node_->get_logger(),
            "Planning...");

        // Always start from the actual current robot state.
        arm_.setStartStateToCurrentState();

        // Set Cartesian target.
        arm_.setPoseTarget(
            target,
            "gripper_base_link");

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        auto result =
            arm_.plan(plan);

        arm_.clearPoseTargets();


        // =====================================================
        // CHECK PLANNING
        // =====================================================

        if (result !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s : PLANNING FAILED",
                stage_name.c_str());

            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "%s : PLANNING SUCCESS",
            stage_name.c_str());


        // =====================================================
        // PLAN ONLY
        // =====================================================

        if (!execute_)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Plan only - robot will NOT move.");

            return true;
        }


        // =====================================================
        // EXECUTE
        // =====================================================

        RCLCPP_WARN(
            node_->get_logger(),
            "Executing %s",
            stage_name.c_str());

        auto execution_result =
            arm_.execute(plan);

        if (execution_result !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s : EXECUTION FAILED",
                stage_name.c_str());

            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "%s : EXECUTION SUCCESS",
            stage_name.c_str());

        std::this_thread::sleep_for(500ms);

        return true;
    }


    // =========================================================
    // MOVE ONLY JOINT 2 TO ZERO
    //
    // IMPORTANT:
    //
    // This function explicitly sets ALL arm joints.
    //
    // joint_1 = CURRENT position
    // joint_2 = 0 radians
    // joint_3 = CURRENT position
    // joint_4 = CURRENT position
    // joint_5 = CURRENT position
    //
    // Therefore MoveIt cannot choose arbitrary values for
    // the other joints at the goal.
    // =========================================================

    bool moveJoint2OnlyToZero()
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "----------------------------------------");

        RCLCPP_INFO(
            node_->get_logger(),
            "STEP 5 : LIFT - ONLY JOINT_2 TO ZERO");

        // =====================================================
        // GET CURRENT ROBOT STATE
        // =====================================================

        arm_.setStartStateToCurrentState();

        std::vector<double> current_joints =
            arm_.getCurrentJointValues();

        const std::vector<std::string> joint_names =
            arm_.getJointNames();


        if (current_joints.empty())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Could not read current arm joint positions.");

            return false;
        }


        if (current_joints.size() != joint_names.size())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Joint position/name count mismatch.");

            return false;
        }


        // =====================================================
        // COPY CURRENT POSITIONS
        //
        // The target initially contains the current position
        // of EVERY arm joint.
        // =====================================================

        std::vector<double> target_joints =
            current_joints;


        // =====================================================
        // FIND JOINT_2
        // =====================================================

        auto joint_2_it =
            std::find(
                joint_names.begin(),
                joint_names.end(),
                "joint_2");


        if (joint_2_it == joint_names.end())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "joint_2 was not found in the arm group.");

            return false;
        }


        const std::size_t joint_2_index =
            std::distance(
                joint_names.begin(),
                joint_2_it);


        // =====================================================
        // CHANGE ONLY JOINT_2
        //
        // 0 radians = 0 degrees
        // =====================================================

        target_joints[joint_2_index] =
            0.0;


        // =====================================================
        // PRINT START AND TARGET
        // =====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "Current arm joint positions:");

        for (std::size_t i = 0;
             i < joint_names.size();
             ++i)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "  %s = %.3f deg",
                joint_names[i].c_str(),
                current_joints[i] * 180.0 / M_PI);
        }


        RCLCPP_INFO(
            node_->get_logger(),
            "Lift target:");

        for (std::size_t i = 0;
             i < joint_names.size();
             ++i)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "  %s = %.3f deg",
                joint_names[i].c_str(),
                target_joints[i] * 180.0 / M_PI);
        }


        // =====================================================
        // SET COMPLETE JOINT TARGET
        // =====================================================

        arm_.setJointValueTarget(
            target_joints);


        // =====================================================
        // PLAN
        // =====================================================

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        auto result =
            arm_.plan(plan);


        if (result !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "LIFT : PLANNING FAILED");

            return false;
        }


        RCLCPP_INFO(
            node_->get_logger(),
            "LIFT : PLANNING SUCCESS");


        // =====================================================
        // PLAN ONLY
        // =====================================================

        if (!execute_)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Plan only - lift will NOT execute.");

            return true;
        }


        // =====================================================
        // EXECUTE
        // =====================================================

        RCLCPP_WARN(
            node_->get_logger(),
            "Executing lift.");

        RCLCPP_WARN(
            node_->get_logger(),
            "ONLY joint_2 is commanded to move to 0 degrees.");


        auto execution_result =
            arm_.execute(plan);


        if (execution_result !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "LIFT : EXECUTION FAILED");

            return false;
        }


        RCLCPP_INFO(
            node_->get_logger(),
            "LIFT : EXECUTION SUCCESS");

        RCLCPP_INFO(
            node_->get_logger(),
            "joint_2 reached the requested 0 degree target.");

        std::this_thread::sleep_for(700ms);

        return true;
    }


    // =========================================================
    // MOVE GRIPPER
    // =========================================================

    bool moveGripper(
        const std::string& stage_name,
        double target_angle)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "----------------------------------------");

        RCLCPP_INFO(
            node_->get_logger(),
            "%s",
            stage_name.c_str());

        RCLCPP_INFO(
            node_->get_logger(),
            "Gripper target: %.4f rad (%.1f deg)",
            target_angle,
            target_angle * 180.0 / M_PI);


        // Use current robot state.
        gripper_.setStartStateToCurrentState();


        // joint_6 is the gripper joint.
        std::vector<double> joint_target;

        joint_target.push_back(
            target_angle);


        gripper_.setJointValueTarget(
            joint_target);


        moveit::planning_interface::MoveGroupInterface::Plan plan;

        auto result =
            gripper_.plan(plan);


        // =====================================================
        // CHECK PLANNING
        // =====================================================

        if (result !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s : GRIPPER PLANNING FAILED",
                stage_name.c_str());

            return false;
        }


        RCLCPP_INFO(
            node_->get_logger(),
            "%s : GRIPPER PLANNING SUCCESS",
            stage_name.c_str());


        // =====================================================
        // PLAN ONLY
        // =====================================================

        if (!execute_)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Plan only - gripper will NOT move.");

            return true;
        }


        // =====================================================
        // EXECUTE
        // =====================================================

        RCLCPP_WARN(
            node_->get_logger(),
            "Executing %s",
            stage_name.c_str());


        auto execution_result =
            gripper_.execute(plan);


        if (execution_result !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "%s : GRIPPER EXECUTION FAILED",
                stage_name.c_str());

            return false;
        }


        RCLCPP_INFO(
            node_->get_logger(),
            "%s : GRIPPER EXECUTION SUCCESS",
            stage_name.c_str());

        std::this_thread::sleep_for(500ms);

        return true;
    }


    // =========================================================
    // RUN PICK AND PLACE
    // =========================================================

    void run()
    {
        // =====================================================
        // GRIPPER CALIBRATION
        // =====================================================

        // OPEN = -21 degrees
        const double gripper_open =
            -21.0 * M_PI / 180.0;

        // CLOSE = -66 degrees
        const double gripper_close =
            -66.0 * M_PI / 180.0;


        // =====================================================
        // INITIAL POSE
        // =====================================================

        auto initial_pose =
            createPose(
                0.008,
                0.106,
                0.209,
                93.621,
                -11.700,
                179.210);


        // =====================================================
        // PICK POSE
        // =====================================================

        auto pick_pose =
            createPose(
                0.007,
                0.185,
                0.044,
                141.509,
                -5.603,
                170.418);


        // =====================================================
        // PLACE POSE
        // =====================================================

        auto place_pose =
            createPose(
                -0.095,
                0.042,
                0.056,
                141.584,
                -0.404,
                -122.081);


        // =====================================================
        // INITIAL ONLY MODE
        // =====================================================

        if (execute_initial_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "========================================");

            RCLCPP_WARN(
                node_->get_logger(),
                "INITIAL POSITION ONLY");

            RCLCPP_WARN(
                node_->get_logger(),
                "========================================");


            moveArm(
                "INITIAL POSITION",
                initial_pose);

            return;
        }


        // =====================================================
        // PICK ONLY MODE
        //
        // PICK
        // CLOSE
        // ONLY JOINT_2 -> 0
        // =====================================================

        if (execute_pick_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "========================================");

            RCLCPP_WARN(
                node_->get_logger(),
                "PICK + CLOSE + JOINT_2 LIFT");

            RCLCPP_WARN(
                node_->get_logger(),
                "========================================");


            // PICK
            if (!moveArm(
                    "PICK POSITION",
                    pick_pose))
            {
                return;
            }


            // CLOSE
            if (!moveGripper(
                    "GRIPPER CLOSE AT PICK",
                    gripper_close))
            {
                return;
            }


            // ONLY JOINT_2 TO ZERO
            if (!moveJoint2OnlyToZero())
            {
                return;
            }


            RCLCPP_INFO(
                node_->get_logger(),
                "PICK + CLOSE + LIFT COMPLETE");

            return;
        }


        // =====================================================
        // PLACE ONLY MODE
        // =====================================================

        if (execute_place_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "========================================");

            RCLCPP_WARN(
                node_->get_logger(),
                "PLACE + OPEN ONLY");

            RCLCPP_WARN(
                node_->get_logger(),
                "========================================");


            if (!moveArm(
                    "PLACE POSITION",
                    place_pose))
            {
                return;
            }


            if (!moveGripper(
                    "GRIPPER OPEN AT PLACE",
                    gripper_open))
            {
                return;
            }


            RCLCPP_INFO(
                node_->get_logger(),
                "PLACE + OPEN COMPLETE");

            return;
        }


        // =====================================================
        // FULL PICK AND PLACE
        // =====================================================

        RCLCPP_WARN(
            node_->get_logger(),
            "========================================");

        RCLCPP_WARN(
            node_->get_logger(),
            "FULL PICK AND PLACE SEQUENCE");

        RCLCPP_WARN(
            node_->get_logger(),
            "========================================");


        // =====================================================
        // STEP 1
        // INITIAL
        // =====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "STEP 1 : MOVE TO INITIAL");


        if (!moveArm(
                "INITIAL POSITION",
                initial_pose))
        {
            return;
        }


        // =====================================================
        // STEP 2
        // OPEN GRIPPER
        // =====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "STEP 2 : OPEN GRIPPER");


        if (!moveGripper(
                "GRIPPER OPEN",
                gripper_open))
        {
            return;
        }


        // =====================================================
        // STEP 3
        // PICK
        // =====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "STEP 3 : MOVE TO PICK");


        if (!moveArm(
                "PICK POSITION",
                pick_pose))
        {
            return;
        }


        // =====================================================
        // STEP 4
        // CLOSE GRIPPER
        // =====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "STEP 4 : CLOSE GRIPPER");


        if (!moveGripper(
                "GRIPPER CLOSE AT PICK",
                gripper_close))
        {
            return;
        }


        // =====================================================
        // STEP 5
        // LIFT
        //
        // ONLY joint_2 -> 0 degrees
        //
        // joint_1, joint_3, joint_4 and joint_5 remain at
        // their current positions.
        // =====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "STEP 5 : LIFT - ONLY JOINT_2 TO ZERO");


        if (!moveJoint2OnlyToZero())
        {
            return;
        }


        // =====================================================
        // STEP 6
        // PLACE
        //
        // IMPORTANT:
        //
        // The robot is now at the lifted configuration.
        //
        // MoveIt starts from that actual configuration and
        // calculates the normal joint configuration required
        // to reach the PLACE Cartesian pose.
        //
        // joint_2 is NOT locked at zero for PLACE.
        // =====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "STEP 6 : MOVE TO PLACE");


        if (!moveArm(
                "PLACE POSITION",
                place_pose))
        {
            return;
        }


        // =====================================================
        // STEP 7
        // OPEN GRIPPER
        // =====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "STEP 7 : OPEN GRIPPER AT PLACE");


        if (!moveGripper(
                "GRIPPER OPEN AT PLACE",
                gripper_open))
        {
            return;
        }


        // =====================================================
        // STEP 8
        // RETURN TO INITIAL
        // =====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "STEP 8 : RETURN TO INITIAL");


        if (!moveArm(
                "RETURN TO INITIAL",
                initial_pose))
        {
            return;
        }


        // =====================================================
        // COMPLETE
        // =====================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "========================================");

        RCLCPP_INFO(
            node_->get_logger(),
            "FULL PICK AND PLACE COMPLETE");

        RCLCPP_INFO(
            node_->get_logger(),
            "========================================");
    }


private:

    rclcpp::Node::SharedPtr node_;

    bool execute_;
    bool execute_initial_;
    bool execute_pick_;
    bool execute_place_;

    moveit::planning_interface::MoveGroupInterface arm_;

    moveit::planning_interface::MoveGroupInterface gripper_;
};


// =============================================================
// MAIN
// =============================================================

int main(
    int argc,
    char* argv[])
{
    rclcpp::init(
        argc,
        argv);


    auto node =
        rclcpp::Node::make_shared(
            "pick_place_node");


    // =========================================================
    // PARAMETERS
    // =========================================================

    bool execute =
        node->declare_parameter<bool>(
            "execute",
            false);


    bool execute_initial =
        node->declare_parameter<bool>(
            "execute_initial",
            false);


    bool execute_pick =
        node->declare_parameter<bool>(
            "execute_pick",
            false);


    bool execute_place =
        node->declare_parameter<bool>(
            "execute_place",
            false);


    // =========================================================
    // CHECK MULTIPLE MODES
    // =========================================================

    int selected_modes = 0;


    if (execute_initial)
    {
        selected_modes++;
    }


    if (execute_pick)
    {
        selected_modes++;
    }


    if (execute_place)
    {
        selected_modes++;
    }


    if (selected_modes > 1)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Multiple execution modes selected.");


        RCLCPP_ERROR(
            node->get_logger(),
            "Use only one of:");


        RCLCPP_ERROR(
            node->get_logger(),
            "execute_initial:=true");


        RCLCPP_ERROR(
            node->get_logger(),
            "execute_pick:=true");


        RCLCPP_ERROR(
            node->get_logger(),
            "execute_place:=true");


        rclcpp::shutdown();

        return 1;
    }


    // =========================================================
    // ROS EXECUTOR
    // =========================================================

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node);


    std::thread spinner(
        [&executor]()
        {
            executor.spin();
        });


    // =========================================================
    // WAIT FOR MOVEIT
    // =========================================================

    std::this_thread::sleep_for(2s);


    // =========================================================
    // CREATE PICK PLACE OBJECT
    // =========================================================

    PickPlace pick_place(
        node,
        execute,
        execute_initial,
        execute_pick,
        execute_place);


    // =========================================================
    // RUN
    // =========================================================

    pick_place.run();


    // =========================================================
    // SHUTDOWN
    // =========================================================

    rclcpp::shutdown();

    spinner.join();


    return 0;
}

