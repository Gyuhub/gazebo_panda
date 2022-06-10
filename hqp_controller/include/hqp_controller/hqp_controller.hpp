#pragma once
#ifndef __HQP_CONTROLLER_HPP__
#define __HQP_CONTROLLER_HPP__

// C++ standard
#include <memory>
#include <string>
#include <vector>

// 3rd-party dependencies
#include "model.h"
#include "control_math.h"
#include "quadratic_programming.h"
#include "trajectory.h"

// ROS2 common
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"

// ROS2 control
// #include "forward_command_controller/forward_command_controller.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "visibility_control.h"

// ROS2 messagee
#include "std_msgs/msg/float64_multi_array.hpp"

namespace hqp_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HQPController : public controller_interface::ControllerInterface
{
public:
    HQP_CONTROLLER_PUBLIC
    HQPController();

    HQP_CONTROLLER_PUBLIC
    controller_interface::return_type init(const std::string & controller_name) override;

    HQP_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    HQP_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    HQP_CONTROLLER_PUBLIC
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    HQP_CONTROLLER_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    HQP_CONTROLLER_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    HQP_CONTROLLER_PUBLIC
    controller_interface::return_type update() override;

protected:
    std::vector<std::string> joint_names_;
    std::vector<std::string> command_interface_types_;
    std::vector<std::string> state_interface_types_;
    std::string interface_name_;

    // To reduce number of variables and to make the code shorter the interfaces are ordered in types
    // as the following constants
    const std::vector<std::string> allowed_interface_types_ = {
        hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_ACCELERATION,
        hardware_interface::HW_IF_EFFORT,
    };

    template <typename T>
    using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

    InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
    InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

    realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64MultiArray>> rt_command_ptr_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscriber_;

    std::string logger_name_;

protected:
    void initialize();
    void update_model();
    void plan_trajectory();
    void calculate_command();
    void set_command();

private:
    // RBDL
    Model model_;
    Eigen::VectorXd q_, qdot_; // angle and angular velocities of joints of the panda
    Eigen::VectorXd tau_, qpos_; // torque of joints of the panda
    Eigen::VectorXd x_, xdot_; // poses and velocities of end-effector of the panda

    Eigen::VectorXd q_des_, qdot_des_; // desired joint space angle and joint angular velocity of panda
    Eigen::VectorXd q_goal_, qdot_goal_; // goal joint space angle and joint angular velocity of panda
    Eigen::VectorXd xddot_star_; // desired task space position and orientation of panda
    Eigen::VectorXd x_des_, xdot_des_; // desired task space position and orientation of panda
    Eigen::VectorXd x_goal_, xdot_goal_; // goal position and orientation which is obtained from the planner
    Eigen::VectorXd x_err_, xdot_err_; // error of the position and orientation

    Eigen::Vector3d pos_err_, posdot_err_;
    Eigen::Vector3d ori_err_, oridot_err_;
    Eigen::VectorXd xddot_ref_, qdot_ref_;
    Eigen::MatrixXd lambda_, null_space_projection_;

    Eigen::MatrixXd J_, J_T_, J_des_, J_T_des_; // jacobian, jacobian transpose matrices of the panda
    Eigen::VectorXd Jdot_qdot_; // Xdot = Jdot * qdot matrices of the panda
    Eigen::MatrixXd T_, J_A_, J_T_A_; // transformation matrix from geometric to analytic jacobian, analytic jacobian and jacobian transpose matices
    Eigen::Matrix3d R_, R_des_, Rdot_des_; // rotation matrix(current, desired, dot desired) of end-effector of the panda
    Eigen::MatrixXd I_dofs_, I_task_; // identity matrix DoFs x DoFs and DoFs of task x DoFs of task
    Eigen::Matrix3d I_3_, O_3_; // identity matrix 3x3 and zero matrix 3x3

    // HQP
    QuadraticProgram hqp_p1_, hqp_p2_;
    //first priority task - franka control, second priority task - joint damping
    Eigen::MatrixXd H1_, H2_, A1_, A2_;
	Eigen::VectorXd g1_, g2_, lbA1_, lbA2_, ubA1_, ubA2_, lb1_, lb2_, ub1_, ub2_;

    // Trajectory
    Trajectory trajectory_;

private:
    bool is_target_reached_; // true when the command and the state of robot is same
    double time_, time_pre_; // current time which is obtained from the mujoco simulator
    double duration_; // time of trajectory interpolation
    double dt_;
    int dofs_; // degrees of the freedom
    double kpj_; //joint control P gain
	double kdj_; //joint control D gain	
	double kp_; //Operational space control P gain
	double kd_; //Operational space control D gain	
};

} // namespace hqp_controller


#endif