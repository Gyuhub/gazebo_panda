#include "hqp_controller/hqp_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace hqp_controller
{

HQPController::HQPController()
: controller_interface::ControllerInterface(), //forward_command_controller::ForwardCommandController()
  rt_command_ptr_(nullptr),
  command_subscriber_(nullptr)
{
}

controller_interface::return_type HQPController::init(
    const std::string & controller_name)
{
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK)
    {
        return ret;
    }

    try
    {
        auto_declare<std::vector<std::string>>("joints", joint_names_);
        auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

        auto_declare<std::string>("interface_name", "");
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::return_type::ERROR;
    }

    initialize(); // initialize the kinematics dynamics model

    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
HQPController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interfaces_config.names.reserve(joint_names_.size() * command_interface_types_.size());
    for (const auto &joint_name : joint_names_)
    {
        for (const auto &interface_type : command_interface_types_)
        {
            command_interfaces_config.names.push_back(joint_name + "/" + interface_type);
        }
    }
    // for (const auto &joint : joint_names_)
    // {
    //     command_interfaces_config.names.push_back(joint + "/" + interface_name_);
    // }

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
HQPController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names.reserve(joint_names_.size() * state_interface_types_.size());
    for (const auto &joint_name : joint_names_)
    {
        for (const auto &interface_type : state_interface_types_)
        {
            state_interfaces_config.names.push_back(joint_name + "/" + interface_type);
        }
    }
    return state_interfaces_config;
}

template <typename T>
bool get_ordered_interfaces(
    std::vector<T> &unordered_interfaces, const std::vector<std::string> &joint_names,
    const std::string &interface_type, std::vector<std::reference_wrapper<T>> &ordered_interfaces)
{
    for (const auto &joint_name : joint_names)
    {
        for (auto &interface : unordered_interfaces)
        {
            if (
                (interface.get_name() == joint_name) && (interface.get_interface_name() == interface_type))
            {
                ordered_interfaces.emplace_back(std::ref(interface));
            }
        }
    }

    return joint_names.size() == ordered_interfaces.size();
}

CallbackReturn HQPController::on_configure(
    const rclcpp_lifecycle::State & /* previous_state */)
{
    joint_names_ = node_->get_parameter("joints").as_string_array();
    auto logger = get_node()->get_logger();
    if (joint_names_.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
        return CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < joint_names_.size(); i++) RCLCPP_INFO(logger, "joint[%d]: %s", i, joint_names_[i].c_str());
    // Specialized, child controllers set interfaces before calling configure function.
    if (interface_name_.empty())
    {
        interface_name_ = node_->get_parameter("interface_name").as_string();
    }

    if (interface_name_.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
        return CallbackReturn::ERROR;
    }

    // Specialized, child controllers set interfaces before calling configure function.
    if (command_interface_types_.empty())
    {
        command_interface_types_ = node_->get_parameter("command_interfaces").as_string_array();
    }

    if (command_interface_types_.empty())
    {
        RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty.");
        return CallbackReturn::FAILURE;
    }
    for (size_t i = 0; i < command_interface_types_.size(); i++) RCLCPP_INFO(logger, "command_interface_types[%d]: %s", i, command_interface_types_[i].c_str());
    state_interface_types_ = node_->get_parameter("state_interfaces").as_string_array();

    if (state_interface_types_.empty())
    {
        RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
        return CallbackReturn::FAILURE;
    }
    for (size_t i = 0; i < state_interface_types_.size(); i++) RCLCPP_INFO(logger, "state_interface_types[%d]: %s", i, state_interface_types_[i].c_str());

    joint_state_interface_.resize(2);
    joint_command_interface_.resize(1);

    command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/commands",
        rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        { 
            for (int i = 0; i < 6; i++) x_goal_(i) = msg.get()->data[i];
            xdot_goal_.setZero();
            RCLCPP_INFO(get_node()->get_logger(), "Received command: %f, %f, %f, %f, %f, %f",
                        x_goal_(0),
                        x_goal_(1),
                        x_goal_(2),
                        x_goal_(3),
                        x_goal_(4),
                        x_goal_(5));
            rt_command_ptr_.writeFromNonRT(msg); });

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
}

CallbackReturn HQPController::on_activate(
    const rclcpp_lifecycle::State & /* previous_state */)
{
    // order all joints in the storage
    for (const auto &interface : command_interface_types_)
    {
        if (!get_ordered_interfaces(
                command_interfaces_, joint_names_, interface, joint_command_interface_[0]))
        {
            RCLCPP_ERROR(
                node_->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", joint_names_.size(),
                interface.c_str(), joint_command_interface_[0].size());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
    }
    for (const auto &interface : state_interface_types_)
    {
        auto it =
            std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
        auto index = std::distance(allowed_interface_types_.begin(), it);
        if (!get_ordered_interfaces(
                state_interfaces_, joint_names_, interface, joint_state_interface_[index]))
        {
            RCLCPP_ERROR(
                node_->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", joint_names_.size(),
                interface.c_str(), joint_state_interface_[index].size());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
    }

    // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
    // if (
    //     !get_ordered_interfaces(
    //         command_interfaces_, joint_names_, interface_name_, ordered_interfaces) ||
    //     command_interfaces_.size() != ordered_interfaces.size())
    // {
    //     RCLCPP_ERROR(
    //         node_->get_logger(), "Expected %zu position command interfaces, got %zu", joint_names_.size(),
    //         ordered_interfaces.size());
    //     return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    // }

    // reset command buffer if a command came through callback when controller was inactive
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64MultiArray>>(nullptr);

    return CallbackReturn::SUCCESS;
}

CallbackReturn HQPController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    joint_command_interface_[0].clear();
    joint_state_interface_[0].clear();

    release_interfaces();
    // reset command buffer
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64MultiArray>>(nullptr);
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type HQPController::update()
{
    time_ = time_ + dt_;
    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        return controller_interface::return_type::OK;
    }
    for (int i = 0; i < dofs_; i++)
    {
        
        q_[i] = joint_state_interface_[0][i].get().get_value();
        qdot_[i] = joint_state_interface_[1][i].get().get_value();
    }
    // RCLCPP_INFO(get_node()->get_logger(), "joint angle: %f, %f, %f, %f, %f, %f, %f",
    //             q_[0],
    //             q_[1],
    //             q_[2],
    //             q_[3],
    //             q_[4],
    //             q_[5],
    //             q_[6]);
    auto commands = rt_command_ptr_.readFromRT();

    if (!commands || !(*commands))
    {
        return controller_interface::return_type::OK;
    }
    // if ((*joint_commands)->data.size() != command_interfaces_.size())
    // {
    //     RCLCPP_ERROR_THROTTLE(
    //     get_node()->get_logger(), *node_->get_clock(), 1000,
    //     "command size (%zu) does not match number of interfaces (%zu)",
    //     (*joint_commands)->data.size(), command_interfaces_.size());
    //     return controller_interface::return_type::ERROR;
    // }

    // for (auto index = 0ul; index < command_interfaces_.size(); ++index)
    // {
    //     command_interfaces_[index].set_value((*joint_commands)->data[index]);
    // }

    /**
     * @brief update functions
     * @todo get model and calculate control command from joint_commands
     */

    update_model();
    plan_trajectory();
    calculate_command();
    set_command();
    
    return controller_interface::return_type::OK;
}

void HQPController::initialize()
{
    hqp_p1_.initialize();
    hqp_p2_.initialize();
    
    time_ = 0.0;
    time_pre_ = 0.0;
    dt_ = 0.001;

    kpj_ = 400.0;
	kdj_ = 40.0;
	kp_ = 400.0;
	kd_ = 40.0;

    // RBDL
    model_.get_model();
    RCLCPP_INFO(get_node()->get_logger(), "dof: %d", dofs_);
    dofs_ = model_.get_dofs();
    model_.initialize();
    RCLCPP_INFO(get_node()->get_logger(), "dof: %d", dofs_);
    trajectory_.initialize(dofs_, dt_);

    q_.setZero(dofs_);
    qdot_.setZero(dofs_);
    tau_.setZero(dofs_);
    qpos_.setZero(dofs_);

    x_.setZero(6);
    xdot_.setZero(6);

    q_des_.setZero(dofs_);
    qdot_des_.setZero(dofs_);
    q_goal_.setZero(dofs_);
    qdot_goal_.setZero(dofs_);
    xddot_star_.setZero(6);
    x_des_.setZero(6);
    xdot_des_.setZero(6);
    x_goal_.setZero(6);
    xdot_goal_.setZero(6);
    x_err_.setZero(6);
    xdot_err_.setZero(6);

    q_goal_(0) = 0.0 * DEG2RAD;
    q_goal_(1) = 0.0 * DEG2RAD;
    q_goal_(2) = 0.0 * DEG2RAD;
    q_goal_(3) = -90.0 * DEG2RAD;
    q_goal_(4) = 0.0 * DEG2RAD;
    q_goal_(5) = 90.0 * DEG2RAD;
    q_goal_(6) = 45.0 * DEG2RAD;
    q_goal_(7) = 0.0;
    q_goal_(8) = 0.0;
    qpos_ = q_goal_;

    pos_err_.setZero();
    posdot_err_.setZero();
    ori_err_.setZero();
    oridot_err_.setZero();
    xddot_ref_.setZero(6);
    qdot_ref_.setZero(dofs_);
    lambda_.setZero(6, 6);
    null_space_projection_.setZero(6, 6);

    J_.setZero(6, dofs_);
    J_T_.setZero(dofs_, 6);
    J_des_.setZero(6, dofs_);
    J_T_des_.setZero(dofs_, 6);
    Jdot_qdot_.setZero(6);
    T_.setZero(6, 6);
    J_A_.setZero(6, dofs_);
    J_T_A_.setZero(dofs_, 6);

    R_.setZero();
    R_des_.setZero();
    Rdot_des_.setZero();
    I_dofs_.setZero(dofs_, dofs_);
    I_task_.setZero(6, 6);
    I_3_.setIdentity();
    O_3_.setZero();

    // HQP
    hqp_p1_.initialize_problem_size(20, 13); //variable size = (joint dof)*2+(task dof), constraint size = (joint dof) + (task dof) 
	H1_.setZero(hqp_p1_.num_var_, hqp_p1_.num_var_);
	g1_.setZero(hqp_p1_.num_var_);
	A1_.setZero(hqp_p1_.num_cons_, hqp_p1_.num_var_);
	lbA1_.setZero(hqp_p1_.num_cons_);
	ubA1_.setZero(hqp_p1_.num_cons_);
	lb1_.setZero(hqp_p1_.num_var_);
	ub1_.setZero(hqp_p1_.num_var_);
	hqp_p2_.initialize_problem_size(23, 20); //variable size = (joint dof)*2+(task dof), constraint size = (joint dof) + (1st prioirty task dof)  + (2nd prioirty task dof) 
	H2_.setZero(hqp_p2_.num_var_, hqp_p2_.num_var_);
	g2_.setZero(hqp_p2_.num_var_);
	A2_.setZero(hqp_p2_.num_cons_, hqp_p2_.num_var_);
	lbA2_.setZero(hqp_p2_.num_cons_);
	ubA2_.setZero(hqp_p2_.num_cons_);
	lb2_.setZero(hqp_p2_.num_var_);
	ub2_.setZero(hqp_p2_.num_var_);

    // Trajectory
    duration_ = 5.0;

    return;
}

void HQPController::update_model()
{
    model_.update_kinematics(q_, qdot_); // TODO: need to get current value of each joint position and velocity (q_, qdot_)
    model_.update_dynamics();
    model_.get_Jacobian();
    model_.get_state();
    J_ = model_.J_;
    J_T_ = J_.transpose();

    x_.head(3) = model_.pos_;
    x_.tail(3) = model_.ori_;
    R_ = model_.R_;

    xdot_.head(3) = model_.posdot_;
    xdot_.tail(3) = model_.oridot_;

    J_A_ = Math::get_analytic_from_geometric_Jacobian(J_, I_3_, O_3_, Math::get_transformation_matrix_from_euler_xyz(x_.tail(3)));
    return;
}

void HQPController::plan_trajectory()
{
    if (trajectory_.is_traj_finished())
    {
        trajectory_.check_size(x_);
        trajectory_.set_start(x_, xdot_, time_); // TODO: set time_
        trajectory_.set_goal(x_goal_, xdot_goal_, time_ + duration_); // TODO: set x_goal, xdot_goal
    }
    trajectory_.update_time(time_);
    x_des_ = trajectory_.get_position_trajectory();
    xdot_des_ = trajectory_.get_orientation_trajectory();
}

void HQPController::calculate_command()
{
    tau_.setZero();

    // TODO: get desired X, Xdot (x_des_, xdot_des_ (6x1))
    kp_ = 400.0;
	kd_ = 20.0;

	x_err_.head(3) = x_des_.head(3) - x_.head(3);
	R_des_ = Math::get_body_rotation_matrix(x_des_(3), x_des_(4), x_des_(5));	
	x_err_.tail(3) = Math::calc_rotation_error(R_, R_des_);

	xdot_err_.head(3) = xdot_des_.head(3) - xdot_.head(3);
	xdot_err_.tail(3) = -xdot_.tail(3); //only damping for orientation	

	xddot_star_.segment(0, 3) = kp_ * x_err_.head(3) + kd_ * xdot_err_.head(3);// position control
	xddot_star_.segment(3, 3) = kp_ * x_err_.tail(3) + kd_ * xdot_err_.tail(3);// orientation control

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Solve rHQP   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	double threshold = 0.001;
	int max_iter = 1000;
	// first priority task QP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// set cost function x^T*H*x + g
	H1_.setZero();
	for (int i = 0; i < DOFS; i++) // torque
	{
		H1_(i, i) = 1.0;
	}
	for (int i = DOFS; i < DOFS + 6; i++) // slack
	{
		H1_(i, i) = 1000000.0;
	}
	g1_.setZero();
	hqp_p1_.update_min_problem(H1_, g1_);

	Eigen::MatrixXd J_Ainv(6, DOFS);
	J_Ainv = J_ * model_.A_.inverse();

	// set A*x <= b
	A1_.setZero();
	lbA1_.setZero();
	ubA1_.setZero();
	A1_.block<6, DOFS>(0, 0) = J_Ainv;
	A1_.block<6, 6>(0, DOFS) = -I_task_;

	for (int i = 0; i < 6; i++)
	{
		lbA1_(i) = -Jdot_qdot_(i) + xddot_star_(i) - threshold;
		ubA1_(i) = -Jdot_qdot_(i) + xddot_star_(i) + threshold;
	}
	hqp_p1_.update_subject_to_Ax(A1_, lbA1_, ubA1_);

	// set lb <= x <= ub
	lb1_.setZero();
	ub1_.setZero();
	// joint torque limit
	for (int i = 0; i < DOFS; i++)
	{
		// torque limit
		//lb1_(i) = Model._min_joint_torque(i) - Model._bg(i);
		//ub1_(i) = Model._max_joint_torque(i) - Model._bg(i);
		lb1_(i) = model_.min_joint_torque_(i);
		ub1_(i) = model_.max_joint_torque_(i);
	}
	lb1_(7) = 0.0 - threshold;
	ub1_(7) = 0.0 + threshold;
	lb1_(8) = 0.0 - threshold;
	ub1_(8) = 0.0 + threshold;
	// NOTE: verify an effect of decreasing the wrist joint inequality constraints
	// lb1_(6) = -1.0-Model._bg(6);
	// ub1_(6) = 1.0-Model._bg(6);
	// lb1_(13) = -1.0-Model._bg(13);
	// ub1_(13) = 1.0-Model._bg(13);
	// task limit
	for (int i = 0; i < 6; i++)
	{
		lb1_(i + DOFS) = -1000000.0;
		ub1_(i + DOFS) = 1000000.0;
	}
	hqp_p1_.update_subject_to_X(lb1_, ub1_);

	// Solve
	hqp_p1_.enable_equality_condition(0.0001);
	hqp_p1_.solve_qpOASES(max_iter);
	//_torque = hqp_p1_._Xopt.segment(0, 15) + Model._bg;

	// second priority task QP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// set cost function x^T*H*x + g
	H2_.setZero();
	for (int i = 0; i < DOFS; i++) // torque
	{
		H2_(i, i) = 1.0;
	}
	for (int i = DOFS; i < DOFS + 6 + 3; i++) // slack
	{
		H2_(i, i) = 1000000.0;
	}
	H2_(7, 7) = 0.0001;
	H2_(8, 8) = 0.0001;
	//H2_.block<15, 15>(0, 0) = Model._A.inverse();
	//H2_.block<15, 15>(15, 15) = Model._A;
	g2_.setZero();
	hqp_p2_.update_min_problem(H2_, g2_);

	// set A*x <= b
	A2_.setZero();
	lbA2_.setZero();
	ubA2_.setZero();
	A2_.block<DOFS, DOFS>(0, 0) = model_.A_.inverse();
	A2_.block<DOFS, DOFS>(0, DOFS) = -I_dofs_;
	A2_.block<6, DOFS>(DOFS, 0) = J_Ainv;

	Eigen::VectorXd joint_acc_des(DOFS);
	joint_acc_des.setZero();
	kdj_ = 20.0;
	kpj_ = 100.0;
	joint_acc_des = kpj_ * ((model_.max_joint_position_ + model_.min_joint_position_) / 2.0 - q_) - kdj_ * qdot_;
	// joint_acc_des = -_kdj * _qdot;
	joint_acc_des(7) = 0.0;
	joint_acc_des(8) = 0.0;
	// joint_acc_des(0) = 400.0 * (_q_home(0)- _q(0)) - _kdj * _qdot(0);

	for (int i = 0; i < DOFS; i++)
	{
		lbA2_(i) = joint_acc_des(i) - threshold;
		ubA2_(i) = joint_acc_des(i) + threshold;
	}
	for (int i = 0; i < 6; i++)
	{
		lbA2_(i + DOFS) = -Jdot_qdot_(i) + xddot_star_(i) + hqp_p1_.X_opt_(i + DOFS) - threshold;
		ubA2_(i + DOFS) = -Jdot_qdot_(i) + xddot_star_(i) + hqp_p1_.X_opt_(i + DOFS) + threshold;
	}
	hqp_p2_.update_subject_to_Ax(A2_, lbA2_, ubA2_);

	// set lb <= x <= ub
	lb2_.setZero();
	ub2_.setZero();

	// joint torque limit
	for (int i = 0; i < DOFS; i++)
	{
		//lb2_(i) = Model._min_joint_torque(i) - Model._bg(i);
		//ub2_(i) = Model._max_joint_torque(i) - Model._bg(i);
		lb2_(i) = model_.min_joint_torque_(i) * 2.0;
		ub2_(i) = model_.max_joint_torque_(i) * 2.0;
	}
	lb2_(7) = 0.0 - threshold;
	ub2_(7) = 0.0 + threshold;
	lb2_(8) = 0.0 - threshold;
	ub2_(8) = 0.0 + threshold;
	// NOTE: verify an effect of decreasing the wrist joint inequality constraints
	// lb2_(6) = -1.0-Model._bg(6);
	// ub2_(6) = 1.0-Model._bg(6);
	// lb2_(13) = -1.0-Model._bg(13);
	// ub2_(13) = 1.0-Model._bg(13);
	// task limit
	for (int i = 0; i < DOFS; i++)
	{
		lb2_(i + DOFS) = -1000000.0;
		ub2_(i + DOFS) = 1000000.0;
	}
	hqp_p2_.update_subject_to_X(lb2_, ub2_);

	// Solve
	// hqp_p2_.EnableEqualityCondition(0.0001);
	hqp_p2_.solve_qpOASES(max_iter);

	if (hqp_p1_.num_state_ == 0 || hqp_p2_.num_state_ == 0)
	{
		tau_ = hqp_p2_.X_opt_.segment(0, DOFS) + model_.bg_;
		// cout << "------------- torque ---------------------\n" << hqp_p2_._Xopt.segment(0, 15). transpose() + Model._bg.transpose() << endl;
	}
	else // when solving HQP failed
	{
		std::cout << "Fault: Cannot solve QP!!" << '\n';
	}
}

void HQPController::set_command()
{
    for (auto index = 0ul; index < command_interfaces_.size(); ++index)
    {
        command_interfaces_[index].set_value(tau_[index]);
    }
    return;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  hqp_controller::HQPController, controller_interface::ControllerInterface)