#include "MPC.hpp"
#include "math.h"
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
MpcNode::MpcNode(std::shared_ptr<ControlNode> control_node, std::shared_ptr<TFSubscriberNode> tf_subscriber_node)
        : Node("control_node"), control_node_(control_node), tf_subscriber_node_(tf_subscriber_node) {
    // 初始化订阅者
    config_yaml_path = "/root/PersonalData/Program/multi-axle-all-wheel-steering-vehicles_ws/src/control_mpc/config/size.yaml";
    YAML::Node config = YAML::LoadFile(config_yaml_path);
    double T = config["T"].as<double>() * 1000;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(T)), std::bind(&MpcNode::TimerCallback, this));

    // 初始化发布者
    wheel_string_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/string_position_controller/commands", 10);
    wheel_speed_publisher_ =  this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_velocity_controller/commands", 10);

    car_info.L = 8.0665;
    car_info.W = 2.73;
    car_info.wheel_x[0] = 1.33679;
    car_info.wheel_x[1] = 0.29679;
    car_info.wheel_x[2] = -0.74321;
    car_info.wheel_x[3] = -1.78321;
    car_info.wheel_x[4] = -2.82321;
    car_info.wheel_y[0] = 0.845;
    car_info.wheel_y[1] = -0.845;
}

void MpcNode::TimerCallback()
{
    if(control_node_->Trajectory.init_points.cols() == 0)
    {
        // ... (stop logic as in your original code)
        array_msg.data.assign(10, 0.0);
        speed_msg.data.assign(10, 0.0);
        wheel_string_publisher_->publish(array_msg);
        wheel_speed_publisher_->publish(speed_msg);
        RCLCPP_INFO(this->get_logger(), "Stop: control_points.cols()=0");
        return;
    }

    YAML::Node config = YAML::LoadFile(config_yaml_path);
    if(config["stop"].as<bool>() == true)
    {
        // ... (stop logic as in your original code)
        control_node_->control_points.resize(3, 0);
        array_msg.data.assign(10, 0.0);
        speed_msg.data.assign(10, 0.0);
        wheel_string_publisher_->publish(array_msg);
        wheel_speed_publisher_->publish(speed_msg);
        RCLCPP_INFO(this->get_logger(), "Stop: stop_bool from config");
        return;
    }

    Odom_Car_Matrix = tf_subscriber_node_ -> Matrix_Read("car_base", "odom");
    Car_Odom_Matrix = tf_subscriber_node_ -> Matrix_Read("odom", "car_base");

    // ... (Trajectory transformation and MINCO setup as in your original code) ...
    Car_Trajectory_Rotation_Translation_Matrix = Odom_Car_Matrix.Rotation_Translation_Read() * control_node_->Trajectory.Odom_Car_Rotation_Translation_Matrix;
    Car_Trajectory_Rotation_Matrix = Car_Trajectory_Rotation_Translation_Matrix.block(0, 0, 3, 3);
    Eigen::MatrixXd init_points = Eigen::MatrixXd::Zero(4, control_node_->Trajectory.init_points.cols());
    init_points.block(0, 0, 3, control_node_->Trajectory.init_points.cols()) = control_node_->Trajectory.init_points;
    init_points.row(3).setOnes();
    Eigen::MatrixXd Transformed_Point_Map = Car_Trajectory_Rotation_Translation_Matrix * init_points;
    for(int i=0; i<control_node_->Trajectory.init_points.cols() ; i++)
    {
        Eigen::MatrixXd init_yaw = Eigen::MatrixXd::Zero(3, 1);
        double yaw_val = control_node_->Trajectory.init_points(2, i);
        init_yaw << cos(yaw_val), sin(yaw_val), 0;
        Eigen::MatrixXd new_yaw = Car_Trajectory_Rotation_Matrix * init_yaw;
        Transformed_Point_Map(2, i) = atan2(new_yaw(1), new_yaw(0));
    }

    Eigen::Matrix3d initState = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d finalState = Eigen::Matrix3d::Zero();
    initState.col(0) = Transformed_Point_Map.block(0, 0, 3, 1);
    finalState.col(0) = Transformed_Point_Map.block(0, Transformed_Point_Map.cols()-1, 3, 1);
    Eigen::MatrixXd MidPoints = Transformed_Point_Map.block(0, 1, 3, Transformed_Point_Map.cols()-2);
    minco.setConditions(initState, finalState, control_node_->Trajectory.pieceN);
    minco.setParameters(MidPoints, control_node_->Trajectory.times);
    double T_step = config["T"].as<double>(); // MPC time step, T was used for this in first example
    int total_steps_trajectory = (int)(control_node_->Trajectory.T_total / T_step);
    Np = std::min(config["Np"].as<int>(), total_steps_trajectory);
    Nc = std::min(config["Nc"].as<int>(), Np);

    if (Np <= 0 || Nc <= 0) {
        RCLCPP_WARN(this->get_logger(), "Np or Nc is zero or negative. Skipping MPC.");
        // Publish zero commands or maintain last command
        array_msg.data.assign(10, 0.0);
        speed_msg.data.assign(10, 0.0);
        wheel_string_publisher_->publish(array_msg);
        wheel_speed_publisher_->publish(speed_msg);
        return;
    }


    Eigen::MatrixXd control_points_ref = Eigen::MatrixXd::Zero(3, total_steps_trajectory);
    for(int i=0; i<total_steps_trajectory; i++)
    {
        int current_segment;
        double local_t;
        double t_val = i * T_step;
        findCurrentSegmentAndLocalTime(t_val, current_segment, local_t, control_node_->Trajectory.pieceN, control_node_->Trajectory.times);
        Eigen::VectorXd T_Position(6);
        T_Position << 1, local_t, pow(local_t,2), pow(local_t,3), pow(local_t,4), pow(local_t,5);
        Eigen::MatrixXd b_segment = minco.b.block(current_segment * 6, 0, 6, 3).transpose();
        control_points_ref.block(0, i, 3, 1) = b_segment * T_Position;
    }
    speed_ratio = config["speed_ratio"].as<double>();
    
    Eigen::Vector3d P_Now = Eigen::Vector3d::Zero(3); // Current state in vehicle frame (x,y,yaw) = (0,0,0) for MPC
    int min_index = -1;
    for(int i=0; i< control_points_ref.cols(); i++)
    {
        Eigen::Vector3d P_ref = control_points_ref.col(i);
        if(P_ref.x() >= 0) // Find first point ahead or at the vehicle
        {
            min_index = i;
            break;
        }
    }
    if(min_index == -1 && control_points_ref.cols() > 0) min_index = control_points_ref.cols() - 1;
    else if (control_points_ref.cols() == 0) {
        RCLCPP_WARN(this->get_logger(), "Reference control points are empty.");
        return;
    }
    
    double delta_lookahead = config["delta"].as<double>();
    int offset=0;
    while (min_index + offset < control_points_ref.cols() && (control_points_ref.col(min_index + offset) - P_Now).x() < delta_lookahead)
    {
        if(min_index + offset >= control_points_ref.cols() - 1) break;
        offset++;
    }
    
    // Ensure Np points are available for reference
    if (min_index + offset + Np > control_points_ref.cols()) {
        Np = std::max(0, (int)control_points_ref.cols() - (min_index + offset));
        Nc = std::min(Nc, Np);
        if (Np <= 0) {
            RCLCPP_WARN(this->get_logger(), "Not enough reference points for MPC horizon. Np adjusted to %d.", Np);
            // Publish zero commands
            array_msg.data.assign(10, 0.0);
            speed_msg.data.assign(10, 0.0);
            wheel_string_publisher_->publish(array_msg);
            wheel_speed_publisher_->publish(speed_msg);
            return;
        }
    }


    Eigen::MatrixXd Yt = Eigen::MatrixXd::Zero(Np * 3, 1);
    for(int row=0; row<Np; row++)
    {
        if (min_index + offset + row < control_points_ref.cols()) {
            Yt.block(row * 3, 0, 3, 1) = control_points_ref.col(min_index + offset + row);
        } else { // Should not happen if Np is adjusted correctly
            Yt.block(row * 3, 0, 3, 1) = control_points_ref.col(control_points_ref.cols()-1);
        }
    }

    Eigen::MatrixXd A_model(3, 3); A_model.setIdentity();
    Eigen::MatrixXd B_model(3, 3); B_model = T_step * Eigen::MatrixXd::Identity(3, 3);

    Eigen::MatrixXd psi = Eigen::MatrixXd::Zero(Np * A_model.rows(), A_model.cols());
    for(int row=0; row<Np; row++) {
        psi.block(row * A_model.rows(), 0, A_model.rows(), A_model.cols()) = matrixPower(A_model, row + 1); // A, A^2, ..., A^Np
    }
    
    Eigen::MatrixXd Theta = Eigen::MatrixXd::Zero(Np * B_model.rows(), Nc * B_model.cols());
    for (int r = 0; r < Np; ++r) {
        for (int c = 0; c < Nc; ++c) {
            if (r >= c) {
                Theta.block(r * B_model.rows(), c * B_model.cols(), B_model.rows(), B_model.cols()) = matrixPower(A_model, r - c) * B_model;
            }
        }
    }

    Eigen::MatrixXd Chi = P_Now; // Current state (x,y,yaw), which is (0,0,0) in vehicle frame for this formulation

    Eigen::MatrixXd Q_cost(3, 3);
    Q_cost << config["Q_x"].as<double>(), 0, 0,
              0, config["Q_y"].as<double>(), 0,
              0, 0, config["Q_yaw"].as<double>();

    Eigen::MatrixXd R_cost(3, 3);
    R_cost << config["R_vx"].as<double>(), 0, 0,
              0, config["R_vy"].as<double>(), 0,
              0, 0, config["R_w"].as<double>();

    Eigen::MatrixXd E_Np = Eigen::MatrixXd::Identity(Np, Np);
    if (Np > 0) E_Np(Np - 1, Np - 1) = config["Final_Position_Weights"].as<double>();
    Eigen::MatrixXd E_Nc = Eigen::MatrixXd::Identity(Nc, Nc);
    Eigen::MatrixXd QQ = kroneckerProduct(E_Np, Q_cost).eval();
    Eigen::MatrixXd RR = kroneckerProduct(E_Nc, R_cost).eval();

    bool Using_Ulack_Variables = config["Using_Ulack_Variables"].as<bool>();

    Eigen::MatrixXd H_qp;
    Eigen::VectorXd g_qp; // Changed from MatrixXd to VectorXd for gradient

    int num_control_vars_total = Nc * B_model.cols();
    int num_decision_vars_total = num_control_vars_total;

    if(Using_Ulack_Variables) {
        num_decision_vars_total += 1;
        H_qp = Eigen::MatrixXd::Zero(num_decision_vars_total, num_decision_vars_total);
        H_qp(num_control_vars_total, num_control_vars_total) = config["rho"].as<double>(); // Slack variable penalty
        g_qp = Eigen::VectorXd::Zero(num_decision_vars_total); // Slack variable gradient term is 0
    } else {
        H_qp = Eigen::MatrixXd::Zero(num_control_vars_total, num_control_vars_total);
        g_qp = Eigen::VectorXd::Zero(num_control_vars_total);
    }
    H_qp.block(0, 0, num_control_vars_total, num_control_vars_total) = Theta.transpose() * QQ * Theta + RR;
    g_qp.block(0, 0, num_control_vars_total, 1) = Theta.transpose() * QQ * (psi * Chi - Yt);
    
    checkMatrix(H_qp); // Ensure H is suitable (e.g. PSD)
    Eigen::SparseMatrix<double> H_sparse = H_qp.sparseView();

    // ------------------- Control Input and Rate Constraint Setup ------------------- //
    int num_vars_per_step = B_model.cols(); // Should be 3 (vx, vy, omega)
    
    // Number of constraints:
    // 1. Bounds on control inputs u_k: Nc * num_vars_per_step
    // 2. Bounds on control input changes delta_u_k: (Nc - 1) * num_vars_per_step (if Nc > 1)
    int n_constraints_abs = Nc * num_vars_per_step;
    int n_constraints_delta = (Nc > 1) ? (Nc - 1) * num_vars_per_step : 0;
    int n_constraints_for_controls = n_constraints_abs + n_constraints_delta;
    int n_total_osqp_constraints = n_constraints_for_controls;

    std::vector<Eigen::Triplet<double>> constraint_triplets;

    // Control input bounds u_min, u_max (vectors of size 3x1)
    Eigen::VectorXd u_min_vals(num_vars_per_step);
    Eigen::VectorXd u_max_vals(num_vars_per_step);
    u_min_vals << config["vx_min"].as<double>(), config["vy_min"].as<double>(), config["omega_min"].as<double>();
    u_max_vals << config["vx_max"].as<double>(), config["vy_max"].as<double>(), config["omega_max"].as<double>(); // Note: original had omega_miax

    // Control rate bounds delta_u_min, delta_u_max (vectors of size 3x1)
    Eigen::VectorXd delta_u_min_vals(num_vars_per_step);
    Eigen::VectorXd delta_u_max_vals(num_vars_per_step);
    // These are rates (change per T_step), not accelerations directly from config (unless config stores them as rates)
    // The first example: a_min * T. Here, config["delta_vx_min"] is likely the change allowed in one T_step.
    delta_u_min_vals << config["delta_vx_min"].as<double>(), config["delta_vy_min"].as<double>(), config["delta_omega_min"].as<double>();
    delta_u_max_vals << config["delta_vx_max"].as<double>(), config["delta_vy_max"].as<double>(), config["delta_omega_max"].as<double>();

    Eigen::VectorXd lower_bound_osqp(n_total_osqp_constraints);
    Eigen::VectorXd upper_bound_osqp(n_total_osqp_constraints);

    // 1. Control Input Bounds: u_min_vals <= u_k <= u_max_vals
    for (int k = 0; k < Nc; ++k) { // For each control step
        for (int var_idx = 0; var_idx < num_vars_per_step; ++var_idx) { // For vx, vy, omega
            int row_idx = k * num_vars_per_step + var_idx;
            int col_idx = k * num_vars_per_step + var_idx; // Index in the decision variable vector (controls part)
            constraint_triplets.emplace_back(row_idx, col_idx, 1.0);
            lower_bound_osqp(row_idx) = u_min_vals(var_idx);
            upper_bound_osqp(row_idx) = u_max_vals(var_idx);
        }
    }

    // 2. Control Input Rate Bounds: delta_u_min_vals <= u_{k+1} - u_k <= delta_u_max_vals
    int constraint_row_offset = n_constraints_abs;
    if (Nc > 1) {
        for (int k = 0; k < Nc - 1; ++k) { // For each pair (u_k, u_{k+1})
            for (int var_idx = 0; var_idx < num_vars_per_step; ++var_idx) {
                int row_idx = constraint_row_offset + k * num_vars_per_step + var_idx;
                // For u_{k+1}
                constraint_triplets.emplace_back(row_idx, (k + 1) * num_vars_per_step + var_idx, 1.0);
                // For -u_k
                constraint_triplets.emplace_back(row_idx, k * num_vars_per_step + var_idx, -1.0);
                
                lower_bound_osqp(row_idx) = delta_u_min_vals(var_idx); // Assumes these are delta per T_step
                upper_bound_osqp(row_idx) = delta_u_max_vals(var_idx); // Assumes these are delta per T_step
            }
        }
    }
    
    Eigen::SparseMatrix<double> A_constraints_osqp(n_constraints_for_controls, num_decision_vars_total); // num_decision_vars_total includes slack col if used
    A_constraints_osqp.setFromTriplets(constraint_triplets.begin(), constraint_triplets.end());


    // Handle slack variable constraint if used (e.g., epsilon >= 0)
    if (Using_Ulack_Variables) {
        n_total_osqp_constraints += 1; // One more constraint for slack
        Eigen::SparseMatrix<double> temp_A = A_constraints_osqp; // Copy previous
        A_constraints_osqp.resize(n_total_osqp_constraints, num_decision_vars_total); // Resize, old values kept
        
        // Add constraint for slack: slack_var >= 0 (or other bounds if needed)
        A_constraints_osqp.insert(n_constraints_for_controls, num_decision_vars_total - 1) = 1.0; // Coeff for slack var

        Eigen::VectorXd temp_lower = lower_bound_osqp;
        Eigen::VectorXd temp_upper = upper_bound_osqp;
        lower_bound_osqp.resize(n_total_osqp_constraints);
        upper_bound_osqp.resize(n_total_osqp_constraints);
        lower_bound_osqp.head(n_constraints_for_controls) = temp_lower;
        upper_bound_osqp.head(n_constraints_for_controls) = temp_upper;

        lower_bound_osqp(n_constraints_for_controls) = 0.0; // Slack >= 0
        upper_bound_osqp(n_constraints_for_controls) = OsqpEigen::INFTY; // No upper bound
    }

    // OSQP Solver Setup
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setAlpha(1.0); // Common setting from example 1, OSQP default is 1.6

    solver.data()->setNumberOfVariables(num_decision_vars_total);
    solver.data()->setNumberOfConstraints(n_total_osqp_constraints);

    if (!solver.data()->setHessianMatrix(H_sparse)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set OSQP Hessian matrix"); return;
    }
    if (!solver.data()->setGradient(g_qp)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set OSQP gradient vector"); return;
    }
    if (!solver.data()->setLinearConstraintsMatrix(A_constraints_osqp)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set OSQP linear constraints matrix"); return;
    }
    if (!solver.data()->setLowerBound(lower_bound_osqp)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set OSQP lower bound vector"); return;
    }
    if (!solver.data()->setUpperBound(upper_bound_osqp)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set OSQP upper bound vector"); return;
    }

    if (!solver.initSolver()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize OSQP solver");
        return;
    }
    
    OsqpEigen::ErrorExitFlag solve_status = solver.solveProblem();
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        RCLCPP_WARN(this->get_logger(), "OSQP solver failed. Status: %d", static_cast<int>(solve_status));
        return;
    }
    Eigen::VectorXd solution_ref = solver.getSolution();
    Eigen::Vector3d control_output = solution_ref.head(3);

    RCLCPP_INFO(this->get_logger(), "Control output: vx=%.2f, vy=%.2f, w=%.2f", control_output(0), control_output(1), control_output(2));
    Control(control_output);
}
void MpcNode::findCurrentSegmentAndLocalTime(double t, int & segment, double & local_t, int pieceN, Eigen::VectorXd times)
{
    int left = 0;
    int right = pieceN - 1;
    double total_time = 0.0;

    // 二分查找
    while (left <= right) {
        int mid = left + (right - left) / 2;
        double segment_end_time = 0.0;

        // 计算到mid段为止的总时间
        for (int i = 0; i <= mid; i++) {
            segment_end_time += times[i];
        }

        if (t < segment_end_time - times[mid]) {
            right = mid - 1;
        } else if (t > segment_end_time) {
            left = mid + 1;
        } else {
            segment = mid;
            total_time = segment_end_time - times[mid];
            break;
        }
    }

    // 如果时间超过了所有轨迹段的总时间，则返回最后一个段
    if (left > right) {
        segment = pieceN - 1;
        total_time = 0.0;
        for (int i = 0; i < pieceN - 1; i++) {
            total_time += times[i];
        }
    }
    // 计算局部时间（相对于当前轨迹段的时间）
    local_t = t - total_time;
}

void MpcNode::Control(Eigen::Vector3d Control)
{
    double velocity_x = Control(0);
    double velocity_y = Control(1);
    double omega = Control(2);

    for(int i=0; i<5; i++)
    {
        double Y_LR = omega * car_info.wheel_x[i]  + velocity_y;
        double X_L = -omega * car_info.wheel_y[0] + velocity_x;
        double X_R = omega * car_info.wheel_y[1] + velocity_x;
        calculateWheelSettings(X_L, Y_LR, car_info.gamma_l[i], car_info.V_l[i]);
        calculateWheelSettings(X_R, Y_LR, car_info.gamma_r[i], car_info.V_r[i]);
        car_info.V_l[i] *= -speed_ratio; 
        car_info.V_r[i] *= -speed_ratio; 
    }
    array_msg.data.resize(10, 0.0);
    array_msg.data = {  car_info.gamma_r[0],
                        car_info.gamma_r[1],
                        car_info.gamma_r[2],
                        car_info.gamma_r[3],
                        car_info.gamma_r[4],
                        car_info.gamma_l[0],
                        car_info.gamma_l[1],
                        car_info.gamma_l[2],
                        car_info.gamma_l[3],
                        car_info.gamma_l[4]};

    speed_msg.data.resize(10, 0.0);
    speed_msg.data = {car_info.V_r[0], car_info.V_r[1], car_info.V_r[2], car_info.V_r[3], car_info.V_r[4],car_info.V_l[0], car_info.V_l[1], car_info.V_l[2], car_info.V_l[3], car_info.V_l[4]};

    wheel_string_publisher_->publish(array_msg);
    wheel_speed_publisher_->publish(speed_msg);
}
void MpcNode::calculateWheelSettings(double vx, double vy, double& angle, double& speed)
{
    speed = -sqrt(vx * vx + vy * vy);
    angle = atan2(vy, vx);
    if (angle > M_PI / 2)         angle -= M_PI;
    else if (angle < -M_PI / 2)   angle += M_PI;
    if (vx < 0)             speed = -speed;
}

// 计算矩阵的幂
Eigen::MatrixXd MpcNode::matrixPower(const Eigen::MatrixXd& matrix, int power)
{
    if (matrix.rows() != matrix.cols()) {
        throw std::runtime_error("Matrix must be square to compute power.");
    }

    Eigen::MatrixXd result = Eigen::MatrixXd::Identity(matrix.rows(), matrix.cols());

    for (int i = 0; i < power; i++) {
        result *= matrix;
    }

    return result;
}

// 计算矩阵的克罗内克积
Eigen::MatrixXd MpcNode::kroneckerProduct(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{
    Eigen::MatrixXd result(A.rows() * B.rows(), A.cols() * B.cols());

    for (int i = 0; i < A.rows(); ++i) {
        for (int j = 0; j < A.cols(); ++j) {
            result.block(i * B.rows(), j * B.cols(), B.rows(), B.cols()) = A(i, j) * B;
        }
    }

    return result;
}

//检查矩阵类型（正定、半正定、负定、不定）
void MpcNode::checkMatrix(Eigen::MatrixXd& eigenH)
{

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(eigenH);

    if (eigenSolver.info() != Eigen::Success) {
        std::cerr << "Matrix decomposition failed." << std::endl;
        isSymmetric(eigenH);
        return;
    }

    Eigen::VectorXd eigenvalues = eigenSolver.eigenvalues();

    bool isPositiveDefinite = (eigenvalues.array() > 0).all();
    bool isPositiveSemidefinite = (eigenvalues.array() >= 0).all();
    bool isNegativeDefinite = (eigenvalues.array() < 0).all();
    bool isNegativeSemidefinite = (eigenvalues.array() <= 0).all();

    if (isPositiveDefinite) {
        std::cout << "The matrix is positive definite." << std::endl;
    } else if (isPositiveSemidefinite) {
        std::cout << "The matrix is positive semidefinite." << std::endl;
    } else if (isNegativeDefinite) {
        std::cout << "The matrix is negative definite." << std::endl;
    } else if (isNegativeSemidefinite) {
        std::cout << "The matrix is negative semidefinite." << std::endl;
    } else {
        std::cout << "The matrix is indefinite." << std::endl;
        // 检查并调整 H 矩阵的正定性
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(eigenH);
        Eigen::VectorXd eigenvalues = eigenSolver.eigenvalues();
        double minEigenvalue = eigenvalues.minCoeff();
        if (minEigenvalue <= 0) {
            eigenH += (-minEigenvalue + 1e-4) * Eigen::MatrixXd::Identity(eigenH.rows(), eigenH.cols());
        }
        std::cout << "The matrix has been made definite." << std::endl;
        bool isPositiveDefinite = (eigenvalues.array() > 0).all();
        if (isPositiveDefinite) {
            std::cout << "The matrix is positive definite." << std::endl;
        } else {
            std::cout << "The matrix is still indefinite." << std::endl;
        }
    }
}

// 检查是否为方阵
void MpcNode::isSymmetric(Eigen::MatrixXd& matrix)
{
    if (matrix.rows() != matrix.cols()) {
        std::cout << "The matrix is not square." << std::endl;
    } else {
        std::cout << "The matrix is square." << std::endl;
    }
    // 检查矩阵是否等于其转置
    if (matrix.isApprox(matrix.transpose())) {
        std::cout << "The matrix is equal to its transpose." << std::endl;
    } else {
        std::cout << "The matrix is not equal to its transpose." << std::endl;
        // 使矩阵对称
        matrix = (matrix + matrix.transpose()) / 2.0;
        std::cout << "The matrix has been made symmetric." << std::endl;
        // 再次检查对称性
        if (matrix.isApprox(matrix.transpose())) {
            std::cout << "The symmetrized matrix is now equal to its transpose." << std::endl;
        } else {
            std::cout << "The symmetrized matrix is still not exactly equal to its transpose due to floating-point precision." << std::endl;
        }
    }
}
