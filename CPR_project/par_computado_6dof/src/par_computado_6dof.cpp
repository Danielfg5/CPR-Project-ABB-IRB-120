#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include "my_custom_msgs/SixJointsValues.h"
#include <time.h>
#include <string>
#include <cmath>
#include <Eigen/Core>
#include <kdl/kdl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames_io.hpp>

#include <moveit_msgs/DisplayTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>

#include "actionlib/server/simple_action_server.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryAction.h>

#include <trajectory_interface/trajectory_interface.h>

#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <joint_trajectory_controller/hardware_interface_adapter.h>

#define N_DOF 6

namespace par_computado_6dof_ns {

    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>         ActionServer;
    typedef boost::shared_ptr<ActionServer>                                            ActionServerPtr;
    typedef ActionServer::GoalHandle                                                   GoalHandle;

    class ParComputado6Dof : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) {

            /* --- Initialize controller matrices to zero --- */
            Kp.setZero();
            Ki.setZero();
            Kd.setZero();
            M_.setZero();
            V_.setZero();
            G_.setZero();
            F_.setZero();

            // Load robot model from the parameter server
            std::string robot_description;

            if(!n.getParam("/robot_description", robot_description)){
                ROS_ERROR("Failed to get robot description.");
                return false;
            }

            
            if (!kdl_parser::treeFromString(robot_description, kdl_tree)) {
                ROS_ERROR("Failed to construct KDL tree from URDF");
                return false;
            }

            // Specify the chain you want to work with
            std::string root_link = "base_link";
            std::string tip_link = "link_6";
            
            if (!kdl_tree.getChain(root_link, tip_link, kdl_chain)) {
                ROS_ERROR("Failed to get KDL chain");
                return false;
            }

            // Create a KDL chainDynParam solver
            KDL::ChainDynParam dyn_param_solver(kdl_chain, KDL::Vector(0, 0, -g));

            /* --- Load parameters from the parameter server --- */
            if(!n.getParam("joints", joint_names_)){
                ROS_ERROR("Failed to get joints parameter.");
                return false;
            }
            if(joint_names_.size() != N_DOF)
            {
                ROS_ERROR_STREAM("This controller requires exactly " << std::to_string(N_DOF) << " joints.");
                return false;
            }

            for(int i=0; i<N_DOF; i++){
                joints_[i] = hw->getHandle(joint_names_[i]);
                q_des_(i) = joints_[i].getPosition();        // Initialize desired position to current position
                dq_des_(i) = 0;
                ddq_des_(i) = 0;

                if(!n.getParam(joint_names_[i] + "_control_params/pid/p", Kp(i,i))){
                    ROS_ERROR_STREAM("Failed to get joint \"" << joint_names_[i] << "\" p parameter value.");
                    return false;
                }
                if(!n.getParam(joint_names_[i] + "_control_params/pid/i", Ki(i,i))){
                    ROS_ERROR_STREAM("Failed to get joint \"" << joint_names_[i] << "\" i parameter value.");
                    return false;
                }
                if(!n.getParam(joint_names_[i] + "_control_params/pid/d", Kd(i,i))){
                    ROS_ERROR_STREAM("Failed to get joint \"" << joint_names_[i] << "\" d parameter value.");
                    return false;
                }
            }

            /* --- Initialize subscribers for desired positions (for manual control) --- */
            sub_q1_des_ = n.subscribe<std_msgs::Float64>("q1_des_command", 1, &ParComputado6Dof::set_q1_des_CB, this);
            sub_q2_des_ = n.subscribe<std_msgs::Float64>("q2_des_command", 1, &ParComputado6Dof::set_q2_des_CB, this);
            sub_q3_des_ = n.subscribe<std_msgs::Float64>("q3_des_command", 1, &ParComputado6Dof::set_q3_des_CB, this);
            sub_q4_des_ = n.subscribe<std_msgs::Float64>("q4_des_command", 1, &ParComputado6Dof::set_q4_des_CB, this);
            sub_q5_des_ = n.subscribe<std_msgs::Float64>("q5_des_command", 1, &ParComputado6Dof::set_q5_des_CB, this);
            sub_q6_des_ = n.subscribe<std_msgs::Float64>("q6_des_command", 1, &ParComputado6Dof::set_q6_des_CB, this);

            /* --- Initialize publishers for datalogging purposes --- */
            pub_q_des_ = n.advertise<my_custom_msgs::SixJointsValues>("q_des" , 1);
            pub_tau_R_ = n.advertise<my_custom_msgs::SixJointsValues>("tau_R" , 1);
            pub_M_tau_R_ = n.advertise<my_custom_msgs::SixJointsValues>("M_tau_R" , 1);
            pub_tau_C_ = n.advertise<my_custom_msgs::SixJointsValues>("tau_C" , 1);
            pub_tau_T_ = n.advertise<my_custom_msgs::SixJointsValues>("tau_T" , 1);
            pub_error_ = n.advertise<my_custom_msgs::SixJointsValues>("error" , 1);

            /* --- Initialize action server --- */
            action_server_.reset(new ActionServer(n, "/arm_group_controller/follow_joint_trajectory", 
                                    boost::bind(&ParComputado6Dof::goalCB,   this, _1),
                                    boost::bind(&ParComputado6Dof::cancelCB, this, _1), false));

            action_server_->start();

            return true;
        }

        void update(const ros::Time& time, const ros::Duration& period){
            
            static Eigen::Matrix<double, N_DOF, 1> ierror = Eigen::Matrix<double, N_DOF, 1>::Zero();

            /* --- Get joint states --- */
            for(int i=0; i<N_DOF; i++){   
                q_(i) = joints_[i].getPosition();
                dq_(i) = joints_[i].getVelocity();
            }

            /* --- Evaluate the dynamics matrices in the current joint states --- */
            KDL::JntArray q_KDL(N_DOF);
            KDL::JntArray dq_KDL(N_DOF);
            KDL::JntArray V_KDL(N_DOF);
            KDL::JntArray G_KDL(N_DOF);
            KDL::JntSpaceInertiaMatrix M_KDL(N_DOF);
            KDL::ChainDynParam dynamics_solver(kdl_chain, KDL::Vector(0, 0, -g));

            q_KDL.data = q_;
            dq_KDL.data = dq_;
            dynamics_solver.JntToMass(q_KDL, M_KDL);
            dynamics_solver.JntToCoriolis(q_KDL, dq_KDL, V_KDL);
            dynamics_solver.JntToGravity(q_KDL, G_KDL);

            M_ = M_KDL.data;
            V_ = V_KDL.data;
            G_ = G_KDL.data;

            /* --- Control law --- */
            error = q_des_ - q_;
            derror = dq_des_ - dq_;
            ierror += error * period.toSec();
            tau_R = ddq_des_ + Kp * error + Ki * ierror + Kd * derror;
            tau_C = V_ + G_ + F_;
            tau_T = M_ * tau_R + tau_C;

            /* --- Apply control torque to the joints --- */
            for(int i=0; i<N_DOF; i++){ 
                joints_[i].setCommand(tau_T(i));
            }

            /* --- Publish messages for datalogging purposes --- */
            publishMessages();
        }

        /* --- Callback functions for desired position subscribers (for manual control) --- */
        void set_q1_des_CB(const std_msgs::Float64ConstPtr& msg){
            q_des_(0) = msg->data;
        }
        void set_q2_des_CB(const std_msgs::Float64ConstPtr& msg){
            q_des_(1) = msg->data;
        }
        void set_q3_des_CB(const std_msgs::Float64ConstPtr& msg){
            q_des_(2) = msg->data;
        }
        void set_q4_des_CB(const std_msgs::Float64ConstPtr& msg){
            q_des_(3) = msg->data;
        }
        void set_q5_des_CB(const std_msgs::Float64ConstPtr& msg){
            q_des_(4) = msg->data;
        }
        void set_q6_des_CB(const std_msgs::Float64ConstPtr& msg){
            q_des_(5) = msg->data;
        }

        void goalCB(GoalHandle gh)
        {
            // Cancel current goal (if active)
            if (has_active_goal_){
                active_goal_.setCanceled();
            }

            gh.setAccepted();
            active_goal_ = gh;
            has_active_goal_ = true;

            trajectory_msgs::JointTrajectory traj = gh.getGoal()->trajectory;

            int n = traj.points.size();            
            for (int i=0; i<6; i++){                
                q_des_(i) = traj.points[n-1].positions[i];
                dq_des_(i) = traj.points[n-1].velocities[i];
                // ddq_des_(i) = traj.points[n-1].accelerations[i];
            }

        }

        void cancelCB(GoalHandle gh)
        {
            if (active_goal_ == gh){
                active_goal_.setCanceled();
                has_active_goal_ = false;
            }
        }

        void starting(const ros::Time& time) {}
        void stopping(const ros::Time& time) {}

        void publishMessages(void){

            my_custom_msgs::SixJointsValues q_des_msg;
                q_des_msg.joint1 = q_des_(0);
                q_des_msg.joint2 = q_des_(1);
                q_des_msg.joint3 = q_des_(2);
                q_des_msg.joint4 = q_des_(3);
                q_des_msg.joint5 = q_des_(4);
                q_des_msg.joint6 = q_des_(5);
                pub_q_des_.publish(q_des_msg);

            my_custom_msgs::SixJointsValues tau_R_msg;
                tau_R_msg.joint1 = tau_R(0);
                tau_R_msg.joint2 = tau_R(1);
                tau_R_msg.joint3 = tau_R(2);
                tau_R_msg.joint4 = tau_R(3);
                tau_R_msg.joint5 = tau_R(4);
                tau_R_msg.joint6 = tau_R(5);
                pub_tau_R_.publish(tau_R_msg);

            my_custom_msgs::SixJointsValues M_tau_R_msg;
                M_tau_R_msg.joint1 = (M_ * tau_R)(0);
                M_tau_R_msg.joint2 = (M_ * tau_R)(1);
                M_tau_R_msg.joint3 = (M_ * tau_R)(2);
                M_tau_R_msg.joint4 = (M_ * tau_R)(3);
                M_tau_R_msg.joint5 = (M_ * tau_R)(4);
                M_tau_R_msg.joint6 = (M_ * tau_R)(5);
                pub_M_tau_R_.publish(M_tau_R_msg);

            my_custom_msgs::SixJointsValues tau_C_msg;
                tau_C_msg.joint1 = tau_C(0);
                tau_C_msg.joint2 = tau_C(1);
                tau_C_msg.joint3 = tau_C(2);
                tau_C_msg.joint4 = tau_C(3);
                tau_C_msg.joint5 = tau_C(4);
                tau_C_msg.joint6 = tau_C(5);
                pub_tau_C_.publish(tau_C_msg);

            my_custom_msgs::SixJointsValues tau_T_msg;
                tau_T_msg.joint1 = tau_T(0);
                tau_T_msg.joint2 = tau_T(1);
                tau_T_msg.joint3 = tau_T(2);
                tau_T_msg.joint4 = tau_T(3);
                tau_T_msg.joint5 = tau_T(4);
                tau_T_msg.joint6 = tau_T(5);
                pub_tau_T_.publish(tau_T_msg);

            my_custom_msgs::SixJointsValues error_msg;
                error_msg.joint1 = error(0);
                error_msg.joint2 = error(1);
                error_msg.joint3 = error(2);
                error_msg.joint4 = error(3);
                error_msg.joint5 = error(4);
                error_msg.joint6 = error(5);
                pub_error_.publish(error_msg);
        }

        private:

            const double g = 9.81;

            Eigen::Matrix<double, N_DOF, 1> q_;
            Eigen::Matrix<double, N_DOF, 1> dq_;
            Eigen::Matrix<double, N_DOF, 1> q_des_;
            Eigen::Matrix<double, N_DOF, 1> dq_des_;
            Eigen::Matrix<double, N_DOF, 1> ddq_des_;
            Eigen::Matrix<double, N_DOF, 1> error;
            Eigen::Matrix<double, N_DOF, 1> derror;

            Eigen::Matrix<double, N_DOF, N_DOF> M_;
            Eigen::Matrix<double, N_DOF, 1> V_;
            Eigen::Matrix<double, N_DOF, 1> G_;
            Eigen::Matrix<double, N_DOF, 1> F_;
            Eigen::Matrix<double, N_DOF, N_DOF> Kp;
            Eigen::Matrix<double, N_DOF, N_DOF> Ki;
            Eigen::Matrix<double, N_DOF, N_DOF> Kd;
            Eigen::Matrix<double, N_DOF, 1> tau_C;
            Eigen::Matrix<double, N_DOF, 1> tau_R;
            Eigen::Matrix<double, N_DOF, 1> tau_T;

            ros::Subscriber sub_q1_des_;
            ros::Subscriber sub_q2_des_;
            ros::Subscriber sub_q3_des_;
            ros::Subscriber sub_q4_des_;
            ros::Subscriber sub_q5_des_;
            ros::Subscriber sub_q6_des_;

            ros::Publisher pub_q_des_;
            ros::Publisher pub_tau_R_;
            ros::Publisher pub_M_tau_R_;
            ros::Publisher pub_tau_C_;
            ros::Publisher pub_tau_T_;
            ros::Publisher pub_error_;

            hardware_interface::JointHandle joints_[N_DOF];
            std::vector<std::string> joint_names_;

            KDL::Tree kdl_tree;
            KDL::Chain kdl_chain;

            ActionServerPtr action_server_;
            bool has_active_goal_ = false;
            GoalHandle active_goal_;
    };

    PLUGINLIB_EXPORT_CLASS(par_computado_6dof_ns::ParComputado6Dof, controller_interface::ControllerBase);
}

