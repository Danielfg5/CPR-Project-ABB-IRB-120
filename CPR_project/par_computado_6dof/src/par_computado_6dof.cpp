#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <time.h>
#include <string>
#include <cmath>
#include "Eigen/Core"

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

#define N_DOF   6

namespace par_computado_6dof_ns {

    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>         ActionServer;
    typedef boost::shared_ptr<ActionServer>                                            ActionServerPtr;
    typedef ActionServer::GoalHandle                                                   GoalHandle;

    class ParComputado6Dof : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) {

            /* --- Initialize controller gain matrices to zero --- */
            Kp.setZero();
            Ki.setZero();
            Kd.setZero();

            /********** DEBUG **********/
            M_ = Eigen::Matrix<double, N_DOF, N_DOF>::Identity();
            V_.setZero();
            G_.setZero();
            F_.setZero();
            /***************************/

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
                q_des_(i) = joints_[i].getPosition();        // Initialize desired (target) position to current position
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

            /* --- Print on console output --- */
            std::cout << std::endl << "************************************" << std::endl;
            std::cout << "------- KP MATRIX -------" << std::endl;
            std::cout << Kp << std::endl;
            std::cout << "------- KI MATRIX -------" << std::endl;
            std::cout << Ki << std::endl;
            std::cout << "------- KD MATRIX -------" << std::endl;
            std::cout << Kd << std::endl;

            /* --- Initialize subscribers for desired positions (for manual control) --- */
            sub_q1_des_ = n.subscribe<std_msgs::Float64>("q1_des_command", 1, &ParComputado6Dof::set_q1_des_CB, this);
            sub_q2_des_ = n.subscribe<std_msgs::Float64>("q2_des_command", 1, &ParComputado6Dof::set_q2_des_CB, this);
            sub_q3_des_ = n.subscribe<std_msgs::Float64>("q3_des_command", 1, &ParComputado6Dof::set_q3_des_CB, this);
            sub_q4_des_ = n.subscribe<std_msgs::Float64>("q4_des_command", 1, &ParComputado6Dof::set_q4_des_CB, this);
            sub_q5_des_ = n.subscribe<std_msgs::Float64>("q5_des_command", 1, &ParComputado6Dof::set_q5_des_CB, this);
            sub_q6_des_ = n.subscribe<std_msgs::Float64>("q6_des_command", 1, &ParComputado6Dof::set_q6_des_CB, this);

            /* --- Initialize action server --- */
            action_server_.reset(new ActionServer(n, "/arm_group_controller/follow_joint_trajectory", 
                                    boost::bind(&ParComputado6Dof::goalCB,   this, _1),
                                    boost::bind(&ParComputado6Dof::cancelCB, this, _1), false));

            action_server_->start();

            return true;
        }

        void update(const ros::Time& time, const ros::Duration& period){
            
            Eigen::Matrix<double, N_DOF, 1> q_;
            Eigen::Matrix<double, N_DOF, 1> dq_;
            Eigen::Matrix<double, N_DOF, 1> error;
            Eigen::Matrix<double, N_DOF, 1> derror;
            static Eigen::Matrix<double, N_DOF, 1> ierror = Eigen::Matrix<double, N_DOF, 1>::Zero();

            // **************  DEBUG ********************
            ros::Time begin = ros::Time::now();
            // ******************************************

            /* --- Get joint states --- */
            for(int i=0; i<N_DOF; i++){   
                q_(i) = joints_[i].getPosition();
                dq_(i) = joints_[i].getVelocity();
            }

            /* --- Evaluate the dynamics matrices in the current joint states --- */
            M_ = compute_M(q_);
            V_ = compute_V(q_, dq_);
            G_ = compute_G(q_);
            F_ = compute_F(q_, dq_);

            /* --- Control law --- */
            error = q_des_ - q_;
            derror = dq_des_ - dq_;
            ierror += error * period.toSec();
            tauR = ddq_des_ + Kp * error + Ki * ierror + Kd * derror;
            tauC = V_ + G_ + F_;
            tauT = M_ * tauR + tauC;

            /* --- Apply control torque to the joints --- */
            for(int i=0; i<N_DOF; i++){ 
                joints_[i].setCommand(tauT(i));
            }

            // **************  DEBUG ********************
            ros::Time end = ros::Time::now();
            ros::Duration duration = end - begin;
            // std::cout << duration.toNSec() << std::endl;
            // ******************************************
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

        Eigen::Matrix<double, N_DOF, N_DOF> compute_M(Eigen::Matrix<double, N_DOF, 1> q)
        {
            Eigen::Matrix<double, N_DOF, N_DOF> M;

            /********** DEBUG **********/
            M = Eigen::Matrix<double, N_DOF, N_DOF>::Identity();
            /***************************/

            /* --------- EXAMPLE --------- */
            M(0,0) = 0.22356*cos(q(2)) - 0.056921*sin(2.0*q(2)) - 0.059165*sin(2.0*q(1)) + 0.0059579*cos(q(4)) - 0.42012*sin(q(2)) + 0.16114*cos(q(2))*cos(q(3)) - 0.0053266*cos(q(4))*sin(q(2)) - 0.78465*pow(cos(q(1)),2) - 0.15558*pow(cos(q(2)),2) - 0.070238*pow(cos(q(3)),2) - 0.22356*pow(cos(q(1)),2)*cos(q(2)) + 0.041776*pow(cos(q(1)),2)*cos(q(3)) - 0.0059579*pow(cos(q(1)),2)*cos(q(4)) + 0.041776*pow(cos(q(2)),2)*cos(q(3)) - 0.0059579*pow(cos(q(2)),2)*cos(q(4)) + 0.42012*pow(cos(q(1)),2)*sin(q(2)) + 0.31115*pow(cos(q(1)),2)*pow(cos(q(2)),2) + 0.070238*pow(cos(q(1)),2)*pow(cos(q(3)),2) + 0.070238*pow(cos(q(2)),2)*pow(cos(q(3)),2)  - 0.16114*pow(cos(q(1)),2)*cos(q(2))*cos(q(3)) + 0.22768*cos(q(1))*pow(cos(q(2)),2)*sin(q(1)) + 0.22768*pow(cos(q(1)),2)*cos(q(2))*sin(q(2)) - 0.001381*pow(cos(q(1)),2)*cos(q(3))*sin(q(4)) - 0.001381*pow(cos(q(2)),2)*cos(q(3))*sin(q(4)) - 0.083552*pow(cos(q(1)),2)*pow(cos(q(2)),2)*cos(q(3)) + 0.011916*pow(cos(q(1)),2)*pow(cos(q(2)),2)*cos(q(4)) + 0.42012*cos(q(1))*cos(q(2))*sin(q(1)) - 0.18032*cos(q(1))*cos(q(3))*sin(q(1)) - 0.001381*cos(q(1))*cos(q(4))*sin(q(1)) - 0.18032*cos(q(2))*cos(q(3))*sin(q(2)) - 0.001381*cos(q(2))*cos(q(4))*sin(q(2)) - 0.0053266*cos(q(2))*cos(q(3))*sin(q(4)) - 0.14048*pow(cos(q(1)),2)*pow(cos(q(2)),2)*pow(cos(q(3)),2) + 0.36065*cos(q(1))*pow(cos(q(2)),2)*cos(q(3))*sin(q(1)) + 0.0027619*cos(q(1))*pow(cos(q(2)),2)*cos(q(4))*sin(q(1)) + 0.36065*pow(cos(q(1)),2)*cos(q(2))*cos(q(3))*sin(q(2)) + 0.0027619*pow(cos(q(1)),2)*cos(q(2))*cos(q(4))*sin(q(2)) + 0.0053266*pow(cos(q(1)),2)*cos(q(2))*cos(q(3))*sin(q(4)) + 0.0027619*pow(cos(q(1)),2)*pow(cos(q(2)),2)*cos(q(3))*sin(q(4)) + 0.0011684*pow(cos(q(1)),2)*pow(cos(q(2)),2)*cos(q(5))*sin(q(4)) + 0.0053266*cos(q(1))*cos(q(2))*cos(q(4))*sin(q(1)) - 0.31115*cos(q(1))*cos(q(2))*sin(q(1))*sin(q(2)) + 0.16114*cos(q(1))*cos(q(3))*sin(q(1))*sin(q(2)) + 0.0059579*cos(q(1))*cos(q(3))*sin(q(1))*sin(q(4)) + 0.0059579*cos(q(2))*cos(q(3))*sin(q(2))*sin(q(4)) + 0.14048*cos(q(1))*cos(q(2))*pow(cos(q(3)),2)*sin(q(1))*sin(q(2)) - 0.011916*cos(q(1))*pow(cos(q(2)),2)*cos(q(3))*sin(q(1))*sin(q(4)) - 0.011916*pow(cos(q(1)),2)*cos(q(2))*cos(q(3))*sin(q(2))*sin(q(4))  - 0.0011684*cos(q(1))*pow(cos(q(2)),2)*sin(q(1))*sin(q(3))*sin(q(5)) - 0.0011684*pow(cos(q(1)),2)*cos(q(2))*sin(q(2))*sin(q(3))*sin(q(5)) + 0.083552*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(2)) - 0.011916*cos(q(1))*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(2)) - 0.0053266*cos(q(1))*cos(q(3))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.0027619*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.0011684*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4)) + 0.0011684*cos(q(1))*pow(cos(q(2)),2)*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1)) + 0.0011684*pow(cos(q(1)),2)*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2)) - 0.0011959*cos(q(1))*pow(cos(q(2)),2)*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(4));
            M(1,1) = 1;
            M(2,2) = 1;
            M(3,3) = 1;
            M(4,4) = 1;
            M(5,5) = 1;

            M(0,1) = 0;
            M(0,2) = 0;
            M(0,3) = 0;
            M(0,4) = 0;
            M(0,5) = 0;
            M(1,2) = 0;
            M(1,3) = 0;
            M(1,4) = 0;
            M(1,5) = 0;
            M(2,3) = 0;
            M(2,4) = 0;
            M(2,5) = 0;
            M(3,4) = 0;
            M(3,5) = 0;
            M(4,5) = 0;

            M(1,0) = M(0,1);
            M(2,0) = M(0,2);
            M(3,0) = M(0,3);
            M(4,0) = M(0,4);
            M(5,0) = M(0,5);
            M(2,1) = M(1,2);
            M(3,1) = M(1,3);
            M(4,1) = M(1,4);
            M(5,1) = M(1,5);
            M(3,2) = M(2,3);
            M(4,2) = M(2,4);
            M(5,2) = M(2,5);
            M(4,3) = M(3,4);
            M(5,3) = M(3,5);
            M(5,4) = M(4,5);
            /* --------------------------- */

            return M;
        }
        Eigen::Matrix<double, N_DOF, 1> compute_V(Eigen::Matrix<double, N_DOF, 1> q, Eigen::Matrix<double, N_DOF, 1> dq)
        {
            Eigen::Matrix<double, N_DOF, 1> V;

            /********** DEBUG **********/
            V.setZero();
            /***************************/

            /* --------- EXAMPLE --------- */
            V(0) = 0;
            V(1) = 0;
            V(2) = 0;
            V(3) = 0;
            V(4) = 0;
            V(5) = 0;
            /* --------------------------- */

            return V;
        }
        Eigen::Matrix<double, N_DOF, 1> compute_G(Eigen::Matrix<double, N_DOF, 1> q)
        {
            Eigen::Matrix<double, N_DOF, 1> G;

            /********** DEBUG **********/
            G.setZero();
            /***************************/

            /* --------- EXAMPLE --------- */
            G(0) = 0;
            G(1) = 0.1*g*cos(q(1));
            G(2) = 0;
            G(3) = 0;
            G(4) = 0;
            G(5) = 0;
            /* --------------------------- */

            return G;
        }
        Eigen::Matrix<double, N_DOF, 1> compute_F(Eigen::Matrix<double, N_DOF, 1> q, Eigen::Matrix<double, N_DOF, 1> dq)
        {
            Eigen::Matrix<double, N_DOF, 1> F;

            /********** DEBUG **********/
            F.setZero();
            /***************************/

            /* --------- EXAMPLE --------- */
            F(0) = 0;
            F(1) = 0;
            F(2) = 0;
            F(3) = 0;
            F(4) = 0;
            F(5) = 0;
            /* --------------------------- */

            return F;
        }

        private:
            hardware_interface::JointHandle joints_[N_DOF];
            
            Eigen::Matrix<double, N_DOF, 1> q_des_;
            Eigen::Matrix<double, N_DOF, 1> dq_des_;
            Eigen::Matrix<double, N_DOF, 1> ddq_des_;

            Eigen::Matrix<double, N_DOF, N_DOF> M_;
            Eigen::Matrix<double, N_DOF, 1> V_;
            Eigen::Matrix<double, N_DOF, 1> G_;
            Eigen::Matrix<double, N_DOF, 1> F_;
            Eigen::Matrix<double, N_DOF, N_DOF> Kp;
            Eigen::Matrix<double, N_DOF, N_DOF> Ki;
            Eigen::Matrix<double, N_DOF, N_DOF> Kd;
            Eigen::Matrix<double, N_DOF, 1> tauC;
            Eigen::Matrix<double, N_DOF, 1> tauR;
            Eigen::Matrix<double, N_DOF, 1> tauT;

            const double g = 9.81;

            ros::Subscriber sub_q1_des_;
            ros::Subscriber sub_q2_des_;
            ros::Subscriber sub_q3_des_;
            ros::Subscriber sub_q4_des_;
            ros::Subscriber sub_q5_des_;
            ros::Subscriber sub_q6_des_;

            std::vector<std::string> joint_names_;

            ActionServerPtr action_server_;
            bool has_active_goal_ = false;
            GoalHandle active_goal_;
    };

    PLUGINLIB_EXPORT_CLASS(par_computado_6dof_ns::ParComputado6Dof, controller_interface::ControllerBase);
}

