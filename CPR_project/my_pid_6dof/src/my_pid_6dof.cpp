#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include "my_custom_msgs/SixJointsValues.h"
#include <control_toolbox/pid.h>
#include <time.h>
#include <string>

#include <moveit_msgs/DisplayTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryAction.h>

#include <trajectory_interface/trajectory_interface.h>

#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <joint_trajectory_controller/hardware_interface_adapter.h>

#define N_DOF   6

namespace my_pid_6dof_ns {

    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
    typedef boost::shared_ptr<ActionServer>                                                     ActionServerPtr;
    typedef ActionServer::GoalHandle                                                            GoalHandle;

    class MyPid6Dof : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

        /* ----- Controller initialization function ----- */
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) {

            /* ----- Load parameters from the parameter server ----- */
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
                q_des_[i] = joints_[i].getPosition();        // Initialize desired position to current position
                dq_des_[i] = 0;

                if (!n.getParam(joint_names_[i] + "_control_params/pid/p", pid_params_[i].p)){
                    ROS_ERROR_STREAM("Failed to get joint \"" << joint_names_[i] << "\" p parameter value.");
                    return false;
                }
                if (!n.getParam(joint_names_[i] + "_control_params/pid/i", pid_params_[i].i)){
                    ROS_ERROR_STREAM("Failed to get joint \"" << joint_names_[i] << "\" i parameter value.");
                    return false;
                }
                if (!n.getParam(joint_names_[i] + "_control_params/pid/d", pid_params_[i].d)){
                    ROS_ERROR_STREAM("Failed to get joint \"" << joint_names_[i] << "\" d parameter value.");
                    return false;
                }

                /* ----- Initialize PID controllers ----- */
                pidControllers_[i].initPid(pid_params_[i].p, pid_params_[i].i, pid_params_[i].d, 100.0, -100.0);
            }

            /* ----- Initialize subscribers for desired positions (for manual control) ----- */
            sub_q1_des_ = n.subscribe<std_msgs::Float64>("q1_des_command", 1, &MyPid6Dof::set_q1_des_CB, this);
            sub_q2_des_ = n.subscribe<std_msgs::Float64>("q2_des_command", 1, &MyPid6Dof::set_q2_des_CB, this);
            sub_q3_des_ = n.subscribe<std_msgs::Float64>("q3_des_command", 1, &MyPid6Dof::set_q3_des_CB, this);
            sub_q4_des_ = n.subscribe<std_msgs::Float64>("q4_des_command", 1, &MyPid6Dof::set_q4_des_CB, this);
            sub_q5_des_ = n.subscribe<std_msgs::Float64>("q5_des_command", 1, &MyPid6Dof::set_q5_des_CB, this);
            sub_q6_des_ = n.subscribe<std_msgs::Float64>("q6_des_command", 1, &MyPid6Dof::set_q6_des_CB, this);

            /* ----- Initialize publishers for datalogging purposes ----- */
            pub_q_ = n.advertise<my_custom_msgs::SixJointsValues>("q" , 1);
            pub_q_des_ = n.advertise<my_custom_msgs::SixJointsValues>("q_des" , 1);
            pub_error_ = n.advertise<my_custom_msgs::SixJointsValues>("error" , 1);
            pub_commanded_effort_ = n.advertise<my_custom_msgs::SixJointsValues>("commanded_effort" , 1);

            /* ----- Initialize action server ----- */
            action_server_.reset(new ActionServer(n, "/arm_group_controller/follow_joint_trajectory", 
                                    boost::bind(&MyPid6Dof::goalCB,   this, _1),
                                    boost::bind(&MyPid6Dof::cancelCB, this, _1), false));
            action_server_->start();

            return true;
        }

        /* ----- Controller loop update function ----- */
        void update(const ros::Time& time, const ros::Duration& period){

            for(int i=0; i<N_DOF; i++){
                q_[i] = joints_[i].getPosition();
                dq_[i] = joints_[i].getVelocity();
                error[i] = q_des_[i] - q_[i];
                derror[i] = dq_des_[i] - dq_[i];
                commanded_effort[i] = pidControllers_[i].computeCommand(error[i], derror[i], period);
                joints_[i].setCommand(commanded_effort[i]);
            }

            publishMessages();
        }

        /* --- Callback functions for desired position subscribers (for manual control) --- */
        void set_q1_des_CB(const std_msgs::Float64ConstPtr& msg){
            q_des_[0] = msg->data;
        }
        void set_q2_des_CB(const std_msgs::Float64ConstPtr& msg){
            q_des_[1] = msg->data;
        }
        void set_q3_des_CB(const std_msgs::Float64ConstPtr& msg){
            q_des_[2] = msg->data;
        }
        void set_q4_des_CB(const std_msgs::Float64ConstPtr& msg){
            q_des_[3] = msg->data;
        }
        void set_q5_des_CB(const std_msgs::Float64ConstPtr& msg){
            q_des_[4] = msg->data;
        }
        void set_q6_des_CB(const std_msgs::Float64ConstPtr& msg){
            q_des_[5] = msg->data;
        }

        /* ----- Callback functions to process Moveit! commands ----- */
        void goalCB(GoalHandle gh)
        {
            trajectory_msgs::JointTrajectory trajectory = gh.getGoal()->trajectory;

            int n = trajectory.points.size();            
            for (int i=0; i<6; i++){                
                q_des_[i] = trajectory.points[n-1].positions[i];
                dq_des_[i] = trajectory.points[n-1].velocities[i];
            }
        }
        void cancelCB(GoalHandle gh)
        {
            gh.setCanceled();
        }

        void starting(const ros::Time& time) {}
        void stopping(const ros::Time& time) {}

        /* ----- Function to publish messages (for datalogging purposes) ----- */
        void publishMessages(void){

            my_custom_msgs::SixJointsValues q_msg;
                q_msg.joint1 = q_[0];
                q_msg.joint2 = q_[1];
                q_msg.joint3 = q_[2];
                q_msg.joint4 = q_[3];
                q_msg.joint5 = q_[4];
                q_msg.joint6 = q_[5];
                pub_q_.publish(q_msg);

            my_custom_msgs::SixJointsValues q_des_msg;
                q_des_msg.joint1 = q_des_[0];
                q_des_msg.joint2 = q_des_[1];
                q_des_msg.joint3 = q_des_[2];
                q_des_msg.joint4 = q_des_[3];
                q_des_msg.joint5 = q_des_[4];
                q_des_msg.joint6 = q_des_[5];
                pub_q_des_.publish(q_des_msg);

            my_custom_msgs::SixJointsValues error_msg;
                error_msg.joint1 = error[0];
                error_msg.joint2 = error[1];
                error_msg.joint3 = error[2];
                error_msg.joint4 = error[3];
                error_msg.joint5 = error[4];
                error_msg.joint6 = error[5];
                pub_error_.publish(error_msg);

            my_custom_msgs::SixJointsValues commanded_effort_msg;
                commanded_effort_msg.joint1 = commanded_effort[0];
                commanded_effort_msg.joint2 = commanded_effort[1];
                commanded_effort_msg.joint3 = commanded_effort[2];
                commanded_effort_msg.joint4 = commanded_effort[3];
                commanded_effort_msg.joint5 = commanded_effort[4];
                commanded_effort_msg.joint6 = commanded_effort[5];
                pub_commanded_effort_.publish(commanded_effort_msg);
        }

        private:

            ActionServerPtr action_server_;

            std::vector<std::string> joint_names_;
            hardware_interface::JointHandle joints_[N_DOF];

            double q_[N_DOF];
            double dq_[N_DOF];
            double q_des_[N_DOF];
            double dq_des_[N_DOF];
            double error[N_DOF];
            double derror[N_DOF];
            double commanded_effort[N_DOF];

            control_toolbox::Pid pidControllers_[N_DOF];

            struct {
                double p;
                double i;
                double d;
            } pid_params_[N_DOF];

            ros::Subscriber sub_q1_des_;
            ros::Subscriber sub_q2_des_;
            ros::Subscriber sub_q3_des_;
            ros::Subscriber sub_q4_des_;
            ros::Subscriber sub_q5_des_;
            ros::Subscriber sub_q6_des_;

            ros::Publisher pub_q_;
            ros::Publisher pub_q_des_;
            ros::Publisher pub_error_;
            ros::Publisher pub_commanded_effort_;
    };

    PLUGINLIB_EXPORT_CLASS(my_pid_6dof_ns::MyPid6Dof, controller_interface::ControllerBase);
}

