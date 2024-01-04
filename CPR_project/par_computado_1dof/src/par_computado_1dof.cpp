#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <control_toolbox/pid.h>
#include <time.h>
#include <string>
#include <cmath>

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


namespace par_computado_1dof_ns {

    class ParComputado1Dof : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

        typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
        typedef boost::shared_ptr<ActionServer>                                                     ActionServerPtr;
        typedef ActionServer::GoalHandle                                                            GoalHandle;

        ActionServerPtr    action_server_;

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) {

            /* --- Load parameters from the parameter server --- */
            if(!n.getParam("joint", joint_name_)){
                ROS_ERROR("Failed to get joint parameter.");
                return false;
            }
            
            joint_ = hw->getHandle(joint_name_);
            q_des_ = joint_.getPosition();        // Initialize desired (target) position to current position
            dq_des_ = 0;
            ddq_des_ = 0;

            if (!n.getParam(joint_name_ + "_control_params/pid/p", pid_params_.p)){
                ROS_ERROR_STREAM("Failed to get joint \"" << joint_name_ << "\" p parameter value.");
                return false;
            }
            if (!n.getParam(joint_name_ + "_control_params/pid/i", pid_params_.i)){
                ROS_ERROR_STREAM("Failed to get joint \"" << joint_name_ << "\" i parameter value.");
                return false;
            }
            if (!n.getParam(joint_name_ + "_control_params/pid/d", pid_params_.d)){
                ROS_ERROR_STREAM("Failed to get joint \"" << joint_name_ << "\" d parameter value.");
                return false;
            }
            if (!n.getParam(joint_name_ + "_control_params/model_params/I", model_params_.I)){
                ROS_ERROR_STREAM("Failed to get joint \"" << joint_name_ << "\" I parameter value.");
                return false;
            }
            if (!n.getParam(joint_name_ + "_control_params/model_params/b", model_params_.b)){
                ROS_ERROR_STREAM("Failed to get joint \"" << joint_name_ << "\" b parameter value.");
                return false;
            }
            if (!n.getParam(joint_name_ + "_control_params/model_params/c", model_params_.c)){
                ROS_ERROR_STREAM("Failed to get joint \"" << joint_name_ << "\" c parameter value.");
                return false;
            }
            if (!n.getParam(joint_name_ + "_control_params/model_params/m", model_params_.m)){
                ROS_ERROR_STREAM("Failed to get joint \"" << joint_name_ << "\" m parameter value.");
                return false;
            }
            if (!n.getParam(joint_name_ + "_control_params/model_params/l", model_params_.l)){
                ROS_ERROR_STREAM("Failed to get joint \"" << joint_name_ << "\" l parameter value.");
                return false;
            }

            /* --- Initialize PID controller --- */
            pidController_.initPid(pid_params_.p, pid_params_.i, pid_params_.d, 100.0, -100.0);            

            /* --- Initialize subscriber for desired position (for manual control) --- */
            sub_q_des_ = n.subscribe<std_msgs::Float64>("q_des_command", 1, &ParComputado1Dof::setCommandCB, this);

            /* --- Initialize publishers for datalogging purposes --- */
            pub_q_des_ = n.advertise<std_msgs::Float64>("q_des" , 1);
            pub_dq_des_ = n.advertise<std_msgs::Float64>("dq_des" , 1);
            pub_ddq_des_ = n.advertise<std_msgs::Float64>("ddq_des" , 1);
            pub_comp_torque_ = n.advertise<std_msgs::Float64>("comp_torque" , 1);
            pub_fb_torque_ = n.advertise<std_msgs::Float64>("fb_torque" , 1);
            pub_comm_effort_ = n.advertise<std_msgs::Float64>("commanded_effort" , 1);
            pub_error_= n.advertise<std_msgs::Float64>("error" , 1);
            pub_p_error_ = n.advertise<std_msgs::Float64>("p_error" , 1);
            pub_i_error_ = n.advertise<std_msgs::Float64>("i_error" , 1);
            pub_d_error_ = n.advertise<std_msgs::Float64>("d_error" , 1);

            /* --- Initialize action server --- */
            action_server_.reset(new ActionServer(n, "/pendulum_controller/follow_joint_trajectory", 
                                    boost::bind(&ParComputado1Dof::goalCB,   this, _1),
                                    boost::bind(&ParComputado1Dof::cancelCB, this, _1), false));

            action_server_->start();

            return true;
        }

        void update(const ros::Time& time, const ros::Duration& period){
            
            const double g = 9.81;
            
            double q_ = joint_.getPosition();
            double dq_ = joint_.getVelocity();

            double error = q_des_ - q_;
            double derror = dq_des_ - dq_;
            
            double computed_torque = model_params_.b * dq_ - 0.5*model_params_.m*g*model_params_.l*sin(q_);
            double feedback_torque = ddq_des_ + pidController_.computeCommand(error, derror, period);
            double commanded_effort = model_params_.I * feedback_torque + computed_torque;
            
            joint_.setCommand(commanded_effort);

            std_msgs::Float64 q_des_msg;
            std_msgs::Float64 dq_des_msg;
            std_msgs::Float64 ddq_des_msg;
            std_msgs::Float64 comp_torque_msg;
            std_msgs::Float64 fb_torque_msg;
            std_msgs::Float64 comm_effort_msg;
            std_msgs::Float64 error_msg;
            std_msgs::Float64 p_error_msg;
            std_msgs::Float64 i_error_msg;
            std_msgs::Float64 d_error_msg;
            double pe, ie, de;
            pidController_.getCurrentPIDErrors(&pe, &ie, &de);
            q_des_msg.data = q_des_;
            dq_des_msg.data = dq_des_;
            ddq_des_msg.data = ddq_des_;
            comp_torque_msg.data = computed_torque;
            fb_torque_msg.data = model_params_.I * feedback_torque;
            comm_effort_msg.data = commanded_effort;
            error_msg.data = error;
            p_error_msg.data = pe;
            i_error_msg.data = ie;
            d_error_msg.data = de;
            pub_q_des_.publish(q_des_msg);
            pub_dq_des_.publish(dq_des_msg);
            pub_ddq_des_.publish(ddq_des_msg);
            pub_comp_torque_.publish(comp_torque_msg);
            pub_fb_torque_.publish(fb_torque_msg);
            pub_comm_effort_.publish(comm_effort_msg);
            pub_error_.publish(error_msg);
            pub_p_error_.publish(p_error_msg);
            pub_i_error_.publish(i_error_msg);
            pub_d_error_.publish(d_error_msg);
        }

        /* --- Callback function for desired position subscriber (for manual control) --- */
        void setCommandCB(const std_msgs::Float64ConstPtr& msg){

            q_des_ = msg->data;
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
            q_des_ = traj.points[n-1].positions[0];
            dq_des_ = traj.points[n-1].velocities[0];
            // ddq_des_ = traj.points[n-1].accelerations[0];
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

        private:
            hardware_interface::JointHandle joint_;
            double q_des_;
            double dq_des_;
            double ddq_des_;

            ros::Subscriber sub_q_des_;
            ros::Publisher pub_q_des_;
            ros::Publisher pub_dq_des_;
            ros::Publisher pub_ddq_des_;
            ros::Publisher pub_comp_torque_;
            ros::Publisher pub_fb_torque_;
            ros::Publisher pub_comm_effort_;
            ros::Publisher pub_error_;
            ros::Publisher pub_p_error_;
            ros::Publisher pub_i_error_;
            ros::Publisher pub_d_error_;

            control_toolbox::Pid pidController_;

            struct {
                double p;
                double i;
                double d;
            } pid_params_;

            struct {
                double I;
                double b;
                double c;
                double m;
                double l;
            } model_params_;

            std::string joint_name_;

            bool has_active_goal_ = false;
            GoalHandle active_goal_;
    };

    PLUGINLIB_EXPORT_CLASS(par_computado_1dof_ns::ParComputado1Dof, controller_interface::ControllerBase);
}

