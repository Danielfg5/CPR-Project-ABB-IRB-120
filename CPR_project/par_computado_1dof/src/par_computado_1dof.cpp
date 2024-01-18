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

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryAction.h>

#include <trajectory_interface/trajectory_interface.h>

#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <joint_trajectory_controller/hardware_interface_adapter.h>


namespace par_computado_1dof_ns {

    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
    typedef boost::shared_ptr<ActionServer>                                                     ActionServerPtr;
    typedef ActionServer::GoalHandle                                                            GoalHandle;

    class ParComputado1Dof : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

        /* ----- Controller initialization function ----- */
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) {

            /* ----- Load parameters from the parameter server ----- */
            if(!n.getParam("joint", joint_name_)){
                ROS_ERROR("Failed to get joint parameter.");
                return false;
            }
            
            joint_ = hw->getHandle(joint_name_);
            q_des_ = joint_.getPosition();        // Initialize desired position to current position
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

            /* ----- Initialize PID controller ----- */
            pidController_.initPid(pid_params_.p, pid_params_.i, pid_params_.d, 100.0, -100.0);            

            /* ----- Initialize subscriber for desired position (for manual control) ----- */
            sub_q_des_ = n.subscribe<std_msgs::Float64>("q_des_command", 1, &ParComputado1Dof::set_q_des_CB, this);

            /* ----- Initialize publishers for datalogging purposes ----- */
            pub_q_ = n.advertise<std_msgs::Float64>("q" , 1);
            pub_q_des_ = n.advertise<std_msgs::Float64>("q_des" , 1);
            pub_comm_effort_ = n.advertise<std_msgs::Float64>("commanded_effort" , 1);

            /* ----- Initialize action server ----- */
            action_server_.reset(new ActionServer(n, "/pendulum_controller/follow_joint_trajectory", 
                                    boost::bind(&ParComputado1Dof::goalCB,   this, _1),
                                    boost::bind(&ParComputado1Dof::cancelCB, this, _1), false));
            action_server_->start();

            return true;
        }

        /* ----- Controller loop update function ----- */
        void update(const ros::Time& time, const ros::Duration& period){
            
            const double g = 9.81;
            
            double q_ = joint_.getPosition();
            double dq_ = joint_.getVelocity();

            double error = q_des_ - q_;
            double derror = dq_des_ - dq_;

            /* ----- PAR COMPUTADO CONTROL LAW ----- */
            double computed_torque = model_params_.b * dq_ - 0.5*model_params_.m*g*model_params_.l*sin(q_);
            double feedback_torque = ddq_des_ + pidController_.computeCommand(error, derror, period);
            double commanded_effort = model_params_.I * feedback_torque + computed_torque;
            
            /* ----- Apply control torque to the joint --- */
            joint_.setCommand(commanded_effort);

            /* --- Publish messages for datalogging purposes --- */
            std_msgs::Float64 q_msg;
            std_msgs::Float64 q_des_msg;
            std_msgs::Float64 comm_effort_msg;
            q_msg.data = q_;
            q_des_msg.data = q_des_;
            comm_effort_msg.data = commanded_effort;
            pub_q_.publish(q_msg);
            pub_q_des_.publish(q_des_msg);
            pub_comm_effort_.publish(comm_effort_msg);
        }

        /* ----- Callback function for desired position subscriber (for manual control) ----- */
        void set_q_des_CB(const std_msgs::Float64ConstPtr& msg){
            q_des_ = msg->data;
        }

        /* ----- Callback functions to process Moveit! commands ----- */
        void goalCB(GoalHandle gh)
        {
            trajectory_msgs::JointTrajectory trajectory = gh.getGoal()->trajectory;

            int n = trajectory.points.size();                    
            q_des_ = trajectory.points[n-1].positions[0];
            dq_des_ = trajectory.points[n-1].velocities[0];
            // ddq_des_ = trajectory.points[n-1].accelerations[0];
        }
        void cancelCB(GoalHandle gh)
        {
            gh.setCanceled();
        }

        void starting(const ros::Time& time) {}
        void stopping(const ros::Time& time) {}

        private:

            ActionServerPtr action_server_;
            
            std::string joint_name_;
            hardware_interface::JointHandle joint_;

            double q_des_;
            double dq_des_;
            double ddq_des_;

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

            ros::Subscriber sub_q_des_;
            ros::Publisher pub_q_;
            ros::Publisher pub_q_des_;
            ros::Publisher pub_comm_effort_;
    };

    PLUGINLIB_EXPORT_CLASS(par_computado_1dof_ns::ParComputado1Dof, controller_interface::ControllerBase);
}

