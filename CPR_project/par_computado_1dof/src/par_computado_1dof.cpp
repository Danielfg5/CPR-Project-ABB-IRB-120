#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <control_toolbox/pid.h>
#include <time.h>
#include <string>

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
            command_ = joint_.getPosition();        // Initialize desired (target) position to current position

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

            /* --- Initialize PID controller --- */
            pidController_.initPid(pid_params_.p, pid_params_.i, pid_params_.d, 100.0, -100.0); // TODO: integrator limits, anti-windup
            // pidController_.initPid(pid_params_.p, 0.0, 0.0, 100.0, -100.0);  // DEBUG
            

            /* --- Initialize subscriber for desired position (for manual control) --- */
            sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &ParComputado1Dof::setCommandCB, this);

            /* --- Initialize action server --- */
            action_server_.reset(new ActionServer(n, "/pendulum_controller/follow_joint_trajectory", 
                                    boost::bind(&ParComputado1Dof::goalCB,   this, _1),
                                    boost::bind(&ParComputado1Dof::cancelCB, this, _1), false));

            action_server_->start();

            return true;
        }

        void update(const ros::Time& time, const ros::Duration& period){
            
            double error = command_ - joint_.getPosition();
            double commanded_effort = pidController_.computeCommand(error, period);
            joint_.setCommand(commanded_effort);
        }

        /* --- Callback function for desired position subscriber (for manual control) --- */
        void setCommandCB(const std_msgs::Float64ConstPtr& msg){

            command_ = msg->data;
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
            command_ = traj.points[n-1].positions[0];
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
            double command_;

            ros::Subscriber sub_command_;

            control_toolbox::Pid pidController_;

            struct {
                double p;
                double i;
                double d;
            } pid_params_;

            std::string joint_name_;

            bool has_active_goal_ = false;
            GoalHandle active_goal_;
    };

    PLUGINLIB_EXPORT_CLASS(par_computado_1dof_ns::ParComputado1Dof, controller_interface::ControllerBase);
}

