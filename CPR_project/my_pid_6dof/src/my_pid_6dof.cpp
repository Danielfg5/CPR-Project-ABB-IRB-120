#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <control_toolbox/pid.h>
#include <time.h>
#include <string>

#define N_DOF   6

namespace my_pid_6dof_ns {

    class MyPid6Dof : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) {
            
            std::vector<std::string> joint_names_;

            /* --- Load parameters from the parameter server (set through joints_controllers.yaml) --- */
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
                commands_[i] = joints_[i].getPosition();        // Initialize desired (target) position to current position

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

                /* --- Initialize PID controllers --- */
                pidControllers_[i].initPid(pid_params_[i].p, pid_params_[i].i, pid_params_[i].d, 100.0, -100.0);
            }

            /* --- Initialize subscribers for desired positions --- */
            sub_command1_ = n.subscribe<std_msgs::Float64>("command1", 1, &MyPid6Dof::setCommand1CB, this);
            sub_command2_ = n.subscribe<std_msgs::Float64>("command2", 1, &MyPid6Dof::setCommand2CB, this);
            sub_command3_ = n.subscribe<std_msgs::Float64>("command3", 1, &MyPid6Dof::setCommand3CB, this);
            sub_command4_ = n.subscribe<std_msgs::Float64>("command4", 1, &MyPid6Dof::setCommand4CB, this);
            sub_command5_ = n.subscribe<std_msgs::Float64>("command5", 1, &MyPid6Dof::setCommand5CB, this);
            sub_command6_ = n.subscribe<std_msgs::Float64>("command6", 1, &MyPid6Dof::setCommand6CB, this);

            return true;
        }

        void update(const ros::Time& time, const ros::Duration& period){
            
            for(int i=0; i<N_DOF; i++){
                double error = commands_[i] - joints_[i].getPosition();
                double commanded_effort = pidControllers_[i].computeCommand(error, period);
                joints_[i].setCommand(commanded_effort);
            }
        }

        /* --- Callback functions for desired position subscribers --- */
        void setCommand1CB(const std_msgs::Float64ConstPtr& msg){

            commands_[0] = msg->data;
        }
        void setCommand2CB(const std_msgs::Float64ConstPtr& msg){

            commands_[1] = msg->data;
        }
        void setCommand3CB(const std_msgs::Float64ConstPtr& msg){

            commands_[2] = msg->data;
        }
        void setCommand4CB(const std_msgs::Float64ConstPtr& msg){

            commands_[3] = msg->data;
        }
        void setCommand5CB(const std_msgs::Float64ConstPtr& msg){

            commands_[4] = msg->data;
        }
        void setCommand6CB(const std_msgs::Float64ConstPtr& msg){

            commands_[5] = msg->data;
        }

        void starting(const ros::Time& time) {}
        void stopping(const ros::Time& time) {}

        private:
            hardware_interface::JointHandle joints_[N_DOF];
            double commands_[N_DOF];

            ros::Subscriber sub_command1_;
            ros::Subscriber sub_command2_;
            ros::Subscriber sub_command3_;
            ros::Subscriber sub_command4_;
            ros::Subscriber sub_command5_;
            ros::Subscriber sub_command6_;

            control_toolbox::Pid pidControllers_[N_DOF];

            struct {
                double p;
                double i;
                double d;
            } pid_params_[N_DOF];
    };

    PLUGINLIB_EXPORT_CLASS(my_pid_6dof_ns::MyPid6Dof, controller_interface::ControllerBase);
}

