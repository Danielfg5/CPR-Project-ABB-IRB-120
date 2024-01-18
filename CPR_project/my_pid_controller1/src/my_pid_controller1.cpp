#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <control_toolbox/pid.h>
#include <time.h>

namespace my_pid_controller1_ns {

    class MyPidController1 : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

        /* ----- Controller initialization function ----- */
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) {
            
            /* --- Load parameters from the parameter server --- */
            std::string joint_name_;
            if(!n.getParam("joint", joint_name_)){
                ROS_ERROR("Could not find joint name.");
                return false;
            }
            joint_ = hw->getHandle(joint_name_);
            command_ = joint_.getPosition();    // Initialize desired (target) position to current position

            if(!n.getParam("gain", gain_)){
                ROS_ERROR("Could not find parameter value (gain).");
                return false;
            }
            if(!n.getParam("integral", integral_)){
                ROS_ERROR("Could not find parameter value (integral).");
                return false;
            }

            if(!n.getParam("derivative", derivative_)){
                ROS_ERROR("Could not find parameter value (derivative).");
                return false;
            }

            /* --- Initialize subscriber for desired position --- */
            sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &MyPidController1::setCommandCB, this);

            /* --- Initialize PID controller --- */
            pidController_.initPid(gain_, integral_, derivative_, 100.0, -100.0);

            return true;
        }

        /* ----- Controller loop update function ----- */
        void update(const ros::Time& time, const ros::Duration& period){
            
            double error = command_ - joint_.getPosition();
            double commanded_effort = pidController_.computeCommand(error, period);
            joint_.setCommand(commanded_effort);
        }

        /* ----- Callback function for desired position subscriber ----- */
        void setCommandCB(const std_msgs::Float64ConstPtr& msg){

            command_ = msg->data;
        }

        void starting(const ros::Time& time) {}
        void stopping(const ros::Time& time) {}

        private:
            hardware_interface::JointHandle joint_;
            double gain_;
            double integral_;
            double derivative_;
            double command_;
            ros::Subscriber sub_command_;
            control_toolbox::Pid pidController_;
    };

    PLUGINLIB_EXPORT_CLASS(my_pid_controller1_ns::MyPidController1, controller_interface::ControllerBase);
}

