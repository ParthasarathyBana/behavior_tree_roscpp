#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_tree_roscpp/ConditionMoveAction.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <string>
#include <stdlib.h>

enum Status {RUNNING, SUCCESS, FAILURE};

class ConditionMoveServer
{
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<behavior_tree_roscpp::ConditionMoveAction> server_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    behavior_tree_roscpp::ConditionMoveFeedback feedback_;  // action feedback (SUCCESS, FAILURE)
    behavior_tree_roscpp::ConditionMoveResult result_;  // action feedback  (same as feedback for us)
    int control_mode;
    ros::Subscriber control_mode_sub;

public:
    explicit ConditionMoveServer(std::string name) :
        server_(nh_, name, boost::bind(&ConditionMoveServer::execute_callback, this, _1), false),
        action_name_(name)
    {
        ROS_INFO("Initializing condition_move_action_server...");
    	control_mode = 21;
    	control_mode_sub = nh_.subscribe<std_msgs::Int8>("/control_mode", 10, &ConditionMoveServer::control_mode_callback, this);
        // Starts the action server
        server_.start();
    }

    ~ConditionMoveServer(void)
    {}

    void execute_callback(const behavior_tree_roscpp::ConditionMoveGoalConstPtr &goal)
    {
        // publish info to the console for the user
        ROS_INFO("Starting Action");

        // start executing the action
        if (control_mode == 21)
        {
            set_status(SUCCESS);
        }
        else
        {
            set_status(FAILURE);
        }
    }


    // returns the status to the client (Behavior Tree)
    void set_status(int status)
    {
        // Set The feedback and result of BT.action
        feedback_.status = status;
        result_.status = feedback_.status;
        // publish the feedback
        server_.publishFeedback(feedback_);
        // setSucceeded means that it has finished the action (it has returned SUCCESS or FAILURE).
        server_.setSucceeded(result_);

        switch (status)  // Print for convenience
        {
        case SUCCESS:
            ROS_INFO("Action %s Succeeded", ros::this_node::getName().c_str() );
            break;
        case FAILURE:
            ROS_INFO("Action %s Failed", ros::this_node::getName().c_str() );
            break;
        default:
            break;
        }
    }

    void control_mode_callback(const std_msgs::Int8::ConstPtr& msg)
    {
    	control_mode = msg->data;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "condition_move_action_server");
    ConditionMoveServer condition_move_action(ros::this_node::getName());
    ros::spin();
    return 0;
}
