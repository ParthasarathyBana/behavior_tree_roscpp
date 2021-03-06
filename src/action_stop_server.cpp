#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_tree_roscpp/RobotStopAction.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

enum Status {RUNNING, SUCCESS, FAILURE};

class RobotStopServer
{
protected:
	ros::NodeHandle nh_;
	// NodeHandle instance must be created before this line. Otherwise strange error occurs.
	actionlib::SimpleActionServer<behavior_tree_roscpp::RobotStopAction> server_;
	std::string action_name_;
	// create messages that are used to published feedback/result
	behavior_tree_roscpp::RobotStopFeedback feedback_;
	behavior_tree_roscpp::RobotStopResult result_;
	
	ros::Publisher move_pub;
    geometry_msgs::Twist msg;
	float speed_;

public:
	RobotStopServer(std::string name) :
	server_(nh_, name, boost::bind(&RobotStopServer::executeCB, this, _1), false),
	action_name_(name)
	{
		ROS_INFO("Initializing action_stop_server...");
		server_.start();
		move_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	}

	~RobotStopServer(void)
	{
	}

	void executeCB(const behavior_tree_roscpp::RobotStopGoalConstPtr &goal)
	{
		// publish info to the console for the user
        ROS_INFO("Starting Action");

		speed_ = 0;

		while(ros::ok())
		{
			msg.linear.x = speed_;
			msg.angular.z = 0;

			move_pub.publish(msg);

			ROS_INFO_STREAM("Sending velocity command:" << " linear = " << msg.linear.x << " angular = " << msg.angular.z);
			
			ros::Duration(1.5).sleep();
			
			

			if (msg.linear.x == speed_ && msg.angular.z == 0)
			{
				set_status(SUCCESS);
				break;
			}
			else
			{
				set_status(FAILURE);
				break;
			}
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
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_stop_action_server");
	ros::NodeHandle n;
	RobotStopServer robot_stop_action(ros::this_node::getName());
    ros::spin();
    return 0;
}