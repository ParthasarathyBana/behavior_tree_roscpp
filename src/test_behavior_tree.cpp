#include <behavior_tree_roscpp/bt_service_node.h>
#include <behavior_tree_roscpp/bt_action_node.h>
#include <ros/ros.h>
#include <behavior_tree_roscpp/ConditionMoveAction.h>
#include <behavior_tree_roscpp/RobotMoveAction.h>
#include <behavior_tree_roscpp/ConditionStopAction.h>
#include <behavior_tree_roscpp/RobotStopAction.h>
#include "ros/package.h"

using namespace BT;

class RobotMoveServer: public RosActionNode<behavior_tree_roscpp::RobotMoveAction>
{
public:
	RobotMoveServer(ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
	RosActionNode<behavior_tree_roscpp::RobotMoveAction>(handle, node_name, conf) {}

	// static PortsList providedPorts()
	// {
	// }

	bool sendGoal(GoalType& goal) override
	{
		ROS_INFO("robot_move_action: sending request.");
		return true;
	}

	NodeStatus onResult(const ResultType& res) override
	{
		ROS_INFO("robot_move_action: result received.");
		return NodeStatus::SUCCESS;
	}

	virtual NodeStatus onFailedRequest(FailureCause failure) override
	{
		ROS_ERROR("RobotMoveAction request failed %d", static_cast<int>(failure));
		return NodeStatus::FAILURE;
	}

	void halt() override
	{
		if( status() == NodeStatus::RUNNING )
		{
			ROS_WARN("robot_move_action halted");
			BaseClass::halt();
		}
	}

};

class RobotStopServer: public RosActionNode<behavior_tree_roscpp::RobotStopAction>
{
public:
	RobotStopServer(ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
	RosActionNode<behavior_tree_roscpp::RobotStopAction>(handle, node_name, conf) {}

	// static PortsList providedPorts()
	// {
	// }

	bool sendGoal(GoalType& goal) override
	{
		ROS_INFO("robot_stop_action: sending request.");
		return true;
	}

	NodeStatus onResult(const ResultType& res) override
	{
		ROS_INFO("robot_stop_ction: result received.");
		return NodeStatus::SUCCESS;
	}

	virtual NodeStatus onFailedRequest(FailureCause failure) override
	{
		ROS_ERROR("RobotStopAction request failed %d", static_cast<int>(failure));
		return NodeStatus::FAILURE;
	}

	void halt() override
	{
		if( status() == NodeStatus::RUNNING )
		{
			ROS_WARN("robot_stop_action halted");
			BaseClass::halt();
		}
	}

};

class ConditionMoveServer: public RosActionNode<behavior_tree_roscpp::ConditionMoveAction>
{
public:
	ConditionMoveServer(ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
	RosActionNode<behavior_tree_roscpp::ConditionMoveAction>(handle, node_name, conf) {}
	// static PortsList providedPorts()
	// {
	// 	return { InputPort<int>("control_mode_no") };
	// }

	bool sendGoal(GoalType& goal) override
	{
		// if (!getInput<int>("control_mode_no", goal.parameter))
		// {
		// 	ROS_ERROR("port not provided with input");
		// 	return false;
		// }
		ROS_INFO("condition_move_action: sending request.");
		return true;
	}

	NodeStatus onResult(const ResultType& res) override
	{
		if (res.status)
		{
			ROS_INFO("condition_move_action: result correct.");
			return NodeStatus::SUCCESS;
		}
		else
		{
			ROS_INFO("condition_move_action: result incorrect.");
			return NodeStatus::SUCCESS;
		}
		
	}

};

class ConditionStopServer: public RosActionNode<behavior_tree_roscpp::ConditionStopAction>
{
public:
	ConditionStopServer(ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
	RosActionNode<behavior_tree_roscpp::ConditionStopAction>(handle, node_name, conf) {}

	// static PortsList providedPorts()
	// {
	// }

	bool sendGoal(GoalType& goal) override
	{
		ROS_INFO("condition_stop_action: sending request.");
		return true;
	}

	NodeStatus onResult(const ResultType& res) override
	{
		ROS_INFO("condition_stop_action: result received.");
		return NodeStatus::SUCCESS;
	}

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_behavior_tree");
	ros::NodeHandle nh("~");
	ROS_INFO("Started Node");
	std::string tree_path;
	nh.getParam("tree_path", tree_path);
	
	BehaviorTreeFactory factory;
	
	ROS_INFO("Registered Factory Node");

	RegisterRosAction<ConditionMoveServer>(factory, "condition_move_action", nh);
	RegisterRosAction<RobotMoveServer>(factory, "robot_move_action", nh);
	RegisterRosAction<ConditionStopServer>(factory, "condition_stop_action", nh);
	RegisterRosAction<RobotStopServer>(factory, "robot_stop_action", nh);

	ROS_INFO("Recieved RosActionNode Response");

	auto tree = factory.createTreeFromFile(tree_path);

	NodeStatus status = NodeStatus::IDLE;

	while( ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
	{
		ROS_INFO("Entered while loop");
		ros::spinOnce();
		status = tree.tickRoot();
		std::cout << status << std::endl;
		ros::Duration sleep_time(0.01);
		sleep_time.sleep();
	}

	return 0;
}