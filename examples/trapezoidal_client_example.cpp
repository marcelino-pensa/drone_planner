#include "ros/ros.h"

// ROS Action types
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <drone_planner/trapezoidal_p2pAction.h>

// Services
#include <drone_planner/publish_pose.h>

geometry_msgs::Point current_pos_;
double current_yaw_;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const drone_planner::trapezoidal_p2pResultConstPtr& result) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Success: %i", result->success);
}

// Called once when the goal becomes active
void activeCb() {
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const drone_planner::trapezoidal_p2pFeedbackConstPtr& fb) {
  const double current_time = fb->current_time;
  const double final_time = fb->final_time;
  current_pos_ = fb->current_position;
  current_yaw_ = fb->current_yaw;
  printf("\rTime: %4.2f/%4.2f", current_time,final_time);
  // ROS_INFO("Executing: %f/%f", current_time, final_time);
  // ROS_INFO("Pos: %4.2f %4.2f %4.2f", current_pos.x,
  //           current_pos.y, current_pos.z);
  // ROS_INFO("Yaw: %4.2f", current_yaw);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "drone_planner_client");
	ros::NodeHandle node("~");

    // Create action client
	std::string action_name = "/trapezoidal_p2p_action";
    actionlib::SimpleActionClient<drone_planner::trapezoidal_p2pAction> 
    		trapezoidal_p2p_client(action_name, true);
    drone_planner::trapezoidal_p2pGoal action_msg_goal;
    
    // Variable declaration
    geometry_msgs::Point init_pt, final_pt;

    // Wait until server is running
    ROS_INFO("Waiting for server...");
    trapezoidal_p2p_client.waitForServer();
    ROS_INFO("Server found!");

    // Request to start publishing into px4
    ros::ServiceClient client = 
        node.serviceClient<drone_planner::publish_pose>("/drone_planner/publish_pose");
    drone_planner::publish_pose msg;
    init_pt.x = 1.0;  init_pt.y = 0.0;  init_pt.z = 0.0;
    msg.request.pos_ref = init_pt;
    msg.request.yaw_ref = 0.0;
    msg.request.publish_pose = true;
    ROS_INFO("Requesting to start publishing pose!");
    if (client.call(msg)) {
        ROS_INFO("Px4 reference pose publisher started!");
    }

    // Sleep for a few seconds before requesting the trajectory 
    // (not needed, just part of the example)
    ros::Duration(3.5).sleep();

    // Set trajectory properties (these can be changed, 
    // but I am keeping them fixed in this example)
    action_msg_goal.max_vel          = 0.3;
    action_msg_goal.max_acc          = 0.3;
    action_msg_goal.max_yaw_vel      = M_PI/6;
    action_msg_goal.max_yaw_accel    = M_PI/6;
    action_msg_goal.publish_freq     = 50;
    action_msg_goal.sleep_after_traj = 2;

    // Request trajectory
    ROS_INFO("Sending first goal!");
    init_pt.x = 1.0;  init_pt.y = 0.0;  init_pt.z = 0.0;
    final_pt.x = 3.0; final_pt.y = 0.0; final_pt.z = 0.0;
    action_msg_goal.initial_point    = init_pt;
    action_msg_goal.final_point      = final_pt;
    action_msg_goal.initial_yaw      = 0.0;
    action_msg_goal.final_yaw        = M_PI/2;
    trapezoidal_p2p_client.sendGoal(action_msg_goal, doneCb, activeCb, feedbackCb);

    // Wait until the previous trajectory is done
    trapezoidal_p2p_client.waitForResult();

    // Send a new trajectory request
    ROS_INFO("Sending new goal!");
    init_pt.x = 3.0;  init_pt.y = 0.0;  init_pt.z = 0.0;
    final_pt.x = 3.0; final_pt.y = 1.5; final_pt.z = 0.0;
    action_msg_goal.initial_point = init_pt;
    action_msg_goal.final_point   = final_pt;
    action_msg_goal.initial_yaw   = M_PI/2;
    action_msg_goal.final_yaw     = M_PI/2;
    trapezoidal_p2p_client.sendGoal(action_msg_goal, doneCb, activeCb, feedbackCb);

    // Sleep for a few seconds before halting the trajectory
    ros::Duration(3.5).sleep();

    // Preempt trajectory
    trapezoidal_p2p_client.cancelAllGoals();

    // Create trajectory that starts from pre-empted spot and goes up in z direction
    ROS_INFO("Sending final goal!");
    init_pt = current_pos_;
    final_pt = init_pt;
    final_pt.z = final_pt.z + 1.5;
    action_msg_goal.initial_point = init_pt;
    action_msg_goal.final_point   = final_pt;
    action_msg_goal.initial_yaw   = M_PI/2;
    action_msg_goal.final_yaw     = 0.0;
    trapezoidal_p2p_client.sendGoal(action_msg_goal, doneCb, activeCb, feedbackCb);

    // Wait until the previous trajectory is done
    trapezoidal_p2p_client.waitForResult();

    // Request the publisher to stop publishing into mavros
    ROS_INFO("Requesting to stop publishing pose!");
    msg.request.publish_pose = false;
    if (client.call(msg)) {
        ROS_INFO("Px4 reference pose publisher interrupted!");
    }

	return 0;
}