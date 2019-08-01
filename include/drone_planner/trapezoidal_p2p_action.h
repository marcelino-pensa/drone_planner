#include <mutex>
#include <thread>

// ROS action definitions
#include <actionlib/server/simple_action_server.h>
#include <drone_planner/trapezoidal_p2pAction.h>

// ROS msg types
#include <geometry_msgs/PoseStamped.h>

// Service types
#include <drone_planner/trapezoidal_p2p.h>
#include <drone_planner/publish_pose.h>

// Helper library
#include <drone_planner/helper.h>

// POSIX-style event signaling library
#include <pevents/pevents.h>

class trapezoidal_p2pAction {
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<drone_planner::trapezoidal_p2pAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to publish feedback/result
  drone_planner::trapezoidal_p2pFeedback feedback_;
  drone_planner::trapezoidal_p2pResult result_;
  ros::Publisher pose_pub_;
  ros::ServiceClient trap_client_;  // Trapezoidal client
  ros::ServiceServer stop_pose_pub_;

  // Thread-related variables
  std::thread h_pub_thread_;
  std::mutex m_pose_;
  pevents::event_t e_publish_pose_, e_new_rate_;
  geometry_msgs::PoseStamped cur_pose_;
  double pub_rate_;

public:

  trapezoidal_p2pAction(std::string name);

  ~trapezoidal_p2pAction(void);

  // geometry_msgs::PoseStamped set_pose(
  //     const geometry_msgs::Point &pos, const double &yaw);

  void executeCB(const drone_planner::trapezoidal_p2pGoalConstPtr &goal);

  bool publish_pose_srv(drone_planner::publish_pose::Request &req,
                        drone_planner::publish_pose::Response &res);

  void pose_pub_thread ();

};
