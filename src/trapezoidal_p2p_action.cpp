
#include <drone_planner/trapezoidal_p2p_action.h>

trapezoidal_p2pAction::trapezoidal_p2pAction(std::string name) :
    as_(nh_, name, boost::bind(&trapezoidal_p2pAction::executeCB, this, _1), false),
    action_name_(name) {
    as_.start();
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
    trap_client_ = nh_.serviceClient<drone_planner::trapezoidal_p2p>("/drone_planner/trapezoidal_solver");
    e_publish_pose_ = pevents::CreateEvent(pevents::MANUAL_RESET, pevents::INIT_STATE_FALSE);
    e_new_rate_ = pevents::CreateEvent(pevents::AUTO_RESET, pevents::INIT_STATE_FALSE);
    h_pub_thread_ = std::thread(&trapezoidal_p2pAction::pose_pub_thread, this);
    stop_pose_pub_ = nh_.advertiseService("/drone_planner/publish_pose",
                                          &trapezoidal_p2pAction::publish_pose_srv, this);
  }

  trapezoidal_p2pAction::~trapezoidal_p2pAction(void) {
    h_pub_thread_.join();
  }

  geometry_msgs::PoseStamped trapezoidal_p2pAction::set_pose(
      const geometry_msgs::Point &pos, const double &yaw) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "fcu";
    pose.pose.position = pos;
    pose.pose.orientation.w = cos(yaw/2.0);
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = sin(yaw/2.0);
    return pose;
  }

  void trapezoidal_p2pAction::executeCB(const drone_planner::trapezoidal_p2pGoalConstPtr &goal) {
    // Boolean for success
    result_.success = true;

    // Get sampling time
    double sampling_freq = goal->publish_freq;
    if (sampling_freq <= 0) {
      ROS_WARN("[%s] Cannot execute Trapezoidal trajectory with sampling frequency %f", 
                action_name_.c_str(), goal->publish_freq);
      sampling_freq = 50;
      ROS_WARN("[%s] Executing trajectory with sampling frequency %f.", action_name_.c_str(), sampling_freq);
    } else {
      ROS_INFO("[%s] Executing trajectory with sampling frequency %f.", action_name_.c_str(), sampling_freq);
    }

    // helper variables
    double dt = 1.0/sampling_freq;
    ros::Rate r(sampling_freq);
    m_pose_.lock();
        pub_rate_ = sampling_freq;
        SetEvent(e_new_rate_);
    m_pose_.unlock();

    // Call the service that returns the trapezoidal trajectory ------------------------------------
    drone_planner::trapezoidal_p2p trap_srv;

    // Waypoints
    trap_srv.request.init_pos = goal->initial_point;
    trap_srv.request.final_pos = goal->final_point;
    trap_srv.request.init_yaw = goal->initial_yaw;
    trap_srv.request.final_yaw = goal->final_yaw;

    trap_srv.request.max_vel = goal->max_vel;
    trap_srv.request.max_acc = goal->max_acc;
    trap_srv.request.max_yaw_vel = goal->max_yaw_vel;
    trap_srv.request.max_yaw_acc = goal->max_yaw_accel;

    trap_srv.request.sampling_freq = sampling_freq;
    trap_srv.request.visualize_output = false;

    ROS_INFO("[%s] Calling service %s!", action_name_.c_str(), trap_client_.getService().c_str());
    if (!trap_client_.call(trap_srv)) {
        ROS_WARN("Error when calling the trajectory optimizer! Server might not be running!");
        result_.success = false;
        as_.setSucceeded(result_);
        return;
    }

    if(trap_srv.response.final_time == 0) {
        ROS_WARN("[%s] Trajectory final time equal to zero! Check the waypoint inputs!", action_name_.c_str());
        result_.success = false;
        as_.setSucceeded(result_);
        return;
    }

    // Create the publisher to send positions to Px4 -----------------------------------------------
    feedback_.final_time = trap_srv.response.final_time;

    // publish info to the console for the user
    ROS_INFO("[%s]: Executing trajectory with final time %f!",
        action_name_.c_str(), feedback_.final_time);
    std::vector<drone_planner::PVA_4d> pva_vec = trap_srv.response.pva_vec;

    // Start executing the trajectory
    for(int i=0; i < pva_vec.size(); i++) {

      // Check that halt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("[%s]: Preempted", action_name_.c_str());
        
        // set the action state to preempted
        as_.setPreempted();

        return;
      }

      // Send pose to px4
      m_pose_.lock();
        cur_pose_ = this->set_pose(pva_vec[i].pos, pva_vec[i].yaw);
        cur_pose_.header.seq = i;
      m_pose_.unlock();

      // Trigger the thread to publish pose
      if (i == 0) {
        SetEvent(e_publish_pose_);
      }

      // publish the feedback
      feedback_.current_position = pva_vec[i].pos;
      feedback_.current_yaw = pva_vec[i].yaw;
      feedback_.current_time = pva_vec[i].time;
      as_.publishFeedback(feedback_);

      // Sleep until publishing the next reference
      r.sleep();
    }

    // Sleep for a requested amount of seconds
    ros::Time t0 = ros::Time::now();
    ROS_INFO("[%s]: Sleeping for %4.2f seconds...", action_name_.c_str(), goal->sleep_after_traj);
    while ((ros::Time::now() - t0).toSec() < goal->sleep_after_traj) {
      // Check that halt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("[%s]: Preempted", action_name_.c_str());
        
        // set the action state to preempted
        as_.setPreempted();

        return;
      }
      r.sleep();
    }

    // Set the action state to succeeded
    ROS_INFO("[%s]: Succeeded", action_name_.c_str());
    as_.setSucceeded(result_);
  }

  bool trapezoidal_p2pAction::publish_pose_srv(drone_planner::publish_pose::Request &req,
                                               drone_planner::publish_pose::Response &res) {
    ROS_INFO("publish_pose_srv called!");
    if (req.publish_pose == true) {
      ROS_INFO("Requested to keep publishing pose!");
      // Send pose to px4
      m_pose_.lock();
        cur_pose_ = this->set_pose(req.pos_ref, req.yaw_ref);
      m_pose_.unlock();
      SetEvent(e_publish_pose_);
    } else {
      pevents::ResetEvent(e_publish_pose_);
      ROS_INFO("[%s]: Interrupting pose publishing into %s",
            action_name_.c_str(), pose_pub_.getTopic().c_str());
    }
    res.success = true;
    
    return true;
  }

  void trapezoidal_p2pAction::pose_pub_thread () {

    // Wait until this thread can publish pose
    ros::Rate wait_rate(50);

    geometry_msgs::PoseStamped pose;
    while (ros::ok()) {
      // Wait until the signal for publishing poses has been triggered
      while (pevents::WaitForEvent(e_publish_pose_, 0) != 0) {
        wait_rate.sleep();
      }

      // Check if the publish rate has changed (update it)
      if (pevents::WaitForEvent(e_new_rate_, 0) == 0) {
        m_pose_.lock();
          wait_rate = ros::Rate(pub_rate_);
        m_pose_.unlock();
        ROS_WARN("[%s]: Thread publishing poses at rate %4.2f Hz!", 
                  action_name_.c_str(), 1.0/wait_rate.expectedCycleTime().toSec());
      }

      // Get the pose to publish
      m_pose_.lock();
        pose = cur_pose_;
      m_pose_.unlock();
      pose_pub_.publish(pose);

      wait_rate.sleep();
    }

  }