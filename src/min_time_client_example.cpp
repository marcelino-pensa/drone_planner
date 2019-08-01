
// ROS
#include "ros/ros.h"

#include "drone_planner/min_time.h"
#include "drone_planner/helper.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "min_time_client");
    ros::NodeHandle node("~");

    // ros::spin();
    ros::Rate loop_rate(10);
    loop_rate.sleep();


    ros::ServiceClient client = node.serviceClient<drone_planner::min_time>("/drone_planner/min_time_solver");
    p4_ros::min_time req;

    // Simple takeoff
    // req.request.pos_array.push_back(p4_helper::ros_point(0.0,  0.0, 0.0));
    // req.request.pos_array.push_back(p4_helper::ros_point(0.25, 0.1, 0.75));
    // req.request.pos_array.push_back(p4_helper::ros_point(0.50, 0.2, 1.5));

    // Scan Example
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, -0.3, -0.7));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, 2, -0.7));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, 4, -0.7));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, 6.2, -0.7));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, 6.2, -1.4));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, 4, -1.4));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, 2, -1.4));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, 0.2, -1.4));

    // // Scan example 2
    req.request.pos_array.push_back(p4_helper::ros_point(0.0, 0.0, 1.0));
    req.request.pos_array.push_back(p4_helper::ros_point(1.0, 0.0, 1.0));
    req.request.pos_array.push_back(p4_helper::ros_point(2.0, 0.0, 1.0));
    req.request.pos_array.push_back(p4_helper::ros_point(3.0, 0.0, 1.0));
    req.request.pos_array.push_back(p4_helper::ros_point(4.0, 0.0, 1.0));
    req.request.pos_array.push_back(p4_helper::ros_point(4.0, 0.0, 1.5));
    req.request.pos_array.push_back(p4_helper::ros_point(3.0, 0.0, 1.5));
    req.request.pos_array.push_back(p4_helper::ros_point(2.0, 0.0, 1.5));
    req.request.pos_array.push_back(p4_helper::ros_point(1.0, 0.0, 1.5));
    req.request.pos_array.push_back(p4_helper::ros_point(0.0, 0.0, 1.5));
    req.request.pos_array.push_back(p4_helper::ros_point(0.0, 0.0, 2.0));
    req.request.pos_array.push_back(p4_helper::ros_point(1.0, 0.0, 2.0));
    req.request.pos_array.push_back(p4_helper::ros_point(2.0, 0.0, 2.0));
    req.request.pos_array.push_back(p4_helper::ros_point(3.0, 0.0, 2.0));
    req.request.pos_array.push_back(p4_helper::ros_point(4.0, 0.0, 2.0));

    req.request.sampling_freq = 30;
    req.request.corridor_width = 0.1;
    req.request.max_vel = 0.3;
    req.request.max_acc = 0.3;
    req.request.max_jerk = 0.3;
    req.request.visualize_output = false;

    for (uint i = 0; i < 1; i++) {
        ros::Time t0 = ros::Time::now();
        ROS_INFO("[min_time_client] Calling service %s!", client.getService().c_str());
        std::cout << "attempt " << i << std::endl;
        if (!client.call(req)) {
            ROS_WARN("[min_time_client] Error when calling the trajectory optimizer!");
        }
        ros::Time t1 = ros::Time::now();
        ROS_INFO("[min_time_client] Solution time: %f", (t1 - t0).toSec());

        if(req.response.final_time <= 0) {
            ROS_WARN("[min_time_client] Solution failed!");
            break;
        }
    }
    
    // while (ros::ok()) {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

  return 0;
}