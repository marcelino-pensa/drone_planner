#include "drone_planner/visualization_functions.h"


namespace visualization {

TrajPublisher::TrajPublisher() { }

TrajPublisher::TrajPublisher(ros::NodeHandle *nh) {
    vis_traj_width_ = 0.10;
    yaw_vec_length_ = 0.5;

    // Declare trajectory publishers
    wp_traj_vis_pub_ = nh->advertise<visualization_msgs::Marker>("spatial_trajectory", 1);
    wp_path_vis_pub_ = nh->advertise<visualization_msgs::Marker>("waypoint_path"     , 1);
    vis_pos_pub_     = nh->advertise<visualization_msgs::Marker>("desired_position", 1);    
    vis_vel_pub_     = nh->advertise<visualization_msgs::Marker>("desired_velocity", 1);    
    vis_acc_pub_     = nh->advertise<visualization_msgs::Marker>("desired_acceleration", 1);
    vis_yaw_pub_     = nh->advertise<visualization_msgs::Marker>("desired_yaw", 1);

    // Set the structures for the path publishers
    vis_pos_.id = vis_vel_.id = vis_acc_.id = 0;
    vis_pos_.header.frame_id = vis_vel_.header.frame_id = "/map";
    vis_acc_.header.frame_id = vis_yaw_.header.frame_id = "/map";
    
    vis_pos_.ns = "pos";
    vis_pos_.type   = visualization_msgs::Marker::SPHERE_LIST;
    vis_pos_.action = visualization_msgs::Marker::ADD;
    vis_pos_.color = Color::Blue();
    vis_pos_.pose.orientation.w = 1.0;
    vis_pos_.scale.x = 0.2; vis_pos_.scale.y = 0.2; vis_pos_.scale.z = 0.2;

    vis_vel_.ns = "vel";
    vis_vel_.type = visualization_msgs::Marker::ARROW;
    vis_vel_.action = visualization_msgs::Marker::ADD;
    vis_vel_.color = Color::Green();
    vis_vel_.scale.x = 0.1; vis_vel_.scale.y = 0.2; vis_vel_.scale.z = 0.2;

    vis_acc_.ns = "acc";
    vis_acc_.type = visualization_msgs::Marker::ARROW;
    vis_acc_.action = visualization_msgs::Marker::ADD;
    vis_acc_.color = Color::Yellow();
    vis_acc_.scale.x = 0.1; vis_acc_.scale.y = 0.2; vis_acc_.scale.z = 0.2;

    vis_yaw_.ns = "yaw";
    vis_yaw_.type = visualization_msgs::Marker::ARROW;
    vis_yaw_.action = visualization_msgs::Marker::ADD;
    vis_yaw_.color = Color::White();
    vis_yaw_.scale.x = 0.1; vis_yaw_.scale.y = 0.2; vis_yaw_.scale.z = 0.2;

    traj_vis_.header.frame_id = "map";
    traj_vis_.ns = "trajectory_waypoints";
    traj_vis_.id = 0;
    traj_vis_.type = visualization_msgs::Marker::SPHERE_LIST;
    traj_vis_.action = visualization_msgs::Marker::ADD;
    traj_vis_.scale.x = vis_traj_width_;
    traj_vis_.scale.y = vis_traj_width_;
    traj_vis_.scale.z = vis_traj_width_;
    traj_vis_.pose.orientation.w = 1.0;
    traj_vis_.color = Color::Red();
}

void TrajPublisher::VisualizePath(const Eigen::MatrixXd &polyCoeff, const Eigen::VectorXd &time) {
    traj_vis_.header.stamp = ros::Time::now();

    double traj_len = 0.0;
    int count = 0;
    Eigen::Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    traj_vis_.points.clear();
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;

    const uint poly_num_coeff = polyCoeff.cols()/3;
    for(int i = 0; i < time.size(); i++ )
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
          pos = this->getPosPoly(polyCoeff, i, t, poly_num_coeff);
          cur(0) = pt.x = pos(0);
          cur(1) = pt.y = pos(1);
          cur(2) = pt.z = pos(2);
          traj_vis_.points.push_back(pt);

          if (count) traj_len += (pre - cur).norm();
          pre = cur;
        }
    }

    wp_traj_vis_pub_.publish(traj_vis_);
}

void TrajPublisher::VisualizePath(const std::vector<Eigen::Vector3d> pos_hist) {
    traj_vis_.header.stamp = ros::Time::now();
    traj_vis_.points.clear();
    geometry_msgs::Point pt;

    for(int i = 0; i < pos_hist.size(); i++ ) {
        pt.x = pos_hist[i](0);
        pt.y = pos_hist[i](1);
        pt.z = pos_hist[i](2);
        traj_vis_.points.push_back(pt);
    }
    wp_traj_vis_pub_.publish(traj_vis_);
}

void TrajPublisher::VisualizeWaypoints(const Eigen::MatrixXd &path) {
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "/map";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_path";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;

    
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < path.rows(); i++){
      geometry_msgs::Point p;
      p.x = path(i, 0);
      p.y = path(i, 1); 
      p.z = path(i, 2); 

      points.points.push_back(p);

      if( i < (path.rows() - 1) )
      {
          geometry_msgs::Point p_line;
          p_line = p;
          line_list.points.push_back(p_line);
          p_line.x = path(i+1, 0);
          p_line.y = path(i+1, 1); 
          p_line.z = path(i+1, 2);
          line_list.points.push_back(p_line);
      }
    }

    wp_path_vis_pub_.publish(points);
    wp_path_vis_pub_.publish(line_list);
}

// Publish trajectory (real-time visualization)
void TrajPublisher::PubPVA_Markers(const geometry_msgs::Point &pos, const geometry_msgs::Vector3 &vel,
                                   const geometry_msgs::Vector3 &acc) {   

    vis_pos_.header.stamp = ros::Time::now();
    vis_vel_.header.stamp = ros::Time::now();
    vis_acc_.header.stamp = ros::Time::now();
    vis_pos_.points.clear();
    vis_vel_.points.clear();
    vis_acc_.points.clear();

    vis_pos_.points.push_back(pos);
    vis_vel_.points.push_back(pos);
    vis_acc_.points.push_back(pos);

    geometry_msgs::Point pt;
    pt.x = pos.x + vel.x;
    pt.y = pos.y + vel.y;
    pt.z = pos.z + vel.z;
    vis_vel_.points.push_back(pt);

    pt.x = pos.x + acc.x;
    pt.y = pos.y + acc.y;
    pt.z = pos.z + acc.z;
    vis_acc_.points.push_back(pt);
    
    vis_pos_pub_.publish(vis_pos_);
    vis_vel_pub_.publish(vis_vel_);
    vis_acc_pub_.publish(vis_acc_);
}

void TrajPublisher::PubYaw_Marker(const geometry_msgs::Point &pos, 
                                  const double &yaw) {
    vis_yaw_.header.stamp = ros::Time::now();
    vis_yaw_.points.clear();
    vis_yaw_.points.push_back(pos);
    geometry_msgs::Point pt;
    pt.x = pos.x + yaw_vec_length_*cos(yaw);
    pt.y = pos.y + yaw_vec_length_*sin(yaw);
    pt.z = pos.z;
    vis_yaw_.points.push_back(pt);
    vis_yaw_pub_.publish(vis_yaw_);
}

// void TrajPublisher::PubRealTimeTraj(const std::vector<p4_ros::PVA> &pva_vec,
//                                     const double &sampling_freq,
//                                     const double &final_time) {
//     ROS_INFO("[p4_services] Publishing trajectory...");
//     ros::Rate rate(sampling_freq);
//     for (uint i = 0; i < pva_vec.size(); i++) {
//         p4_ros::PVA cur_pva = pva_vec[i];
//         printf("\rPublishing trajectory for time %4.2f/%4.2f sec", cur_pva.time, final_time);
//         this->PubPVA_Markers(cur_pva.pos, cur_pva.vel, cur_pva.acc);
//         rate.sleep();
//     }
//     printf("\n\r");
// }

void TrajPublisher::PubRealTimeTraj(const std::vector<drone_planner::PVA_4d> &pva_vec,
                                    const double &sampling_freq,
                                    const double &final_time) {
    ROS_INFO("[drone_planner] Publishing trajectory...");
    ros::Rate rate(sampling_freq);
    for (uint i = 0; i < pva_vec.size(); i++) {
        drone_planner::PVA_4d cur_pva = pva_vec[i];
        // std::cout << cur_pva.pos.x << " " << cur_pva.pos.y << " " << cur_pva.pos.z << std::endl;
        printf("\rPublishing trajectory for time %4.2f/%4.2f sec", cur_pva.time, final_time);
        this->PubPVA_Markers(cur_pva.pos, cur_pva.vel, cur_pva.acc);
        this->PubYaw_Marker(cur_pva.pos, cur_pva.yaw);
        rate.sleep();
    }
    printf("\n\r");
}

// void TrajPublisher::plot_results_gnuplot(const std::vector<p4_ros::PVA> &pva_vec) {
//     {
//         std::vector<double> t_hist, x_hist, y_hist, z_hist;
//         for (uint i = 0; i < pva_vec.size(); i++) {
//             t_hist.push_back(pva_vec[i].time);
//             x_hist.push_back(pva_vec[i].pos.x);
//             y_hist.push_back(pva_vec[i].pos.y);
//             z_hist.push_back(pva_vec[i].pos.z);
//         }

//         Gnuplot gp;
//         gp << "plot '-' using 1:2 with lines title 'X-Position'";
//         gp << ", '-' using 1:2 with lines title 'Y-Position'";
//         gp << ", '-' using 1:2 with lines title 'Z-Position'";
//         gp << std::endl;
//         gp.send1d(boost::make_tuple(t_hist, x_hist));
//         gp.send1d(boost::make_tuple(t_hist, y_hist));
//         gp.send1d(boost::make_tuple(t_hist, z_hist));
//         gp << "set grid" << std::endl;
//         gp << "replot" << std::endl;
//     }

//     {
//          std::vector<double> t_hist, x_hist, y_hist, z_hist;
//         for (uint i = 0; i < pva_vec.size(); i++) {
//             t_hist.push_back(pva_vec[i].time);
//             x_hist.push_back(pva_vec[i].vel.x);
//             y_hist.push_back(pva_vec[i].vel.y);
//             z_hist.push_back(pva_vec[i].vel.z);
//         }

//         Gnuplot gp;
//         gp << "plot '-' using 1:2 with lines title 'X-Velocity'";
//         gp << ", '-' using 1:2 with lines title 'Y-Velocity'";
//         gp << ", '-' using 1:2 with lines title 'Z-Velocity'";
//         gp << std::endl;
//         gp.send1d(boost::make_tuple(t_hist, x_hist));
//         gp.send1d(boost::make_tuple(t_hist, y_hist));
//         gp.send1d(boost::make_tuple(t_hist, z_hist));
//         gp << "set grid" << std::endl;
//         gp << "replot" << std::endl;
//     }

//     {
//          std::vector<double> t_hist, x_hist, y_hist, z_hist;
//         for (uint i = 0; i < pva_vec.size(); i++) {
//             t_hist.push_back(pva_vec[i].time);
//             x_hist.push_back(pva_vec[i].acc.x);
//             y_hist.push_back(pva_vec[i].acc.y);
//             z_hist.push_back(pva_vec[i].acc.z);
//         }

//         Gnuplot gp;
//         gp << "plot '-' using 1:2 with lines title 'X-Acceleration'";
//         gp << ", '-' using 1:2 with lines title 'Y-Acceleration'";
//         gp << ", '-' using 1:2 with lines title 'Z-Acceleration'";
//         gp << std::endl;
//         gp.send1d(boost::make_tuple(t_hist, x_hist));
//         gp.send1d(boost::make_tuple(t_hist, y_hist));
//         gp.send1d(boost::make_tuple(t_hist, z_hist));
//         gp << "set grid" << std::endl;
//         gp << "replot" << std::endl;
//     }
// }

Eigen::Vector3d TrajPublisher::getPosPoly(const Eigen::MatrixXd &polyCoeff, 
                           const int &k, const double &t, const uint &n_coeff) {
    Eigen::Vector3d ret;


    for ( int dim = 0; dim < 3; dim++ ) {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * n_coeff, n_coeff );
        Eigen::VectorXd time  = Eigen::VectorXd::Zero( n_coeff );
        
        for(int j = 0; j < n_coeff; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}

}  // namespace visualization