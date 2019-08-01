#include "drone_planner/time_optimizer_class.h"


namespace time_optimizer {

TimeOptimizerClass::TimeOptimizerClass(
	               const double &max_vel, const double &max_acc,
	               const double &max_jerk, const double &d_s,
	               const double &rho, const uint &poly_order,
	               const double &sampling_freq, const Eigen::MatrixXd &polyCoeff,
	               const Eigen::VectorXd &polyTime, const bool &visualize_output,
                   std::vector<drone_planner::PVA> *pva_vec, float *final_time) {
	ros::NodeHandle nh("~");
	max_vel_ = max_vel;
	max_acc_ = max_acc;
	max_jerk_ = max_jerk;
    max_vel_sample_ = 1.25*max_vel_;
    max_acc_sample_ = 1.25*max_acc_;
	d_s_ = d_s;
	rho_ = rho;
	poly_num_coeff_ = poly_order + 1;
	polyCoeff_ = polyCoeff;
	polyTime_ = polyTime;
	num_segments_ = polyTime.size() - 1;
	sampling_freq_ = sampling_freq;

    if(sampling_freq_ <= 0) {
        sampling_freq_ = 50;
    }

    // Solve minimum time optimization problem
    // bool success = this->SolveMinTimeOpt("mosek");
    bool success = this->SolveMinTimeOpt("ecos");

    // Retrieve position, velocity and acceleration from optimal solution
    if (success) {
        this->GetTrajectoryPVA(pva_vec, final_time);
    } else {
        *final_time = -1.0;
    }
}

bool TimeOptimizerClass::SolveMinTimeOpt(const std::string &solver) {
	// Structure for the time optimizer
	TrajPolyMono polyTraj(polyCoeff_, polyTime_);

	// run the time optimizer
    ros::Time time_3 = ros::Time::now();
    bool success;
    ecos_sol::MinimumTimeOptimizer time_optimizer;
    success = time_optimizer.MinimumTimeGeneration( polyTraj, max_vel_, max_acc_, max_jerk_, d_s_, rho_);
    time_allocator_ = time_optimizer.GetTimeAllocation();
    ros::Time time_4 = ros::Time::now();

    if(success) {
        // _has_traj = true;    
        ROS_WARN("[drone_planner] Temporal trajectory generated");
        cout<<"[drone_planner] time spent in temporal trajectory is: "<<(time_4 - time_3).toSec()<<endl;

        final_time_ = 0.0;
        for(int i = 0; i < time_allocator_->time.rows(); i++) {   
            int K = time_allocator_->K(i);
            final_time_ += time_allocator_->time(i, K - 1);
        }
        std::cout << "Final time: " << final_time_ << std::endl;

        // cout<<"[TimeOptimizer DEMO] now start publishing commands"<<endl;
        return true;
    } else {
        cout<<"[drone_planner] temporal optimization fail"<<endl;
        cout<<"[drone_planner] possible reasons : " << "\n" <<
        "1 - please check the spatial trajectory,"     <<  "\n" <<
        "2 - numerical issue of the solver, try setting a larger d_s"<<endl;
        return false;
    }
}

void TimeOptimizerClass::GetTrajectoryPVA(std::vector<drone_planner::PVA> *pva_vec, float *final_time) {
    const double dt = 1.0/sampling_freq_;
    geometry_msgs::Point pos;
    geometry_msgs::Vector3 vel, acc;
    double t = 0.0;
    drone_planner::PVA pva;
    while (t < final_time_) {
        this->GetPVAatTime(t, &pva.pos, &pva.vel, &pva.acc);
        pva.time = t;
        pva_vec->push_back(pva);
        t = t + dt;
    }
    this->GetPVAatTime(final_time_, &pva.pos, &pva.vel, &pva.acc);
    pva.time = final_time_;
    pva.vel = helper::ros_vector3(0.0, 0.0, 0.0);
    pva.acc = helper::ros_vector3(0.0, 0.0, 0.0);
    pva_vec->push_back(pva);
    *final_time = final_time_;
}

Eigen::Vector3d TimeOptimizerClass::getPosPoly(const Eigen::MatrixXd &polyCoeff, 
	                       const int &k, const double &t) {
    Eigen::Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ ) {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * poly_num_coeff_, poly_num_coeff_ );
        Eigen::VectorXd time  = VectorXd::Zero( poly_num_coeff_ );
        
        for(int j = 0; j < poly_num_coeff_; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}

Eigen::Vector3d TimeOptimizerClass::getVelPoly(const Eigen::MatrixXd &polyCoeff, 
	                       const int &k, const double &t) {
    Eigen::Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * poly_num_coeff_, poly_num_coeff_ );
        Eigen::VectorXd time  = VectorXd::Zero( poly_num_coeff_ );
        
        for(int j = 0; j < poly_num_coeff_; j ++)
            if(j==0)
                time(j) = 0.0;
            else
                time(j) = j * pow(t, j-1);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}

Eigen::Vector3d TimeOptimizerClass::getAccPoly(const Eigen::MatrixXd &polyCoeff,
	                       const int &k, const double &t) {
    Eigen::Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * poly_num_coeff_, poly_num_coeff_ );
        Eigen::VectorXd time  = VectorXd::Zero( poly_num_coeff_ );

        for(int j = 0; j < poly_num_coeff_; j ++)
            if( j==0 || j==1 )
                time(j) = 0.0;
            else
                time(j) = j * (j - 1) * pow(t, j-2);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}

void TimeOptimizerClass::GetPVAatTime(
				const double &time_in, geometry_msgs::Point *pos,
				geometry_msgs::Vector3 *vel, geometry_msgs::Vector3 *acc) {
	Eigen::MatrixXd time     = time_allocator_->time;
    Eigen::MatrixXd time_acc = time_allocator_->time_acc;
    double t = time_in;

    int idx;
    for(idx = 0; idx < num_segments_; idx++)
    {   
        int K = time_allocator_->K(idx);
        if( t  > time(idx, K - 1))
            t -= time(idx, K - 1);
        else
            break;
    }
    double t_tmp = t;     

    int grid_num = time_allocator_->K(idx);
    
    // now we need to find which grid the time instance belongs to
    int grid_idx;
    for(grid_idx = 0; grid_idx < time_allocator_->K(idx); grid_idx++){
        if (t > time(idx, grid_idx)) continue;
        else{ 
            if(grid_idx > 0) t -= time(idx, grid_idx - 1);
            else             t -= 0.0;
            break;
        }
    }
    
    double delta_t;
    if(grid_idx > 0){	
      delta_t = (time(idx, grid_idx) - time(idx, grid_idx - 1));
    } else {
      delta_t = time(idx, grid_idx) - 0.0;
    }
    
    double delta_s = t * time_allocator_->s_step / delta_t;
    double s = time_allocator_->s(idx, grid_idx) + delta_s;

    // get position data 
    Eigen::Vector3d position   = getPosPoly(polyCoeff_, idx, s);

    // get velocity data
    double s_k   = time_allocator_->s(idx, grid_idx);
    double s_k_1 = time_allocator_->s(idx, grid_idx + 1);
    double b_k   = time_allocator_->b(idx, grid_idx);
    double b_k_1 = time_allocator_->b(idx, grid_idx + 1);

    Vector3d velocity_s1 = getVelPoly(polyCoeff_, idx, s_k  ); 
    Vector3d velocity_s2 = getVelPoly(polyCoeff_, idx, s_k_1);

    Vector3d velocity1   = velocity_s1 * sqrt(b_k);
    Vector3d velocity2   = velocity_s2 * sqrt(b_k_1);
    Vector3d velocity   = velocity1 + (velocity2 - velocity1) * t / delta_t;

// ### NOTE: From what above we get the position and velocity easily.
// ###       positions are the same as the trajectory before re-timing; and velocity are obtained by interpolation between each grid.
// ###       In what follows, we will get the accleration. It's more complicated since each acceleration ais evaluated at the middle of a grid        
    // reset grid_idx and t for time acceleration axis
    t = t_tmp;
    for(grid_idx = 0; grid_idx < time_allocator_->K(idx); grid_idx++)
    {
        if (t > time_acc(idx, grid_idx)) continue;
        else{ 
            if(grid_idx > 0) t -= time_acc(idx, grid_idx - 1);
            else             t -= 0.0;
            break;
        }
    }
    
    if(grid_idx == grid_num)
        t -= time_acc(idx, grid_num - 1);

    // prepare to do accleration interpolation
    Vector3d velocity_s, acceleration_s, acceleration1, acceleration2;
    Vector3d acceleration;

    double a_k;
    if( grid_idx == 0 && idx == 0 ) {   
        // # special case 1: the very first grid of all segments of the trajectory, do interpolation in one grid
        s_k   = time_allocator_->s(idx, 0);
        s_k_1 = time_allocator_->s(idx, 0 + 1);
        
        a_k   = time_allocator_->a(idx, 0);
        b_k   = time_allocator_->b(idx, 0);
        b_k_1 = time_allocator_->b(idx, 0 + 1);

        velocity_s     = getVelPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
        acceleration_s = getAccPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
        acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
        acceleration1 << 0.0, 0.0, 0.0;
        
        acceleration   = acceleration1 + (acceleration2 - acceleration1) * t / time_acc(0, 0); 
    } else if( grid_idx == grid_num && idx == (num_segments_) ) {
        // # special case 2: the very last grid of all segments of the trajectory, do interpolation in one grid
        s_k   = time_allocator_->s(idx, grid_num - 1);
        s_k_1 = time_allocator_->s(idx, grid_num);
        
        a_k   = time_allocator_->a(idx, grid_num - 1);
        b_k   = time_allocator_->b(idx, grid_num - 1);
        b_k_1 = time_allocator_->b(idx, grid_num    );

        velocity_s     = getVelPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
        acceleration_s = getAccPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
        acceleration = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
    } else if(idx > num_segments_) {
        velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
        acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);
    } else  {
        // # regular case: do interpolation between two grids   
        // sub-case 1: two grids are in the same segment
        if(grid_idx < grid_num && grid_idx > 0) {
            // take average accleration in a same segment
            delta_t = (time_acc(idx, grid_idx) - time_acc(idx, grid_idx - 1));
            
            s_k   = time_allocator_->s(idx, grid_idx - 1);
            s_k_1 = time_allocator_->s(idx, grid_idx + 0);
            
            a_k   = time_allocator_->a(idx, grid_idx - 1);
            b_k   = time_allocator_->b(idx, grid_idx - 1);
            b_k_1 = time_allocator_->b(idx, grid_idx + 0);

            velocity_s     = getVelPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration_s = getAccPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration1 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

            s_k   = time_allocator_->s(idx, grid_idx + 0);
            s_k_1 = time_allocator_->s(idx, grid_idx + 1);

            a_k   = time_allocator_->a(idx, grid_idx + 0);
            b_k   = time_allocator_->b(idx, grid_idx + 0);
            b_k_1 = time_allocator_->b(idx, grid_idx + 1);              

            velocity_s     = getVelPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration_s = getAccPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
            acceleration   = acceleration1 + (acceleration2 - acceleration1) * t / delta_t;   
        } else if(grid_idx == grid_num) {
            // sub-case 2: two grids are in consecutive segment, the current grid is in a segment's tail
            // take average accleration between two segments
            delta_t = (time(idx, grid_num - 1) - time_acc(idx, grid_num - 1) + time_acc(idx + 1, 0) );
            
            s_k   = time_allocator_->s(idx, grid_idx - 1);
            s_k_1 = time_allocator_->s(idx, grid_idx);
            
            a_k   = time_allocator_->a(idx, grid_idx - 1);
            b_k   = time_allocator_->b(idx, grid_idx - 1);
            b_k_1 = time_allocator_->b(idx, grid_idx);

            velocity_s     = getVelPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration_s = getAccPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration1 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
            s_k   = time_allocator_->s(idx + 1, 0);
            s_k_1 = time_allocator_->s(idx + 1, 1);

            a_k   = time_allocator_->a(idx + 1, 0);
            b_k   = time_allocator_->b(idx + 1, 0);
            b_k_1 = time_allocator_->b(idx + 1, 1);              

            velocity_s     = getVelPoly(polyCoeff_, idx + 1, (s_k + s_k_1 ) / 2.0);
            acceleration_s = getAccPoly(polyCoeff_, idx + 1, (s_k + s_k_1 ) / 2.0);
            acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
            acceleration  = acceleration1 + (acceleration2 - acceleration1) * t / delta_t;        
        } else if(grid_idx == 0) {
            // sub-case 3: two grids are in consecutive segment, the current grid is in a segment's head
            // take average accleration between two segments
            int grid_num_k = time_allocator_->K(idx - 1);
            delta_t = (time(idx - 1, grid_num_k - 1) - time_acc(idx - 1, grid_num_k - 1) + time_acc(idx, 0) );
            
            s_k   = time_allocator_->s(idx - 1, grid_num_k - 1);
            s_k_1 = time_allocator_->s(idx - 1, grid_num_k    );
            
            a_k   = time_allocator_->a(idx - 1, grid_num_k - 1);
            b_k   = time_allocator_->b(idx - 1, grid_num_k - 1);
            b_k_1 = time_allocator_->b(idx - 1, grid_num_k    );

            velocity_s     = getVelPoly(polyCoeff_, idx - 1, (s_k + s_k_1 ) / 2.0);
            acceleration_s = getAccPoly(polyCoeff_, idx - 1, (s_k + s_k_1 ) / 2.0);
            acceleration1  = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

            s_k   = time_allocator_->s(idx, 0);
            s_k_1 = time_allocator_->s(idx, 0 + 1);
            
            a_k   = time_allocator_->a(idx, 0);
            b_k   = time_allocator_->b(idx, 0);
            b_k_1 = time_allocator_->b(idx, 0 + 1);

            velocity_s     = getVelPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration_s = getAccPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration2  = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
            acceleration   = acceleration1 + (acceleration2 - acceleration1) * (t + time(idx - 1, grid_num_k - 1) - time_acc(idx - 1, grid_num_k - 1)) / delta_t;   
        } else {
            // no else
            ROS_WARN("I got here!!");
        }
    }

    // if ((std::fabs(acceleration(0)) > max_acc_sample_) || (std::fabs(acceleration(1)) > max_acc_sample_) ||
    //     (std::fabs(acceleration(2)) > max_acc_sample_)) {
    //     ROS_WARN("Something weird has happened...");
    //     std::cout << "time_in: " << time_in << std::endl;
    //     std::cout << "grid_idx: " << grid_idx << std::endl;
    //     std::cout << "grid_num: " << grid_idx << std::endl;
    //     std::cout << "num_segments_: " << num_segments_ << std::endl;
    //     std::cout << "idx: " << idx << std::endl;
    //     std::cout << "s_k: " << s_k << std::endl;
    //     std::cout << "s_k_1: " << s_k_1 << std::endl;
    //     std::cout << "a_k: " << a_k << std::endl;
    //     std::cout << "b_k: " << b_k << std::endl;
    //     std::cout << "b_k_1: " << b_k_1 << std::endl;
    //     // cin.get();
    // }

    pos->x = position(0);
    pos->y = position(1);
    pos->z = position(2);
    vel->x = helper::saturate(velocity(0), -max_vel_sample_, max_vel_sample_);
    vel->y = helper::saturate(velocity(1), -max_vel_sample_, max_vel_sample_);
    vel->z = helper::saturate(velocity(2), -max_vel_sample_, max_vel_sample_);
    acc->x = helper::saturate(acceleration(0), -max_acc_sample_, max_acc_sample_);
    acc->y = helper::saturate(acceleration(1), -max_acc_sample_, max_acc_sample_);
    acc->z = helper::saturate(acceleration(2), -max_acc_sample_, max_acc_sample_);
}

}  // namespace time_optimizer