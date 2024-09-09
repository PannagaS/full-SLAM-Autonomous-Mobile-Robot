#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>

/*
An action model takes in an action and uses it to approximate where
the robot actually is given its previous estimated probabilities.


*/

ActionModel::ActionModel(void)
: k1_(0.005f)
, k2_(0.025f)
, min_dist_(0.025)
, min_theta_(0.08)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}

void ActionModel::resetPrevious(const mbot_lcm_msgs::pose2D_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose2D_t& odometry)
{
    if (!initialized_)
    {
        resetPrevious(odometry);
        initialized_ = true;
    }

    float dx_ = odometry.x - previousPose_.x; //changed this from previousPose_.y to previousPose_.x
    float dy_ = odometry.y - previousPose_.y;
    float dtheta_ = odometry.theta - previousPose_.theta;
    
    
    // Maybe add code here to reverse direction if turn is too much (greater than pi/2) -> ???
    
    //pans 
    float direction = 1.0; 
    if (std::abs(delta_rot1) > M_PI/2.0)
    {
        delta_rot1 = angle_diff(M_PI, delta_rot1);
        direction = -1.0;
    }


    delta_rot1 = angle_diff(std::atan2(dy_,dx_), previousPose_.theta); 
    delta_trans = sqrt(dx_*dx_+dy_*dy_);
    delta_rot2 = angle_diff(dtheta_, delta_rot1);

    // Check if we've moved
    // bool moved = abs(dtheta_) > min_theta_ || abs(delta_trans) > min_dist_;
    bool moved = (dx_ !=0.0) || (dy_ != 0.0) ||(dtheta_ != 0.0);
    if (moved)
    {
        //Update stds for rot1, trans, and rot2
        rot1Std_ = std::sqrt(k1_ * std::abs(delta_rot1));
        transStd_ = std::sqrt(k2_ * std::abs(delta_trans));
        rot2Std_ = std::sqrt(k1_ * std::abs(delta_rot2));

    }

    delta_trans = delta_trans * direction; //pans
    resetPrevious(odometry);
    utime_ = odometry.utime;
    return moved;   
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    //particle_t contains pose2D_t pose, pose2D_t parent_pose, and double weight
    
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;
    
    //Sample rot1, trans, and rot2 from normal distibutions
    float sampledRot1 = std::normal_distribution<>(delta_rot1, rot1Std_)(numberGenerator_);
    float sampledTrans = std::normal_distribution<>(delta_trans, transStd_)(numberGenerator_);
    float sampledRot2 = std::normal_distribution<>(delta_rot2, rot2Std_)(numberGenerator_);

    // Use the action model equations in Lec16 Slide 21 to get new pose info
    newSample.pose.x += sampledTrans * cos(sample.pose.theta + sampledRot1);
    newSample.pose.y += sampledTrans * sin(sample.pose.theta + sampledRot1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampledRot1 + sampledRot2);

    // Update newSample's utime and parent_pose
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;

    return newSample;
}
