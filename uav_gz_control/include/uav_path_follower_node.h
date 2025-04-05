#ifndef UAV_PATH_FOLLOWER_NODE_H
#define UAV_PATH_FOLLOWER_NODE_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace uav_control{

class UAVPathFollowerNode{
  public:
    UAVPathFollowerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~UAVPathFollowerNode();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Publishers
    ros::Publisher cmd_pose_pub_;
    ros::Publisher cmd_pose_vis_pub_;

    // Subscribers
    ros::Subscriber cmd_path_sub_;
    ros::Subscriber odometry_sub_;


    ros::Timer command_timer_;

    double trans_tolerance_ = 0.25;
    double rot_tolerance_ = 0.5;

    std::vector<geometry_msgs::Pose> poses_;
    geometry_msgs::Pose current_pose_;
    size_t current_pose_index_ = 0;

    void publishPoseCommand(const ros::TimerEvent& e);
    void odometryCallback(const nav_msgs::Odometry& odo);
    void pathCallback(const nav_msgs::Path& path);
    bool isAtPosition(const geometry_msgs::Pose& target_pose);

    inline void truncateAngle(double &angle)
    {
      if (angle < -M_PI)
        angle += 2 * M_PI;
      else if (angle > M_PI)
        angle -= 2*M_PI;
    }
};

}

#endif // UAV_PATH_FOLLOWER_NODE_H
