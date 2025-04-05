#include <ros/ros.h>
#include <uav_path_follower_node.h>

namespace uav_control{

UAVPathFollowerNode::UAVPathFollowerNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),private_nh_(private_nh){

    cmd_path_sub_ = nh_.subscribe("command/path", 1,
        &UAVPathFollowerNode::pathCallback, this);

    odometry_sub_ = nh_.subscribe("odometry", 1,
        &UAVPathFollowerNode::odometryCallback, this);

    cmd_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("command/pose", 1);
    cmd_pose_vis_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose_vis", 1);

    command_timer_ = nh_.createTimer(ros::Duration(0.05), &UAVPathFollowerNode::publishPoseCommand, this);

}

UAVPathFollowerNode::~UAVPathFollowerNode() { }

void UAVPathFollowerNode::pathCallback(const nav_msgs::Path& path) {
    poses_.clear();
    current_pose_index_ = 0;
    for (int i=0; i < path.poses.size(); i++){
        geometry_msgs::Pose pose;
        pose.position.x = path.poses[i].pose.position.x;
        pose.position.y = path.poses[i].pose.position.y;
        pose.position.z = path.poses[i].pose.position.z;
        pose.orientation.x = path.poses[i].pose.orientation.x;
        pose.orientation.y = path.poses[i].pose.orientation.y;
        pose.orientation.z = path.poses[i].pose.orientation.z;
        pose.orientation.w = path.poses[i].pose.orientation.w;
        poses_.push_back(pose);
    }

}

void UAVPathFollowerNode::publishPoseCommand(const ros::TimerEvent& e) {
    if (current_pose_index_ < poses_.size()) {
        geometry_msgs::Pose target_pose = poses_[current_pose_index_];
        cmd_pose_pub_.publish(target_pose);
        geometry_msgs::PoseStamped target_pose_vis;
        target_pose_vis.header.frame_id = "world";
        target_pose_vis.pose = target_pose;
        cmd_pose_vis_pub_.publish(target_pose_vis);
        // ROS_INFO("Moving to pose %d/%d", current_pose_index_ + 1, (int)poses_.size());

        if (isAtPosition(target_pose)) {
            current_pose_index_++;
        }
    }
}

bool UAVPathFollowerNode::isAtPosition(const geometry_msgs::Pose& target_pose) {
    double dx = current_pose_.position.x - target_pose.position.x;
    double dy = current_pose_.position.y - target_pose.position.y;
    double dz = current_pose_.position.z - target_pose.position.z;
    double distance = sqrt(dx*dx + dy*dy + dz*dz);

    double target_yaw = tf::getYaw(target_pose.orientation);
    double current_yaw = tf::getYaw(current_pose_.orientation);
    double d_yaw = current_yaw - target_yaw;
    truncateAngle(d_yaw);

    return (distance < trans_tolerance_) && (std::abs(d_yaw) < rot_tolerance_);
}

void UAVPathFollowerNode::odometryCallback(const nav_msgs::Odometry& odo) {
    ROS_INFO_ONCE("UAVPathFollowerNode got first odometry message.");
    current_pose_.position.x = odo.pose.pose.position.x;
    current_pose_.position.y = odo.pose.pose.position.y;
    current_pose_.position.z = odo.pose.pose.position.z;
    current_pose_.orientation.x = odo.pose.pose.orientation.x;
    current_pose_.orientation.y = odo.pose.pose.orientation.y;
    current_pose_.orientation.z = odo.pose.pose.orientation.z;
    current_pose_.orientation.w = odo.pose.pose.orientation.w;
    // std::cout << "Current Pose: x= " << current_pose_.position.x << " , y= " << current_pose_.position.y << " , z= " << current_pose_.position.z << std::endl;
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "uav_path_follower_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  uav_control::UAVPathFollowerNode uav_path_follower_node(nh, private_nh);

  ros::spin();

  return 0;
}
