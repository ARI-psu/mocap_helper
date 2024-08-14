#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf/transform_datatypes.h"

ros::Publisher odom_replay, vicon_studio_odometry_replay;
ros::Subscriber mocap_sub;

void replay_odom(nav_msgs::Odometry odom_msg) {
    // ROS_INFO("Sending pose...");
    
    // have to do the transformation
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg.header;
    pose_stamped.header.frame_id = "base_link";
    // because the onboard computer is not coneected with internet, 
    // the time in it would be inconsistent with vicon system
    pose_stamped.header.stamp = ros::Time::now();

    pose_stamped.pose = odom_msg.pose.pose;

    odom_replay.publish(pose_stamped);
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sim_odom_replay");

    ros::NodeHandle n;

    std::string topic_name = argv[1];

    mocap_sub = n.subscribe(topic_name, 1, replay_odom);
    odom_replay = n.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

}