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

tf2::Quaternion yaw_positive_half_pi_rotation;

void replay_odom(geometry_msgs::TransformStamped transform_msg) {
    // ROS_INFO("Sending pose...");
    
    // have to do the transformation
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = transform_msg.header;
    pose_stamped.header.frame_id = "base_link";
    // because the onboard computer is not coneected with internet, 
    // the time in it would be inconsistent with vicon system
    pose_stamped.header.stamp = ros::Time::now();

    pose_stamped.pose.position.x = -transform_msg.transform.translation.y;
    pose_stamped.pose.position.y = transform_msg.transform.translation.x;
    pose_stamped.pose.position.z = transform_msg.transform.translation.z;

    // this rotation given by vicon is in NED frame, to represent the rotation in the ros ENU
    // we have to rotate the pose in 90 degree of z axis(up axis).
    tf2::Quaternion desired_orientation, new_orientation_in_neu_ros_frame;
    tf2::convert(transform_msg.transform.rotation, desired_orientation);   
    new_orientation_in_neu_ros_frame = yaw_positive_half_pi_rotation * desired_orientation;
    new_orientation_in_neu_ros_frame.normalize();
    tf2::convert(new_orientation_in_neu_ros_frame, pose_stamped.pose.orientation);
    
    odom_replay.publish(pose_stamped);
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vicon_replay");

    ros::NodeHandle n;

    std::string topic_name = argv[1];

    mocap_sub = n.subscribe(topic_name, 1, replay_odom);
    odom_replay = n.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);

    yaw_positive_half_pi_rotation.setRPY(0, 0, 3.14159 / 2);

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

}