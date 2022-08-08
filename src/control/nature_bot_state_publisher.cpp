#include <string>
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::Pose pose;
void OdometryCallback(const nav_msgs::Odometry::ConstPtr& rcv_odom){
    std::cout<<"State publisher recieved odometry "<<std::endl;
	pose = rcv_odom->pose.pose;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "nature_state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber odom_sub = n.subscribe("nature/odometry",100, OdometryCallback);

    tf::TransformBroadcaster broadcaster;
    

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    // set up parent and child frames
    tf::StampedTransform tf_map_to_odom;
    tf_map_to_odom.frame_id_ = std::string("map");
    tf_map_to_odom.child_frame_id_ = std::string("odom");

    ros::Rate loop_rate(100.0);
    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(3);
        joint_state.position.resize(3);
        joint_state.name[0] ="lidar_joint";
        joint_state.position[0] = 0.0;
        
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = pose.position.x;
        odom_trans.transform.translation.y = pose.position.y;
        odom_trans.transform.translation.z = pose.position.z;
        odom_trans.transform.rotation = pose.orientation;

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // map to odom broadcast transform
        tf_map_to_odom.stamp_ = ros::Time::now();
        //tf_map_to_odom.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
        //tf_map_to_odom.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y,
        //                                          pose.orientation.z,pose.orientation.w));
        tf_map_to_odom.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf_map_to_odom.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

        broadcaster.sendTransform(tf_map_to_odom);

        // This will adjust as needed per iteration
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}