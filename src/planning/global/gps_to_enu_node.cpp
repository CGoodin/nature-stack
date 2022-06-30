/**
 * \file gps_to_enu_node.cpp
 * 
 * ROS node for reading in a list of waypoints, converting to UTM, subscribing to a UTM transform, and republishing in ENU
 * 
 * \author Chris Goodin
 *
 * \contact dwc2@cavs.msstate.edu
 * 
 * \date 1/21/2022
 */

// ros includes
//#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#include "avt_341/node/ros_types.h"
//#include "avt_341/node/node_proxy.h"
// local includes
#include "avt_341/avt_341_utils.h"
#include "avt_341/planning/global/coord_conversions/coord_conversions.h"
// c++ includes
#include <fstream>

bool fix_rcvd = false;
float lat_rcvd = 0.0f;
float lon_rcvd = 0.0f;
float alt_rcvd = 0.0f;
nav_msgs::Odometry current_odom;
bool odom_rcvd = false;

void NavSatCallback(const sensor_msgs::NavSatFix::ConstPtr& rcv_fix){
    if (!fix_rcvd){
        lat_rcvd = rcv_fix->latitude;
        lon_rcvd = rcv_fix->longitude;
        alt_rcvd = rcv_fix->altitude;
    }
  fix_rcvd = true;
}

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& rcv_odom){
    current_odom = *rcv_odom;
    odom_rcvd = true;
}

int main(int argc, char **argv){

    //auto n = avt_341::node::init_node(argc,argv,"gps_to_enu_node");
    ros::init(argc, argv, "gps_to_enu_node");
	ros::NodeHandle n;

    //auto path_pub = n->create_publisher<avt_341::msg::Path>("avt_341/enu_waypoints", 10);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/avt_341/enu_waypoints",10);

    //ros::Subscriber navsat_sub = n.subscribe("/piksi_imu/navsatfix_best_fix", 10, NavSatCallback);

    ros::Subscriber odometry_sub = n.subscribe("/avt_341/odometry", 10, OdometryCallback);

    // tf2 transform utm->odom needed
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    std::vector<double> gps_waypoints_lat, gps_waypoints_lon;

    if (n.hasParam("gps_waypoints_lon")){
		n.getParam("gps_waypoints_lon", gps_waypoints_lon);
	}
    if (n.hasParam("gps_waypoints_lat")){
		n.getParam("gps_waypoints_lat", gps_waypoints_lat);
	}

    float waypoint_spacing = 10.0f; // meters
    if (ros::param::has("~waypoint_spacing")){
		ros::param::get("~waypoint_spacing", waypoint_spacing);
	}

    if (gps_waypoints_lat.size()!=gps_waypoints_lon.size()){
        std::cerr<<"ERROR! IN THE GPS TO ENU WP FILE, THE NUMBER OF LAT AND LON ENTRIES WAS NOT THE SAME. EXITING."<<std::endl;
        return 1;
    }

    std::vector< std::vector<double> > path;
    avt_341::coordinate_system::CoordinateConverter converter;
    std::vector<avt_341::coordinate_system::UTM> utm_waypoints;
    for (int i=0;i<gps_waypoints_lat.size();i++){
        avt_341::coordinate_system::LLA gps_wp;
        gps_wp.latitude = gps_waypoints_lat[i];
        gps_wp.longitude = gps_waypoints_lon[i];
        gps_wp.altitude = 80.0f; // approximate elevation for Starkville, MS
        avt_341::coordinate_system::UTM utm_wp = converter.LLA2UTM(gps_wp);
        utm_waypoints.push_back(utm_wp);
        std::vector<double> point;
        point.push_back(utm_wp.x);
        point.push_back(utm_wp.y);
        path.push_back(point);
    }
    if (path.size()<1){
        std::cerr<<"ERROR: NO WAYPOINTS WERE GIVEN TO THE GPS TO ENU NODE. EXITING."<<std::endl;
        exit(1);
    }
    ros::Rate rate(10.0);


    int count = 0;
    float utm_north = 0.0f;
    float utm_east = 0.0f;
    //avt_341::msg::PoseStamped first_pose, second_pose;
    nav_msgs::Path ros_path;
    ros_path.header.frame_id = "odom";
    ros_path.poses.clear();
    while (ros::ok()){

        // skip until transform available

     	if (!tfBuffer.canTransform("odom","utm",ros::Time::now(),ros::Duration(30))){
     		ROS_WARN("No utm->odom transform!");
     		continue;
     	}

        if (odom_rcvd){
            if (count==0){

                // log waypoints to file
                std::ofstream fout;
                fout.open("gps_convert_log.txt");
                geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform("odom","utm",ros::Time(0));
                fout<<"UTM Origin: (" << transform.transform.translation.x <<", "<< transform.transform.translation.y <<")"<<std::endl;
                fout<<"Waypoints: "<<std::endl;
                // waypoints are in UTM currently, need to be in odom for next step
                for (auto& utm_wp : path ){
                    // pack into ROS Pose msg
                    geometry_msgs::PoseStamped utm_pose, odom_pose;
                    // metadata so TF can transform correctly
                    utm_pose.header.frame_id = "utm";
                    utm_pose.header.stamp = ros::Time::now();
                    utm_pose.pose.position.x = utm_wp[0]; // UTM
                    utm_pose.pose.position.y = utm_wp[1]; // UTM
                    utm_pose.pose.position.z = 0; // assume 2D waypoints for now
                    // apply transform
                    tfBuffer.transform(utm_pose, odom_pose, "odom", ros::Duration(60)); // 1 minute timeout to apply the transform
                    // this is a repulsive hack, but it minimizes code changes for now
                    utm_wp[0] = odom_pose.pose.position.x;
                    utm_wp[1] = odom_pose.pose.position.y;
                    // logging
                    fout<<"(" << utm_wp[0] <<", "<< utm_wp[1] << ")" << std::endl;
                } // end for

                // translations of transform
                float to_veh_x = current_odom.pose.pose.position.x - path[0][0];
                float to_veh_y = current_odom.pose.pose.position.y - path[0][1];
                // distance to first waypoint
                float tvm = sqrtf(to_veh_x*to_veh_x  + to_veh_y*to_veh_y);
                // normalized components
                float tvx = to_veh_x/tvm;
                float tvy = to_veh_y/tvm;
                // initialize x,y to 3 meters away along path to first waypoint   
                float x = current_odom.pose.pose.position.x + tvx*3.0f;
                float y = current_odom.pose.pose.position.y + tvy*3.0f;

                // ensure waypoints are close enough together
                int current_waypoint = 0;
                bool finished = false;
                int num_loops = 0;
                while (!finished){
                    if (current_waypoint>=path.size()) break; // end condition
                    
                    // distance to next waypoint from current limit
                    float tx = path[current_waypoint][0] - x;
                    float ty = path[current_waypoint][1] - y;
                    float normt = sqrtf(tx*tx + ty*ty);

                    // add waypoint                    
                    avt_341::msg::PoseStamped pose;
                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = 0.0f;
                    // orientation doesn't matter for waypoints, so leave default orientation
                    pose.pose.orientation.w = 1.0f;
                    pose.pose.orientation.x = 0.0f;
                    pose.pose.orientation.y = 0.0f;
                    pose.pose.orientation.z = 0.0f;
                    ros_path.poses.push_back(pose);
                    // if next waypoint is close enough, skip it
                    if (normt<1.25f*waypoint_spacing){
                        current_waypoint += 1;
                        if (current_waypoint>=path.size())finished = true;
                    }
                    // walk towards next waypoint
                    x += waypoint_spacing*tx/normt;
                    y += waypoint_spacing*ty/normt;
                    // cap waypoints to 1000 tops
                    if (num_loops>1000){
                        finished = true;
                    }
                    num_loops++;
                }
                avt_341::msg::PoseStamped pose;
                pose.pose.position.x = path.back()[0];
                pose.pose.position.y = path.back()[1];
                pose.pose.position.z = 0.0f;
                pose.pose.orientation.w = 1.0f;
                pose.pose.orientation.x = 0.0f;
                pose.pose.orientation.y = 0.0f;
                pose.pose.orientation.z = 0.0f;
                ros_path.poses.push_back(pose);
            }
/*
            if (count==0){
                // first time only 
                avt_341::coordinate_system::LLA gps_origin;
                gps_origin.latitude = lat_rcvd;
                gps_origin.longitude = lon_rcvd;
                gps_origin.altitude = alt_rcvd; // approximate elevation for Starkville, MS
                avt_341::coordinate_system::UTM utm_origin = converter.LLA2UTM(gps_origin);
                utm_east = utm_origin.x;
                utm_north = utm_origin.y;
                std::ofstream fout;
                fout.open("gps_convert_log.txt");
                fout<<"UTM Origin: ("<<utm_east<<", "<<utm_north<<")"<<std::endl;
                fout<<"Waypoints: "<<std::endl;
                for (int32_t i = 0; i < path.size(); i++){
                    fout<<"("<<path[i][0] - utm_east<<", "<< path[i][1] - utm_north<<")"<<std::endl;
                }
                fout.close();

                // make the first waypoint be the initial pose
                float to_veh_x = current_odom.pose.pose.position.x - (path[0][0] - utm_east);
                float to_veh_y = current_odom.pose.pose.position.y - (path[0][1] - utm_north); 
                float tvm = sqrtf(to_veh_x*to_veh_x  + to_veh_y*to_veh_y);
                float tvx = to_veh_x/tvm;
                float tvy = to_veh_y/tvm;
                first_pose.pose.position.x = current_odom.pose.pose.position.x + tvx*3.0f;
                first_pose.pose.position.y = current_odom.pose.pose.position.y + tvy*3.0f;
                first_pose.pose.position.z = 0.0f;
                first_pose.pose.orientation.w = 1.0f;
                first_pose.pose.orientation.x = 0.0f;
                first_pose.pose.orientation.y = 0.0f;
                first_pose.pose.orientation.z = 0.0f;
                second_pose = first_pose;
                second_pose.pose.position.x = current_odom.pose.pose.position.x - tvx*5.0f;
                second_pose.pose.position.y = current_odom.pose.pose.position.y - tvy*5.0f;
            }

            nav_msgs::Path ros_path;
            ros_path.header.frame_id = "odom";
            ros_path.poses.clear();
            
            if (path.size()==1){
                ros_path.poses.push_back(first_pose);
                ros_path.poses.push_back(second_pose);
            }

            for (int32_t i = 0; i < path.size(); i++){
                avt_341::msg::PoseStamped pose;
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = path[i][0] - utm_east;
                pose.pose.position.y = path[i][1] - utm_north;
                pose.pose.position.z = 0.0f;
                pose.pose.orientation.w = 1.0f;
                pose.pose.orientation.x = 0.0f;
                pose.pose.orientation.y = 0.0f;
                pose.pose.orientation.z = 0.0f;
                ros_path.poses.push_back(pose);
            } 

            */

            ros_path.header.stamp =  ros::Time::now();
            ros_path.header.seq = count;

            for (int i = 0; i < ros_path.poses.size(); i++){
                ros_path.poses[i].header = ros_path.header;
            }
            path_pub.publish(ros_path);
            count++;
        }

        rate.sleep();
		ros::spinOnce();
    }
}
