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
#include <tf/transform_listener.h>
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
// local includes
#include "avt_341/avt_341_utils.h"
#include "avt_341/planning/global/coord_conversions/coord_conversions.h"

int main(int argc, char **argv){

    

    auto n = avt_341::node::init_node(argc,argv,"path_manager");

    auto path_pub = n->create_publisher<avt_341::msg::Path>("avt_341/enu_waypoints", 10);
    
    std::vector<double> gps_waypoints_lat, gps_waypoints_lon;
    n->get_parameter("/gps_waypoints_lon", gps_waypoints_lon, std::vector<double>(0));
    n->get_parameter("/gps_waypoints_lat", gps_waypoints_lat, std::vector<double>(0));

    if (gps_waypoints_lat.size()!=gps_waypoints_lon.size()){
        std::cerr<<"ERROR! IN THE GPS TO ENU WP FILE, THE NUMBER OF LAT AND LON ENTRIES WAS NOT THE SAME. EXITING."<<std::endl;
        return 1;
    }

    std::vector< std::vector<double> > path;
    double utm_east = 333990.45;
    double utm_north = 3704867.39;
    avt_341::coordinate_system::CoordinateConverter converter;
    std::vector<avt_341::coordinate_system::UTM> utm_waypoints;
    for (int i=0;i<gps_waypoints_lat.size();i++){
        avt_341::coordinate_system::LLA gps_wp;
        gps_wp.latitude = gps_waypoints_lat[i];
        gps_wp.longitude = gps_waypoints_lon[i];
        gps_wp.altitude = 100.0f; // approximate elevation for Starkville, MS
        avt_341::coordinate_system::UTM utm_wp = converter.LLA2UTM(gps_wp);
        std::cout<<"POINT "<<i<<" "<<gps_waypoints_lat[i]<<" "<<gps_waypoints_lon[i]<<" "<<utm_wp.x<<" "<<utm_wp.y<<std::endl;
        utm_waypoints.push_back(utm_wp);
        std::vector<double> point;
        point.push_back(utm_wp.x);
        point.push_back(utm_wp.y);
        path.push_back(point);
    }

    avt_341::msg::Path enu_path;

    tf::TransformListener listener;

    avt_341::node::Rate loop_rate(10);


    int count = 0;
    while(avt_341::node::ok()){

        //std::vector<std::vector<float>> path = {{0, 10}, {10, 20}, {10, 50}};
        tf::StampedTransform transform;
        listener.lookupTransform("/odom", "/utm",  ros::Time(0), transform);
        std::cout<<transform.getOrigin().x() <<" "<<transform.getOrigin().y()<<std::endl;
        avt_341::msg::Path ros_path;
        ros_path.header.frame_id = "odom";
        ros_path.poses.clear();
        for (int32_t i = 0; i < path.size(); i++){
            avt_341::msg::PoseStamped pose;
            //pose.pose.position.x = static_cast<float>(path[i][0]);
            //pose.pose.position.y = static_cast<float>(path[i][1]);
            pose.pose.position.x = path[i][0] - utm_east;
            pose.pose.position.y = path[i][1] - utm_north;
            pose.pose.position.z = 0.0f;
            pose.pose.orientation.w = 1.0f;
            pose.pose.orientation.x = 0.0f;
            pose.pose.orientation.y = 0.0f;
            pose.pose.orientation.z = 0.0f;
            ros_path.poses.push_back(pose);
        } 

        ros_path.header.stamp = n->get_stamp();
        avt_341::node::set_seq(ros_path.header, count);

        for (int i = 0; i < ros_path.poses.size(); i++){
            ros_path.poses[i].header = ros_path.header;
        }

        path_pub->publish(ros_path);
        count++;
        
        n->spin_some();
        loop_rate.sleep();
    }
}
