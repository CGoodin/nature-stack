// c++ includes
//#include <math.h>
//#include <iostream>
//#include <fstream>
//#include <vector>
//#include <algorithm>
// ros includes
#include "nature/node/ros_types.h"
#include "nature/node/node_proxy.h"

std::vector<double> obs_x_list, obs_y_list, obs_r_list;
float grid_res, grid_llx, grid_lly;
float grid_width, grid_height;

nature::msg::OccupancyGrid CreateGrid(bool row_major){
	nature::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = grid_res;
	int nx = (int)(grid_width/grid_res);
	int ny = (int)(grid_height/grid_res);
  grid.info.width = nx;
  grid.info.height = ny;
  grid.info.origin.position.x = grid_llx;
  grid.info.origin.position.y = grid_lly;
  grid.info.origin.orientation.w = 1.0;
  grid.info.origin.orientation.x = 0.0;
  grid.info.origin.orientation.y = 0.0;
  grid.info.origin.orientation.z = 0.0;
  grid.data.resize(nx*ny);
  int c = 0;

  if(row_major){
    for (int j=0;j<ny;j++){
			double y = grid_lly + (j+0.5)*grid_res;
      for (int i=0;i<nx;i++){
				double x = grid_llx + (i+0.5)*grid_res;
				int cost = 0;
				for (int k=0;k<obs_x_list.size();k++){
					double dx = x - obs_x_list[k];
					double dy = y - obs_y_list[k];
					double d = sqrt(dx*dx + dy*dy);
					if (d<obs_r_list[k]+grid_res) cost = 100;
				}
        grid.data[c++] = cost;
      }
    }
  }
	else {
    for (int i=0;i<nx;i++){
			double x = grid_llx + (i+0.5)*grid_res;
      for (int j=0;j<ny;j++){
				double y = grid_lly + (j+0.5)*grid_res;
        int cost = 0;
				for (int k=0;k<obs_x_list.size();k++){
					double dx = x - obs_x_list[k];
					double dy = y - obs_y_list[k];
					double d = sqrt(dx*dx + dy*dy);
					if (d<obs_r_list[k]+grid_res) cost = 100;
				}
        grid.data[c++] = cost;
      }
    }
  }
  return grid;
}

int main(int argc, char *argv[]) {

	auto n = nature::node::init_node(argc, argv, "nature_map_publisher_node");
  auto grid_pub = n->create_publisher<nature::msg::OccupancyGrid>("nature/occupancy_grid", 1);
  
  n->get_parameter("~grid_width", grid_width, 200.0f);
  n->get_parameter("~grid_height", grid_height, 200.0f);
  n->get_parameter("~grid_res", grid_res, 1.0f);
  n->get_parameter("~grid_llx", grid_llx, -100.0f);
  n->get_parameter("~grid_lly", grid_lly, -100.0f);

	std::string display;
  n->get_parameter("~display", display, std::string("image"));

  n->get_parameter("/obstacles_x", obs_x_list, std::vector<double>(0));
  n->get_parameter("/obstacles_y", obs_y_list, std::vector<double>(0));
	n->get_parameter("/obstacles_r", obs_r_list, std::vector<double>(0));
	
	if ( ! (obs_x_list.size()==obs_y_list.size() && obs_x_list.size()==obs_r_list.size())){
		std::cerr<<"ERROR: The list of obstacle position and radius did not match in the input file. "<<std::endl;
		exit(29);
	}

  bool use_rviz = display == "rviz";
  std::shared_ptr<nature::node::Publisher<nature::msg::OccupancyGrid>> grid_pub_vis;
  if(use_rviz){
    grid_pub_vis = n->create_publisher<nature::msg::OccupancyGrid>("nature/occupancy_grid_vis", 1);
  }

	nature::msg::OccupancyGrid grd = CreateGrid(false);
	nature::msg::OccupancyGrid grd_vis = CreateGrid( true);

	int nloops = 0;
	nature::node::Rate rate(100.0);
	while (nature::node::ok()){

    grd.header.stamp = n->get_stamp();
		grid_pub->publish(grd);

		if(use_rviz && nloops % 10 == 0){
			grd_vis.header.stamp = n->get_stamp();
			grid_pub_vis->publish(grd_vis);
		}
		nloops++;
		n->spin_some();
		rate.sleep();
	}

	return 0;
}