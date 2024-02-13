/*
Non-Commercial License - Mississippi State University Off-Road Traversability Algorithm

REPO: https://gitlab.com/cgoodin/off_road_traversability

CONTACT: cgoodin@cavs.msstate.edu

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

CITATION:
Goodin, C., Dabbiru, L., Hudson, C., Mason, G., Carruth, D., & Doude, M. (2021, April).
Fast terrain traversability estimation with terrestrial lidar in off-road autonomous navigation.
In Unmanned Systems Technology XXIII (Vol. 11758, p. 117580O). International Society for Optics and Photonics.

NOTICE:
Do not share or distribute. Software is authorized for use only by the approved recepient.

Copyright 2022 (C) Mississippi State University
*/
// class header
#include "nature/perception/ftte/voxel_grid.h"
// C++ includes
#include <algorithm>
#include <omp.h>
#include <fstream>
// local includes
#include "nature/perception/ftte/moreland.h"

namespace traverselib {

/// Allocate a 2D vector
template <class T>
inline std::vector< std::vector<T> > Allocate2DVector(int nx, int ny, T initval) {
	std::vector< std::vector<T> > A;
	for (int i = 0; i<nx; i++) {
		std::vector<T> row;
		for (int j = 0; j<ny; j++) {
			row.push_back(initval);
		}
		A.push_back(row);
	}
	return A;
};

/// Allocate a 3D vector
template <class T>
inline std::vector< std::vector< std::vector<T> > > Allocate3DVector(int nx, int ny, int nz, T initval) {
	std::vector< std::vector< std::vector<T> > > A;
	for (int i = 0; i<nx; i++) {
		std::vector <std::vector<T> > B;
		for (int j = 0; j<ny; j++) {
			std::vector<T> row;
			for (int k = 0; k < nz; k++) {
				row.push_back(initval);
			}
			B.push_back(row);
		}
		A.push_back(B);
	}
	return A;
};


VoxelGrid::VoxelGrid(glm::vec3 llc, glm::vec3 urc, float res) {
	Initialize(llc,urc,res);
}

void VoxelGrid::Initialize(glm::vec3 llc, glm::vec3 urc, float res){
	initialized_ = true;
	res_ = res;
	res_squared_ = res_ * res_;
	llc_ = llc;
	urc_ = urc;
	averaging_radius_ = 3.0; //meters ~ 10ft

	rci_max_ = 100.0f;
	max_slope_ = 0.0f;
	max_impermeability_ = 0.0f;
	max_roughness_ = 0.0f;
	min_ground_ = std::numeric_limits<float>::max();
	max_ground_ = std::numeric_limits<float>::lowest();

	veg_hi_lo_cutoff_ = glm::vec2(0.3f, 2.5f);

	use_plane_fitting_ = false;
	print_timing_info_ = false;

	for (int i = 0; i < 3; i++) {
		dim_[i] = (int)(ceil((urc_[i] - llc_[i])) / res_);
	}

	TraverseCell cell;
	cells_ = Allocate2DVector(dim_[0], dim_[1], cell);
	ground_ = Allocate2DVector(dim_.x, dim_.y, urc_.z);

	Plane plane;
	fit_planes_ = Allocate2DVector(dim_.x, dim_.y, plane);
	slope_ = Allocate2DVector(dim_.x, dim_.y, 0.0f);
	local_avg_ = Allocate2DVector(dim_.x, dim_.y, 0.0f);
	roughness_ = Allocate2DVector(dim_.x, dim_.y, 0.0f);
	impermeability_ = Allocate2DVector(dim_.x, dim_.y, 0.0f);
	rci_ = Allocate2DVector(dim_.x, dim_.y, rci_max_);

	max_num_points_ = 0;
	elev_avg_ = 0.0f;
	local_avg_complete_ = false;

	default_traversability_ = 0.5f;
}

void VoxelGrid::Move(glm::vec3 new_llc, glm::vec3 new_urc){
	
	float dx = new_llc.x - llc_.x;
	float dy = new_llc.y - llc_.y;
	int dnx = (int)floor(std::fabs(dx/res_));
	int dny = (int)floor(std::fabs(dy/res_));
	

	if (dnx>0 || dny>0){
		dnx = dnx*(int)(dx/std::fabs(dx));
		dny = dny*(int)(dy/std::fabs(dy));
		//std::cout<<" Moving voxel grid by "<<dnx<<" X "<<dny<<" cells. "<<std::endl;
		std::vector<std::vector<TraverseCell> > new_cells;
		std::vector<std::vector<float> > new_ground;
		std::vector<std::vector<float> > new_local_avg;
		std::vector<std::vector<Plane> > new_fit_planes;
		std::vector<std::vector<float> > new_slope;
		std::vector<std::vector<float> > new_roughness;
		std::vector<std::vector<float> > new_impermeability; 
		std::vector<std::vector<float> > new_rci;
		TraverseCell cell;
		new_cells = Allocate2DVector(dim_[0], dim_[1], cell);
		new_ground = Allocate2DVector(dim_.x, dim_.y, elev_avg_);

		Plane plane;
		new_fit_planes = Allocate2DVector(dim_.x, dim_.y, plane);
		new_slope = Allocate2DVector(dim_.x, dim_.y, 0.0f);
		new_local_avg = Allocate2DVector(dim_.x, dim_.y, 0.0f);
		new_roughness = Allocate2DVector(dim_.x, dim_.y, 0.0f);
		new_impermeability = Allocate2DVector(dim_.x, dim_.y, 0.0f);
		new_rci = Allocate2DVector(dim_.x, dim_.y, rci_max_);
		for (int i=0;i<dim_.x; i++){
			int ii = i + dnx;
			for (int j=0;j<dim_.y;j++){
				int jj = j + dny;
				if (ii>=0 && ii<dim_.x && jj>=0 && jj<dim_.y){
					new_cells[i][j] = cells_[ii][jj];
					new_ground[i][j] = ground_[ii][jj];
					new_fit_planes[i][j] = fit_planes_[ii][jj];
					new_slope[i][j] = slope_[ii][jj];
					new_local_avg[i][j] = local_avg_[ii][jj];
					new_roughness[i][j] = roughness_[ii][jj];
					new_impermeability[i][j] = impermeability_[ii][jj];
					new_rci[i][j] = rci_[ii][jj];
				}
			}
		}
		llc_ = new_llc;
		urc_ = new_urc;
		cells_ = new_cells;
		ground_ = new_ground;
		fit_planes_ = new_fit_planes;
		slope_ = new_slope;
		local_avg_ = new_local_avg;
		roughness_ = new_roughness;
		impermeability_ = new_impermeability;
		rci_ = new_rci;
	}
}


bool VoxelGrid::InGrid(glm::ivec3 idx) {
	bool in_grid = false;
	if (idx.x >= 0 && idx.x < dim_.x && idx.y >= 0 && idx.y < dim_.y && idx.z >= 0 && idx.z < dim_.z) in_grid = true;
	return in_grid;
}

bool VoxelGrid::InGrid(glm::ivec2 idx) {
	bool in_grid = false;
	if (idx.x >= 0 && idx.x < dim_.x && idx.y >= 0 && idx.y < dim_.y) in_grid = true;
	return in_grid;
}

glm::ivec3 VoxelGrid::PointToIndex(glm::vec3 p) {
	glm::ivec3 idx;
	for (int i = 0; i < 3; i++) idx[i] = (int)floor((p[i] - llc_[i]) / res_);
	return idx;
}

glm::ivec2 VoxelGrid::PointToIndex(glm::vec2 p) {
	glm::ivec2 idx;
	for (int i = 0; i < 2; i++) idx[i] = (int)floor((p[i] - llc_[i]) / res_);
	return idx;
}

glm::vec3 VoxelGrid::IndexToPoint(glm::ivec3 idx) {
	glm::vec3 p;
	for (int i = 0; i < 3; i++)p[i] = (idx[i] + 0.5f)*res_ + llc_[i];
	return p;
}

void VoxelGrid::RayMarching(glm::vec3 start_point, glm::vec3 end_point) {
	// search through the cells that it didn't hit and add in the "misses"
	// use bresenhams algorithm
	glm::ivec2 idx0 = PointToIndex(glm::vec2(start_point.x, start_point.y));
	glm::ivec2 idx1 = PointToIndex(glm::vec2(end_point.x, end_point.y));

	int x0 = idx0.x;
	int y0 = idx0.y;
	int x1 = idx1.x;
	int y1 = idx1.y;
    int dx = x1 - x0;
    int dy = y1 - y0;

	int xsign,ysign;
	if (dx>0){
		xsign = 1;
	}
	else {
		xsign = -1;
	}
	if (dy>0){
		ysign = 1;
	}
	else{
		ysign = -1;
	}

    dx = abs(dx);
    dy = abs(dy);

	int xx,xy,yx,yy;
    if (dx > dy){
		xx = xsign;
		xy = 0;
		yx = 0;
		yy = ysign;
	}
    else{
		std::swap(dx,dy);
		xx = 0;
		xy = ysign;
		yx = xsign;
		yy = 0;
	}

    int D = 2*dy - dx;
    int y = 0;
	for (int x =0;x<=dx;x++){
        int px =  x0 + x*xx + y*yx;
		int py =  y0 + x*xy + y*yy;
		if (px<0 || px>=dim_.x || py<0 || py>=dim_.y)break;
		cells_[px][py].num_points_total += 1.0f;
        if (D >= 0){
            y += 1;
            D -= 2*dx;
		}
        D += 2*dy;
	}
}


void VoxelGrid::AddRegisteredPoints(std::vector<glm::vec3> &points_in, glm::vec3 position) {
	// add position to the path taken
	path_taken_.push_back(PointToIndex(position));

	vehicle_path_.push_back(glm::vec2(position.x,position.y));

	double t0 = omp_get_wtime();
	//remove points are too close
	std::vector<glm::vec3> points;
	std::vector<float> ranges;
	for (int i = 0; i < points_in.size(); i++) {
		float r = glm::length(points_in[i] - position);
		if (r > 1.0f) {
			points.push_back(points_in[i]);
			ranges.push_back(r);
		}
	}
	double t1 = omp_get_wtime();
	// this section is one of the slowest
	glm::ivec2 pidx = PointToIndex(position);
	// add points to the grid, keeping the highest and lowest
	int np = (int)points.size();
	for (int p = 0; p < np; p++) {
		glm::ivec2 idx = PointToIndex(points[p]);
		if (InGrid(idx)) {
			cells_[idx.x][idx.y].num_points_hit++;
			float dz = points[p].z - cells_[idx.x][idx.y].lowest;
			if (dz > veg_hi_lo_cutoff_.x && dz < veg_hi_lo_cutoff_.y) cells_[idx.x][idx.y].num_veg_hits += 1.0f;
			if (points[p].z < cells_[idx.x][idx.y].lowest) cells_[idx.x][idx.y].lowest = points[p].z;
			if (points[p].z > cells_[idx.x][idx.y].highest) cells_[idx.x][idx.y].highest = points[p].z;
			float zdir = points[p].z - position.z;
			if (zdir < 0.0f)cells_[idx.x][idx.y].ground_candidate = true;
			// Find voxels the rays have "passed through"
			RayMarching(position, points[p]);
			// update the confidence in each cell
			cells_[idx.x][idx.y].conf_accum += 1.0f / (float)sqrt(ranges[p]);
		}
	}
	double t2 = omp_get_wtime();
	// calculate the average measured surface elevation on the whole grid
	float avg_num = 0.0f;
	float avg_den = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			//if (cells_[i][j].num_points_hit > 0) {
			if (cells_[i][j].num_points_hit > 0 && cells_[i][j].ground_candidate) {
				avg_num += cells_[i][j].lowest;
				avg_den += 1.0f;
				if (cells_[i][j].num_points_hit > max_num_points_)max_num_points_ = cells_[i][j].num_points_hit;
			}
		}
	}
	if (avg_den > 0.0f)elev_avg_ = avg_num / avg_den;

	double t3 = omp_get_wtime();
	// calculate the veg density and confidence
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			if (cells_[i][j].conf_accum >= 1.0) {
				cells_[i][j].confidence = 1.0f - (float)sqrt(cells_[i][j].conf_accum) / cells_[i][j].conf_accum;
			}
			else {
				cells_[i][j].confidence = 0.0f;
			}
			if (cells_[i][j].num_points_total == 0.0f) {
				impermeability_[i][j] = 0.0f;
			}
			else {
				impermeability_[i][j] = std::min(1.0f,cells_[i][j].num_veg_hits / cells_[i][j].num_points_total);
			}
		}
	}

	// Calculate other cell variables for use in mobility calculation
	double t4 = omp_get_wtime();
	// this is one of the slowest if using the plane fitting
	CalculateLocalAvg();
	double t5 = omp_get_wtime();
	CalculateGround();
	double t6 = omp_get_wtime();
	CalculateSlope();
	double t7 = omp_get_wtime();
	CalculateRoughness();
	double t8 = omp_get_wtime();
	if (print_timing_info_){
		std::cout<<"Total traversability algorithm time: "<<t8-t0<<std::endl;
		std::cout<<"\t Point Indexing: "<<t1-t0<<std::endl;
		std::cout<<"\t Veg Density: "<<t2-t1<<std::endl;
		std::cout<<"\t Average Elev: "<<t3-t2<<std::endl;
		std::cout<<"\t Confidence: "<<t4-t3<<std::endl;
		std::cout<<"\t Local avg elev: "<<t5-t4<<std::endl;
		std::cout<<"\t Ground calculation: "<<t6-t5<<std::endl;
		std::cout<<"\t Slope calculation: "<<t7-t6<<std::endl;
		std::cout<<"\t Roughness calculation: "<<t8-t7<<std::endl;
	}
}

void VoxelGrid::CalculateGround() {
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			if (cells_[i][j].num_points_hit > 0 && cells_[i][j].ground_candidate) {
				ground_[i][j] = cells_[i][j].lowest;
				if (ground_[i][j] > max_ground_)max_ground_ = ground_[i][j];
				if (ground_[i][j] < min_ground_)min_ground_ = ground_[i][j];
			}
			else {
				if (use_plane_fitting_){
					glm::vec3 p = IndexToPoint(glm::ivec3(i, j, 0));
					float local_z = fit_planes_[i][j].GetHeightAt(p.x, p.y);
					ground_[i][j] = local_z; 
				}
				else{
					ground_[i][j] = local_avg_[i][j]; //elev_avg_;
				}
			}
		}
	}
}

void VoxelGrid::WriteStats(std::string fname) {
	std::ofstream fout(fname.c_str());
	fout << "i,j,ground,slope,roughess,impermeability,rci,confidence" << std::endl;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			fout << i << "," << j << "," << ground_[i][j] << "," << slope_[i][j] << "," << roughness_[i][j] << "," << impermeability_[i][j] << "," << rci_[i][j] << "," << cells_[i][j].confidence << std::endl;
		}
	}

	fout.close();
}

cimg_library::CImg<float> VoxelGrid::DrawConfidence() {
	cimg_library::CImg<float> img;
	img.assign(dim_.x, dim_.y, 1, 3, 0.0f);
	img = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			glm::vec3 color = MorelandGreen(cells_[i][j].confidence, 1.0f, 0.0f);
			img.draw_point(i, j, (float *)&color);
		}
	}
	AddVehiclePathToImage(img);
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);
	return img;	
	//if (conf_disp_.width() == 0 || conf_disp_.height() == 0) {
	//	conf_disp_.assign(img, "Confidence");
	//}
	//else {
	//	conf_disp_ = img;
	//}
}

void VoxelGrid::PlotConfidence() {
	cimg_library::CImg<float> img = DrawConfidence();
	/*img.assign(dim_.x, dim_.y, 1, 3, 0.0f);
	img = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			glm::vec3 color = MorelandGreen(cells_[i][j].confidence, 1.0f, 0.0f);
			img.draw_point(i, j, (float *)&color);
		}
	}
	AddVehiclePathToImage(img);
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);*/
	if (conf_disp_.width() == 0 || conf_disp_.height() == 0) {
		conf_disp_.assign(img, "Confidence");
	}
	else {
		conf_disp_ = img;
	}
}

void VoxelGrid::SaveConfidencePlot(std::string fname) {
	cimg_library::CImg<float> img = DrawConfidence(); //(conf_disp_);
	img.mirror('y');
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);
	img.save(fname.c_str());
}

void VoxelGrid::AddVehiclePathToImage(cimg_library::CImg<float> &img){
	glm::vec3 blue(0.0f, 0.0f, 255.0f);
	for (int i=0;i<(int)vehicle_path_.size()-1;i++){
		glm::ivec2 p0 = PointToIndex(vehicle_path_[i]);
		glm::ivec2 p1 = PointToIndex(vehicle_path_[i+1]);
		img.draw_line(p0.x,p0.y,p1.x,p1.y,(float *)&blue);
	}
}

cimg_library::CImg<float> VoxelGrid::DrawGround() {
	cimg_library::CImg<float> img;
	img.assign(dim_.x, dim_.y, 1, 3, 0.0f);
	img = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			glm::vec3 color = MorelandGreen(ground_[i][j], min_ground_, max_ground_);
			img.draw_point(i, j, (float *)&color);
		}
	}
	AddVehiclePathToImage(img);
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);
	return img;	
//if (ground_disp_.width() == 0 || ground_disp_.height() == 0) {
	//	ground_disp_.assign(img, "Ground");
	//}
	//else {
	//	ground_disp_ = img;
	//}
}

void VoxelGrid::PlotGround() {
	cimg_library::CImg<float> img = DrawGround();
	/*img.assign(dim_.x, dim_.y, 1, 3, 0.0f);
	img = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			glm::vec3 color = MorelandGreen(ground_[i][j], min_ground_, max_ground_);
			img.draw_point(i, j, (float *)&color);
		}
	}
	AddVehiclePathToImage(img);
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);*/
	if (ground_disp_.width() == 0 || ground_disp_.height() == 0) {
		ground_disp_.assign(img, "Ground");
	}
	else {
		ground_disp_ = img;
	}
}

void VoxelGrid::SaveGroundPlot(std::string fname) {
	cimg_library::CImg<float> img = DrawGround(); //(ground_disp_);
	img.mirror('y');
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);
	img.save(fname.c_str());
}

void VoxelGrid::SaveSlicePlot(std::string fname) {
	cimg_library::CImg<float> img(slice_disp_);
	img.mirror('y');
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);
	img.save(fname.c_str());
}

void VoxelGrid::CalculateSlope() {
	if (!local_avg_complete_)CalculateLocalAvg();
	max_slope_ = 0.0f;
	for (int i = 1; i < dim_.x - 1; i++) {
		for (int j = 1; j < dim_.y - 1; j++) {
			if (use_plane_fitting_){
				slope_[i][j] = fit_planes_[i][j].GetSlope();
			}
			else{
				if (cells_[i+1][j].num_points_hit > 0 && 
				    cells_[i-1][j].num_points_hit > 0 &&
					cells_[i][j+1].num_points_hit > 0 &&
					cells_[i][j-1].num_points_hit > 0 ) {
					float dx = (ground_[i+1][j]-ground_[i-1][j])/(2.0f*res_);
					float dy = (ground_[i][j+1]-ground_[i][j-1])/(2.0f*res_);
					slope_[i][j] = (float)sqrt(dx*dx + dy*dy);
				}
				else{
					slope_[i][j] = 0.0f;
				}
			}
			if (slope_[i][j] > max_slope_)max_slope_ = slope_[i][j];
		}
	}
}

bool VoxelGrid::ArePointsColinear(std::vector<glm::vec3> points) {
	// this is not really a test of colinearity
	// the plane-fitting algorithm just doesn't like points in a row
	bool x_equal = true;
	bool y_equal = true;
	for (int i = 0; i < (int)points.size()-1; i++) {
		if (points[i].x != points[i + 1].x)x_equal = false;
		if (points[i].y != points[i + 1].y)y_equal = false;
	}
	return (x_equal || y_equal);
}

void VoxelGrid::CalculateLocalAvg() {
	int pixrad = (int)ceil(0.5f*averaging_radius_ / res_);
#pragma omp parallel for
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			std::vector<glm::vec3> points_in_plane;
			float local_avg = 0.0f;
			float num_avg = 0.0f;
			for (int ii = i - pixrad; ii <= i + pixrad; ii++) {
				for (int jj = j - pixrad; jj <= j + pixrad; jj++) {
					if (InGrid(glm::ivec3(ii, jj, 0))) {
						if (cells_[ii][jj].num_points_hit > 0) {
							if (use_plane_fitting_){
								glm::vec3 p = IndexToPoint(glm::ivec3(ii, jj, 0));
								p.z = ground_[ii][jj];
								points_in_plane.push_back(p);
							}
		
							num_avg += 1.0f;
							local_avg += ground_[ii][jj];
							
						}
					}
				}
			}
			if (use_plane_fitting_){			
				bool colinear = false;
				float this_avg = elev_avg_;
				if (num_avg>0.0f){
					this_avg=local_avg/num_avg;
				}
				if (points_in_plane.size() <= 2 * pixrad+1) {
					colinear = ArePointsColinear(points_in_plane);
				}
				if (points_in_plane.size()>3 && !colinear){
					// this is really slow
					fit_planes_[i][j].FitToPoints(points_in_plane);
					glm::vec3 coeffs = fit_planes_[i][j].GetCoeffs();
					if (std::isnan(coeffs.x)){
						fit_planes_[i][j].SetCoeffs(0.0f, 0.0f, this_avg);
					}
				}
				else {
					fit_planes_[i][j].SetCoeffs(0.0f, 0.0f, this_avg);
				}
			}
			else{
				if (num_avg>0.0f){
					local_avg_[i][j]=local_avg/num_avg;
				}
				else{
					local_avg_[i][j] = elev_avg_;
				}
			}
		}
	}
	local_avg_complete_ = true;
}

void VoxelGrid::CalculateRoughness() {
	max_roughness_ = 0.0f;
	if (!local_avg_complete_)CalculateLocalAvg();
	int pixrad = (int)ceil(0.5f*averaging_radius_ / res_);
#pragma omp parallel for schedule(dynamic) 
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			float roughsum = 0.0f;
			float navg = 0.0f;
			for (int ii = i - pixrad; ii <= i + pixrad; ii++) {
				for (int jj = j - pixrad; jj <= j + pixrad; jj++) {
					if (InGrid(glm::ivec3(ii, jj, 0))) {
						if (use_plane_fitting_){
							glm::vec3 p = IndexToPoint(glm::ivec3(ii, jj, 0));
							float local_z = fit_planes_[ii][jj].GetHeightAt(p.x, p.y);
							roughsum += (float)pow(ground_[ii][jj] - local_z, 2.0f);
						}
						else{
							roughsum += (float)pow(ground_[ii][jj] - local_avg_[ii][jj], 2.0f);
						}
						navg += 1.0f;

					}
				}
			}
			if (navg > 0.0f) {
				roughness_[i][j] = sqrt(roughsum / (navg));
			}
			else {
				roughness_[i][j] = 0.0f;
			}
			if (roughness_[i][j] > max_roughness_)max_roughness_ = roughness_[i][j];
		}
	}
}

cimg_library::CImg<float>VoxelGrid::DrawSlope() {
	cimg_library::CImg<float> img;
	img.assign(dim_.x, dim_.y, 1, 3, 0.0f);
	img = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			glm::vec3 color = MorelandGreen(vehicle_.GetBeta() * slope_[i][j], 0.0f, 1.0);
			img.draw_point(i, j, (float *)&color);
		}
	}
	AddVehiclePathToImage(img);
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);
	return img;
	//if (slope_disp_.width() == 0 || slope_disp_.height() == 0) {
	//	slope_disp_.assign(img, "Slope");
	//}
	//else {
	//	slope_disp_ = img;
	//}
}

void VoxelGrid::PlotSlope() {
	cimg_library::CImg<float> img = DrawSlope();
	/*img.assign(dim_.x, dim_.y, 1, 3, 0.0f);
	img = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			glm::vec3 color = MorelandGreen(vehicle_.GetBeta() * slope_[i][j], 0.0f, 1.0);
			img.draw_point(i, j, (float *)&color);
		}
	}
	AddVehiclePathToImage(img);
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);*/
	if (slope_disp_.width() == 0 || slope_disp_.height() == 0) {
		slope_disp_.assign(img, "Slope");
	}
	else {
		slope_disp_ = img;
	}
}

cimg_library::CImg<float> VoxelGrid::DrawTraversability() {
	cimg_library::CImg<float> img;
	img.assign(dim_.x, dim_.y, 1, 3, 0.0f);
	img = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			glm::vec3 color = MorelandGreen(GetTraversabilityAtCell(i, j), 1.0f, 0.0);
			img.draw_point(i, j, (float *)&color);
		}
	}
	AddVehiclePathToImage(img);
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);
	return img;
	//if (trav_disp_.width() == 0 || trav_disp_.height() == 0) {
	//	trav_disp_.assign(img, "Traversability");
	//}
	//else {
	//	trav_disp_ = img;
	//}
}

void VoxelGrid::PlotTraversability() {
	cimg_library::CImg<float> img = DrawTraversability();
	/*img.assign(dim_.x, dim_.y, 1, 3, 0.0f);
	img = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			glm::vec3 color = MorelandGreen(GetTraversabilityAtCell(i, j), 1.0f, 0.0);
			img.draw_point(i, j, (float *)&color);
		}
	}
	AddVehiclePathToImage(img);
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);*/
	if (trav_disp_.width() == 0 || trav_disp_.height() == 0) {
		trav_disp_.assign(img, "Traversability");
	}
	else {
		trav_disp_ = img;
	}
}

void VoxelGrid::PlotTravAndImage(cimg_library::CImg<float> image, std::string prefix) {
	PlotTraversability();
	cimg_library::CImg<float> trav_img(trav_disp_);
	trav_img.resize(image.height(), image.height());
	cimg_library::CImg<float> combined;
	combined = trav_img.append(image, 'x');
	//combined.display();
	std::string fname = prefix.append("_combined.bmp");
	combined.save(fname.c_str());
}

cimg_library::CImg<float> VoxelGrid::DrawVegDensity() {
	cimg_library::CImg<float> img;
	img.assign(dim_.x, dim_.y, 1, 3, 0.0f);
	img = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			glm::vec3 color = MorelandGreen(impermeability_[i][j], 0.0f, 1.0f);
			img.draw_point(i, j, (float *)&color);
		}
	}
	AddVehiclePathToImage(img);
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);
	return img;
	//if (veg_disp_.width() == 0 || veg_disp_.height() == 0) {
	//	veg_disp_.assign(img, "Veg Density");
	//}
	//else {
	//	veg_disp_ = img;
	//}
}

void VoxelGrid::PlotVegDensity() {
	cimg_library::CImg<float> img = DrawVegDensity();
	/*img.assign(dim_.x, dim_.y, 1, 3, 0.0f);
	img = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			glm::vec3 color = MorelandGreen(impermeability_[i][j], 0.0f, 1.0f);
			img.draw_point(i, j, (float *)&color);
		}
	}
	AddVehiclePathToImage(img);
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);*/
	if (veg_disp_.width() == 0 || veg_disp_.height() == 0) {
		veg_disp_.assign(img, "Veg Density");
	}
	else {
		veg_disp_ = img;
	}
}

cimg_library::CImg<float> VoxelGrid::DrawRoughness() {
	cimg_library::CImg<float> img;
	img.assign(dim_.x, dim_.y, 1, 3, 0.0f);
	img = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			glm::vec3 color = MorelandGreen(vehicle_.GetGamma(roughness_[i][j]), 0.0f, 1.0f);
			img.draw_point(i, j, (float *)&color);
		}
	}
	AddVehiclePathToImage(img);
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);
	return img;
	//if (rough_disp_.width() == 0 || rough_disp_.height() == 0) {
	//	rough_disp_.assign(img, "Roughness");
	//}
	//else {
	//	rough_disp_ = img;
	//}
}

void VoxelGrid::PlotRoughness() {
	cimg_library::CImg<float> img = DrawRoughness();
	/*img.assign(dim_.x, dim_.y, 1, 3, 0.0f);
	img = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			glm::vec3 color = MorelandGreen(vehicle_.GetGamma(roughness_[i][j]), 0.0f, 1.0f);
			img.draw_point(i, j, (float *)&color);
		}
	}
	AddVehiclePathToImage(img);
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);*/
	if (rough_disp_.width() == 0 || rough_disp_.height() == 0) {
		rough_disp_.assign(img, "Roughness");
	}
	else {
		rough_disp_ = img;
	}
}

void VoxelGrid::SaveSlopePlot(std::string fname) {
	cimg_library::CImg<float> img = DrawSlope(); //(slope_disp_);
	img.mirror('y');
	img.save(fname.c_str());
}

void VoxelGrid::SaveTraversabilityPlot(std::string fname) {
	cimg_library::CImg<float> img = DrawTraversability(); //(trav_disp_);
	img.mirror('y');
	img.save(fname.c_str());
}

void VoxelGrid::SaveRoughPlot(std::string fname) {
	cimg_library::CImg<float> img = DrawRoughness(); //(rough_disp_);
	img.mirror('y');
	img.save(fname.c_str());
}

void VoxelGrid::SaveVegDensityPlot(std::string fname) {
	cimg_library::CImg<float> img = DrawVegDensity(); //(veg_disp_);
	img.mirror('y');
	img.save(fname.c_str());
}

cimg_library::CImg<float> VoxelGrid::DrawRci() {
	cimg_library::CImg<float> img;
	img.assign(dim_.x, dim_.y, 1, 3, 0.0f);
	img = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			glm::vec3 color = MorelandGreen(vehicle_.GetEta() / rci_[i][j], 0.0f, 1.0f);
			img.draw_point(i, j, (float *)&color);
		}
	}
	AddVehiclePathToImage(img);
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);
	return img;
	//if (rci_disp_.width() == 0 || rci_disp_.height() == 0) {
	//	rci_disp_.assign(img, "RCI");
	//}
	//else {
//		rci_disp_ = img;
	//}
}

void VoxelGrid::PlotRci() {
	cimg_library::CImg<float> img = DrawRci();
	/*img.assign(dim_.x, dim_.y, 1, 3, 0.0f);
	img = 0.0f;
	for (int i = 0; i < dim_.x; i++) {
		for (int j = 0; j < dim_.y; j++) {
			glm::vec3 color = MorelandGreen(vehicle_.GetEta() / rci_[i][j], 0.0f, 1.0f);
			img.draw_point(i, j, (float *)&color);
		}
	}
	AddVehiclePathToImage(img);
	int new_width = (int)(512.0f*(1.0f*img.width())/(1.0f*img.height()));
	img.resize(new_width,512);*/
	if (rci_disp_.width() == 0 || rci_disp_.height() == 0) {
		rci_disp_.assign(img, "RCI");
	}
	else {
		rci_disp_ = img;
	}
}

void VoxelGrid::SaveRciPlot(std::string fname) {
	cimg_library::CImg<float> img = DrawRci(); //(rci_disp_);
	img.mirror('y');
	img.save(fname.c_str());
}

float VoxelGrid::GetTraversabilityAtCell(int i, int j) {
	glm::ivec3 idx(i, j, 0);
	float trav = default_traversability_; // 0.0f;
	if (InGrid(idx)) {
		trav = vehicle_.GetTraversability(roughness_[i][j], slope_[i][j], impermeability_[i][j], rci_[i][j]);
		// adjust trav by confidence
		trav = (1.0f - cells_[i][j].confidence)*default_traversability_ + trav * cells_[i][j].confidence;
	}
	return trav;
}

float VoxelGrid::GetTraversabilityAtPoint(glm::vec3 p) {
	glm::ivec3 idx = PointToIndex(p);
	float trav = 0.0f;
	if (InGrid(idx)) {
		float z = p.z - ground_[idx.x][idx.y];
		if (z > veg_hi_lo_cutoff_.y) {
			return -1.0f;
		}

		float trav0 = GetTraversabilityAtCell(idx);
		float w0 = 1.0f / glm::length(p - IndexToPoint(idx));

		float trav1 = 0.0f;
		float w1 = 0.0f;
		glm::ivec3 idx1(idx.x + 1, idx.y, idx.z);
		if (InGrid(idx1)) {
			trav1 = GetTraversabilityAtCell(idx1);
			w1 = 1.0f / glm::length(p - IndexToPoint(idx1));
		}

		float trav2 = 0.0f;
		float w2 = 0.0f;
		glm::ivec3 idx2(idx.x, idx.y + 1, idx.z);
		if (InGrid(idx2)) {
			trav2 = GetTraversabilityAtCell(idx2);
			w2 = 1.0f / glm::length(p - IndexToPoint(idx2));
		}

		float trav3 = 0.0f;
		float w3 = 0.0f;
		glm::ivec3 idx3(idx.x + 1, idx.y + 1, idx.z);
		if (InGrid(idx3)) {
			trav3 = GetTraversabilityAtCell(idx3);
			w3 = 1.0f / glm::length(p - IndexToPoint(idx3));
		}

		float trav4 = 0.0f;
		float w4 = 0.0f;
		glm::ivec3 idx4(idx.x - 1, idx.y, idx.z);
		if (InGrid(idx4)) {
			trav4 = GetTraversabilityAtCell(idx4);
			w4 = 1.0f / glm::length(p - IndexToPoint(idx4));
		}

		float trav5 = 0.0f;
		float w5 = 0.0f;
		glm::ivec3 idx5(idx.x, idx.y - 1, idx.z);
		if (InGrid(idx5)) {
			trav5 = GetTraversabilityAtCell(idx5);
			w5 = 1.0f / glm::length(p - IndexToPoint(idx5));
		}

		float trav6 = 0.0f;
		float w6 = 0.0f;
		glm::ivec3 idx6(idx.x - 1, idx.y - 1, idx.z);
		if (InGrid(idx6)) {
			trav6 = GetTraversabilityAtCell(idx6);
			w6 = 1.0f / glm::length(p - IndexToPoint(idx6));
		}

		float trav7 = 0.0f;
		float w7 = 0.0f;
		glm::ivec3 idx7(idx.x + 1, idx.y - 1, idx.z);
		if (InGrid(idx7)) {
			trav7 = GetTraversabilityAtCell(idx);
			w7 = 1.0f / glm::length(p - IndexToPoint(idx7));
		}

		float trav8 = 0.0f;
		float w8 = 0.0f;
		glm::ivec3 idx8(idx.x - 1, idx.y + 1, idx.z);
		if (InGrid(idx8)) {
			trav8 = GetTraversabilityAtCell(idx);
			w8 = 1.0f / glm::length(p - IndexToPoint(idx8));
		}

		trav = (w0*trav0 + w1 * trav1 + w2 * trav2 + w3 * trav3 + w4 * trav4 + w5 * trav5 + w6 * trav6 + w7 * trav7 + w8 * trav8) / (w0 + w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8);

		trav = std::max(0.0f, trav);
		trav = std::min(1.0f, trav);
	}
	return trav;
}

// a1 is line1 start, a2 is line1 end, b1 is line2 start, b2 is line2 end
bool VoxelGrid::LineLineIntersect(glm::vec2 a1, glm::vec2 a2, glm::vec2 b1, glm::vec2 b2) {
	// https://stackoverflow.com/questions/3746274/line-intersection-with-aabb-rectangle
	//intersection = glm::vec2(0.0f, 0.0f);

	glm::vec2 b = a2 - a1;
	glm::vec2 d = b2 - b1;
	float bDotDPerp = b.x * d.y - b.y * d.x;

	// if b dot d == 0, it means the lines are parallel so have infinite intersection points
	if (bDotDPerp == 0)
		return false;

	glm::vec2 c = b1 - a1;
	float t = (c.x * d.y - c.y * d.x) / bDotDPerp;
	if (t < 0 || t > 1)
		return false;

	float u = (c.x * b.y - c.y * b.x) / bDotDPerp;
	if (u < 0 || u > 1)
		return false;

	//intersection = a1 + t * b;

	return true;
}

bool VoxelGrid::LineBoxIntersect(glm::vec2 origin, glm::vec2 endpoint, glm::ivec2 box_idx) {
	// find the four corners of the box
	// p3-------p2
	// |         |
	// |         |
	// |         |
	// p0-------p1
	
	int i = box_idx.x;
	int j = box_idx.y;
	if (!(i >= 0 && i < dim_.x && j >= 0 && j < dim_.y)) return false;
	glm::vec2 p0(llc_.x + i * res_, llc_.y + j * res_);
	glm::vec2 p1(llc_.x + (i + 1)*res_, llc_.y + j * res_);
	glm::vec2 p2(llc_.x + (i + 1)*res_, llc_.y + (j + 1)*res_);
	glm::vec2 p3(llc_.x + i * res_, llc_.y + (j + 1)*res_);
	// first check if origin is inside box
	if (origin.x >= p0.x && origin.x <= p1.x && origin.y >= p0.x && origin.y <= p3.y)return true;
	// then check if endpoint is inside box
	if (endpoint.x >= p0.x && endpoint.x <= p1.x && endpoint.y >= p0.x && endpoint.y <= p3.y)return true;
	// finally, check for intersections with each side
	bool b0 = LineLineIntersect(origin, endpoint, p0, p1);
	if (b0) return true;
	bool b1 = LineLineIntersect(origin, endpoint, p1, p2);
	if (b1) return true;
	bool b2 = LineLineIntersect(origin, endpoint, p2, p3);
	if (b2) return true;
	bool b3 = LineLineIntersect(origin, endpoint, p3, p0);
	if (b3) return true;
	return false;
}

nav_msgs::OccupancyGrid VoxelGrid::GetTraversabilityAsOccupancyGrid(bool row_major) {
	nav_msgs::OccupancyGrid grid;
	grid.data.resize(dim_.x*dim_.y, 0);
	int n = 0;
	if (row_major){
		for (int j = 0; j < dim_.y; j++) {
			for (int i = 0; i < dim_.x; i++) {

				float trav = vehicle_.GetTraversability(roughness_[i][j], slope_[i][j], impermeability_[i][j], rci_[i][j]);
				grid.data[n] = (int)(100.0f*(1.0f - trav));
				n++;
			}
		}
	}
	else {
		for (int i = 0; i < dim_.x; i++) {
			for (int j = 0; j < dim_.y; j++) {

				float trav = vehicle_.GetTraversability(roughness_[i][j], slope_[i][j], impermeability_[i][j], rci_[i][j]);
				grid.data[n] = (int)(100.0f*(1.0f - trav));
				n++;
			}
		}
	}
	grid.info.height = dim_.y;
	grid.info.width = dim_.x;
	grid.info.resolution = res_;
	grid.info.origin.position.x = llc_.x;
	grid.info.origin.position.y = llc_.y;
	grid.info.origin.position.z = llc_.z;
	grid.info.origin.orientation.w = 1.0f;
	grid.info.origin.orientation.x = 0.0f;
	grid.info.origin.orientation.y = 0.0f;
	grid.info.origin.orientation.z = 0.0f;
	return grid;
}

} // namespace traverselib
