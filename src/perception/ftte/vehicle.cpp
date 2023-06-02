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
#include "nature/perception/ftte/vehicle.h"
#include <math.h>
#include <iostream>

namespace traverselib {

Vehicle::Vehicle() {
	// default parameters are for HMMWV
	vci1_ = 25.0f; 
	roof_height_ = 1.99f; 
	max_slope_ = 0.55f;
	mass_ = 3697.0f; //kg
	bumper_height_ = 0.75f;
	beta_ = 0.0f;
	eta_ = 50.0f;
	tire_radius_ = 0.4f;
	slope_coeff_ = 1.0f;
	slope_exp_ = 1.0f;
	soil_coeff_ = 1.0f;
	soil_exp_ = 1.0f;
	veg_coeff_ = 1.0f;
	veg_exp_ = 1.0f;
	rough_coeff_ = 1.0f;
	rough_exp_ = 1.0f;
}

void Vehicle::SetParams(float mass, float bumper_height, float tire_radius, float vci1, float max_slope, float roof_height) {
	beta_ = 1.0f / max_slope;
	eta_ = vci1;
	sigma_max_ = tire_radius; 
	critical_diameter_ = (float)(pow(mass, 1.0 / 3.0)*3.684E-3f*pow(10.86 - 5.34*bumper_height, -1.0 / 3.0));
	roof_height_ = roof_height;
}

void Vehicle::SetGridResolution(float h) {
	res_ = h;
	res_squared_ = h * h;
	rv_ = (float)sqrt(3.0)*res_;
}

float Vehicle::GetGamma(float sigma) {
	float gamma = 1.0f - sigma_max_ / (sigma*sigma + sigma_max_);
	return gamma;
}

float Vehicle::GetTraversability(float rms, float slope, float impermeability, float rci) {
	float nu = 1.0f;
	if (impermeability<1.0f)nu = res_squared_ * (float)(-log(1.0f - impermeability) / (rv_*critical_diameter_));
	//float trav = 1.0f - beta_ * slope - eta_ / rci - GetGamma(rms) - nu;
	float trav = 1.0f - slope_coeff_*pow(beta_ * slope,slope_exp_)
				      - soil_coeff_*pow(eta_ / rci,soil_exp_)
					  - rough_coeff_*pow(GetGamma(rms),rough_exp_) 
					  - veg_coeff_*pow(nu,veg_exp_);
	if (trav < 0.0f)trav = 0.0f;
	if (trav > 1.0f)trav = 1.0f;
	return trav;
}

} // namespace traverselib
