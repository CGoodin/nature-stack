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
#ifndef VEHICLE_H_
#define VEHICLE_H_

namespace traverselib {

class Vehicle {
public:
	Vehicle();

	float GetTraversability(float rms, float slope, float veg_dens, float rci);

	float GetBeta() { return beta_; }

	float GetEta() { return eta_; }

	float GetGamma(float roughness);

	float GetCriticalDiameter() { return critical_diameter_; }

	float GetBumperHeight() { return bumper_height_; }

	float GetRoofHeight() { return roof_height_; }

	void SetParams(float mass, float bumper_height, float tire_radius, float vci1, float max_slope, float roof_height);

	void SetGridResolution(float h);

	void SetSlopeCoeff(float sc){ slope_coeff_ = sc; }

	void SetSlopeExponent(float se){ slope_exp_ = se; }

	void SetSoilCoeff(float sc){ soil_coeff_ = sc; }

	void SetSoilExponent(float se){ soil_exp_ = se; }

	void SetRoughnessCoeff(float rc){ rough_coeff_ = rc; }

	void SetRoughnessExponent(float re){ rough_exp_ = re; }

	void SetVegCoeff(float vc){ veg_coeff_ = vc; }

	void SetVegExponent(float ve){ veg_exp_ = ve; }

private:
	float res_;
	float res_squared_;
	float rv_;
	float mass_;
	float tire_radius_;
	float bumper_height_;
	float vci1_;
	float max_slope_;
	float roof_height_;
	float beta_;
	float eta_;
	float sigma_max_;
	float critical_diameter_;
	float slope_coeff_, slope_exp_;
	float soil_coeff_, soil_exp_;
	float veg_coeff_, veg_exp_;
	float rough_coeff_, rough_exp_;
};

} // namespace traverselib
#endif