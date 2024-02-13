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
#ifndef TRAVERSE_CELL_H
#define TRAVERSE_CELL_H

#include <limits>

namespace traverselib {

class TraverseCell {
public:
	TraverseCell() {
		highest = std::numeric_limits<float>::lowest();
		lowest = std::numeric_limits<float>::max();
		num_points_hit = 0;
		num_points_total = 0.0f;
		num_veg_hits = 0.0f;
		ground_candidate = false;
		confidence = 0.0f;
		conf_accum = 0.0f;
	}
	float highest;
	float confidence;
	float conf_accum;
	float lowest;
	int num_points_hit;
	float num_points_total;
	float num_veg_hits;
	bool ground_candidate;
};

} // namespace traverselib

#endif
