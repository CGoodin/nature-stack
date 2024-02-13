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
#include "nature/perception/ftte/moreland.h"

namespace traverselib {

glm::vec3 MorelandColormap(float x, float xmin, float xmax) {
	// Diverging Color Maps for Scientific Visualization
	// Kenneth Moreland
	x = (x - xmin) / (xmax - xmin);
	if (x < 0.0f) x = 0.0f;
	if (x > 1.0f) x = 1.0f;
	float x2 = x * x;
	float x3 = x2 * x;
	float x4 = x3 * x;
	float x5 = x4 * x;
	float x6 = x5 * x;
	glm::vec3 color;
	color.b = -971.97f*x6 + 1841.3f*x5 - 225.53f*x4 - 804.89f*x3 - 387.13f*x2 + 393.62f*x + 192.0f;
	color.r = 226.07f*x6 + 100.66f*x5 - 883.07f*x4 + 242.79f*x3 + 146.89f*x2 + 288.16f*x + 59.0f;
	color.g = -6584.7f*x6 + 18406.0f*x5 - 18432.0f*x4 + 7588.7f*x3 - 1589.2f*x2 + 541.05f*x + 76.0f;
	return color;
}

glm::vec3 MorelandGreen(float x, float xmin, float xmax) {
	// Diverging Color Maps for Scientific Visualization
	// Kenneth Moreland
	x = (x - xmin) / (xmax - xmin);
	if (x < 0.0f) x = 0.0f;
	if (x > 1.0f) x = 1.0f;
	float x2 = x * x;
	float x3 = x2 * x;
	float x4 = x3 * x;
	float x5 = x4 * x;
	float x6 = x5 * x;
	glm::vec3 color;
	color.g = -971.97f*x6 + 1841.3f*x5 - 225.53f*x4 - 804.89f*x3 - 387.13f*x2 + 393.62f*x + 192.0f;
	color.r = 226.07f*x6 + 100.66f*x5 - 883.07f*x4 + 242.79f*x3 + 146.89f*x2 + 288.16f*x + 59.0f;
	color.b = -6584.7f*x6 + 18406.0f*x5 - 18432.0f*x4 + 7588.7f*x3 - 1589.2f*x2 + 541.05f*x + 76.0f;
	return color;
}

} // namespace traverselib