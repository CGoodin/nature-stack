/**
 * \class Ellipsoid
 * Class that defines parameters of the reference ellipsoid being used.
 * Modified from from the code posted on 
 * "www.gpsy.com/gpsinfo/geotoutm", 4 Dec 2017 by Chuck Gantz
 *
 * \author Chris Goodin
 *
 * \date 12/13/2017
 */

#ifndef AVT_341_ELLIPSOID_H
#define AVT_341_ELLIPSOID_H

#include <vector>
#include <string>

namespace nature{
namespace coordinate_system{

/**
 * The Ellipsoid class defines 23 reference ellipsoids, with
 * corresponding reference numbers.
 * 1, "Airy"
 * 2, "Australian National"
 * 3, "Bessel 1841"
 * 4, "Bessel 1841 (Nambia)"
 * 5, "Clarke 1866"
 * 6, "Clarke 1880"
 * 7, "Everest"
 * 8, "Fischer 1960 (Mercury)"
 * 9, "Fischer 1968"
 * 10, "GRS 1967"
 * 11, "GRS 1980"
 * 12, "Helmert 1906"
 * 13, "Hough"
 * 14, "International"
 * 15, "Krassovsky"
 * 16, "Modified Airy"
 * 17, "Modified Everest"
 * 18, "Modified Fischer 1960"
 * 19, "South American 1969"
 * 20, "WGS 60"
 * 21, "WGS 66"
 * 22, "WGS-72"
 * 23 "WGS-84"
 */
class Ellipsoid{
 public:
  ///Declare an ellipsoid without initializing constants. 
  Ellipsoid();

  /**
   * Declare an ellipsoid with initialization of constants.
   * \param id Unique id number of the ellipsoid.
   * \param name Reference name of the ellipsoid.
   * \param radius Equatorial radius of the refrence, meters.
   * \param ecc Squared eccentricity of the ellipsoid.
   */
  Ellipsoid(int id, std::string name, double radius, double ecc);

  /// Get the equatorial radius of the elllipsoid.
  double GetEquatorialRadius(){return equatorial_radius_;}

  /// Get the squared eccentricity of the ellipsoid
  double GetEccentricitySquared(){return eccentricity_squared_;}

  /// Get the equatorial radius of one of the 23 reference ellipsoids.
  double GetEquatorialRadius(int refnum );

  /// Get the squared of one of the 23 reference ellipsois.
  double GetEccentrictySquared(int refnum);
 
 private: 
  void Init(); 
  int id_;
  std::string ellipsoid_name_;
  double equatorial_radius_; 
  double eccentricity_squared_;  
};

} //namespace coordinate_system
} //namespace nature

#endif
