// class definition
#include <nature/planning/global/coord_conversions/coord_conversions.h>
// c++ includes
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

//#include <mavs_core/math/constants.h>

namespace nature{
namespace coordinate_system{

static const double k2Pi = 6.283185307179586476925286766559;
static const double kRadToDeg = 180.0 / 3.1415926535897932384626433832795;
static const double kDegToRad = 3.1415926535897932384626433832795 / 180.0;

CoordinateConverter::CoordinateConverter(){
  rot_transpose_.Resize(3,3);
  rot_transpose_inverse_.Resize(3,3);
  reference_position_.Resize(3,1);
  
  //use WGS84 by default
  SetReferenceEllipsoid(23);

  //default local origin is starkville, MS
  coordinate_system::LLA lla;
  lla.latitude = 33.4504;
  lla.longitude = -88.8184;
  lla.altitude = 102.108; //meters
  SetLocalOrigin(lla);
}

void CoordinateConverter::SetReferenceEllipsoid(int ref){
  ref_ellips_ = ref;
  a_ = ellipsoid_.GetEquatorialRadius(ref_ellips_);
  e2_ = ellipsoid_.GetEccentrictySquared(ref_ellips_);
  e_ = sqrt(e2_);
  one_minus_e2 = 1.0-e2_;
}

void CoordinateConverter::SetLocalOrigin(coordinate_system::UTM utm){
  local_origin_ecef_ = UTM2ECEF(utm);
  local_origin_lla_ = UTM2LLA(utm);
  SetMatrices();
}

void CoordinateConverter::SetLocalOrigin(coordinate_system::LLA lla){
  local_origin_ecef_ = LLA2ECEF(lla);
  local_origin_lla_ = lla;
  SetMatrices();
}

void CoordinateConverter::SetLocalOrigin(double lat, double lon, double alt){
  coordinate_system::LLA lla;
  lla.latitude = lat;
  lla.longitude = lon;
  lla.altitude = alt;
  SetLocalOrigin(lla);
}

void CoordinateConverter::SetLocalOrigin(coordinate_system::ECEF ecef){
  local_origin_ecef_ = ecef;
  local_origin_lla_ = ECEF2LLA(ecef);
  SetMatrices();
}

void CoordinateConverter::SetMatrices(){
  double latrad = local_origin_lla_.latitude*kDegToRad;
  double lonrad = local_origin_lla_.longitude*kDegToRad;
  double slat = sin(latrad);
  double slon = sin(lonrad);
  double clat = cos(latrad);
  double clon = cos(lonrad);
  math::Matrix R(3,3);
  reference_position_(0,0) = local_origin_ecef_.x;
  reference_position_(1,0) = local_origin_ecef_.y;
  reference_position_(2,0) = local_origin_ecef_.z;
  R(0,0) = -slat*clon;
  R(0,1) = -slon;
  R(0,2) = -clat*clon;
  R(1,0) = -slat*slon;
  R(1,1) = clon;
  R(1,2) = -clat*slon;
  R(2,0) = clat;
  R(2,1) = 0.0;
  R(2,2) = -slat;
  rot_transpose_ = R.Transpose();
  rot_transpose_inverse_ = rot_transpose_.Inverse();
}


coordinate_system::ECEF CoordinateConverter::LLA2ECEF(coordinate_system::LLA
						      lla){
  //From:Department of Defense World Geodetic System1984, p. 4-4
  //see matlab implementation by Michael Kleder on File Exchange
  //BSD 2 clause license
  double lat = kDegToRad*lla.latitude;
  double lon = kDegToRad*lla.longitude;
  double slat = sin(lat);
  double clat = cos(lat);
  double slon = sin(lon);
  double clon = cos(lon);
  double N = a_/sqrt(1-e2_*slat*slat);
  coordinate_system::ECEF xyz;
  double Nac = (N+lla.altitude)*clat;
  xyz.x = Nac*clon;
  xyz.y = Nac*slon;
  xyz.z = (one_minus_e2*N+lla.altitude)*slat;
  return xyz;
}
  
coordinate_system::LLA CoordinateConverter::ECEF2LLA(coordinate_system::ECEF
						     ecef){
  //WGS84 ellipsoid
  //see matlab implementation by Michael Kleder on File Exchange
  //BSD 2 clause license
  double a2 = a_*a_;
  double b = sqrt(a2*one_minus_e2);
  double b2 = b*b;
  double ep = sqrt((a2-b2)/b2);
  double p = sqrt(ecef.x*ecef.x+ecef.y*ecef.y);
  double th = atan2(a_*ecef.z,b*p);
  double lon = atan2(ecef.y, ecef.x);
  double lat = atan2((ecef.z+ep*ep*b*pow(sin(th),3.0)),(p-e2_*a_*pow(cos(th),3.0)));
  double slat = sin(lat);
  double N = a_/sqrt(1-e2_*slat*slat);
  double alt = p/cos(lat) - N;
  while (lon>k2Pi){
    lon = lon - k2Pi;
  }
  coordinate_system::LLA lla;
  lla.altitude = alt;
  lla.longitude = kRadToDeg*lon;
  lla.latitude = kRadToDeg*lat;
  return lla;
}


/*ctg: The following conversions from UTM to LLA and back are by Chuck Gantz, 
 * taken from the code posted on "www.gpsy.com/gpsinfo/geotoutm", 4 Dec 2017
 * No license was given */

/*Reference ellipsoids derived from Peter H. Dana's website- 
http://www.utexas.edu/depts/grg/gcraft/notes/datum/elist.html
Department of Geography, University of Texas at Austin
Internet: pdana@mail.utexas.edu
3/22/95

Source
Defense Mapping Agency. 1987b. DMA Technical Report: Supplement to 
Department of Defense World Geodetic System
1984 Technical Report. Part I and II. Washington, DC: Defense Mapping Agency
*/
  
coordinate_system::UTM CoordinateConverter::LLA2UTM(coordinate_system::LLA lla){
  //converts lat/long to UTM coords.  Equations from USGS Bulletin 1532 
  //East Longitudes are positive, West longitudes are negative. 
  //North latitudes are positive, South latitudes are negative
  //Lat and Long are in decimal degrees
  //Written by Chuck Gantz- chuck.gantz@globalstar.com
  double Lat = lla.latitude;
  double Long = lla.longitude;
  coordinate_system::UTM utm;
  utm.altitude = lla.altitude;
  double UTMNorthing;
  double UTMEasting;
  //char* UTMZone;
  double a = a_;
  double eccSquared = e2_;
  double k0 = 0.9996;
  
  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;
  
  //Make sure the longitude is between -180.00 .. 179.9
  double LongTemp = (Long+180)-int((Long+180)/360)*360-180;
  
  double LatRad = Lat*kDegToRad; //rad;
  double LongRad = LongTemp*kDegToRad; //rad;
  double LongOriginRad;
  int    ZoneNumber;
  ZoneNumber = int((LongTemp + 180)/6) + 1;
  
  if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
    ZoneNumber = 32;
  
  // Special zones for Svalbard
  if( Lat >= 72.0 && Lat < 84.0 ) {
    if(      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
    else if( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
    else if( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
    else if( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
  }
  //+3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;  
  LongOriginRad = LongOrigin * kDegToRad; 

  eccPrimeSquared = (eccSquared)/(1-eccSquared);
  
  N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
  T = tan(LatRad)*tan(LatRad);
  C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
  A = cos(LatRad)*(LongRad-LongOriginRad);
  
  M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64	- 
	  5*eccSquared*eccSquared*eccSquared/256)*LatRad - 
	 (3*eccSquared/8 + 3*eccSquared*eccSquared/32	
	  + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
	 + (15*eccSquared*eccSquared/256 
	    + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad) 
	 - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));
  
  UTMEasting = (double)(k0*N*(A+(1-T+C)*A*A*A/6
			      + (5-18*T+T*T+72*C-58*eccPrimeSquared)
			      *A*A*A*A*A/120)+ 500000.0);
  
  UTMNorthing = (double)(k0*(M+N*tan(LatRad)*
			     (A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
			      + (61-58*T+T*T+600*C-330*eccPrimeSquared)
			      *A*A*A*A*A*A/720)));
  if(Lat < 0)
    UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere

  utm.x = UTMEasting;
  utm.y = UTMNorthing;
  utm.zone_num = ZoneNumber;
  utm.zone_char = UTMLetterDesignator(Lat);
  return utm;
}

char CoordinateConverter::UTMLetterDesignator(double Lat)
{
  //This routine determines the correct UTM letter designator 
  //for the given latitude
  //returns 'Z' if latitude is outside the UTM limits of 84N to 80S
  //Written by Chuck Gantz- chuck.gantz@globalstar.com
  char LetterDesignator;
  if((84 >= Lat) && (Lat >= 72)) LetterDesignator = 'X';
  else if((72 > Lat) && (Lat >= 64)) LetterDesignator = 'W';
  else if((64 > Lat) && (Lat >= 56)) LetterDesignator = 'V';
  else if((56 > Lat) && (Lat >= 48)) LetterDesignator = 'U';
  else if((48 > Lat) && (Lat >= 40)) LetterDesignator = 'T';
  else if((40 > Lat) && (Lat >= 32)) LetterDesignator = 'S';
  else if((32 > Lat) && (Lat >= 24)) LetterDesignator = 'R';
  else if((24 > Lat) && (Lat >= 16)) LetterDesignator = 'Q';
  else if((16 > Lat) && (Lat >= 8)) LetterDesignator = 'P';
  else if(( 8 > Lat) && (Lat >= 0)) LetterDesignator = 'N';
  else if(( 0 > Lat) && (Lat >= -8)) LetterDesignator = 'M';
  else if((-8> Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
  //Z is an error flag to show that the Latitude is outside the UTM limits
  else LetterDesignator = 'Z'; 
  return LetterDesignator;
}


coordinate_system::LLA CoordinateConverter::UTM2LLA(coordinate_system::UTM utm){
  //converts UTM coords to lat/long.  Equations from USGS Bulletin 1532 
  //East Longitudes are positive, West longitudes are negative. 
  //North latitudes are positive, South latitudes are negative
  //Lat and Long are in decimal degrees. 
  //Written by Chuck Gantz- chuck.gantz@globalstar.com
  double UTMNorthing = utm.y;
  double UTMEasting = utm.x;
  double Lat;
  double Long;
  double k0 = 0.9996;
  double a = a_;
  double eccSquared = e2_;;
  double eccPrimeSquared;
  double e1 = (1-sqrt(1-eccSquared))/(1+sqrt(1-eccSquared));
  double N1, T1, C1, R1, D, M;
  double LongOrigin;
  double mu, phi1, phi1Rad;
  double x, y;
  int ZoneNumber = utm.zone_num;
  char ZoneLetter = utm.zone_char;
  int NorthernHemisphere; //1 for northern hemispher, 0 for southern
  
  x = UTMEasting - 500000.0; //remove 500,000 meter offset for longitude
  y = UTMNorthing;
  
  if((ZoneLetter - 'N') >= 0)
    NorthernHemisphere = 1;//point is in northern hemisphere
  else
    {
      NorthernHemisphere = 0;//point is in southern hemisphere
      //remove 10,000,000 meter offset used for southern hemisphere
      y -= 10000000.0;
    }
  //+3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1)*6 - 180 + 3; 
  eccPrimeSquared = (eccSquared)/(1-eccSquared);
  
  M = y / k0;
  mu = M/(a*(1-eccSquared/4-3*eccSquared*eccSquared/
	     64-5*eccSquared*eccSquared*eccSquared/256));
  
  phi1Rad = mu	+ (3*e1/2-27*e1*e1*e1/32)*sin(2*mu) 
    + (21*e1*e1/16-55*e1*e1*e1*e1/32)*sin(4*mu)
    +(151*e1*e1*e1/96)*sin(6*mu);
  phi1 = phi1Rad*kRadToDeg;
  
  N1 = a/sqrt(1-eccSquared*sin(phi1Rad)*sin(phi1Rad));
  T1 = tan(phi1Rad)*tan(phi1Rad);
  C1 = eccPrimeSquared*cos(phi1Rad)*cos(phi1Rad);
  R1 = a*(1-eccSquared)/pow(1-eccSquared*sin(phi1Rad)*sin(phi1Rad), 1.5);
  D = x/(N1*k0);
  
  Lat = phi1Rad - (N1*tan(phi1Rad)/R1)*
    (D*D/2-(5+3*T1+10*C1-4*C1*C1-9*eccPrimeSquared)*D*D*D*D/24
     +(61+90*T1+298*C1+45*T1*T1-252*eccPrimeSquared-3*C1*C1)*D*D*D*D*D*D/720);
  Lat = Lat * kRadToDeg; 
  Long = (D-(1+2*T1+C1)*D*D*D/6
	  +(5-2*C1+28*T1-3*C1*C1+8*eccPrimeSquared+24*T1*T1)
	  *D*D*D*D*D/120)/cos(phi1Rad);
  Long = LongOrigin + Long * kRadToDeg; 
  coordinate_system::LLA lla;
  lla.altitude = utm.altitude;
  lla.latitude = Lat;
  lla.longitude = Long;
  return lla;
}

coordinate_system::ECEF CoordinateConverter::UTM2ECEF(coordinate_system::UTM
						      utm){
  coordinate_system::LLA lla = UTM2LLA(utm);
  coordinate_system::ECEF ecef = LLA2ECEF(lla);
  return ecef;
}
  
coordinate_system::UTM CoordinateConverter::ECEF2UTM(coordinate_system::ECEF
						     ecef){
  coordinate_system::LLA lla = ECEF2LLA(ecef);
  coordinate_system::UTM utm = LLA2UTM(lla);
  return utm;
}
  
coordinate_system::ENU CoordinateConverter::ECEF2ENU(coordinate_system::ECEF
						     ecef){
  math::Matrix p_ecef(3,1),p_ned(3,1);
  p_ecef(0,0) = ecef.x;
  p_ecef(1,0) = ecef.y;
  p_ecef(2,0) = ecef.z;
  p_ned = rot_transpose_*(p_ecef-reference_position_);
  coordinate_system::ENU enu;
  enu.x = p_ned(0,0);
  enu.y = p_ned(1,0);
  enu.z = -p_ned(2,0);
  return enu;
}
  
coordinate_system::ECEF CoordinateConverter::ENU2ECEF(coordinate_system::ENU
						      enu){
  math::Matrix p_ecef(3,1), p_ned(3,1);
  p_ned(0,0) = enu.x;
  p_ned(1,0) = enu.y;
  p_ned(2,0) = -enu.z;
  p_ecef = rot_transpose_inverse_*p_ned + reference_position_;
  coordinate_system::ECEF ecef;
  ecef.x = p_ecef(0,0);
  ecef.y = p_ecef(1,0);
  ecef.z = p_ecef(2,0);
  return ecef;
}

coordinate_system::LLA CoordinateConverter::ENU2LLA(coordinate_system::ENU enu){
  coordinate_system::ECEF ecef = ENU2ECEF(enu);
  coordinate_system::LLA lla = ECEF2LLA(ecef);
  return lla;
}

coordinate_system::ENU CoordinateConverter::LLA2ENU(coordinate_system::LLA lla){
  coordinate_system::ECEF ecef = LLA2ECEF(lla);
  coordinate_system::ENU enu = ECEF2ENU(ecef);
  return enu;
}

coordinate_system::UTM CoordinateConverter::ENU2UTM(coordinate_system::ENU enu){
  coordinate_system::ECEF ecef = ENU2ECEF(enu);
  coordinate_system::UTM utm = ECEF2UTM(ecef);
  return utm;
}

coordinate_system::ENU CoordinateConverter::UTM2ENU(coordinate_system::UTM utm){
  coordinate_system::ECEF ecef = UTM2ECEF(utm);
  coordinate_system::ENU enu = ECEF2ENU(ecef);
  return enu;
}

} //namespace environment
} //namespace nature
