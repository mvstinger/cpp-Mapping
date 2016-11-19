/*
 * MAPP__types.h
 *
 *  Created on: Nov 15, 2016
 *      Author: mvstinger
 */

#ifndef MAPP__TYPES_H_
#define MAPP__TYPES_H_



#include <Eigen/Eigen>
#include "MAPP__datums.h"


namespace Mapping {



typedef unsigned int uint;
typedef unsigned int Idx;



//	ERRORs

//TODO: Convert to logger
enum MappingError {
	NO_ERROR = 0,
	UNK_ERROR = 1,
	WARNING_LAT_OOB = -10,
	WARNING_LON_OOB = -11
};
//const uint WARNING_LAT_OOB = 1;
//const uint WARNING_LON_OOB = 1;



// Forward declarations
class MapSourceInterface;
class MapSourceBase;
class FileMapSource;

class MapInterface;
class MapBase;
class SimpleMap;



struct MapExtents {
	double lat_min;
	double lat_max;
	double lon_min;
	double lon_max;

	MapExtents(void);
	MapExtents(const double, const double, const double, const double);
};



struct MapResolution {
	double lat_res;
	double lon_res;

	MapResolution(void);
	MapResolution(const double, const double);
};



class MapSourceInterface {
public:
	virtual ~MapSourceInterface(void) = 0;

	virtual int open(void) = 0;
	virtual int close(void) = 0;
	virtual int buffer(void) = 0;
};



class MapSourceBase : MapSourceInterface {};



class FileMapSource : MapSourceBase {};



class MapInterface {
public:
	//	Define constructors for initialization of children
	MapInterface(void);
	MapInterface(const MapExtents, const MapResolution);
	MapInterface(const GeodeticDatumInterface*, const MapExtents, const MapResolution);

  //  Make pure virtual (make MapInterface ABC)
  virtual ~MapInterface()=0;

  //  Getters/Setters
  virtual double get_lat_min(void) const = 0;
  virtual double get_lat_max(void) const = 0;
  virtual uint get_lat_count(void) const = 0;
  virtual double get_lat_span(void) const = 0;
  virtual double get_lon_min(void) const = 0;
  virtual double get_lon_max(void) const = 0;
  virtual uint get_lon_count(void) const = 0;
  virtual double get_lon_span(void) const = 0;

  virtual MappingError set_lat_min(const double) = 0;
  virtual MappingError set_lat_max(const double) = 0;
//  virtual MappingError set_lat_count(const double) = 0;
  virtual MappingError set_lon_min(const double) = 0;
  virtual MappingError set_lon_max(const double) = 0;
//  virtual MappingError set_lon_count(const double) = 0;

  //	Convenience methods
  virtual uint write_csv(std::FILE*) const = 0;

  virtual double alt_at(const double, const double) const = 0;
  double radius_at(const double, const double) const;

protected:
  MapExtents map_extents_;
  MapResolution map_resolution_;
  Eigen::MatrixXd map_data_;
  const GeodeticDatumInterface* datum_;

  double CALC_ALT_RES_ = 0.001;
  uint CALC_ALT_MAX_ITER_ = 20;

  virtual MappingError update_lat_reso_(void) = 0;
  virtual MappingError update_lon_reso_(void) = 0;
  virtual MappingError update_map_size_(void) = 0;
  virtual MappingError update_all_(void) = 0;
};



class MapBase : public MapInterface {
public:
	//	Define constructors for initialization of children
	MapBase(void);
	MapBase(const MapExtents, const MapResolution);
	MapBase(const GeodeticDatumInterface*, const MapExtents, const MapResolution);

  //  Make pure virtual (make MapInterface ABC)
  virtual ~MapBase(void) = 0;
  static SimpleMap from_random(const int, const int, const double);

  //  Getters/Setters
  double get_lat_min(void) const;
  double get_lat_max(void) const;
  double get_lat_span(void) const;
  uint get_lat_count(void) const;
  double get_lon_min(void) const;
  double get_lon_max(void) const;
  double get_lon_span(void) const;
  uint get_lon_count(void) const;

  MappingError set_lat_min(const double);
  MappingError set_lat_max(const double);
//  MappingError set_lat_count(const double);
  MappingError set_lon_min(const double);
  MappingError set_lon_max(const double);
//  MappingError set_lon_count(const double);

  uint write_csv(std::FILE*) const;

  double alt_at(const double, const double) const;
  double radius_at(const double, const double) const;
  double height_at(const double, const double) const;

protected:
  MappingError update_lat_reso_(void);
  MappingError update_lon_reso_(void);
  MappingError update_map_size_(void);
  MappingError update_all_(void);
};



class SimpleMap : public MapBase {
public:
	SimpleMap(void);
	SimpleMap(const MapExtents, const MapResolution);
	SimpleMap(const GeodeticDatumInterface*, const MapExtents, const MapResolution);
	SimpleMap(const SimpleMap&);
  ~SimpleMap(void);
};



}



#endif /* MAPP__TYPES_H_ */
