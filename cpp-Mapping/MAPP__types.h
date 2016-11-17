/*
 * MAPP__types.h
 *
 *  Created on: Nov 15, 2016
 *      Author: mvstinger
 */

#ifndef MAPP__TYPES_H_
#define MAPP__TYPES_H_



namespace Mapping {




// Forward declarations
class MapSourceInterface;
class MapSourceBase;
class FileMapSource;

class MapInterface;
class MapBase;
class SimpleMap;

class GeodeticDatum;

struct MapExtents {
	double lat_min;
	double lat_max;
	double lon_min;
	double lon_max;

	MapExtents(void);
	MapExtents(double, double, double, double);
};

struct MapResolution {
	double lat_res;
	double lon_res;

	MapResolution(void);
	MapResolution(double, double);
};



class MapSourceInterface {
public:
	virtual ~MapSourceInterface(void) = 0;

	virtual int open(void) = 0;
	virtual int close(void) = 0;
	virtual int buffer(void) = 0;

protected:
	MapInterface* parent_map_;
};



class MapSourceBase : MapSourceInterface {};



class FileMapSource : MapSourceBase {};



class MapInterface {
public:
	virtual	~MapInterface(void) = 0;

protected:
	MapExtents map_extents_;
	MapResolution map_resolution_;
	MapSourceInterface* map_source_;
};






};




#endif /* MAPP__TYPES_H_ */
