/*
 * MAPP_datums.cc
 *
 *  Created on: Nov 16, 2016
 *      Author: mvstinger
 */



#include <cmath>
#include "../MAPP__datums.h"



namespace Mapping {



GeodeticDatumInterface::~GeodeticDatumInterface(void) {};



GeodeticDatumBase::~GeodeticDatumBase(void) {};



EllipsoidDatum::EllipsoidDatum(const double a, const double b) :
	a_(a),
	b_(b) {};

EllipsoidDatum::~EllipsoidDatum(void) {};

double EllipsoidDatum::radius_at(const double lat, const double lon) const {
	return sqrt( ( (a_*a_*cos(lat)) * (a_*a_*cos(lat)) + (b_*b_*sin(lat)) * (b_*b_*sin(lat)) ) / ( (a_*cos(lat)) * (a_*cos(lat)) + (b_*sin(lat)) * (b_*sin(lat)) ) );
};








};
