/*
 * MAPP_types.cc
 *
 *  Created on: Nov 15, 2016
 *      Author: mvstinger
 */



#include <string>
#include <Eigen/Eigen>
#include "../MAPP__types.h"



namespace Mapping {



using Eigen::MatrixXd;
using Eigen::Matrix2d;
using std::sqrt;
using std::to_string;



MapExtents::MapExtents(void) :
		lat_min(-90.0),
		lat_max(+90.0),
		lon_min(-180.0),
		lon_max(+180.0) {};

MapExtents::MapExtents(const double lat_min, const double lat_max, const double lon_min, const double lon_max) :
		lat_min(lat_min),
		lat_max(lat_max),
		lon_min(lon_min),
		lon_max(lon_max) {};



MapResolution::MapResolution(void) :
		lat_res(1.0),
		lon_res(1.0) {};

MapResolution::MapResolution(const double lat_res, const double lon_res) :
		lat_res(lat_res),
		lon_res(lon_res) {};



MapInterface::MapInterface(void) :
		map_extents_(MapExtents()),
		map_resolution_(MapResolution()),
		datum_(&WGS84) {};

MapInterface::MapInterface(const MapExtents me, const MapResolution res) :
		map_extents_(me),
		map_resolution_(res),
		datum_(&WGS84) {};

MapInterface::MapInterface(const GeodeticDatumInterface* gd, const MapExtents me, const MapResolution res) :
	map_extents_(me),
	map_resolution_(res),
	datum_(gd) {};

MapInterface::~MapInterface() {};



MapBase::MapBase(void) :
		MapInterface(MapExtents(), MapResolution()) {};

MapBase::MapBase(const MapExtents me, const MapResolution res) :
		MapInterface(me, res) {};

MapBase::MapBase(const GeodeticDatumInterface* gd, const MapExtents me, const MapResolution res) :
		MapInterface(gd, me, res) {};

MapBase::~MapBase() {};

SimpleMap MapBase::from_random(const int lat_count, const int lon_count, const double alt_span) {
	//TODO: Improve terrain generation (eg. harmonic-based)
	SimpleMap output_map = SimpleMap();
  output_map.map_data_ = MatrixXd::Random(lat_count, lon_count) * alt_span;
  MapExtents aux_me(-90.0, +90.0, -180.0, +180.0);
  output_map.update_lat_reso_();
  output_map.update_lon_reso_();
//  logger->info(std::string("Created random map of size ") + std::to_string(lat_count) + std::string(", ") + std::to_string(lon_count));
  return output_map;
};

uint MapBase::write_csv(std::FILE* fid) const {
  int error_state=0;
  for(Idx lat_idx=0; lat_idx < (this->get_lat_count()); lat_idx++) {
    for(Idx lon_idx=0; lon_idx < (this->get_lon_count()); lon_idx++) {
//      fprintf(fid, "%6.2f", this->map_data_(lat_idx, lon_idx));
//      fprintf(stdout, "%6.2f", this->map_data_(lat_idx, lon_idx));
//    	fprintf(stdout, "%i, %i\n", this->map_data_.rows(), this->map_data_.cols());
      (lon_idx==(this->get_lon_count()-1)) ? fprintf(fid, "%c", '\n') : fprintf(fid, "%c", ',');
    }
  }
  return error_state;
}

double MapBase::get_lat_min(void) const { return this->map_extents_.lat_min; }

double MapBase::get_lat_max(void) const { return this->map_extents_.lat_max; }

double MapBase::get_lat_span(void) const { return this->map_extents_.lat_max - this->map_extents_.lat_min; }

uint MapBase::get_lat_count(void) const { return ceil(this->get_lat_span() / this->map_resolution_.lat_res); }

double MapBase::get_lon_min(void) const { return this->map_extents_.lon_min; }

double MapBase::get_lon_max(void) const { return this->map_extents_.lon_max; }

double MapBase::get_lon_span(void) const { return this->map_extents_.lon_max - this->map_extents_.lon_min; }

uint MapBase::get_lon_count(void) const { return ceil(this->get_lon_span() / this->map_resolution_.lon_res); }



MappingError MapBase::set_lat_min(const float input_lat) {
	MappingError error_state = NO_ERROR;
  if((input_lat < -90.0) || (input_lat > 90.0)) { error_state = WARNING_LAT_OOB; }
  this->map_extents_.lat_min = input_lat;
  this->update_lat_reso_();
  return error_state;
}

MappingError MapBase::set_lat_max(const float input_lat) {
	MappingError error_state = NO_ERROR;
  if((input_lat < -90.0) || (input_lat > 90.0)) { error_state = WARNING_LAT_OOB; }
  this->map_extents_.lat_max = input_lat;
  this->update_lat_reso_();
  return error_state;
}

//MappingError MapBase::set_lat_count(const float input_count) {
//  MappingError error_state = NO_ERROR;
//  if(input_count<=0) { error_state = ERROR_COUNT_OOB; }
//  this->map_extents_.lat_count = input_count;
//  this->update_lat_reso_();
//  this->update_map_size_();
//  return error_state;
//}

MappingError MapBase::set_lon_min(const float input_lon) {
	MappingError error_state = NO_ERROR;
  if((input_lon < -180) || (input_lon > 90.0)) { error_state = WARNING_LON_OOB; }
  this->map_extents_.lon_min = input_lon;
  this->update_lon_reso_();
  return error_state;
}

MappingError MapBase::set_lon_max(const float input_lon) {
	MappingError error_state = NO_ERROR;
  if((input_lon < -180.0) || (input_lon > 180.0)) { error_state = WARNING_LON_OOB; }
  this->map_extents_.lon_max = input_lon;
  this->update_lon_reso_();
  return error_state;
}

//MappingError MapBase::set_lon_count(const float input_count) {
//  int error_state = NO_ERROR;
//  if(input_count <= 0) { error_state = ERROR_COUNT_OOB; }
//  this->map_extents_.lon_count = input_count;
//  this->update_lon_reso_();
//  this->update_map_size_();
//  return error_state;
//}

double MapBase::alt_at(const double input_lat, const double input_lon) const {
  //	1. Calculate geometric median
	//	1.1 Seed the algorithm with the average altitude
  double lat_pos, lat_neg, lon_pos, lon_neg;
  int lat_pos_idx, lat_neg_idx, lon_pos_idx, lon_neg_idx;

  //	1.1.1 Adjacent latitudes
  lat_pos_idx = ceil( (input_lat - this->map_extents_.lat_min) / this->map_resolution_.lat_res );
  lat_neg_idx = (lat_pos_idx==0) ? lat_pos_idx : lat_pos_idx - 1;
  lat_pos = this->map_extents_.lat_min + lat_pos_idx * this->map_resolution_.lat_res;
  lat_neg = this->map_extents_.lat_min + lat_neg_idx * this->map_resolution_.lat_res;

  // 	1.1.2 Adjacent longitudes
  lon_pos_idx = ceil((input_lon - this->map_extents_.lon_min) / this->map_resolution_.lon_res);
  lon_neg_idx = (lon_pos_idx==0) ? lon_pos_idx : lon_pos_idx - 1;
  lon_pos = this->map_extents_.lon_min + lon_pos_idx * this->map_resolution_.lon_res;
  lon_neg = this->map_extents_.lon_min + lon_neg_idx * this->map_resolution_.lon_res;

  //	1.1.3 Calculate seed altitude
  Matrix2d adjacent_block = this->map_data_.block(lat_neg_idx,lon_neg_idx, 2,2);
  double output_alt = adjacent_block.mean();

  //	1.2 Set algorithm parameters

  //	1.3 Iteratively calculate geometric median
  //	1.3.1 Predimension variables
  double alt_step = 0.25 * (adjacent_block.maxCoeff() - adjacent_block.minCoeff());
  double no_step_dist, pos_step_dist, neg_step_dist;
  //	1.3.2 Calculate fixed lat, lon differences
  double pos_lon_diff_2 = (lon_pos-input_lon)*(lon_pos-input_lon);
  double pos_lat_diff_2 = (lat_pos-input_lat)*(lat_pos-input_lat);
  double neg_lon_diff_2 = (lon_neg-input_lon)*(lon_neg-input_lon);
  double neg_lat_diff_2 = (lat_neg-input_lat)*(lat_neg-input_lat);
  //	1.3.2 Loop/step
  for(Idx iter_idx=0; iter_idx<this->CALC_ALT_MAX_ITER_; iter_idx++) {
  	//	1.3.2.1 Calculate current altitude estimate distance
  	no_step_dist =
  			sqrt( pos_lon_diff_2 + pos_lat_diff_2 + (output_alt - (double)adjacent_block(0, 1))*(output_alt - (double)adjacent_block(0, 1)) ) +
  			sqrt( pos_lon_diff_2 + neg_lat_diff_2 + (output_alt - (double)adjacent_block(1, 1))*(output_alt - (double)adjacent_block(1, 1)) ) +
  			sqrt( neg_lon_diff_2 + neg_lat_diff_2 + (output_alt - (double)adjacent_block(1, 0))*(output_alt - (double)adjacent_block(1, 0)) ) +
				sqrt( neg_lon_diff_2 + pos_lat_diff_2 + (output_alt - (double)adjacent_block(0, 0))*(output_alt - (double)adjacent_block(0, 0)) );

  	//	1.3.2.2 Calculate positive altitude step distance
  	pos_step_dist =
  			sqrt( pos_lon_diff_2 + pos_lat_diff_2 + (output_alt+alt_step - (double)adjacent_block(0, 1))*(output_alt+alt_step - (double)adjacent_block(0, 1)) ) +
				sqrt( pos_lon_diff_2 + neg_lat_diff_2 + (output_alt+alt_step - (double)adjacent_block(1, 1))*(output_alt+alt_step - (double)adjacent_block(1, 1)) ) +
				sqrt( neg_lon_diff_2 + neg_lat_diff_2 + (output_alt+alt_step - (double)adjacent_block(1, 0))*(output_alt+alt_step - (double)adjacent_block(1, 0)) ) +
				sqrt( neg_lon_diff_2 + pos_lat_diff_2 + (output_alt+alt_step - (double)adjacent_block(0, 0))*(output_alt+alt_step - (double)adjacent_block(0, 0)) );

  	//	1.3.2.3 Calculate negative altitude step distance
  	neg_step_dist =
  			sqrt( pos_lon_diff_2 + pos_lat_diff_2 + (output_alt-alt_step - (double)adjacent_block(0, 1))*(output_alt-alt_step - (double)adjacent_block(0, 1)) ) +
				sqrt( pos_lon_diff_2 + neg_lat_diff_2 + (output_alt-alt_step - (double)adjacent_block(1, 1))*(output_alt-alt_step - (double)adjacent_block(1, 1)) ) +
				sqrt( neg_lon_diff_2 + neg_lat_diff_2 + (output_alt-alt_step - (double)adjacent_block(1, 0))*(output_alt-alt_step - (double)adjacent_block(1, 0)) ) +
				sqrt( neg_lon_diff_2 + pos_lat_diff_2 + (output_alt-alt_step - (double)adjacent_block(0, 0))*(output_alt-alt_step - (double)adjacent_block(0, 0)) );

  	//	1.3.2.4 Calculate new step size
  	//	NOTE: Only one or neither of positive and negative altitude steps will
  	//				result in a shorter summed-distances.
  	if(pos_step_dist < no_step_dist) {
  		//	Positive altitude step resulted in shorter summed-distances
  		//		-> Update altitude estimate
  		//alt_step = alt_step;
  		output_alt += alt_step;
  	} else if(neg_step_dist < no_step_dist) {
  		//	Negative altitude step resulted in shorter summed-distances
  		//		-> Update altitude estimate
  		//alt_step = alt_step;
  		output_alt -= alt_step;
  	} else {
  		//	Neither postive or negative steps resulted in a shorter summed-distances
  		//		-> Reduce step size, and recalculate
  		alt_step /= 2;
  		//output_alt = output_alt;
  	}

  	//	1.3.3 Exit loop if desired resolution has been achieved
  	if(alt_step < this->CALC_ALT_RES_) { break; }
  }

  //  Return value
  return output_alt;
}

double MapBase::radius_at(const double input_lat, const double input_lon) const {
	return this->datum_->radius_at(input_lat, input_lon);
}

double MapBase::height_at(const double input_lat, const double input_lon) const {
	return this->datum_->radius_at(input_lat, input_lon) +
			this->alt_at(input_lat, input_lon);
}

uint MapBase::update_lat_reso_(void) {
  uint error_state=0;
  this->map_resolution_.lat_res = (this->map_extents_.lat_max - this->map_extents_.lat_min) / this->get_lat_count();
  return error_state;
};

uint MapBase::update_lon_reso_(void) {
  uint error_state=0;
  this->map_resolution_.lon_res = (this->map_extents_.lon_max - this->map_extents_.lon_min) / this->get_lon_count();
  return error_state;
};

uint MapBase::update_map_size_(void) {
  uint error_state = 0;
  this->map_data_.resize((long long int)this->get_lat_count(), (long long int)this->get_lon_count());
  return error_state;
}

uint MapBase::update_all_(void) {
  uint error_state=0;
  error_state |= this->update_map_size_();
  error_state |= this->update_lat_reso_();
  error_state |= this->update_lon_reso_();
  return error_state;
}



SimpleMap::SimpleMap() {};

SimpleMap::SimpleMap(const MapExtents me, const MapResolution res) : MapBase(&WGS84, me, res) {};

SimpleMap::SimpleMap(const GeodeticDatumInterface* emi, const MapExtents me, const MapResolution res) : MapBase(emi, me, res) {};

SimpleMap::~SimpleMap() {};



}
