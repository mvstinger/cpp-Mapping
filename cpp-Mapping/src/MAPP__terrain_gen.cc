/*
 * MAPP__terrain_gen.cc
 *
 *  Created on: Nov 19, 2016
 *      Author: mvstinger
 */



#include <cstdlib>
#include <Eigen/Eigen>
#include "../MAPP__types.h"
#include "../MAPP__helpers.h"
#include "../MAPP__terrain_gen.h"



using std::ceil;
using std::log2;
using std::max;
using std::pow;
using Eigen::MatrixXd;
using Eigen::Ref;



namespace Mapping {



SDConfiguration::SDConfiguration(void) :
		roughness(1.0),
		macro_slope(1.00),
		micro_slope(0.01) {};

SDConfiguration::SDConfiguration(const double roughness, const double macro_m, const double micro_m) :
		roughness(roughness),
		macro_slope(macro_m),
		micro_slope(micro_m) {};



MappingError terrain_gen_SD(Ref<MatrixXd> output_map, const double alt_span, const SDConfiguration config) {
	fprintf(stdout, "[ terrain_gen_SD()     ]\toutput_map ptr: 0x%08X\n", output_map);
	MappingError error_state = NO_ERROR;
	//	Create empty map to work with
	//		Round up size to be of square size 2^n + 1
	int output_map_rows = output_map->rows();
	int output_map_cols = output_map->cols();
	fprintf(stdout, "[ terrain_gen_SD()     ]\toutput_map size: (%6.3f x %6.3f)\n", output_map->rows(), output_map->cols());
	int recursions = ceil(log2(max(output_map_rows, output_map_cols)));
	long long int map_size = pow(2, recursions) + 1;
	fprintf(stdout, "[ terrain_gen_SD()     ]\tmap_size: %6.3f\n", map_size);
	//TODO: Break larger space into smaller pieces for more efficiency?
	//	Set four corners
	(*output_map)(0,						0)					= rand_btwn(-alt_span/2, +alt_span/2);
	(*output_map)(0,						map_size-1) = rand_btwn(-alt_span/2, +alt_span/2);
	(*output_map)(map_size-1,	0)					= rand_btwn(-alt_span/2, +alt_span/2);
	(*output_map)(map_size-1,	map_size-1) = rand_btwn(-alt_span/2, +alt_span/2);
	//	Calculate offset multiplier
	double offset_multiplier = pow(config.micro_slope / config.macro_slope, 1.0/recursions);
	//	Prepare for looping
	double offset = config.macro_slope;
	int span = map_size - 1;
	int half_span = 0;
	//	Start looping over blocks
	while(span > 2) {
		half_span = ceil(span/2);
		//  logger->info(std::string("from_random(): Creating random map of size ") + std::to_string(lat_count) + std::string(", ") + std::to_string(lon_count));
		fprintf(stdout, "[ terrain_gen_SD()     ]\tspan: %6.3f\thalf_span: %6.3f\n", span, half_span);
		//	Step over square/"ecks" (x) blocks
		for(Idx row_idx = half_span; row_idx < map_size; row_idx += half_span ) {
			for(Idx col_idx = half_span; col_idx < map_size; col_idx += half_span ) {
				ecks(output_map, row_idx, col_idx, span, offset);
			}
		}
		//	Step over diamond/"plus" (+) blocks : part 1
		for(Idx row_idx = 0; row_idx < map_size; row_idx += span ) {
			for(Idx col_idx = half_span; col_idx < map_size; col_idx += span ) {
				plus(output_map, row_idx, col_idx, span, offset);
			}
		}
		//	Step over diamond/"plus" (+) blocks : part 2
		for(Idx row_idx = half_span; row_idx < map_size; row_idx += span ) {
			for(Idx col_idx = 0; col_idx < map_size; col_idx += span ) {
				plus(output_map, row_idx, col_idx, span, offset);
			}
		}
		//	~Halve span
		span = ceil(span/2);
		//	Calculate new offset
		offset *= offset_multiplier;
	}
	return error_state;
};



void ecks(MatrixXd* map, const int row, const int col, const int span, const double offset) {
	double ul, ur, bl, br;
	ul = (*map)(row - span/2, col - span/2);
	ur = (*map)(row - span/2, col + span/2);
	bl = (*map)(row + span/2, col - span/2);
	br = (*map)(row + span/2, col - span/2);
	(*map)(row, col) = (ul + ur + bl + br) / 4 + offset;
}

void plus(MatrixXd* map, const int row, const int col, const int span, const double offset) {
	int divisor = 4;
	double top, right, bottom, left;
	if( row==0 ) {
		divisor = 3;
		top = 0;
	} else {
		top = (*map)(row - span/2, col);
	}	//	END TOP
	if(row==(map->rows())) {
		divisor = 3;
		bottom = 0;
	} else {
		bottom = (*map)(row + span/2, col);
	}	//	END BOTTOM
	if( col==0 ) {
		divisor = 3;
		right = 0;
	} else {
		right = (*map)(row, col + span/2);
	}	//	END RIGHT
	if(col==(map->cols())) {
		divisor = 3;
		left = 0;
	} else {
		left = (*map)(row, col - span/2);
	}	//	END LEFT
	(*map)(row, col) = (top + right + bottom + left) / divisor + offset;
}

}
