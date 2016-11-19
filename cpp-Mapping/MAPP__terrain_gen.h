/*
 * MAPP__terrain_gen.h
 *
 *  Created on: Nov 19, 2016
 *      Author: mvstinger
 */

#ifndef MAPP__TERRAIN_GEN_H_
#define MAPP__TERRAIN_GEN_H_



#include <Eigen/Eigen>
#include "MAPP__types.h"



using Eigen::MatrixXd;



namespace Mapping {



struct SDConfiguration {
	double roughness;
	double macro_slope;
	double micro_slope;

	SDConfiguration(void);
	SDConfiguration(const double, const double, const double);
};



MappingError terrain_gen_SD(MatrixXd*, const double, const SDConfiguration);



void ecks(MatrixXd*, const int, const int, const int, const double);

void plus(MatrixXd*, const int, const int, const int, const double);



}



#endif /* MAPP__TERRAIN_GEN_H_ */
