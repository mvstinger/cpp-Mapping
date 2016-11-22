/*
 * MAPP__helpers.cc
 *
 *  Created on: Nov 19, 2016
 *      Author: mvstinger
 */



#include <cstdlib>
#include "../MAPP__helpers.h"



namespace Mapping {



double rand_btwn(const double low, const double high) {
	return (high - low) * (double)std::rand() / RAND_MAX + low;
}

double randu(void) {
	return (double)std::rand() / RAND_MAX;
};



}
