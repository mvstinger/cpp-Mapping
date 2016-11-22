/*
 * Mapping.h
 *
 *  Created on: Nov 15, 2016
 *      Author: mvstinger
 */

#ifndef MAPPING_H_
#define MAPPING_H_



#include "Logger.h"
#include "MAPP__datums.h"
#include "MAPP__types.h"
#include "MAPP__helpers.h"
#include "MAPP__terrain_gen.h"




namespace Mapping {



//	Assigned in Mapping.cc-
//TODO: Will this avoid creation of new instances of 'logger' for each #include of 'Mapping.h' file?
//TODO: Will this extend scope of variable beyond Mapping?
//extern Logger::LoggerInterface* logger;



};




#endif /* MAPPING_H_ */
