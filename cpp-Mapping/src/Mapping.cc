/*
 * Mapping.cc
 *
 *  Created on: Nov 19, 2016
 *      Author: mvstinger
 */



#include "../Mapping.h"
#include "Logger.h"



namespace Mapping {



//	Setup logger
Logger::NullLogger logger_proxy = Logger::NullLogger();
Logger::LoggerInterface* logger = &logger_proxy;



}
