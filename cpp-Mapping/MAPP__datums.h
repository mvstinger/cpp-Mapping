/*
 * MAPP__datums.h
 *
 *  Created on: Nov 16, 2016
 *      Author: mvstinger
 */

#ifndef MAPP__DATUMS_H_
#define MAPP__DATUMS_H_



namespace Mapping {



class GeodeticDatumInterface {
public:
	virtual ~GeodeticDatumInterface(void) = 0;

	virtual double radius_at(const double, const double) const = 0;
};



class GeodeticDatumBase : public GeodeticDatumInterface {
public:
	virtual ~GeodeticDatumBase(void) = 0;

	virtual double radius_at(const double, const double) const = 0;
};



class EllipsoidDatum : public GeodeticDatumBase {
public:
	EllipsoidDatum(const double, const double);
	~EllipsoidDatum(void);

	double radius_at(const double, const double) const;


private:
	double a_;
	double b_;
};



const EllipsoidDatum WGS84 = EllipsoidDatum(6378137.0, 6356752.314245);



};



#endif /* MAPP__DATUMS_H_ */
