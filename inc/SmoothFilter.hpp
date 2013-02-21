/*
 * SmoothFilter.h
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef SMOOTHFILTER_HPP_
#define SMOOTHFILTER_HPP_

#include "FeatureFilter.hpp"

class SmoothFilter: public virtual FeatureFilter {
public:
	SmoothFilter();
	virtual ~SmoothFilter();
};

#endif /* SMOOTHFILTER_H_ */
