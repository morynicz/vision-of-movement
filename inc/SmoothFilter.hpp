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
        double _maxDeviation;
        double _deviantThreshold;
        double _minLength;
    public:
        SmoothFilter(const double &maxDeviation, const double &maxDeviants,
                const double &minLength);
        SmoothFilter(const SmoothFilter &toCopy);
        virtual ~SmoothFilter();
        virtual FeatureFilter *constructCopy() const;
        virtual std::vector<std::list<cv::Point2f> > filterFeatures(
                const std::vector<std::list<cv::Point2f> > &features);
};

#endif /* SMOOTHFILTER_H_ */
