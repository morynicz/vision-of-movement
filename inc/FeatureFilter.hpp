/**
 * \file
 * \date 21.02.2013
 * \author Michał Orynicz
 */

#ifndef FEATUREFILTER_HPP_
#define FEATUREFILTER_HPP_

#include <vector>
#include <list>
#include <opencv2/core/core.hpp>

/// Base class for feature filters
class FeatureFilter {
    public:
        FeatureFilter();
        /// Method creates a pointer to a copy of this object
        virtual FeatureFilter *constructCopy() const = 0;
        /// Method removes features that do not meet certain criteria

        /**
         * Method removes features that do not meet set criteria
         * @param[in] features - features to be filtered
         * @return container with only those features, which meet
         * the criteria
         */
        virtual std::vector<std::list<cv::Point2f> > filterFeatures(
                const std::vector<std::list<cv::Point2f> > &features)=0;
        virtual ~FeatureFilter();
};

#endif /* FEATUREFILTER_HPP_ */
