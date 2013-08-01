/**
 * \file
 * \date 21.02.2013
 * \author Michał Orynicz
 */

#ifndef FEATURETRACKER_HPP_
#define FEATURETRACKER_HPP_

#include <vector>
#include <list>
#include <opencv2/core/core.hpp>

/// Base class for feature trackers
class FeatureTracker {
    public:
        FeatureTracker();
        /// Method creates a pointer to a copy of this object
        virtual FeatureTracker *constructCopy() const = 0;
        /// Method finds new positions of tracked features on new image frame

        /**
         * Method finds new positions of tracked features based on their past
         * positions and current and previous frames
         * @param oldInput - previous image frame
         * @param newInput - current image frame
         * @param trackedFeatures - container with tracked features
         */
        virtual void trackFeatures(const cv::Mat &oldInput,
                const cv::Mat &newInput,
                std::vector<std::list<cv::Point2f> > &trackedFeatures)=0;
        virtual ~FeatureTracker();
};

#endif /* FEATURETRACKER_HPP_ */
