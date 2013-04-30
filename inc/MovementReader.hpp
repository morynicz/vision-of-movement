/*
 * MovementReader.hpp
 *
 *  Created on: Mar 16, 2013
 *      Author: link
 */

#ifndef MOVEMENTREADER_HPP_
#define MOVEMENTREADER_HPP_

#include <vector>
#include <list>

#include <opencv2/core/core.hpp>

#include "FeatureExtractor.hpp"
#include "FeatureTracker.hpp"
#include "FeatureFilter.hpp"

class MovementReader {
    protected:
        std::vector<std::list<cv::Point2f> > _trackedFeatures;
        FeatureTracker *_tracker;
        FeatureExtractor *_extractor;
        std::list<FeatureFilter*> _filters;
        unsigned int _maxFeatures;
        cv::Mat _oldFrame;
        MovementReader(const FeatureTracker &tracker,
                const FeatureExtractor &extractor,
                const std::list<FeatureFilter*> &filters,
                const unsigned int &maxFeatures,
                const std::vector<std::list<cv::Point2f> > &trackedFeatures =
                        std::vector<std::list<cv::Point2f> >());
        MovementReader(const MovementReader &toCopy);
    public:
        MovementReader();
        std::vector<std::list<cv::Point2f> > getTrackedFeatures() const;
        void refillFeatures(const int &maxFeatures);
        virtual ~MovementReader();
};

#endif /* MOVEMENTREADER_HPP_ */
