/**
 * \file
 * \file 21.02.2013
 * \author Michał Orynicz
 */

#include "RotationReader.hpp"

RotationReader::RotationReader(const FeatureTracker &tracker,
        const FeatureExtractor &extractor,
        const std::list<FeatureFilter*> &filters,
        const unsigned int &maxFeatures,
        const std::vector<std::list<cv::Point2f> > &trackedFeatures) :
        MovementReader(tracker, extractor, filters, maxFeatures,
                trackedFeatures) {
}

RotationReader::RotationReader(const RotationReader &toCopy) :
        MovementReader(toCopy) {
}

RotationReader::~RotationReader() {
}

