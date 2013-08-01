/**
 * \file
 * \date 16.03.2013
 * \author Michał Orynicz
 */

#include "MovementReader.hpp"
MovementReader::MovementReader(const FeatureTracker &tracker,
        const FeatureExtractor &extractor,
        const std::list<FeatureFilter*> &filters,
        const unsigned int &maxFeatures,
        const std::vector<std::list<cv::Point2f> > &trackedFeatures) :
        _trackedFeatures(trackedFeatures), _tracker(
                tracker.constructCopy()), _extractor(
                extractor.constructCopy()), _maxFeatures(maxFeatures) {
    for (std::list<FeatureFilter*>::const_iterator it =
            filters.begin(); it != filters.end(); ++it) {
        _filters.push_back((*it)->constructCopy());
    }
}

MovementReader::MovementReader(const MovementReader &toCopy) :
        _trackedFeatures(toCopy._trackedFeatures), _tracker(
                toCopy._tracker->constructCopy()), _extractor(
                toCopy._extractor->constructCopy()), _maxFeatures(
                toCopy._maxFeatures) {
    for (std::list<FeatureFilter*>::const_iterator it =
            toCopy._filters.begin(); it != toCopy._filters.end();
            ++it) {
        _filters.push_back((*it)->constructCopy());
    }
}

MovementReader::~MovementReader() {
    if (_tracker) {
        delete _tracker;
        _tracker = NULL;
    }
    if (_extractor) {
        delete _extractor;
        _extractor = NULL;
    }

    for (std::list<FeatureFilter*>::const_iterator it =
            _filters.begin(); it != _filters.end(); ++it) {
        delete *it;
    }
    _filters.clear();
}

std::vector<std::list<cv::Point2f> > MovementReader::getTrackedFeatures() const {
    return _trackedFeatures;
}
