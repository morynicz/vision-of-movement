/*
 * TangentRotationReader.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "TangentRotationReader.hpp"

TangentRotationReader::TangentRotationReader(const FeatureExtractor &extractor,
		const FeatureTracker &tracker,
		const std::list<FeatureFilter*> &filters) {
	_tracker = tracker.constructCopy();
	_trackedFeatures.clear();
	_extractor = extractor.constructCopy();
	for (std::list<FeatureFilter*>::const_iterator it = filters.begin();
			it != filters.end(); ++it) {
		_filters.push_back(*it);
	}
}

TangentRotationReader::~TangentRotationReader() {
	if (NULL != _tracker) {
		delete _tracker;
	}
	if (NULL != _extractor) {
		delete _extractor;
	}
	for (std::list<FeatureFilter*>::const_iterator it = _filters.begin();
			it != _filters.end(); ++it) {
		delete *it;
	}
}

TangentRotationReader::TangentRotationReader(
		const TangentRotationReader &toCopy) {
	_trackedFeatures = toCopy._trackedFeatures;
	_tracker = toCopy._tracker->constructCopy();
	_extractor = toCopy._extractor->constructCopy();
}

RotationReader *TangentRotationReader::constructCopy() const {
	return new TangentRotationReader(*this);
}

float TangentRotationReader::readRotation(const cv::Mat &newFrame) {
	float result = 0;

	return result;
}
