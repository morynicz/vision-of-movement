/*
 * TangentRotationReader.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "TangentRotationReader.hpp"

TangentRotationReader::TangentRotationReader(const FeatureExtractor &extractor,
		const FeatureTracker &tracker) {
	_tracker = tracker.constructCopy();
	_trackedFeatures.clear();
	_extractor=extractor.constructCopy();
}

TangentRotationReader::~TangentRotationReader() {
	if (NULL != _tracker) {
		delete _tracker;
	}
	if (NULL != _extractor) {
		delete _extractor;
	}
}

TangentRotationReader::TangentRotationReader(
		const TangentRotationReader &toCopy) {
	_tracker = toCopy._tracker;
	_trackedFeatures = toCopy._trackedFeatures;
	_extractor = toCopy._extractor;
}

RotationReader *TangentRotationReader::constructCopy() const {
	return new TangentRotationReader(*this);
}

float TangentRotationReader::readRotation(const cv::Mat &newFrame) {
	float result = 0;

	return result;
}
