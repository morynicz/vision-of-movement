/*
 * TangentRotationReader.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "TangentRotationReader.hpp"

#include "DrawingFunctions.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

TangentRotationReader::TangentRotationReader(
        const FeatureTracker &tracker,
        const FeatureExtractor &extractor,
        const std::list<FeatureFilter*> &filters,
        const unsigned int &maxFeatures, const double &focalLength,
        const std::vector<std::list<cv::Point2f> > &trackedFeatures) :
        RotationReader(tracker, extractor, filters, maxFeatures,
                trackedFeatures), _focalLength(focalLength) {
}

TangentRotationReader::~TangentRotationReader() {
}

TangentRotationReader::TangentRotationReader(
        const TangentRotationReader &toCopy) :
        RotationReader(toCopy), _focalLength(toCopy._focalLength) {
}

RotationReader *TangentRotationReader::constructCopy() const {
	return new TangentRotationReader(*this);
}

float TangentRotationReader::readRotation(const cv::Mat &newFrame) {
	float result = 0;

	return result;
}
