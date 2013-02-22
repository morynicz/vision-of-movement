/*
 * TangentRotationReader.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "TangentRotationReader.hpp"

TangentRotationReader::TangentRotationReader(const FeatureTracker &tracker) {
	_tracker = tracker.constructCopy();
	_trackedFeatures.clear();

}

TangentRotationReader::~TangentRotationReader() {
	if(NULL!=_tracker){
		delete _tracker;
	}
}

TangentRotationReader::TangentRotationReader(const TangentRotationReader &toCopy){
	_tracker = toCopy._tracker;
	_trackedFeatures = toCopy._trackedFeatures;
}

RotationReader *TangentRotationReader::constructCopy() const{
	return new TangentRotationReader(*this);
}

float TangentRotationReader::readRotation(const cv::Mat &newFrame){
	float result=0;

	return result;
}
