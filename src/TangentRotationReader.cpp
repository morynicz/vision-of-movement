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
    if (!_oldFrame.empty()) {
        _extractor->refillFeatures(_oldFrame, _trackedFeatures,
                _maxFeatures);
        _tracker->trackFeatures(_oldFrame, newFrame,
                _trackedFeatures);
        if (!_trackedFeatures.empty()) {
            for (std::list<FeatureFilter*>::iterator it =
                    _filters.begin(); it != _filters.end(); ++it) {
                _trackedFeatures = (*it)->filterFeatures(
                        _trackedFeatures);
            }
//            cv::Mat shown = newFrame.clone();
//            drawFeatureHistory(shown, _trackedFeatures, 5);
//            cv::imshow("tangent", shown);
//            cv::waitKey(1);
            std::vector<float> rotations = computeRotationVectors();
            int vectorHalf = rotations.size() / 2;
            std::nth_element(rotations.begin(),
                    rotations.begin() + vectorHalf, rotations.end());
            result = rotations[vectorHalf];
        }
    }
    _oldFrame = newFrame.clone();
    return result;
}

std::vector<float> TangentRotationReader::computeRotationVectors() {
    std::vector<cv::Point2f> oldFeatures(_trackedFeatures.size()),
            newFeatures(_trackedFeatures.size());
    std::vector<float> result(oldFeatures.size());

    for (unsigned int i = 0; i < result.size(); ++i) {
        std::list<cv::Point2f>::const_iterator it =
                _trackedFeatures[i].begin();
        if (_trackedFeatures[i].size() > 2) {
            result[i] = atan2(it->x - _oldFrame.cols / 2,
                    _focalLength);
            ++it;
            result[i] -= atan2(it->x - _oldFrame.cols / 2, _focalLength);
       //     std::cerr << result[i] << std::endl;
        }
    }
    return result;
}
