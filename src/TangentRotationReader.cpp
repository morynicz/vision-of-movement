/*
 * TangentRotationReader.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "TangentRotationReader.hpp"
#include "ImageEdgeFilter.hpp"

TangentRotationReader::TangentRotationReader(
        const FeatureTracker &tracker,
        const FeatureExtractor &extractor,
        const std::list<FeatureFilter*> &filters,
        const unsigned int &maxFeatures, const double &focalLength,
        const cv::Size& imageSize, const double &margin,
        const std::vector<std::list<cv::Point2f> > &trackedFeatures) :
        RotationReader(tracker, extractor, filters, maxFeatures,
                trackedFeatures), _focalLength(focalLength), _imageSize(
                imageSize) {
    _filters.push_front(
            new ImageEdgeFilter(cv::Mat::eye(cv::Size(3, 3), CV_32F),
                    _imageSize, margin));
}

TangentRotationReader::~TangentRotationReader() {
}

TangentRotationReader::TangentRotationReader(
        const TangentRotationReader &toCopy) :
        RotationReader(toCopy), _focalLength(toCopy._focalLength), _imageSize(
                toCopy._imageSize) {
}

RotationReader *TangentRotationReader::constructCopy() const {
    return new TangentRotationReader(*this);
}

float TangentRotationReader::readRotation(const cv::Mat &newFrame) {
    float result = 0;
    if (!_oldFrame.empty()) {
        _extractor->refillFeatures(_oldFrame, _trackedFeatures,
                _maxFeatures);
        if (!_trackedFeatures.empty()) {
            _tracker->trackFeatures(_oldFrame, newFrame,
                    _trackedFeatures);
            if (!_trackedFeatures.empty()) {
                for (std::list<FeatureFilter*>::iterator it =
                        _filters.begin(); it != _filters.end();
                        ++it) {
                    _trackedFeatures = (*it)->filterFeatures(
                            _trackedFeatures);
                }

                if (!_trackedFeatures.empty()) {
                    std::vector<float> rotations =
                            computeRotationVectors();
                    int vectorHalf = rotations.size() / 2;
                    std::nth_element(rotations.begin(),
                            rotations.begin() + vectorHalf,
                            rotations.end());
                    result = rotations[vectorHalf];
                }
            }
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
            result[i] -= atan2(it->x - _oldFrame.cols / 2,
                    _focalLength);
        }
    }
    return result;
}
