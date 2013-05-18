/*
 * ImageEdgeFilter.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "ImageEdgeFilter.hpp"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
ImageEdgeFilter::ImageEdgeFilter(const cv::Mat &transformMatrix,
        const cv::Size &imageSize, const double &margin) :
        _margin(margin) {
    std::vector<cv::Point2f> tempCorners(5), corners;
    _a.resize(4);
    _b.resize(4);
    _c.resize(4);

    tempCorners[0] = (cv::Point2f(0, 0));
    tempCorners[1] = (cv::Point2f(imageSize.width, 0));
    tempCorners[2] = (cv::Point2f(imageSize.width, imageSize.height));
    tempCorners[3] = (cv::Point2f(0, imageSize.height));
    tempCorners[4] = tempCorners[0];
    cv::perspectiveTransform(tempCorners, corners,
            transformMatrix.inv());

    for (int i = 0; i < 4; ++i) {
        cv::Point2f tmp = corners[i + 1] - corners[i];
        if (0 != tmp.x) {
            _a[i] = (corners[i + 1].y - corners[i].y)
                    / (corners[i + 1].x - corners[i].x);
            _b[i] = -1;
            _c[i] = (corners[i].y * corners[i + 1].x
                    - corners[i + 1].y * corners[i].x)
                    / (corners[i + 1].x - corners[i].x);

        } else {
            _b[i] = 0;
            _a[i] = -1;
            _c[i] = corners[i].x;
        }
    }
}

ImageEdgeFilter::ImageEdgeFilter(const ImageEdgeFilter & toCopy) :
        _a(toCopy._a), _b(toCopy._b), _c(toCopy._c), _margin(
                toCopy._margin) {
}

ImageEdgeFilter::~ImageEdgeFilter() {
    // TODO Auto-generated destructor stub
}

std::vector<std::list<cv::Point2f> > ImageEdgeFilter::filterFeatures(
        const std::vector<std::list<cv::Point2f> >& features) {
    std::vector<std::list<cv::Point2f> > result(features);
    for (unsigned int i = 0; i < result.size(); ++i) {
        for (unsigned int j = 0; j < _a.size(); ++j) {
            double d = abs(
                    _a[j] * result.at(i).front().x
                            + _b[j] * result.at(i).front().y + _c[j])
                    / sqrt(_a[j] * _a[j] + _b[j] * _b[j]);
            if (_margin > d) {
                result.erase(result.begin() + i);
                --i;
                break;
            }
        }
    }
    return result;
}

FeatureFilter *ImageEdgeFilter::constructCopy(void) const {
    FeatureFilter *result = new ImageEdgeFilter(*this);
    return result;
}
