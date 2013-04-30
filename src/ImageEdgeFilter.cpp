/*
 * ImageEdgeFilter.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "ImageEdgeFilter.hpp"
#include <iostream>

ImageEdgeFilter::ImageEdgeFilter(const cv::Mat &transformMatrix,
        const cv::Size &imageSize, const double &margin) :
        _margin(margin) {
    std::vector<cv::Point2f> tempCorners(5), corners;
    _a.resize(4);
    _c.resize(4);

    tempCorners[0] = (cv::Point2f(0, 0));
    tempCorners[1] = (cv::Point2f(imageSize.width, 0));
    tempCorners[2] = (cv::Point2f(0, imageSize.height));
    tempCorners[3] = (cv::Point2f(imageSize.width, imageSize.height));
    tempCorners[4] = tempCorners[0];
    cv::perspectiveTransform(tempCorners, corners, transformMatrix);
    for (int i = 0; i < 4; ++i) {
        cv::Point2f tmp = corners[i + 1] - corners[i];
        if (0 != tmp.x) {
            _a[i] = (corners[i + 1].y - corners[i].y)
                    / (corners[i + 1].x - corners[i].x);
            _c[i] = (corners[i].y * corners[i+1].x
                    - corners[i+1].y * corners[i].x)
                    / (corners[i+1].x - corners[i].x);
            std::cerr << "corn" << corners[i] << " " << corners[i + 1]
                    << std::endl;
            std::cerr << _a[i] << " " << _c[i] << std::endl;
            std::cerr << (_a[i] * corners[i + 1].x + _c[i]) - corners[i + 1].y
                    << std::endl;
            std::cerr << (_a[i] * corners[i + 1].x + _c[i]) << " "
                    << corners[i + 1].y << std::endl;
            std::cerr << (_a[i] * corners[i].x + _c[i]) - corners[i].y
                    << std::endl;

        } else {
            cv::Exception ex(0, "napraw to do cholery, linia pionowa!",
                    __func__, __FILE__, __LINE__);
            throw ex;
        }
    }
}

ImageEdgeFilter::ImageEdgeFilter(const ImageEdgeFilter & toCopy) :
        _a(toCopy._a), _c(toCopy._c), _margin(toCopy._margin) {
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
                    _a[j] * result.at(i).front().x - result.at(i).front().y
                            + _c[j]) / sqrt(_a[j] * _a[j] + 1);
        //    std::cerr<<"D"<<d<<std::endl;
            if (_margin > d) {
                //std::cerr << "killed" << result.at(i).front() << std::endl;
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
