/*
 * VisualOdometer.h
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef VISUALODOMETER_H_
#define VISUALODOMETER_H_

#include "TranslationReader.hpp"
#include "RotationReader.hpp"
#include "FeatureFilter.hpp"
#include "SmoothFilter.hpp"
#include "ImageEdgeFilter.hpp"
#include "FeatureTracker.hpp"
#include "FeatureExtractor.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <list>
#include <cmath>

class VisualOdometer {
        TranslationReader *_translationReader;
        RotationReader *_rotationReader;
        int _horizonHeight;
        int _deadZoneWidth;
    public:
        VisualOdometer(const RotationReader &rotationReader,
                const TranslationReader &translationReader,
                const int &horizonHeight, const int &deadZoneWidth);
        VisualOdometer(const VisualOdometer &toCopy);
        cv::Point3f calculateDisplacement(const cv::Mat &newFrame);
        std::vector<std::list<cv::Point2f> > getRotationFeatures() const;
        std::vector<std::list<cv::Point2f> > getTranslationFeatures() const;
        virtual ~VisualOdometer();
        VisualOdometer &operator=(
                const VisualOdometer &toCopy);
};

#endif /* VISUALODOMETER_H_ */
