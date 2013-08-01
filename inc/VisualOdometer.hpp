/**
 * \file
 * \date 21.02.2013
 * \author Michał Orynicz
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

/**
 * Class encapsulates reading translation and rotation of camera
 * with provided readers.
 */
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
        /**
         * Calculate displacement using previously passed frame and
         * newFrame.
         * @param newFrame new frame to read displacement from
         * @return Translation in x and y axes and rotation in radians
         * (x,y,alpha)
         */
        cv::Point3f calculateDisplacement(const cv::Mat &newFrame);
        /**
         * Get features tracked by rotation reader
         * @return Vector of tracked features locations
         */
        std::vector<std::list<cv::Point2f> > getRotationFeatures() const;
        /**
         * Get features tracked by translation reader
         * @return Vector of tracked features locations
         */
        std::vector<std::list<cv::Point2f> > getTranslationFeatures() const;
        virtual ~VisualOdometer();
        VisualOdometer &operator=(const VisualOdometer &toCopy);
};

#endif /* VISUALODOMETER_H_ */
