/*
 * BirdsEyeTranslationReader.hpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef BIRDSEYETRANSLATIONREADER_HPP_
#define BIRDSEYETRANSLATIONREADER_HPP_

#include <TranslationReader.hpp>

class BirdsEyeTranslationReader: public TranslationReader {
        std::vector<cv::Point2f> _translations;
        cv::Point2f _rotationCenter;
        cv::Size _viewSize;
        cv::Mat _homography;
        std::vector<cv::Point2f> computeTranslationVectors(
                const cv::Mat &rotationMatrix);
        void filterBoarderFeatures();
        void filterFeatures();
    public:
        BirdsEyeTranslationReader(
                const BirdsEyeTranslationReader &toCopy);
        BirdsEyeTranslationReader(const cv::Mat &homography,
                const FeatureExtractor &extractor,
                const FeatureTracker &tracker,
                const unsigned int& maxFeatures,
                const std::list<FeatureFilter*>& filters,
                cv::Point2f rotationCentre, const cv::Size &imageSize,
                const double &margin, const cv::Size &viewSize =
                        cv::Size(1000, 1000),
                const std::vector<std::list<cv::Point2f> > &trackedFeatures =
                        std::vector<std::list<cv::Point2f> >());
        virtual TranslationReader *constructCopy() const;
        virtual cv::Point3f readTranslation(const cv::Mat &newFrame,
                const double& rotation);
        cv::Mat getBirdsEyeView(const cv::Mat &input);
        virtual ~BirdsEyeTranslationReader();
};

#endif /* BIRDSEYETRANSLATIONREADER_HPP_ */
