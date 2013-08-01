/**
 * \file
 * \date 21.02.2013
 * \author Michał Orynicz
 */

#ifndef TANGENTROTATIONREADER_HPP_
#define TANGENTROTATIONREADER_HPP_

#include <RotationReader.hpp>

class TangentRotationReader: public RotationReader {
        double _focalLength;
        cv::Size _imageSize;
        std::vector<float> computeRotationVectors();
    public:
        TangentRotationReader(const FeatureTracker &tracker,
                const FeatureExtractor &extractor,
                const std::list<FeatureFilter*> &filters,
                const unsigned int &maxFeatures,
                const double &focalLength, const cv::Size& imageSize,
                const double &margin,
                const std::vector<std::list<cv::Point2f> > &trackedFeatures =
                        std::vector<std::list<cv::Point2f> >());
        TangentRotationReader(const TangentRotationReader &toCopy);
        virtual RotationReader *constructCopy() const;
        virtual float readRotation(const cv::Mat &newFrame);
        virtual ~TangentRotationReader();
};

#endif /* TANGENTROTATIONREADER_HPP_ */
