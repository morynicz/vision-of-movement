/**
 * \file
 * \date 21.02.2013
 * \author Michał Orynicz
 */

#ifndef TANGENTROTATIONREADER_HPP_
#define TANGENTROTATIONREADER_HPP_

#include <RotationReader.hpp>
/// Reads rotation using tangens function instead of projection

/**
 * Class reads rotation of camera based on two consecutive
 * images, by instead of projecting moved points on a cylindric surface
 * calculating the angular displacement based on linear displacement,
 * focal length and arcus tangent between them.
 */
class TangentRotationReader: public RotationReader {
        double _focalLength;
        cv::Size _imageSize; ///< size of processed images
        std::vector<float> computeRotationVectors();
    public:
        /**
         * Constructor
         * @param tracker
         * @param extractor
         * @param filters
         * @param maxFeatures
         * @param focalLength
         * @param imageSize size of processed image used for
         * built in ImageEdgeFilter
         * @param margin margin for built in ImageEdgeFilter
         * @param trackedFeatures
         */
        TangentRotationReader(const FeatureTracker &tracker,
                const FeatureExtractor &extractor,
                const std::list<FeatureFilter*> &filters,
                const unsigned int &maxFeatures,
                const double &focalLength, const cv::Size& imageSize,
                const double &margin,
                const std::vector<std::list<cv::Point2f> > &trackedFeatures =
                        std::vector<std::list<cv::Point2f> >());
        TangentRotationReader(const TangentRotationReader &toCopy);
        /**
         * Create a deep copy of self
         * @return Pointer to the copy
         */
        virtual RotationReader *constructCopy() const;
        /**
         * Read rotation from previous passed frame and current.
         * @param newFrame frame from which rotation will be read
         * @return Read rotation angle
         */
        virtual float readRotation(const cv::Mat &newFrame);
        virtual ~TangentRotationReader();
};

#endif /* TANGENTROTATIONREADER_HPP_ */
