/**
 * \file BirdsEyeTranslationReader.hpp
 *
 *  \date 21.02.2013
 *  \author Michał Orynicz
 */

#ifndef BIRDSEYETRANSLATIONREADER_HPP_
#define BIRDSEYETRANSLATIONREADER_HPP_

#include <TranslationReader.hpp>

/// Class of objects reading translation from given images using
///     birds eye perspective transform and rotation subtraction

/**
 * Class of objects reading translation of the camera, by first
 * transforming the image analyzed to a birds eye perspective
 * and compensating rotation that occurred since last frame.
 */
class BirdsEyeTranslationReader: public TranslationReader {
        cv::Point2f _rotationCenter;
        cv::Size _viewSize; ///< size of transformed image
        cv::Mat _homography; ///< transformation matrix
        /// Method computes translation vectors based on new
        ///     and old feature positions and rotation matrix
        /**
         * Method computes translation vectors between new
         * and old feature positions with compensation
         * rotation using rotation matrix.
         *
         * @param rotationMatrix - transformation matrix for
         *      rotation compensation
         * @retval vector of 2d points representing translation
         * vectors
         */
        std::vector<cv::Point2f> computeTranslationVectors(
                const cv::Mat &rotationMatrix);
    public:
        /// Deep copy constructor
        BirdsEyeTranslationReader(
                const BirdsEyeTranslationReader &toCopy);
        /// Constructor
        /**
         * Constructor
         * @param homography - transformation matrix for birds eye view
         * transformation
         * @param extractor - feature extractor
         * @param tracker - feature tracker
         * @param maxFeatures - upper limit for number of tracked features
         * @param filters - feature filter list
         * @param rotationCentre - coordinates of center of rotation of
         * analyzed images
         * @param imageSize - input image size
         * @param margin - how far from image edge should the features
         * be
         * @param viewSize - size of transformation result image
         * @param trackedFeatures - the tracked features
         */
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
        /// Factory method for getting pointer to a copy of this object
        /**
         * Factory method returning pointer to a deep copy of the object
         * @return Pointer to a deep copy of this object
         */
        virtual TranslationReader *constructCopy() const;
        /// Get translation based on new frame and rotation
        /**
         * Method calculates translation that occurred since last frame
         * based on rotation occurred since last frame and current frame.
         * @param newFrame - current frame to read translation from.
         * @param rotation - rotation that occurred since last frame.
         * @return 3d point representing translation and rotation that
         * occurred since last frame.
         */
        virtual cv::Point3f readTranslation(const cv::Mat &newFrame,
                const double& rotation);
        /// Get transformed image view
        /**
         * Transform input to birds eye perspective
         * @param input - image to be transformed
         * @return transformed input
         */
        cv::Mat getBirdsEyeView(const cv::Mat &input);
        ///Destructor
        virtual ~BirdsEyeTranslationReader();
};

#endif /* BIRDSEYETRANSLATIONREADER_HPP_ */
