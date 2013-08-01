/**
 * \file
 * \date 21.02.2013
 * \author Michał Orynicz
 */

#ifndef TRANSLATIONREADER_H_
#define TRANSLATIONREADER_H_

#include <vector>
#include <list>
#include <opencv2/core/core.hpp>

#include "MovementReader.hpp"

#include "FeatureExtractor.hpp"
#include "FeatureTracker.hpp"
#include "FeatureFilter.hpp"
/**
 * Base class for objects reading translation from image
 */
class TranslationReader: public MovementReader {
    protected:
        ///Constructor
        /**
         * Constructor
         * @param tracker - feature tracker
         * @param extractor - feature extractor
         * @param filters - list of feature filter pointers
         * @param maxFeatures - upper limit for number of tracked features
         * @param trackedFeatures - tracked features
         */
        TranslationReader(const FeatureTracker &tracker,
                const FeatureExtractor &extractor,
                const std::list<FeatureFilter*> &filters,
                const unsigned int &maxFeatures,
                const std::vector<std::list<cv::Point2f> > &trackedFeatures =
                        std::vector<std::list<cv::Point2f> >());
        ///Deep copy constructor
        /**
         * Deep copy constructor
         * @param toCopy - object to initialize with
         */
        TranslationReader(const TranslationReader& toCopy);
    public:
        /// Factory method for copies of this object
        /**
         * Factory method for getting a pointer to deep copy of
         * this object
         * @return
         */
        virtual TranslationReader *constructCopy() const =0;
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
                const double& rotation) =0;
        ///Destructor
        virtual ~TranslationReader();
};

#endif /* TRANSLATIONREADER_H_ */
