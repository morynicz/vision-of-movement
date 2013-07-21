/**
 * \file MovementReader.hpp
 *
 * \date 16.03.2013
 * \author Michał Orynicz
 */

#ifndef MOVEMENTREADER_HPP_
#define MOVEMENTREADER_HPP_

#include <vector>
#include <list>

#include <opencv2/core/core.hpp>

#include "FeatureExtractor.hpp"
#include "FeatureTracker.hpp"
#include "FeatureFilter.hpp"

/**
 * Base class for objects reading position changes of the camera
 * based on image from it.
 */
class MovementReader {
    protected:
        /// Features tracked
        std::vector<std::list<cv::Point2f> > _trackedFeatures;
        FeatureTracker *_tracker; ///<Feature tracker
        FeatureExtractor *_extractor; ///<Feature extractor
        std::list<FeatureFilter*> _filters; ///<List of feature filters
        ///Upper limit on number of features to track
        unsigned int _maxFeatures;
        cv::Mat _oldFrame;        ///<Previous frame
        ///Constructor
        /**
         * Constructor
         * @param tracker - feature tracker
         * @param extractor - feature extractor
         * @param filters - list of feature filter pointers
         * @param maxFeatures - upper limit for number of tracked features
         * @param trackedFeatures - tracked features
         */
        MovementReader(const FeatureTracker &tracker,
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
        MovementReader(const MovementReader &toCopy);
    public:
        ///Default constructor
        MovementReader();
        /// Getter method for tracked features
        /**
         * Getter method for tracked features
         * @return Tracked features container
         */
        std::vector<std::list<cv::Point2f> > getTrackedFeatures() const;
        /// Complement missing features up to maxFeatures
        /**
         * Complement the collection of tracked features so it contains
         * up to maxFeatures of features.
         * @param maxFeatures Upper limit for number of features tracked
         */
        void refillFeatures(const int &maxFeatures);
        ///Destructor
        virtual ~MovementReader();
};

#endif /* MOVEMENTREADER_HPP_ */
