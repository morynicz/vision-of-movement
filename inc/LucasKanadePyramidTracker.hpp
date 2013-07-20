/**
 * \file LucasKanadePyramidTracker.hpp
 *
 * \date 21.02.2013
 * \author Michał Orynicz
 */

#ifndef LUCASKANADEPYRAMIDTRACKER_HPP_
#define LUCASKANADEPYRAMIDTRACKER_HPP_

#include <FeatureTracker.hpp>
#include "CvTypesIo.hpp"

/// Class tracking features using cv::calcOpticalFlowPyrLK function
class LucasKanadePyramidTracker: public FeatureTracker {
        cv::Size _windowSize; ///< size of matching window
        unsigned int _maxLevel; ///< max pyramid level
        int _flags; ///< flags for cv::calcOpticalFlowPyrLK
        /// Threshold below which features are discarded
        double _minEigenvalueThreshold;
        /**
         * max dissimilarity between matched features above which the feature
         * is discarded
         */
        double _maxErrorThreshold;
        /**
         * Termination criteria for cv::calcOpticalFlowPyrLK
         */
        cv::TermCriteria _termCrit;
    public:
        LucasKanadePyramidTracker();
        /**
         * Copy constructor
         * @param toCopy - object to copy
         */
        LucasKanadePyramidTracker(
                const LucasKanadePyramidTracker &toCopy);
        /**
         * Constructor
         * @param windowSize - size of matching window
         * @param maxLevel - max pyramid level
         * @param flags - flags for cv::calcOpticalFlowPyrLK
         * @param terminationCriteria - Threshold below which features
         * are discarded
         * @param minEigenvalueThreshold -  Threshold below which features are discarded
         * @param maxErrorValue - Max dissimilarity between matched features
         */
        LucasKanadePyramidTracker(const cv::Size &windowSize,
                const unsigned int &maxLevel, const int &flags,
                const cv::TermCriteria &terminationCriteria,
                const double &minEigenvalueThreshold,
                const double &maxErrorValue);
        /// Method creates a pointer to a copy of this object
        virtual FeatureTracker *constructCopy() const;
        /// Method finds new positions of tracked features on new image frame

        /**
         * Method finds new positions of tracked features based on their past
         * positions and current and previous frames
         * @param oldInput - previous image frame
         * @param newInput - current image frame
         * @param trackedFeatures - container with tracked features
         */
        virtual void trackFeatures(const cv::Mat &oldInput,
                const cv::Mat &newInput,
                std::vector<std::list<cv::Point2f> > &trackedFeatures);
        virtual ~LucasKanadePyramidTracker();
};

class LucasKanadeParameters {
    public:
        int maxLevel;
        int flags;
        cv::Size winSize;
        double minEigenvalueThresh;
        double maxErrorValue;
        cv::TermCriteria termCrit;
};

void operator>>(const cv::FileNode &node,
        LucasKanadeParameters &lucasKanade);

cv::FileStorage &operator<<(cv::FileStorage &fs,
        const LucasKanadeParameters &lucasKanade);
#endif /* LUCASCANDAEPYRAMIDTRACKER_HPP_ */
