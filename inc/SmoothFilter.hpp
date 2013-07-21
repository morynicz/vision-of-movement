/**
 * \file SmoothFilter.hpp
 *
 * \date 21.02.2013
 * \author Michał Orynicz
 */

#ifndef SMOOTHFILTER_HPP_
#define SMOOTHFILTER_HPP_

#include "FeatureFilter.hpp"
/**
 * Class checks smoothness of features using feature position now,
 * one, three and seven frames ago, creating vectors of displacement.
 * Angles between vectors are measured, and if a threshold is violated,
 * a feature is marked a deviant and should be removed.
 */
class SmoothFilter: public virtual FeatureFilter {
        double _maxDeviation; ///< Max deviation angle between feature vectors
        /**
         * Max proportion of deviants to overall features, above which no deviants
         * are filtered out (global unsmoothness of movement)
         */
        double _deviantThreshold;
        /**
         * Min length of feature vector (not used)
         */
        double _minLength;
        /**
         * Min length of feature vector (not used)
         */
        double _maxLength;
    public:
        /**
         * Constructor
         * @param maxDeviation - Max deviation angle between feature vectors
         * @param maxDeviants - proportion of deviants above which they are
         * not punished for unsmoothness (global movement unsmoothness)
         * @param minLength - minimal length of feature vector (not used)
         * @param maxLength - maximal length of feature vector (not used)
         */
        SmoothFilter(const double &maxDeviation,
                const double &maxDeviants, const double &minLength,
                const double &maxLength);
        /**
         * Copy constructor
         * @param toCopy - object to copy
         */
        SmoothFilter(const SmoothFilter &toCopy);
        virtual ~SmoothFilter();
        /// Method creates a pointer to a copy of this object
        virtual FeatureFilter *constructCopy() const;
        /// Method removes features that do not meet certain criteria

        /**
         * Method removes features that do not meet set criteria
         * @param[in] features - features to be filtered
         * @return container with only those features, which meet
         * the criteria
         */
        virtual std::vector<std::list<cv::Point2f> > filterFeatures(
                const std::vector<std::list<cv::Point2f> > &features);
};

#endif /* SMOOTHFILTER_H_ */
