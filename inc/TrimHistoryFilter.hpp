/**
 * \file
 *  \date 30.03.2013
 *  \author Michał Orynicz
 */

#ifndef TRIMHISTORYFILTER_HPP_
#define TRIMHISTORYFILTER_HPP_

#include <FeatureFilter.hpp>

/// Filter keeps feature history from being too long
class TrimHistoryFilter: public FeatureFilter {
        unsigned _maxLength; ///< Max number of feature positions remembered
    public:
        /**
         * Constructor
         * @param maxLength - max number of remembered feature positions
         */
        TrimHistoryFilter(const unsigned int &maxLength);
        /**
         * Copy constructor
         * @param toCopy - object to copy
         */
        TrimHistoryFilter(const TrimHistoryFilter &toCopy);
        /// Method removes features that do not meet certain criteria

        /**
         * Method removes features that do not meet set criteria
         * @param[in] features - features to be filtered
         * @return container with only those features, which meet
         * the criteria
         */
        std::vector<std::list<cv::Point2f> > filterFeatures(
                const std::vector<std::list<cv::Point2f> > &features);
        /// Method creates a pointer to a copy of this object
        FeatureFilter* constructCopy() const;
        virtual ~TrimHistoryFilter();
};

#endif /* TRIMHISTORYFILTER_HPP_ */
