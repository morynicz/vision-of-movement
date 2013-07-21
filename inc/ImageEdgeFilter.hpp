/**
 * \file ImageEdgeFilter.hpp
 *
 *  \date 21.02.2013
 *  \author Michał Orynicz
 */

#ifndef IMAGEEDGEFILTER_HPP_
#define IMAGEEDGEFILTER_HPP_

#include <opencv2/core/core.hpp>
#include "FeatureFilter.hpp"
/**
 * Filter removing features that are too lose to edge of the image
 */
class ImageEdgeFilter: public FeatureFilter {
        std::vector<double> _a; ///< a parameters of image edge lines
        std::vector<double> _b; ///< b parameters of image edge lines
        std::vector<double> _c; ///< c parameters of image edge lines
        double _margin; ///< safe distance from edge
    public:
        /**
         * Constructor
         * @param transformMatrix - matrix of image transformation used on
         * the image
         * @param imageSzie - size of the image
         * @param margin - margin width in pixels
         */
        ImageEdgeFilter(const cv::Mat& transformMatrix,
                const cv::Size &imageSzie, const double &margin);
        /**
         * Copy constructor
         * @param toCopy - object to copy
         */
        ImageEdgeFilter(const ImageEdgeFilter &toCopy);
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
        FeatureFilter *constructCopy() const;
        virtual ~ImageEdgeFilter();
};

#endif /* IMAGEEDGEFILTER_HPP_ */
