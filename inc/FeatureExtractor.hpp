/**
 * \file
 * \date 21.02.2013
 * \author Michał Orynicz
 */

#ifndef FEATUREEXTRACTOR_HPP_
#define FEATUREEXTRACTOR_HPP_

#include <vector>
#include <list>
#include <opencv2/core/core.hpp>

/// Code indicating that no features were found.
const int NO_FEATURES_FOUND = -10;

/// Base class for feature extractors
class FeatureExtractor {
public:
	FeatureExtractor();
	/// Method creates a pointer to a copy of this object
	virtual FeatureExtractor *constructCopy() const =0;
	/// Method extracts features from given image

	/**
	 * Method extracts features from the given image
	 * @param[in] input - image from which features will be extracted
	 * @param[in] maxCorners - max features to be extracted
	 * @return vector of 2d points indicating locations of the features
	 */
	virtual std::vector<cv::Point2f> extractFeatures(const cv::Mat &input,
			const int & maxCorners) const=0;

	/// Method attempts to increase number of tracked features up to
	/// maxFeatures limit

	/**
	 * Method attempts to increase number of features tracked if there are
	 * less than maxFeaturesfeatures tracked
	 * @param[in] oldFrame - image on which features will be searched for
	 * @param[in,out] features - container with tracked features
	 * @param[in] maxFeatures - limit on number of tracked features
	 */
	void refillFeatures(const cv::Mat& oldFrame,
			std::vector<std::list<cv::Point2f> >& features,
			const unsigned int& maxFeatures);

	///Destructor
	virtual ~FeatureExtractor();
};

#endif /* FEATUREEXTRACTOR_HPP_ */
