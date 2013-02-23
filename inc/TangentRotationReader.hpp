/*
 * TangentRotationReader.hpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef TANGENTROTATIONREADER_HPP_
#define TANGENTROTATIONREADER_HPP_

#include <RotationReader.hpp>

class TangentRotationReader: public RotationReader {
public:
	TangentRotationReader(const FeatureExtractor&extractor,
			const FeatureTracker &tracker);
	TangentRotationReader(const TangentRotationReader &toCopy);
	virtual RotationReader *constructCopy() const;
	virtual float readRotation(const cv::Mat &newFrame);
	virtual ~TangentRotationReader();
};

#endif /* TANGENTROTATIONREADER_HPP_ */
