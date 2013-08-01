/**
 * \file
 * \date 21.02.2013
 * \author Michał Orynicz
 */

#include "TranslationReader.hpp"

TranslationReader::TranslationReader(const FeatureTracker &tracker,
        const FeatureExtractor &extractor,
        const std::list<FeatureFilter*> &filters,
        const unsigned int &maxFeatures,
        const std::vector<std::list<cv::Point2f> > &trackedFeatures) :
        MovementReader(tracker,extractor,filters,maxFeatures,trackedFeatures){}

TranslationReader::TranslationReader(const TranslationReader& toCopy): MovementReader(toCopy){}

TranslationReader::~TranslationReader() {}

