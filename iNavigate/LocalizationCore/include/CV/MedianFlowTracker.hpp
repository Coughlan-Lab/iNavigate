/*
 Copyright 2015 The Smith-Kettlewell Eye Research Institute
 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at
 
 http://www.apache.org/licenses/LICENSE-2.0
 
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */
/**
 * @file MedianFlowTracker.hpp
 *
 *  Created on: @date Mar 12, 2013
 *      Author: @author ender tekin
 */

#ifndef MEDIANFLOWTRACKER_HPP_
#define MEDIANFLOWTRACKER_HPP_

#include <cstdint>
#include "opencv2/core/core.hpp"


    
typedef cv::Mat_<std::uint8_t> MatUint8;        ///< grayscale image

/// Median flow tracker.
/// Based on http://www3.ee.surrey.ac.uk/CVSSP/Publications/papers/Kalal-ICPR-2010.pdf
/// @param[in] loc location of the object in the previous frame
/// @param[in] prevImg grayscale representation of the previous frame
/// @param[in] currentImg grayscale representation of the current frame
/// @return estimated location of the object in currentImg. If object is lost, returns cv::Rect()
/// TODO: See if we can do away with the whole frame needed in prevImg, and just use the previous patch instead
cv::Rect trackMedianFlow(const cv::Rect& loc, const MatUint8& prevImg, const MatUint8& currentImg);



#endif /* MEDIANFLOWTRACKER_HPP_ */
