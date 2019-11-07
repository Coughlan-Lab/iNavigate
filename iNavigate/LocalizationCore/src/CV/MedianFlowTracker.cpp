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
 * @file MedianFlowTracker.cpp
 *
 *  Created on: @date Mar 12, 2013
 *      Author: @author ender tekin
 */

#include "MedianFlowTracker.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <algorithm>
#include <vector>
#include <cassert>
#include <tuple>


namespace
{
    //=========================
    //
    // HELPER DECLARATIONS
    //
    //=========================
    
    ///Number of tracked points
    static const int N_ROWS = 10, N_COLS = 10;
    static const int N_POINTS = N_ROWS * N_COLS;
    
    typedef std::pair<cv::Point2f, cv::Point2f> PointCorrespondence;
    
    ///Returns the motion between a pair of points
    template<typename T>
    inline cv::Point_<T> getMotion(const std::pair<cv::Point_<T>, cv::Point_<T>>& pc)
    {
        return pc.second - pc.first;
    }
    
    /**
     * Initializes the point grid to track
     * @param[in] bbox bounding box
     */
    void initializePointGrid(std::vector<cv::Point2f> &pts, const cv::Rect &bbox)
    {
        pts.clear();
        cv::Point2f step( (float) bbox.width / (float) (N_COLS + 1), (float) bbox.height / (float) (N_ROWS + 1) );
        cv::Point2f pt(bbox.x, bbox.y);
        for (int i = 0; i < N_ROWS; ++i)
        {
            pt.x = bbox.x;
            pt.y += step.y;
            for (int j = 0; j < N_COLS; ++j)
            {
                pt.x += step.x;
                pts.push_back(pt);
            }
        }
    }

    template<size_t N>
    struct OpticalFlowData
    {
        //Vector of points to track, their new locations, and backtracked locations from the new locations
        std::vector<cv::Point2f> points, trackedPoints, backTrackedPoints;
        //Status of the tracked points in the forward/backward optical flow
        std::vector<std::uint8_t> fStatus, bStatus;
        //Calculated error of each tracked point
        std::vector<float> error;
        OpticalFlowData(const cv::Rect& bbox)
        {
            points.reserve(N);
            trackedPoints.reserve(N);
            backTrackedPoints.reserve(N);
            fStatus.reserve(N);
            bStatus.reserve(N);
            error.reserve(N);
            //Initialize point grid
            initializePointGrid(points, bbox);
        }
    };
    
    template<typename Iterator, typename Compare>
    Iterator calculateMedian(Iterator begin, const Iterator end, const Compare compare)
    {
        assert( begin != end );
        auto size = std::distance(begin, end);
        auto median = begin;
        std::advance(median, (size >> 1) );
        std::nth_element(begin, median, end, compare);
        return median;
    }

    template<typename Iterator>
    Iterator calculateMedian(Iterator begin, const Iterator end)
    {
        return calculateMedian( begin, end, std::less<typename std::iterator_traits<Iterator>::value_type>() );
    }

    template<typename Iterator>
    float calculateNormalizedCrossCorrelation(Iterator it1, const Iterator end1, Iterator it2)
    {
        //Using CV_TM_CCOEFF_NORMED, See http://docs.opencv.org/modules/imgproc/doc/object_detection.html?highlight=matchtemplate
        if (it1 == end1)
            return 0.f;
        float ncc = 0.f, m1 = 0.f, m2 = 0.f, v1 = 0.f, v2 = 0.f;
        auto n = std::distance(it1, end1);
        while (it1 != end1)
        {
            m1 += (*it1);
            m2 += (*it2);
            ncc += (float)(*it1) * (float)(*it2);
            v1 += (float)(*it1) * (float)(*it1);
            v2 += (float)(*it2) * (float)(*it2);
            ++it1;
            ++it2;
        }
        ncc -= (m1 * m2 / n);
        v1 -= (m1 * m1 / n);
        v2 -= (m2 * m2 / n);
        float v = v1 * v2;
        return ( v <= 0.f ? 0.f : ncc / sqrt(v) );
    }

    
    std::vector<PointCorrespondence> calculateCorrespondences(const MatUint8 &prevImg, const MatUint8 &aImg, const cv::Rect &bbox)
    {
        std::vector<PointCorrespondence> correspondences;
        if (bbox.area() < 1)
            return correspondences;
        correspondences.reserve(N_POINTS >> 2); //since median filtering should limit the results to at most a quarter of the actual pts.
        struct CorrespondenceErrors
        {
            int index;
            float dist;
            float ncc;
        };
        std::vector<CorrespondenceErrors> errors;
        errors.reserve(N_POINTS);
        
        OpticalFlowData<N_POINTS> data(bbox);
        //Calculate forward optical flow
        static const cv::Size OPTICAL_FLOW_WINDOW_SIZE_(21,21);
        calcOpticalFlowPyrLK(prevImg, aImg, data.points, data.trackedPoints, data.fStatus, data.error, OPTICAL_FLOW_WINDOW_SIZE_, 3);
        //Calculate backward optical flow
        calcOpticalFlowPyrLK(aImg, prevImg, data.trackedPoints, data.backTrackedPoints, data.bStatus, data.error, OPTICAL_FLOW_WINDOW_SIZE_, 3);
        //Calculate the NCC error and return the matched pixels
        static const int PATCH_SIZE = 16;
        const cv::Size patchSize(PATCH_SIZE, PATCH_SIZE);
        MatUint8 patch1(patchSize), patch2(patchSize);
        //assert(patch1.data != patch2.data);
        for (int i = 0; i < N_POINTS; i++)
        {
           	//if either the forward or backward flow has failed for this point, ignore.
            if (data.fStatus[i] && data.bStatus[i])
            {
                //SPEEDUP: Round the pixel and calculate cross correlation directly
                getRectSubPix( aImg, patchSize, data.points[i], patch1 );
                getRectSubPix( aImg, patchSize, data.trackedPoints[i], patch2 );
                //Calculate normalized cross correlation
                assert( patch1.isContinuous() && patch2.isContinuous() );
                float ncc = calculateNormalizedCrossCorrelation( patch1.begin(), patch1.end(), patch2.begin() );
                float dist = cv::norm(data.points[i] - data.backTrackedPoints[i]);
                errors.push_back( {i, dist, ncc} );
            }
        }
        if ( errors.size() < 4 )
        {
            return correspondences; //cannot track
        }
        //Filter the points according to errors
        auto itMedian = calculateMedian(errors.begin(), errors.end(),
                                             [](const CorrespondenceErrors &e1, const CorrespondenceErrors &e2) {return (e1.dist < e2.dist); } );
        itMedian = calculateMedian(errors.begin(), itMedian,
                                        [](const CorrespondenceErrors &e1, const CorrespondenceErrors &e2) {return (e1.ncc < e2.ncc); } );
        errors.erase(itMedian, errors.end());
        //Return the best matched points
        for (const auto& err : errors)
        {
            correspondences.emplace_back(data.points[err.index], data.trackedPoints[err.index]);
        }
        return correspondences;
    }
    
    /**
     * Calculates the bounding box from point correspondences
     * @param[in] correspondences set of point correspondences
     * @param[in] maxMotion maximum allowed motion pixel-wise.
     * @return new bounding box estimate
     */
    cv::Rect calculateBoundingBox(const std::vector<PointCorrespondence>&correspondences, const cv::Rect& bbox, float maxMotion)
    {
        //Get the median motion and scale
        int nPoints = (int) correspondences.size();
        //TSizeInt originalSize = bbox.size();
        static const int MIN_CORRESPONDENCES = 10;
        if ( (nPoints < MIN_CORRESPONDENCES) || (bbox.area() < 1) )
        {
            return cv::Rect();
        }
        std::vector<float> xDisp, yDisp, scales;
        xDisp.reserve(nPoints);
        yDisp.reserve(nPoints);
        scales.reserve(nPoints * (nPoints - 1) / 2);
        for (int n = 0; n < nPoints; ++n)
        {
            cv::Point2f disp = getMotion(correspondences[n]);
            xDisp.push_back(disp.x);
            yDisp.push_back(disp.y);
            for (int k = 0; k < n; ++k)
            {
                cv::Point2f disp2 = getMotion(correspondences[k]);
                float angle = ( (norm(disp) > 0) && (norm(disp2) > 0) ? acos ( (disp.ddot(disp2)) / (norm(disp) * norm(disp2)) ) : 0.f);
                if (angle < 0.3)
                {
                    float dist = cv::norm(correspondences[n].first - correspondences[k].first);
                    assert(dist > 0.f);
                    float nextDist = cv::norm(correspondences[n].second - correspondences[k].second);
                    scales.push_back( nextDist / dist );
                }
            }
        }
        assert( xDisp.size() == nPoints );
        assert( yDisp.size() == nPoints );
        //Calculate median motion
        cv::Point2f medianMotion( *calculateMedian(xDisp.begin(), xDisp.end()), *calculateMedian(yDisp.begin(), yDisp.end()) );
        //Calculate median scale
        float scale = (scales.empty() ? 1.f : *calculateMedian( scales.begin(), scales.end() ) );
        //Move the box
        const float c = .5f * (scale - 1.f);
        cv::Rect res( round(bbox.x + medianMotion.x - bbox.width * c),
                     round(bbox.y + medianMotion.y - bbox.height * c),
                     round(bbox.width / scale),
                     round(bbox.height / scale)
                     );
        return res;
    }
    
}   //end anon namespace

//=========================
//
// DEFINITIONS
//
//=========================

cv::Rect trackMedianFlow(const cv::Rect& loc, const MatUint8& prevImg, const MatUint8& currentImg)
{
    //Calculate estimated motion
    auto pointCorrespondences = calculateCorrespondences(prevImg, currentImg, loc);
    //Calculate and return new bounding box, ensuring a minimum number of point correspondences and a maximum
    const float maxMotion = currentImg.cols/ 30; //limit max trackable motion to around 2 degrees for a camera with a 60deg FOV.
    auto res = calculateBoundingBox(pointCorrespondences, loc , maxMotion);
    //ensure box is still inside the frame
    res &= cv::Rect(0, 0, currentImg.cols, currentImg.rows);

    return res;
}

