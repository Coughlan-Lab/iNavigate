//
//  ArucoMarkerDetector.hpp
//  LocalizationCore
//
//  Created by Giovanni Fusco on 11/6/18.
//  Copyright © 2018 SKERI. All rights reserved.
//

#ifndef ArucoMarkerDetector_h
#define ArucoMarkerDetector_h

#include "opencv2/aruco.hpp"

#include "CVDetector.h"

namespace compvis {
    
    class ArucoMarkerDetector : public CVDetector{
    
    public:
        ArucoMarkerDetector(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs): _cameraMatrix(cameraMatrix), _distCoeffs(distCoeffs){
            // set up ARuco detector parameters
            _arucoDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
            _arucoParams = cv::aruco::DetectorParameters::create();
            _arucoParams->adaptiveThreshWinSizeMin = 15;
            _arucoParams->minMarkerPerimeterRate = 0.08;
            _arucoParams->maxMarkerPerimeterRate = 2;
        }
        
        std::vector<CVObservation> detect(const cv::Mat& frame) throw (std::runtime_error){
            cv::Mat grayFrame;
            if (frame.channels() > 1)
                cv::cvtColor(frame,grayFrame,cv::COLOR_RGBA2GRAY);
            else
                grayFrame = frame;
            
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f> > corners;
            cv::aruco::detectMarkers(grayFrame, _arucoDictionary, corners, ids, _arucoParams);
            
            std::vector<CVObservation> obs;
            for ( int i = 0; i < ids.size(); i++)
                obs.push_back(CVObservation("aruco", std::to_string(ids[i]), corners[i]));
          return obs;
        }
        
        bool feedRotatedImage = true;
                              
    private:
        cv::Ptr<cv::aruco::Dictionary> _arucoDictionary;
        cv::Ptr<cv::aruco::DetectorParameters> _arucoParams;
        cv::Mat _cameraMatrix;
        cv::Mat _distCoeffs;
    };
} // ::compvis

#endif /* ArucoMarkerDetector_h */
