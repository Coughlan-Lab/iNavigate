#if !defined(CVOBSERVATIONSMANAGER_HPP)
#define CVOBSERVATIONSMANAGER_HPP

#define _USE_MATH_DEFINES
#include "CVObservation.hpp"
#include "CVDetector.h"
#include "VIOMeasurements.hpp"
#include "ObjDetector.h"
#include "ArucoMarkerDetector.hpp"
#include <cmath>
#include <iostream>
#include <vector>

namespace compvis{

class CVObservationsManager{

    public:
        CVObservationsManager(){;}
        void inline attach(CVDetector* detector) { _detectors.push_back(detector); }
        cv::Mat img;
    
        std::vector<CVObservation> update(const locore::VIOMeasurements& vioData){
            
            cv::Mat frame;
            vioData.getFrame().copyTo(frame);
            // this is to fix the orientation of the frame coming from the camera sensor on the phone
            cv::rotate(frame, frame, cv::ROTATE_90_CLOCKWISE);
            
            cv::Mat R = cv::getRotationMatrix2D(cv::Point2f(frame.size[0]/2, frame.size[1]/2), 90+vioData.getRoll()*180/M_PI, 1);
            cv::Mat rotFrame;
            cv::warpAffine(frame, rotFrame, R, cv::Size(frame.size[1], frame.size[0]));
           
            _observations.clear();
            rotFrame.copyTo(img);
            for(CVDetector* detector : _detectors){
                
                //if (detector->feedRotatedImage == true){
                    
                //}
                //else
                //    frame.copyTo(img);
                
                auto obs = detector->detect(img);
                if (obs.size() > 0)
                    _observations.insert(std::end(_observations), std::begin(obs), std::end(obs));       
            }
            return _observations;
        }

    private:
        std::vector<CVDetector*> _detectors;
        std::vector<CVObservation> _observations;
};

} // ::compvis

#endif // CVOBSERVATIONSMANAGER_HPP
