#ifndef _LOCALIZATIONSYSTEM_HPP_
#define _LOCALIZATIONSYSTEM_HPP_

#include <opencv2/opencv.hpp>

#include "ParticleFilter.hpp"
#include "VIOMeasurements.hpp"
#include "../CV/CVObservationsManager.hpp"
#include "../CV/ObjDetector.h"
#include "../CV/ArucoMarkerDetector.hpp"
#include "../LogTools/Visualizer.hpp"
#include "../Sensors/FloorChangeDetector.hpp"
#include "../Utils/MatUtils.hpp"
#include "NavGraph.hpp"

#include <stdio.h>
#include <iostream>

namespace locore{
    
class LocalizationSystem{
   
    public:
    
    struct PeakYaw_t{
        double yaw;
        double variance;
        bool valid;
    };
    
        LocalizationSystem(std::string resFolder, int numParticles, InitMode initMode, int startFloor, double posU, double posV, float initMotionYaw, float initYawNoise, std::shared_ptr<maps::MapManager> mapManager) : _initMode(initMode), _mapManager(mapManager) {
            _resFolder = resFolder;
            cv::Point2d posMt = cv::Point2f(posU, posV);
            signDetected = false;
            _init(numParticles, startFloor, posMt, initMotionYaw, initYawNoise); 
            _floorChangeDetector = sensors::FloorChangeDetector(startFloor);
            
            yawMapTimeStamp = high_resolution_clock::now();
        }
    
        ~LocalizationSystem(){;}

        // TODO: refactor this, so that the output is the estimated location
        cv::Mat step(const VIOMeasurements& vioData){
            if (_partFilter->particlesInitialized()){
                _vioData.update(vioData);
    //          std::cerr << "VIODATA DELTA YAW: " << _vioData.getDeltaYaw()*180/CV_PI << "\n";
                std::cerr << ">>> _obsManagerCV.update " << "\n";
                std::vector<compvis::CVObservation> detections = _obsManagerCV.update(_vioData);
                std::cerr << "<<< _obsManagerCV.update " << "\n";
                _vioData.getFrame().copyTo(_currFrame);
                signDetected = false;
                if (detections.size() > 0)
                    signDetected = true;
                std::cerr << ">>> _partFilter->step " << "\n";
                _partFilter->step(_vioData, detections);
                std::cerr << "<<< _partFilter->step " << "\n";
                std::cerr << ">>> _partFilter->getHeatMap " << "\n";
                cv::Mat kde = _partFilter->getHeatMap();
                std::cerr << "<<< _partFilter->getHeatMap " << "\n";
                std::cerr << ">>> LocalizationSystem::computeYawMap " << "\n";
                //computeYawMap();
                std::cerr << "<<< LocalizationSystem::computeYawMap " << "\n";
                _cvDetectorOutputFrame = _obsManagerCV.img;
                cv::cvtColor(_cvDetectorOutputFrame, _cvDetectorOutputFrame, cv::COLOR_GRAY2RGBA);
                
                for (auto it = detections.begin(); it!=detections.end(); ++it){
                    cv::rectangle(_cvDetectorOutputFrame, it->getROI(), cv::Scalar(255,0,0,255), 5);
                    cv::circle(_cvDetectorOutputFrame, it->getROICenter(), 3, cv::Scalar(0,255,0,255));
                    cv::putText(_cvDetectorOutputFrame, std::to_string(_partFilter->estimatedDistanceToSign),  it->getROICenter(), cv::FONT_HERSHEY_SIMPLEX, 1.25, cv::Scalar(255,0,0,255) );
                }
//              cv::Mat markers;
//              markers = _mapManager->getWallsImageRGB().clone();
//              return getParticlesYawMap();
                
                //cv::rotate(kde, kde, cv::ROTATE_90_CLOCKWISE);
                return kde;
            }
            else{
                _vioData = vioData;
                _partFilter->init(vioData);
//              _mapManager.showFeatures();
                return _mapManager->getWallsImageRGB();
            }
        }
    
        inline cv::Mat getCVDetectorOutputFrame() { return _cvDetectorOutputFrame; }
        cv::Mat getCurrentFrame() { return _currFrame; }
    
        cv::Mat getParticlesYawMap(){
            cv::Mat img = _mapManager->getWallsImageRGB().clone();
            img = _partFilter->plotParticlesYaw(img, 5);
            return img; 
        }
    
        inline bool isLocalizationUncertain() { return _partFilter->isPeakUncertain(); }
        inline bool isPeakHard(){ return _partFilter->isPeakHard(); }
    
        inline PeakDetector::peak_t getPeak(){return _partFilter->getPeak();}
    
        ParticleFilter::PFParameters getParticleFilterParameters() { return _partFilter->pfParameters; }
    
        void setParticleFilterParameters(ParticleFilter::PFParameters params) { _partFilter->pfParameters = params; }
        inline void setCameraHeight(float cameraHeightMt) { _cameraHeightMt = cameraHeightMt; }
    
        const std::vector<Particle> getParticles() { return _partFilter->getParticles(); }
        double getDistanceToSign() { return _partFilter->estimatedDistanceToSign; }
        bool signDetected;
    
        float getDeltaYawFromVIO() { return _vioData.getDeltaYaw(); }
    
        PeakYaw_t getYawAtPeak(double var_thr){
            // http://inka.mssm.edu/~mezei/proj/cv/
            PeakYaw_t yawPeak;
            yawPeak.valid = false;
            yawPeak.yaw = 1e6;
            yawPeak.variance = 0;
    
            
            PeakDetector::peak_t peak = getPeak();
            
            if (peak.valid){
                double sumX = 0, sumY = 0;
                std::vector<std::vector<double>> yawMap = computeYawMap();
                std::cerr << "\t\t yawMap.size() = " << yawMap.size() << "\n";
                int vcnt = 0;
                for (auto it = yawMap.begin(); it != yawMap.end(); ++it)
                    for (auto it2 = it->begin(); it2 != it->end(); ++it2){
                        sumX += cos(*it2);
                        sumY += sin(*it2);
                        vcnt++;
                    }
                yawPeak.variance = 1 - (sqrt(sumX*sumX + sumY*sumY))/vcnt;
                
     
                if (yawPeak.variance <= var_thr){
                    yawPeak.valid = true;
                    int ind = peak.pxCoord.x*_mapManager->getMapSizePixels().height+peak.pxCoord.y;
                    std::cerr << "\t\t ind = " << ind << "\n";
                    std::cerr << "[>] yawMap[ind]" << "\n";
                    std::vector<double> yaws = yawMap[ind];
                    std::cerr << "[<] yawMap[ind]" << "\n";
                    int cnt = 0;
                    double v[2] = {0, 0};
                    for (double y : yaws){
                        cnt++;
                        v[0] += cos(y);
                        v[1] += sin(y);
                    }
                    if (cnt > 0){
                        v[0] /= cnt;
                        v[1] /= cnt;
                        yawPeak.yaw = atan2(v[1], v[0]);
                        yawPeak.yaw = fmod( yawPeak.yaw*180/CV_PI, 360);
                        if (yawPeak.yaw < 0)
                            yawPeak.yaw += 360;
                    }
//                    yawMap.clear();
                }
            }
            return yawPeak;
        }
    
    private:
    
        std::unique_ptr<locore::ParticleFilter> _partFilter;
        compvis::CVObservationsManager _obsManagerCV;
        std::shared_ptr<maps::MapManager> _mapManager;
        logtools::Visualizer _visualizer;
        VIOMeasurements _vioData;
        sensors::FloorChangeDetector _floorChangeDetector;
        
        std::string _resFolder;
        cv::Mat _currFrame;
        cv::Mat _cvDetectorOutputFrame;
        std::chrono::time_point<std::chrono::high_resolution_clock> yawMapTimeStamp;
        int _arucoID;
        int _targetID;
      
        // estimated location in meters (relative to map origin)
//        float _posx;
//        float _posz;
    
        // estimated location in pixels (on map)
//        int _posu;
//        int _posv;
    
        float _cameraHeightMt;
    
        cv::Mat _cameraMatrix;
        cv::Mat _distCoeffs;
    
        InitMode _initMode;
            
        std::vector<std::vector<double>> computeYawMap(){
            std::vector<std::vector<double>> yawMap;
            


            
            yawMap = std::vector<std::vector<double>>(_mapManager->getMapSizePixels().width * _mapManager->getMapSizePixels().height);
            for (const Particle& p : _partFilter->getParticles()){
                if (p.valid){
                    cv::Point2i ppos = _mapManager->uv2pixels(p.getPositionPoint());
                    int ind = ppos.x*_mapManager->getMapSizePixels().height+ppos.y;
                    yawMap[ind].push_back(p.cameraYaw);
                    int yaw = trunc(p.cameraYaw* 180/CV_PI);
                    if (yaw >= 360)
                        yaw -= 360;
                }
            }
            return yawMap;
        }
    
        void _init(int numParticles, int floor, cv::Point2d posMt, float initMotionYaw, float initYawNoise){
            _arucoID = -1;
            
            std::string configFile = "exit_sign_config.yaml";
            std::string configFileRestroom = "restroom_sign_config.yaml";
            
            _obsManagerCV = compvis::CVObservationsManager();
            
            // *** todo : read this from a config file
            double cm[3][3] = {
                                    {496.76392161,  0.,           179.4307037},
                                    {0.,            492.06136884, 318.7074295 },
                                    {0.0,           0.0,          1.0         }
                              };
            
            double dcoeff[5] = {2.79707782e-01, -1.51142752e+00, -1.93469830e-03, -1.46837375e-03, 2.63094958e+00};
            // ***
            
            double w = 480/2;
            //double fx = 496.76392161 * 3/4;
            double fx = 525.33 * 3/4;
            //double fx = 496.76 * 3/4;
            double tan_alpha = w / fx;
            double anglePerPixel = atan(tan_alpha)/w;
            
            _cameraMatrix = cv::Mat(3,3,CV_64F, cm);
            _distCoeffs = cv::Mat(1,5, CV_64F, dcoeff);
            float radius = 1.;
            
           _obsManagerCV.attach(new compvis::ObjDetector(configFile, _resFolder));
            // _obsManagerCV.attach(new compvis::ObjDetector(configFileRestroom, _resFolder));
          //  _obsManagerCV.attach(new compvis::ArucoMarkerDetector(_cameraMatrix, _distCoeffs));
            
//            cv::Point2i startLoc_px = _mapManager->uv2pixels(posMt);
            _partFilter = std::make_unique<locore::ParticleFilter>(numParticles, _initMode, _mapManager, initMotionYaw, initYawNoise, radius, posMt);
            _partFilter->setCameraAPP(anglePerPixel, fx);
            
        }
};
    
} // ::locore

#endif
