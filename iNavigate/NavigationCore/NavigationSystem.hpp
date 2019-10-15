//
//  NavigationSystem.hpp
//  iLocalize
//
//  Created by Giovanni Fusco on 4/1/19.
//  Copyright © 2019 SKERI. All rights reserved.
//

#ifndef NavigationSystem_hpp
#define NavigationSystem_hpp

#include <stdio.h>
#include "MapManager.hpp"
#include "LocalizationSystem.hpp"
#include "NavGraph.hpp"
#include "NavigationTracker.hpp"

namespace navgraph{
    
    class NavigationSystem{
    public:
        
        int destinationId;
        
        NavigationSystem(std::string mapFolder, int floor, bool exploreMode){
            _mapManager = std::make_shared<maps::MapManager>();
            _mapManager->init(mapFolder, floor);
            _exploreMode = exploreMode;
            std::string graphJSONFile = mapFolder + "/navgraph.json";
            _navGraph = std::make_unique<NavGraph>(graphJSONFile, _mapManager);
            destinationId = -1;
        }
        
        std::vector<float> getNodeUVPosition(int nodeID){
            std::vector<float> pos;
            auto nodes = _navGraph->getNodes();
            pos.push_back(nodes[nodeID].positionUV.x);
            pos.push_back(nodes[nodeID].positionUV.y);
            return pos;
        }
        
        void initLocalizationSystem(std::string resFolder, int numParticles, locore::InitMode initMode, double posU, double posV, float initMotionYaw, float initYawNoise){
            _locSystem = std::make_unique<locore::LocalizationSystem>(resFolder, numParticles, initMode, _mapManager->currentFloor, posU, posV, initMotionYaw, initYawNoise, _mapManager);
            
            //std::shared_ptr<locore::LocalizationSystem>( new locore::LocalizationSystem(resFolder, numParticles, initMode, startFloor, posU, posV, initMotionYaw, initYawNoise, _mapManager));
            
        }
        
        inline void setCameraHeight(float cameraHeight) { _locSystem->setCameraHeight(cameraHeight); }
        
        NavGraph::SnappedPosition* step(const locore::VIOMeasurements& vioData, int deltaFloors){
            updateCurrentFloor(deltaFloors);
            
            _navigationImage = _locSystem->step(vioData);
            cv::Point2f peak = _locSystem->getRobustPeakUV();
            
            //if ((destinationId >= 0) && (peak.x > -1e6 && peak.y > -1e6)){
            if (peak.x > -1e6 && peak.y > -1e6){
                _currSnappedPosition = _navGraph->snapUV2Graph(peak,  _locSystem->getDeltaYawFromVIO()*180/CV_PI, _mapManager->currentFloor, true);
                
                if (_currSnappedPosition.srcNodeId >= 0 && _currSnappedPosition.destNodeId >= 0){
                    if (destinationId >=0){
                    _path = _navGraph->getPathFromCurrentLocation(_currSnappedPosition, destinationId);
                        if (_path.size() > 1){
                            NavigationTracker::TurnDirection turnDir = _navTrack.update(_currSnappedPosition, _path[0], _path[1]);
                            
//                            std::cerr << "\n heading: " << _currSnappedPosition.heading << ", refAngle: " << _currSnappedPosition.refAngle << "\n";
//                            std::cerr << "[][][] Heading diff: " << _currSnappedPosition.refAngle - _currSnappedPosition.heading << "\n";
                            
                            switch (turnDir) {
                                case NavigationTracker::Forward:
                                    _currSnappedPosition.instruction = "Go Forward";
                                    break;
                                case NavigationTracker::TurnAround:
                                    _currSnappedPosition.instruction = "Please turn around";
                                    break;
                                case NavigationTracker::Left:
                                _currSnappedPosition.instruction = "turn left";
                                break;
                                case NavigationTracker::EasyLeft:
                                    _currSnappedPosition.instruction = "make an easy left";
                                break;
                                case NavigationTracker::Right:
                                    _currSnappedPosition.instruction = "turn right";
                                break;
                                case NavigationTracker::EasyRight:
                                    _currSnappedPosition.instruction = "make an easy right";
                                break;
                                    
                                default:
                                    _currSnappedPosition.instruction = " ";
                                    break;
                            }
                        }
                        
                        drawNavigationGraph();
                    }
                    return &_currSnappedPosition;
                }
                else{
                    return nullptr;
                }
            }
            else{
                return nullptr;
            }
        }
        
        void drawNavigationGraph(){
//            std::vector<int> path = _navGraph->getPathFromCurrentLocation(_currSnappedPosition, destinationId);
            cv::Point2i pt = _mapManager->uv2pixels(_currSnappedPosition.uvPos);
            cv::drawMarker(_navigationImage, cv::Point2i(pt.y, pt.x), cv::Scalar(0,255,0), cv::MARKER_DIAMOND, 10, 3);
            
            // find path and plot it
            for (int i=0; i < _path.size()-1; i++){
                cv::Point2i npos1 = _mapManager->uv2pixels(_navGraph->getNode(_path[i]).positionUV);
                cv::Point2i npos2 = _mapManager->uv2pixels(_navGraph->getNode(_path[i+1]).positionUV);
                cv::drawMarker(_navigationImage, cv::Point2i(npos1.y, npos1.x), cv::Scalar(0,255,255), cv::MARKER_TRIANGLE_DOWN, 10, 2);
                cv::line(_navigationImage, cv::Point2i(npos1.y, npos1.x), cv::Point2i(npos2.y, npos2.x), cv::Scalar(0,255,255), 2);
                if (i == 0)
                    cv::line(_navigationImage, cv::Point2i(pt.y, pt.x), cv::Point2i(npos1.y, npos1.x), cv::Scalar(255,0,255), 2, cv::LINE_4);
            }
            cv::Point2i npos = _mapManager->uv2pixels(_navGraph->getNode(_path[_path.size()-1]).positionUV);
            cv::drawMarker(_navigationImage, cv::Point2i(npos.y, npos.x), cv::Scalar(255,0,255), cv::MARKER_TRIANGLE_DOWN, 10, 3);
            cv::rotate(_navigationImage, _navigationImage, cv::ROTATE_90_CLOCKWISE);
        }
        
        
        inline cv::Mat getCVDetectorOutputFrame() { return _locSystem->getCVDetectorOutputFrame(); }
        
        const std::shared_ptr<maps::MapManager> getMapManager() { return _mapManager; }
        
        float getParticlesYaw(){
            const std::vector<locore::Particle> particles = _locSystem->getParticles();
            locore::Particle p = particles[0];
            float yaw = p.getCameraYaw();
            return yaw;
        }
        
        void dumpParticles(std::string outFolder, unsigned long int cnt){
            
            std::ofstream myfile;
            std::string filename = outFolder + "/" + std::to_string(cnt) + ".txt";
            myfile.open (filename);
            if (myfile.is_open()){
                const std::vector<locore::Particle> particles = _locSystem->getParticles();
                myfile << "x, y, score, yaw, heading, detection \n";
                for (locore::Particle p : particles){
                    // width, height, score, sign 0/1, pred dist, est. distance, det yaw, pred yaw,
                    cv::Point2i pt = _mapManager->uv2pixels(p.getPositionPoint());
                    myfile << std::to_string(pt.y) << ", " << std::to_string(pt.x) << ", " << std::to_string(p.getScore()) << ", " << std::to_string(p.getCameraYaw()) << ", " << std::to_string(p.getCourse()) << ", " << _locSystem->signDetected <<
                       _locSystem->getDistanceToSign() << ", " << std::to_string(p.bestEstimatedDistance) << ", " <<
                    std::to_string(p.bestDetYaw) << ", " << std::to_string(p.bestPredYaw) << "\n";
                    
                }
                myfile.close();
            }
        }
        
        void updateCurrentFloor(int floorNumDelta){
            _mapManager->currentFloor += floorNumDelta;
        }
        
        inline cv::Mat getNavigationGraph() { return _navigationImage; }
    
    private:
        std::unique_ptr<NavGraph> _navGraph;
        std::unique_ptr<locore::LocalizationSystem> _locSystem;
        std::shared_ptr<maps::MapManager> _mapManager;
        bool _exploreMode;
        cv::Point2f _snapPositionToGraph(cv::Point2f pos) { return _navGraph->snapUV2Graph(pos, _mapManager->currentFloor, false).uvPos; }
        NavGraph::SnappedPosition _currSnappedPosition;
        cv::Mat _navigationImage;
        std::vector<int> _path;
        NavigationTracker _navTrack;
    
    };
    
} /* namespace navgraph */

#endif /* NavigationUI_hpp */
