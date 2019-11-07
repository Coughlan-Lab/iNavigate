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
        
        enum TurnDirection { Forward = 0, TurnAround = -1, Left = 1, Right = 2, EasyLeft = 3, EasyRight = 4, None = -2,
                             Arrived = 100, LeftToDest = 101, RightToDest = 102 };
        
        struct navigation_t {
            float course;
            float refAngle;
            float angleError;
            TurnDirection instruction;
            NavGraph::NodeType approachingNodeType;
            float distanceToApproachingNode;
            std::string comments;
            std::string nodeLabel;
            bool valid;
            bool destThroughDoor;
            float userUVPos[2];
            float nodeUVPos[2];
        };
        
        NavigationSystem(std::string mapFolder, int floor, bool exploreMode, float motionThreshold){
            _mapManager = std::make_shared<maps::MapManager>();
            _mapManager->init(mapFolder, floor);
            _exploreMode = exploreMode;
            std::string graphJSONFile = mapFolder + "/navgraph.json";
            _navGraph = std::make_unique<NavGraph>(graphJSONFile, _mapManager);
            destinationId = -1;
            _distanceMoved = 0;
            _course = 1000;
            _refAngle = 1000;
            _prevCourse = 1000;
            _firstCourseEstimate = true;
            _motionThreshold = motionThreshold;
            
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
        }
        
        inline void setCameraHeight(float cameraHeight) { _locSystem->setCameraHeight(cameraHeight); }
        inline void setInitialCourse(float yaw) { _course = yaw; }
        
        navigation_t step(const locore::VIOMeasurements& vioData, int deltaFloors){
            navigation_t navData;
            navData.instruction = None;
            navData.distanceToApproachingNode = 1e6;
            navData.approachingNodeType = NavGraph::Undef;
            navData.comments = "";
            navData.nodeLabel = "";
            navData.valid = false;
            updateCurrentFloor(deltaFloors);
            
            
            // run the localization and get an estimated location for the user
            // if peak.valid == false it means that we do not have a localization estimate
            _navigationImage = _locSystem->step(vioData);
            locore::PeakDetector::peak_t peak = _locSystem->getPeak();
            
            if (destinationId >= 0 && peak.valid){
                // project the peak to the navigation graph
                _currSnappedPosition = _navGraph->snapUV2Graph(peak.uvCoord, 0, _mapManager->currentFloor, true);
               
                if (_currSnappedPosition.srcNodeId >= 0 && _currSnappedPosition.destNodeId >= 0){
                    //need to calculate the route angle based on the path
                    _path = _navGraph->getPathFromCurrentLocation(_currSnappedPosition, destinationId);
                    navData.approachingNodeType = _navGraph->getNode(_path[0]).type;
                    navData.nodeLabel = _navGraph->getNode(_path[0]).label;
                    navData.comments = _navGraph->getNode(_path[0]).comments;
                    navData.valid = true;
                    navData.destThroughDoor = false;
                    navData.userUVPos[0] = peak.uvCoord.x;
                    navData.userUVPos[1] = peak.uvCoord.y;
                    float distToNextNode = getDistanceToNextNode();
                    navData.distanceToApproachingNode = distToNextNode;
                    // if we are far enough from the next node, refer to the current edge
                    if (_path.size()>1 && (distToNextNode > 1.5 || _goingBackward)){
                        _refAngle = getCurrentEdgeAngle(_path[0]);
                        navData.nodeUVPos[0] = _navGraph->getNode(_path[0]).positionUV.x;
                        navData.nodeUVPos[1] = _navGraph->getNode(_path[0]).positionUV.y;
                    }
                    else if (_path.size()>1 && !_goingBackward) {
                        _refAngle = getEdgeAngle(_path[0], _path[1]);
                        navData.nodeUVPos[0] = _navGraph->getNode(_path[1]).positionUV.x;
                        navData.nodeUVPos[1] = _navGraph->getNode(_path[1]).positionUV.y;
                    }
                    _prevCourse = _course;
                    computeUserCourse(peak, vioData);
                    float courseDiff = atan2(sin((_prevCourse-_course)*CV_PI/180), cos((_prevCourse-_course)*CV_PI/180));
                    
                    float diffAngle = atan2(sin((_refAngle-_course)*CV_PI/180), cos((_refAngle-_course)*CV_PI/180));
                    if (_path[0] == destinationId && distToNextNode <= 1.){
                        _refAngle = getCurrentEdgeAngle(_path[0]);
                        navData.nodeUVPos[0] = _navGraph->getNode(_path[0]).positionUV.x;
                        navData.nodeUVPos[1] = _navGraph->getNode(_path[0]).positionUV.y;
                        navData.instruction = Arrived;
                    }
                    else if (_path.size() == 2 && !_goingBackward){
                        navData.instruction = calculateTurn(diffAngle*180/CV_PI, 25, true);
                        navData.destThroughDoor = _navGraph->getNode(_path[1]).isDoor;
                    }
                    else if (!_goingBackward || courseDiff*180/CV_PI > 10)
                        navData.instruction = calculateTurn(diffAngle*180/CV_PI, 25, false);
                    else
                        navData.instruction = TurnAround;
                    navData.angleError = diffAngle*180/CV_PI;
                    drawNavigationGraph();
                }
            }
            
            navData.course = _course;
            navData.refAngle = _refAngle;
            return navData;
        }
        
        
        float getDistanceToNextNode(){
            cv::Point2f pos = _currSnappedPosition.realuvPos;
            NavGraph::Node n = _navGraph->getNode( _path[0]);
            cv::Point2f nodePos = n.positionUV;
            float d = cv::norm(nodePos - pos);
            return d;
        }
        
        float getEdgeAngle(int srcNodeId, int destNodeId){
            NavGraph::Node n = _navGraph->getNode(srcNodeId);
            return n.edges[destNodeId].angleDeg;
        }
        
              
        // handles the ambiguity of getting snapped to an edge withouth knowing precisely which way we should head
        float getCurrentEdgeAngle(int nodeId){
            if (_currSnappedPosition.destNodeId == nodeId){
                NavGraph::Node n = _navGraph->getNode(_currSnappedPosition.srcNodeId);
                return n.edges[_currSnappedPosition.destNodeId].angleDeg;
            }
            else{
                NavGraph::Node n = _navGraph->getNode(_currSnappedPosition.destNodeId);
                return n.edges[_currSnappedPosition.srcNodeId].angleDeg;
            }
        }
        
        void computeUserCourse(locore::PeakDetector::peak_t peak, const locore::VIOMeasurements& vioData){
            if (peak.valid){
                std::cerr << "got peak" << "\n";
                if (_firstCourseEstimate){
                    _prevPeak = peak;
                    _firstCourseEstimate = false;
//                    _course = fmod(_locSystem->getYawAtPeak()*180/CV_PI, 360);
//                    if (_course < 0)
//                        _course += 360;
                }
                else{
                    float t = cv::norm(peak.uvCoord - _prevPeak.uvCoord);
                    _distanceMoved += t;
                    if (_distanceMoved >= _motionThreshold){
                        _distanceMoved = 0;
                        _course = atan2(peak.uvCoord.x - _prevPeak.uvCoord.x, peak.uvCoord.y - _prevPeak.uvCoord.y)*180/CV_PI;
                        _prevPeak = peak;
                        _course = fmod(_course, 360);
                        if (_course < 0)
                            _course += 360;
                    }
                    else{
//                        _course = fmod(_locSystem->getYawAtPeak()*180/CV_PI, 360);
//                                           if (_course < 0)
//                                               _course += 360;
                        _course = fmod(_course + _locSystem->getDeltaYawFromVIO()*180/CV_PI, 360);
                        if (_course < 0)
                        _course += 360;
                    }
                }
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
//            cv::rotate(_navigationImage, _navigationImage, cv::ROTATE_90_CLOCKWISE);
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
        
        bool _exploreMode;
        bool _firstCourseEstimate;
        float _distanceMoved;
        float _course;
        float _refAngle;
        float _motionThreshold;
        bool _goingBackward;
        float _prevCourse;
        
        locore::PeakDetector::peak_t _prevPeak;
        NavGraph::SnappedPosition _currSnappedPosition;
        cv::Mat _navigationImage;

        std::vector<int> _path;
        
        std::unique_ptr<NavGraph> _navGraph;
        std::unique_ptr<locore::LocalizationSystem> _locSystem;
        std::shared_ptr<maps::MapManager> _mapManager;
       
        cv::Point2f _snapPositionToGraph(cv::Point2f pos) { return _navGraph->snapUV2Graph(pos, _mapManager->currentFloor, false).uvPos; }
        
        TurnDirection calculateTurn(float  diffAngleDeg, float thresholdDeg, bool toDestination){
            _goingBackward = false;
            if (diffAngleDeg > thresholdDeg && diffAngleDeg <= 120){
                if (toDestination)
                    return LeftToDest;
                else
                    return Left;
            }
            else if (diffAngleDeg >= -120 && diffAngleDeg < -thresholdDeg){
                if (toDestination)
                        return RightToDest;
                    else
                        return Right;
            }
            else if (diffAngleDeg <= -125 || diffAngleDeg >= 125 ){
                _goingBackward = true;
                return TurnAround;
            }
//            if (diffAngleDeg > thresholdDeg && diffAngleDeg <= 45)
//                return EasyLeft;
//            else if (diffAngleDeg > 45 && diffAngleDeg >= 90 + thresholdDeg)
//                return Left;
//            else if (diffAngleDeg > 90 + thresholdDeg)
//                return TurnAround;
//            else if (diffAngleDeg >= -45 && diffAngleDeg < -thresholdDeg)
//                return EasyRight;
//            else if (diffAngleDeg >= -90 - thresholdDeg && diffAngleDeg < -45)
//                return Right;
//            else if (diffAngleDeg < -90 - thresholdDeg)
//                return TurnAround;
            else
                return None;
        }
    };
    
} /* namespace navgraph */

#endif /* NavigationUI_hpp */
