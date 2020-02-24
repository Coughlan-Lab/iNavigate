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
                             Arrived = 100, LeftToDest = 101, RightToDest = 102, ForwardToDest = 103 };
        
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
            bool validTurnNode;
            float userUVPos[2];
            float nodeUVPos[2];
            float turnNodeUVPos[2];
            double yawVariance;
            navigation_t() : instruction(None), distanceToApproachingNode(1e6), approachingNodeType(NavGraph::Undef),
            comments(""), nodeLabel(""), valid(false), validTurnNode(false), destThroughDoor(false){};
        };
        
        struct Course_t{
            double angleDeg;
            double angle;
            double prevAngle;
            bool valid;
            bool init;
            Course_t() : angleDeg(0), angle(0), prevAngle(0), valid(false), init(false){};
        };
        
        NavigationSystem(std::string mapFolder, int floor, bool exploreMode, float motionThreshold){
            _mapManager = std::make_shared<maps::MapManager>();
            _mapManager->init(mapFolder, floor);
            _exploreMode = exploreMode;
            std::string graphJSONFile = mapFolder + "/navgraph.json";
            _navGraph = std::make_unique<NavGraph>(graphJSONFile, _mapManager);
            destinationId = -1;
            _distanceMoved = 0;
            peakYaw.valid = false;
            _refAngle = 1000;
            _prevPeakYaw = 1000;
            _firstCourseEstimate = true;
            _motionThreshold = motionThreshold;
            _goingBackward = false;
            navigation_t navData = navigation_t();
            loggingEvent = false;
            logCounter = 0;
            course.init = false;
            course.valid = false;
            
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

        void computeUserCourse(const locore::VIOMeasurements& vioData){
            float t = cv::norm(peak.uvCoord - prevPeak.uvCoord);
            _distanceMoved += t;
            if (course.init){
                if (_distanceMoved >= _motionThreshold){
                    _distanceMoved = 0;
                    course.prevAngle = course.angle;
                    course.angleDeg = atan2(peak.uvCoord.x - prevPeak.uvCoord.x, peak.uvCoord.y - prevPeak.uvCoord.y)*180/CV_PI;
                    course.angleDeg = fmod(course.angleDeg, 360);
                    if (course.angleDeg < 0)
                        course.angleDeg += 360;
                    course.valid = true;
                    course.angle = course.angleDeg * CV_PI/180;
                }
            }
            else{
                // we now have the first peak
                course.init = true;
                course.valid = false;
            }
//            if (_distanceMoved >= _motionThreshold){
//                _distanceMoved = 0;
//                _course = atan2(peak.uvCoord.x - _prevPeak.uvCoord.x, peak.uvCoord.y - _prevPeak.uvCoord.y)*180/CV_PI;
//                _prevPeak = peak;
//                _course = fmod(_course, 360);
//                if (_course < 0)
//                    _course += 360;
//            }
        }
        
        navigation_t step(const locore::VIOMeasurements& vioData, int deltaFloors, bool useYaw, bool logParticles){
            navData.valid = false;
            navData.validTurnNode = false;
            
            updateCurrentFloor(deltaFloors);
            
            // run the localization and get an estimated location for the user
            // if peak.valid == false it means that we do not have a localization estimate
            _navigationImage = _locSystem->step(vioData, logParticles);
            
            prevPeak = peak;
            peak = _locSystem->getPeak();
            computeUserCourse(vioData);
            
            _prevPeakYaw = peakYaw.yaw;
            peakYaw = computeYaw(peak, vioData);
            navData.yawVariance = peakYaw.variance;
            
            if (destinationId >= 0 && peak.valid && (peakYaw.valid || !useYaw)){ //we have a destination and the peak is valid and the variance of the peak yaw is acceptable
                navData.course = useYaw==true? peakYaw.yaw : course.angle;
                // project the peak location on the navigation graph
                _currSnappedPosition = _navGraph->snapUV2Graph(peak.uvCoord, 0, _mapManager->currentFloor, true);

                if (_currSnappedPosition.srcNodeId >= 0 && _currSnappedPosition.destNodeId >= 0){
                    //need to calculate the route angle based on the path
                    _path = _navGraph->getPathFromCurrentLocation(_currSnappedPosition, destinationId);
                    
                    std::cerr << "Path Len: " << _path.size() << "\n";
                    std::cerr << "_path[0]:" << _path[0] << ", destId:" << destinationId << "\n";
                    navData.approachingNodeType = _navGraph->getNode(_path[0]).type;
                    navData.nodeLabel = _navGraph->getNode(_path[0]).label;
                    navData.comments = _navGraph->getNode(_path[0]).comments;
                    navData.valid = useYaw || course.valid;
                    navData.userUVPos[0] = peak.uvCoord.x;
                    navData.userUVPos[1] = peak.uvCoord.y;
                    navData.turnNodeUVPos[0] = -1;
                    navData.turnNodeUVPos[1] = -1;
                    float distToNextNode = getDistanceToNextNode(peak);
                    float distToDestination = getDistanceToNode(_path[_path.size()-1], peak);
                    std::cerr << "Dist to Dest: " << distToDestination << "\n";
                    navData.distanceToApproachingNode = distToNextNode;
                    
                    setReferenceAngle(distToNextNode);
                   
                    if (_path.size()>2){ // for sonification, only sonify node before a turn
                        NavGraph::Node turnNode = getNextTurnNode();
                        navData.turnNodeUVPos[0] = turnNode.positionUV.x;
                        navData.turnNodeUVPos[1] = turnNode.positionUV.y;
                        navData.validTurnNode = true;
                    }

//                    float yawDelta = fabs(atan2(sin((_prevPeakYaw-peakYaw.yaw)*CV_PI/180), cos((_prevPeakYaw-peakYaw.yaw)*CV_PI/180)));
                    float yawDelta = fabs(_prevPeakYaw-navData.course)*180/CV_PI;

                    float diffAngle = atan2(sin((_refAngle-navData.course)*CV_PI/180), cos((_refAngle-navData.course)*CV_PI/180));
                    
                    // if the next node is the destination and we are close enough, we have arrived
//                    if (_path[0] == destinationId && distToNextNode <= 1.){
//                        _refAngle = getCurrentEdgeAngle(_path[0]);
//                        navData.nodeUVPos[0] = _navGraph->getNode(_path[0]).positionUV.x;
//                        navData.nodeUVPos[1] = _navGraph->getNode(_path[0]).positionUV.y;
//                        navData.instruction = Arrived;
//                        std::cerr << "NagivationSystem::ARRIVED!!!" << "\n";
//                    }
                    if (_path.size() < 2 && distToDestination < 0.75){
                        _refAngle = getCurrentEdgeAngle(_path[0]);
                        navData.nodeUVPos[0] = _navGraph->getNode(_path[0]).positionUV.x;
                        navData.nodeUVPos[1] = _navGraph->getNode(_path[0]).positionUV.y;
                        navData.instruction = Arrived;
                        std::cerr << "NagivationSystem::ARRIVED!!!" << "\n";
                    }
                    
                    //if there are only two nodes left in the path and the last node is marked as door, give heads up
                    else if (_path.size() == 2 && !_goingBackward && distToNextNode <= 1.){
                        navData.instruction = calculateTurn(diffAngle*180/CV_PI, 55, true);
                        navData.destThroughDoor = _navGraph->getNode(_path[1]).isDoor;
                    }
                    // otherwise, just calculate next instruction based on angle wrt the current graph edge
//                    else if ((_path.size() > 2) && (!_goingBackward || yawDelta > 10))
//                         navData.instruction = calculateTurn(diffAngle*180/CV_PI, 25, false);
                    else if  (yawDelta > 10) {
                           navData.instruction = calculateTurn(diffAngle*180/CV_PI, 55, false);
//                        navData.instruction = TurnAround;
//                        std::cerr << "turn around" << "\n";
                    }
                    navData.angleError = diffAngle*180/CV_PI;
                    drawNavigationGraph();
                }
            }
            else{
                    navData.valid = false;
            }
//            navData.course = peakYaw.yaw;
            navData.refAngle = _refAngle;
            return navData;
        }
        
        
        void setReferenceAngle(float distToNextNode){
            // if we are far enough from the next node, refer to the current edge
            if (_path.size()>1 && (distToNextNode > 1 || _goingBackward)){
                _refAngle = getCurrentEdgeAngle(_path[0]);
                navData.nodeUVPos[0] = _navGraph->getNode(_path[0]).positionUV.x;
                navData.nodeUVPos[1] = _navGraph->getNode(_path[0]).positionUV.y;
            }
            // otherwise if we are less than a meter from the next node, move to the next segment of the path
            else if (_path.size()>1 && !_goingBackward) {
                _refAngle = getEdgeAngle(_path[0], _path[1]);
                navData.nodeUVPos[0] = _navGraph->getNode(_path[1]).positionUV.x;
                navData.nodeUVPos[1] = _navGraph->getNode(_path[1]).positionUV.y;
            }
        }
        
        
        float getDistanceToNextNode(locore::PeakDetector::peak_t peak){
            return getDistanceToNode(_path[0], peak);
        }
        
        float getDistanceToNode(int nodeId, locore::PeakDetector::peak_t peak){
//            cv::Point2f pos = _currSnappedPosition.realuvPos;
            cv::Point2f pos = peak.uvCoord;
            NavGraph::Node n = _navGraph->getNode(nodeId);
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
        
        locore::LocalizationSystem::PeakYaw_t computeYaw(locore::PeakDetector::peak_t peak, const locore::VIOMeasurements& vioData){
            if (peak.valid)
                return _locSystem->getYawAtPeak(0.5);
            else
                return locore::LocalizationSystem::PeakYaw_t();
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
                cv::putText(_navigationImage,std::to_string(_path[i]), cv::Point2i(npos1.y, npos1.x), cv::FONT_HERSHEY_PLAIN, 1., cv::Scalar(255,0,0));
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
            
            if (loggingEvent || _locSystem->signDetected){
                if (_locSystem->signDetected)
                    logCounter = 5;
                loggingEvent = true;
                // we dump the previous log info
                std::ofstream myfile;
                std::string filename = outFolder + "/" + std::to_string(cnt) + ".txt";
                myfile.open (filename);
                std::cerr << "Logging to file";
                if (myfile.is_open()){
                    myfile << _locSystem->getParticlesLog();
                }
            }
            // prepare new string, every time
//            prevParticlesLog.clear();
            prevParticlesLog.str(std::string());
            
            prevParticlesLog.str(_locSystem->getParticlesLog());
            
            if (loggingEvent && !_locSystem->signDetected){
                logCounter--;
                loggingEvent = (logCounter >= 0);
            }
                
        }
        
        
        locore::ParticleFilter::ParticlesStats getParticlesStats(){
            return _locSystem->getParticlesStats();
        }
        
        
        void updateCurrentFloor(int floorNumDelta){
            _mapManager->currentFloor += floorNumDelta;
        }
        
        
        inline cv::Mat getNavigationGraph() { return _navigationImage; }
    
        
    private:
        
        bool _exploreMode;
        bool _firstCourseEstimate;
        float _distanceMoved;
        float _refAngle;
        float _motionThreshold;
        bool _goingBackward;
        float _prevPeakYaw;
        navigation_t navData;
        std::ostringstream prevParticlesLog;
        bool loggingEvent;
        int logCounter;
        
        locore::PeakDetector::peak_t prevPeak, peak;
        locore::LocalizationSystem::PeakYaw_t peakYaw;
        Course_t course;
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
            else if (diffAngleDeg >= -20 && diffAngleDeg <= 20){
                if (toDestination)
                    return ForwardToDest;
                return Forward;
                
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
        
        
        NavGraph::Node getNextTurnNode(){
            NavGraph::Node node, destNode;
            if (_path[0] == _currSnappedPosition.srcNodeId){
                for (int i=0; i < _path.size()-2; i++){
                    node = _navGraph->getNode(_path[i]);
                    float angle1 = node.edges[_path[i+1]].angleDeg;
                    node = _navGraph->getNode(_path[i+1]);
                    float angle2 = node.edges[_path[i+2]].angleDeg;
                    float diffAngle = atan2(sin((angle1-angle2)*CV_PI/180), cos((angle1-angle2)*CV_PI/180));
                    
                    if (abs(diffAngle*180/CV_PI) > 20){
                        std::cerr << "Diff Angle next turn: " << abs(diffAngle*180/CV_PI) <<  "ID: " << _path[i] << "\n";
                        return _navGraph->getNode(_path[i+1]);
                    }
                }
            }
            else{
                node = _navGraph->getNode(_currSnappedPosition.srcNodeId);
                float angle1 = node.edges[_path[0]].angleDeg;
                for (int i=0; i < _path.size()-1; i++){
                    node = _navGraph->getNode(_path[i]);
                    float angle2 = node.edges[_path[i+1]].angleDeg;
                    
                    float diffAngle = atan2(sin((angle1-angle2)*CV_PI/180), cos((angle1-angle2)*CV_PI/180));
                    
                    if (abs(diffAngle*180/CV_PI) > 20){
                        std::cerr << "*Diff Angle next turn: " << abs(diffAngle*180/CV_PI) << "ID: " << _path[i] << "\n";
                        return _navGraph->getNode(_path[i]);
                    }
                    angle1 = angle2;
                }
            }
            std::cerr << "turn node not found." << "\n";
            return node;
        }
    };
    
} /* namespace navgraph */

#endif /* NavigationUI_hpp */
