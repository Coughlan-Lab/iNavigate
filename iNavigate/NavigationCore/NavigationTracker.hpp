//
//  NavigationTracker.hpp
//  iNavigate
//
//  Created by Giovanni Fusco on 6/27/19.
//  Copyright © 2019 Smith-Kettlewell Eye Research Institute. All rights reserved.
//

#ifndef NavigationTracker_h
#define NavigationTracker_h

#include <stdio.h>
#include "MapManager.hpp"
#include "LocalizationSystem.hpp"
#include "NavigationSystem.hpp"
#include "NavGraph.hpp"

namespace navgraph {
    
    class NavigationTracker{
    public:
        
        enum TurnDirection { Forward = 0, TurnAround = -1, Left = 1, Right = 2, EasyLeft = 3, EasyRight = 4, None = -2 };
        enum CompassOrientation { N = 0, NW = 1, W = 2, SW = 3, S = 4, SE =5, E = 6, NE = 7, UNDEF = -1 };
        
        NavigationTracker(){ first = true;  distanceMoved = 0;}
        ~NavigationTracker(){;}
        
        //TurnDirection update(cv::Point2f peak, NavGraph::SnappedPosition& pos, int firstNodeId, int nextNodeId){
        float update(cv::Point2f peak, NavGraph::SnappedPosition& pos, int firstNodeId, int nextNodeId){
            if (first){
                prevPosition = pos;
                prevPeak = peak;
                first = false;
                return None;
            }
            else{
                
                //float t = cv::norm(pos.uvPos - prevPosition.uvPos);
                float t = cv::norm(peak - prevPeak);
                distanceMoved += t;
                //!  WARNING: Possible confusion with reference System
                if (distanceMoved > 0.5){
                    distanceMoved = 0;
                    heading = atan2(peak.x - prevPeak.x, peak.y - prevPeak.y)*180/CV_PI;
                    prevPeak = peak;
    //                else if ((abs(t) < 0.1) && (abs(t) >0)){
    //                    heading += pos.deltaYaw*180/CV_PI;
    //                }
                    heading = fmod(heading, 360);
                   
                    // first make sure the user is walking in the right direction
//                    TurnDirection turn;
//                    if (pos.srcNodeId == firstNodeId)
//                        turn = _getTurnDirection(pos.destNode, firstNodeId, pos);
//                    else
//                        turn = _getTurnDirection(pos.srcNode, firstNodeId, pos);
                    
                    
                    prevPosition = pos;
    //                if (turn == Forward){
    //                    std::cerr << "Looking good! \n";
    //                }
//                    return turn;
                    return heading;
                }
                else
                    return 1e10;
            }
        }
        
        
    private:
        NavGraph::SnappedPosition prevPosition;
        cv::Point2f prevPeak;
        float heading;
        bool first;
        float distanceMoved;
        
        TurnDirection _getTurnDirection(NavGraph::Node srcNode, int destId, NavGraph::SnappedPosition& pos){
            NavGraph::Edge edge = srcNode.edges[destId];
            float refAngle = edge.angleDeg;
            if (refAngle < 0)
                refAngle += 360;
            if (heading <0)
                heading += 360;
            
            pos.heading = heading;
            pos.refAngle = refAngle;
            float diffAngle = atan2(sin((refAngle-heading)*3.14/180), cos((refAngle-heading)*3.14/180));
            std::cerr << "\n heading: " << heading << ", refAngle: " << refAngle << "\n";
//            std::cerr << "[][][] Heading diff: " << refAngle - heading << "\n";
            pos.headingDiff = diffAngle * 180/3.14;
//            CompassOrientation refOrientation = _angleToCompass(refAngle);
//            CompassOrientation userOrientation = _angleToCompass(heading);
//            CompassOrientation headingCorrection = _angleToCompass(diffAngle);
//            if headingCorrection
            TurnDirection turn = calculateTurn(pos.headingDiff, 15);
            return turn;
        }
        
        TurnDirection calculateTurn(float  diffAngleDeg, float threshold){
            if (diffAngleDeg > threshold && diffAngleDeg <= 45)
                return EasyLeft;
            else if (diffAngleDeg > 45 && diffAngleDeg >= 90 + threshold)
                return Left;
            else if (diffAngleDeg > 90 + threshold)
                return TurnAround;
            else if (diffAngleDeg >= -45 && diffAngleDeg < -threshold)
                return EasyRight;
            else if (diffAngleDeg >= -90 - threshold && diffAngleDeg < -45)
                return Right;
            else if (diffAngleDeg < -90 - threshold)
                return TurnAround;
            else
                return None;
        }
        
        TurnDirection calculateTurn(CompassOrientation refOrientation, CompassOrientation userOrientation){
            switch(refOrientation){
                case N:
                    if (userOrientation == N)
                        return Forward;
                    else if (userOrientation == NW)
                        return EasyLeft;
                    else if (userOrientation == W)
                        return Left;
                    else if (userOrientation == SW || userOrientation == S || userOrientation == SE)
                        return TurnAround;
                    else if (userOrientation == E)
                        return Left;
                    else if (userOrientation == NE)
                        return EasyLeft;
                    break;
                case S:
                    if (userOrientation == S)
                        return Forward;
                    else if (userOrientation == SE)
                        return EasyLeft;
                    else if (userOrientation == E)
                        return Left;
                    else if (userOrientation == NE || userOrientation == N || userOrientation == NE)
                        return TurnAround;
                    else if (userOrientation == W)
                        return Left;
                    else if (userOrientation == SW)
                        return EasyLeft;
                    break;
                case W:
                    if (userOrientation == W)
                        return Forward;
                    else if (userOrientation == NW)
                        return EasyLeft;
                    else if (userOrientation == N)
                        return Left;
                    else if (userOrientation == NE || userOrientation == E || userOrientation == SE)
                        return TurnAround;
                    else if (userOrientation == S)
                        return Right;
                    else if (userOrientation == SW)
                        return EasyRight;
                    break;
                case E:
                    if (userOrientation == E)
                        return Forward;
                    else if (userOrientation == SE)
                        return EasyLeft;
                    else if (userOrientation == S)
                        return Left;
                    else if (userOrientation == SW || userOrientation == W || userOrientation == NW)
                        return TurnAround;
                    else if (userOrientation == N)
                        return Right;
                    else if (userOrientation == NE)
                        return EasyRight;
                    break;
                case NE:
                    if (userOrientation == NE)
                        return Forward;
                    else if (userOrientation == E)
                        return EasyLeft;
                    else if (userOrientation == SE)
                        return Left;
                    else if (userOrientation == SW || userOrientation == S || userOrientation == W)
                        return TurnAround;
                    else if (userOrientation == NW)
                        return Right;
                    else if (userOrientation == N)
                        return EasyRight;
                    break;
                case SE:
                    if (userOrientation == SE)
                        return Forward;
                    else if (userOrientation == S)
                        return EasyLeft;
                    else if (userOrientation == SW)
                        return Left;
                    else if (userOrientation == NW || userOrientation == N || userOrientation == W)
                        return TurnAround;
                    else if (userOrientation == NE)
                        return Right;
                    else if (userOrientation == E)
                        return EasyRight;
                    break;
                case SW:
                    if (userOrientation == SW)
                        return Forward;
                    else if (userOrientation == W)
                        return EasyLeft;
                    else if (userOrientation == NW)
                        return Left;
                    else if (userOrientation == NE || userOrientation == N || userOrientation == E)
                        return TurnAround;
                    else if (userOrientation == SE)
                        return Right;
                    else if (userOrientation == S)
                        return EasyRight;
                    break;
                case NW:
                    if (userOrientation == NW)
                        return Forward;
                    else if (userOrientation == N)
                        return EasyLeft;
                    else if (userOrientation == NE)
                        return Left;
                    else if (userOrientation == SE || userOrientation == S || userOrientation == E)
                        return TurnAround;
                    else if (userOrientation == SW)
                        return Right;
                    else if (userOrientation == W)
                        return EasyRight;
                    break;
                default:
                    return Forward;
                    break;
            }
            return Forward;
        }
        
        CompassOrientation _angleToCompass(float angle){
            float T = 22.5;
            if (angle >= 90 - T && angle <= 90 + T)
                return N;
            else if (angle >= T && angle <= 3*T)
                return NE;
            else if ( (angle >=0 && angle<T) || (angle >= 360-T && angle <= 360) )
                return E;
            else if (angle >= 270+T && angle <= 270 + 3*T)
                return SE;
            else if (angle >= 270-T && angle <= 270+T)
                return S;
            else if (angle >= 180+T && angle <= 180+3*T)
                return SW;
            else if (angle >= 180-T && angle <= 180+T)
                return W;
            else if (angle >= 90+T && angle <= 90 + 3*T)
                return NW;
            else return UNDEF;
        }
        
    };
    
} // end navgraph namespace

#endif /* NavigationTracker_h */
