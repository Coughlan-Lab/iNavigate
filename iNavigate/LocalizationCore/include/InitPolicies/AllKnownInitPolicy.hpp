//
//  AllKnownInitPolicy.hpp
//  iNavigate
//
//  Created by Giovanni Fusco on 4/4/19.
//  Copyright © 2019 Smith-Kettlewell Eye Research Institute. All rights reserved.
//

#ifndef AllKnownInitPolicy_h
#define AllKnownInitPolicy_h

#include <random>

#include "InitPolicy.h"
namespace initpol{
class AllKnownInitPolicy : public InitPolicy{
    
public:
    AllKnownInitPolicy() { { _radiusMeters = 2.; _initPos = cv::Point2i(0,0); _initPosMt = cv::Point2f(0,0);  _yaw = 0; _yawNoise = 0;} }
    ~AllKnownInitPolicy(){;}
    
    bool initializeParticles(int numParticles, const locore::VIOMeasurements vioData, std::vector<locore::Particle>& particles, std::shared_ptr<maps::MapManager> mapManager, float minGlobalScaleFactor, float maxGlobalScaleFactor){
        
//        cv::Size mapSizeMt = mapManager->mapSizeMeters();
        std::default_random_engine generator;
        float radiusPx = mapManager->getScale() * _radiusMeters;
        std::normal_distribution<double> dist_radius(0., radiusPx);
        std::uniform_real_distribution<double> globalScaleCorrFactorDist(minGlobalScaleFactor, maxGlobalScaleFactor);
        std::uniform_real_distribution<double> initYawNoise(-_yawNoise, _yawNoise);
        
        int particlesLeft = numParticles;
        while (particlesLeft > 0){
            int y = _initPos.y + dist_radius(generator);
            int x = _initPos.x + dist_radius(generator);
            
            cv::Point2i pt = cv::Point2i(x,y);
            cv::Point2d pt_mt = mapManager->pixels2uv(pt);
            
            //check that the location x,y is walkable
            if (mapManager->isWalkable(pt)){
                particlesLeft -= 1;
                double yawNoise = initYawNoise(generator);
                double course = _yaw + yawNoise;
                double cameraYaw = _yaw + yawNoise;
                double gscf = globalScaleCorrFactorDist(generator);
                particles.push_back(locore::Particle(pt_mt, cameraYaw, course, gscf));
            }
            
        }
        return true;
    }
};

} //end initpol namespace
#endif /* AllKnownInitPolicy_h */
