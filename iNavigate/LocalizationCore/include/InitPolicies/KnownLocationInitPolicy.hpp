//
//  KnownLocationInitPolicy.h
//  LocalizationCore
//
//  Created by Giovanni Fusco on 1/25/19.
//  Copyright © 2019 SKERI. All rights reserved.
//

#ifndef KnownLocationInitPolicy_h
#define KnownLocationInitPolicy_h

#include <random>

#include "InitPolicy.h"

namespace initpol {
    
    class KnownLocationInitPolicy : public InitPolicy{
        
    public:
        KnownLocationInitPolicy(){ _radiusMeters = 2.; _initPos = cv::Point2i(0,0); _initPosMt = cv::Point2d(0,0);}
        ~KnownLocationInitPolicy(){;}
        
        
        
        // out: particles vector
        bool initializeParticles(int numParticles, const locore::VIOMeasurements vioData, std::vector<locore::Particle>& particles, std::shared_ptr<maps::MapManager> mapManager, float minGlobalScaleFactor, float maxGlobalScaleFactor){
            std::default_random_engine generator;
            
            float radiusPx = mapManager->getScale() * _radiusMeters;
            std::normal_distribution<double> dist_radius(0., radiusPx);
            std::uniform_real_distribution<double> dist_yaw(0.0, 2 * CV_PI);
            std::uniform_real_distribution<double> globalScaleCorrFactorDist(minGlobalScaleFactor, maxGlobalScaleFactor);
            
            int particlesLeft = numParticles;
            while (particlesLeft > 0){
                double y = _initPos.y + dist_radius(generator);
                double x = _initPos.x + dist_radius(generator);
                
                //check that the location x,y is walkable
                cv::Point2i pt = cv::Point2i(x,y);
                cv::Point2d pt_mt = mapManager->pixels2uv(pt);
                
                if (mapManager->isWalkable(pt)){
                    particlesLeft -= 1;
                    double cameraYaw = dist_yaw(generator);
                    cameraYaw = 0;
//                    double course = dist_yaw(generator);
                    double course = cameraYaw;
                    double gscf = globalScaleCorrFactorDist(generator);
                    particles.push_back(locore::Particle(pt_mt, cameraYaw, course, gscf));
                }
                
            }
            return true;
        }        
    };
} // ::initpol

#endif /* KnownLocationInitPolicy_h */
