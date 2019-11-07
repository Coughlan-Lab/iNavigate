//
//  UniformWithYawInitPolicy.h
//  LocalizationCore
//
//  Created by Giovanni Fusco on 11/12/18.
//  Copyright © 2018 SKERI. All rights reserved.
//

#ifndef UniformWithYawInitPolicy_hpp
#define UniformWithYawInitPolicy_hpp

#include "InitPolicy.h"

namespace initpol{
    
    class UniformWithYawInitPolicy : public InitPolicy{
        
    public:
        UniformWithYawInitPolicy() { _yaw = 0; _yawNoise = 0;}
        UniformWithYawInitPolicy(float yaw, float initYawNoise) { _yaw = yaw; _yawNoise = initYawNoise; }
        ~UniformWithYawInitPolicy() { ; }
       
        // out: particles vector
        bool initializeParticles(int numParticles, const locore::VIOMeasurements vioData, std::vector<locore::Particle>& particles, std::shared_ptr<maps::MapManager> mapManager, float minGlobalScaleFactor, float maxGlobalScaleFactor){
            std::default_random_engine generator;
            
            int rows = mapManager->getWallsImage().rows;
            int cols = mapManager->getWallsImage().cols;
            std::uniform_real_distribution<double> dist_width(0, cols-1);
            std::uniform_real_distribution<double> dist_height(0, rows-1);
            std::uniform_real_distribution<double> globalScaleCorrFactorDist(minGlobalScaleFactor, maxGlobalScaleFactor);
            std::uniform_real_distribution<double> initYawNoise(-_yawNoise, _yawNoise);
            
            int particlesLeft = numParticles;
            while (particlesLeft > 0){
                int y = dist_width(generator);
                int x = dist_height(generator);
                cv::Point2i pt = cv::Point2i(x,y);
                cv::Point2d pt_mt = mapManager->pixels2uv(pt);
                double yawNoise = initYawNoise(generator);
                
                //check that the location x,y is walkable
                if (pt.x < mapManager->getWallsImage().size().height && pt.y < mapManager->getWallsImage().size().width){
                    if (mapManager->isWalkable(pt)){
                        particlesLeft -= 1;
                        double cameraYaw = _yaw + yawNoise;
                        double motionYaw = _yaw + yawNoise;
                        double gscf = globalScaleCorrFactorDist(generator);
                        particles.push_back(locore::Particle(pt_mt, cameraYaw, motionYaw, gscf));
                    }
                }
                
                
            }
            return true;
        }

    };
}

#endif /* UniformWithYawInitPolicy_hpp */
