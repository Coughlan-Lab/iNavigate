//
//  InitUniformPolicy.hpp
//  LocalizationCore
//
//  Created by Giovanni Fusco on 11/9/18.
//  Copyright © 2018 SKERI. All rights reserved.
//

#ifndef UniformInitPolicy_h
#define UniformInitPolicy_h

#include <random>

#include "InitPolicy.h"

namespace initpol {
    
    class UniformInitPolicy : public InitPolicy{
        
    public:
        UniformInitPolicy(){;}
        ~UniformInitPolicy(){;}
        
        
        // out: particles vector
        bool initializeParticles(int numParticles, const locore::VIOMeasurements vioData, std::vector<locore::Particle>& particles, std::shared_ptr<maps::MapManager> mapManager, float minGlobalScaleFactor, float maxGlobalScaleFactor){
            std::default_random_engine generator;
            
            int rows = mapManager->getWallsImage().rows;
            int cols = mapManager->getWallsImage().cols;
            std::uniform_real_distribution<double> dist_width(0, cols-1);
            std::uniform_real_distribution<double> dist_height(0, rows-1);
            std::uniform_real_distribution<double> globalScaleCorrFactorDist(minGlobalScaleFactor, maxGlobalScaleFactor);
            std::cerr << "Yaw noise: " << _yawNoise << "\n";
            std::uniform_real_distribution<double> initYawNoise(-_yawNoise, _yawNoise);
            std::uniform_real_distribution<double> dist_yaw(0.0, 2 * CV_PI);
            
            int particlesLeft = numParticles;
            while (particlesLeft > 0){
                double y = dist_width(generator);
                double x = dist_height(generator);
                
                //check that the location x,y is walkable
                cv::Point2i pt = cv::Point2i(x,y);
                cv::Point2d pt_mt = mapManager->pixels2uv(pt);
                
                if (mapManager->isWalkable(pt)){
                    particlesLeft -= 1;
                    double cameraYaw = dist_yaw(generator);
                    cameraYaw = 0;
                    std::cerr << "camera yaw: " << cameraYaw << "\n";
//                    double course = cameraYaw;
                    double course = dist_yaw(generator);
                    double gscf = globalScaleCorrFactorDist(generator);
                    particles.push_back(locore::Particle(pt_mt, cameraYaw, course, gscf));
                }
            
            }
            return true;
        }
    };
} // ::initpol



#endif /* UniformInitPolicy_h */
