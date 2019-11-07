//
//  InitPolicy.h
//  LocalizationCore
//
//  Created by Giovanni Fusco on 11/8/18.
//  Copyright © 2018 SKERI. All rights reserved.
//

#ifndef InitPolicy_h
#define InitPolicy_h

#include "../Core/VIOMeasurements.hpp"
#include "../Core/Particle.hpp"
#include "../Maps/MapManager.hpp"

namespace initpol {
    
    class InitPolicy{
  
    public:
        InitPolicy(){;}
        virtual ~InitPolicy(){;}
        virtual bool initializeParticles(int numParticles, locore::VIOMeasurements vioData, std::vector<locore::Particle>& particles, std::shared_ptr<maps::MapManager> mapManager, float minGlobalScaleFactor, float maxGlobalScaleFactor) = 0;
        inline void setInitialMotionYaw(float yaw, float noise) { _yaw = yaw; _yawNoise = noise; }
        void setRadius(float radiusMt) { _radiusMeters = radiusMt; }
        void setInitialPosition(cv::Point2i posPx) { _initPos = cv::Point2i(posPx.x, posPx.y); }
        void setInitialPosition(cv::Point2f posMt, std::shared_ptr<maps::MapManager> mapManager) {
            _initPos = mapManager->uv2pixels(posMt);
            _initPosMt = posMt;
        }
    protected:
        float _yaw;
        float _yawNoise;
        float _radiusMeters;
        cv::Point2i _initPos;
        cv::Point2d _initPosMt;
    };
}


#endif /* InitPolicy_h */
