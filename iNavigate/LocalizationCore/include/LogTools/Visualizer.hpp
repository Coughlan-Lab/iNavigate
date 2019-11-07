//
//  Visualizer.hpp
//  LocalizationCore
//
//  Created by Giovanni Fusco on 11/9/18.
//  Copyright © 2018 SKERI. All rights reserved.
//

#ifndef Visualizer_h
#define Visualizer_h

#include<opencv2/opencv.hpp>
#include "../Maps/MapManager.hpp"
#include "../Core/Particle.hpp"

namespace logtools {
    
    class Visualizer{
        
    public:
        Visualizer() { ; }
        Visualizer(const maps::MapManager& mapManager) : _mapManager(mapManager) {
            plotDelay = 1;
            cv::Mat map;
            _mapManager.getWallsImage().copyTo(map);
            if (map.channels() == 1)
                cv::cvtColor(map, _mapRGB, cv::COLOR_GRAY2RGB);
            
        }
        
        const void showParticles(const std::vector<locore::Particle>& particles){
            cv::Mat lmap;
            _mapRGB.copyTo(lmap);
            for(const locore::Particle& p : particles){
                cv::Point2i pt = _mapManager.uv2pixels(cv::Vec2d(p.getPositionVector()));
                if (p.valid)
                    cv::circle(lmap, cv::Point(pt.y, pt.x), 4, cv::Scalar(0,0,255));
                else
                    cv::circle(lmap, cv::Point(pt.y, pt.x), 4, cv::Scalar(128,0,128));
            }
            cv::imshow("Particles", lmap);
            cv::waitKey(plotDelay);
        }
        
        int plotDelay;
        
    private:
        maps::MapManager _mapManager;
        cv::Mat _mapRGB;
            
    };
} // ::logtools

#endif /* Visualizer_h */
