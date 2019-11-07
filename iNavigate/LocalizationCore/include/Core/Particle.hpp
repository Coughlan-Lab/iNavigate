
#if !defined(PARTICLE_HPP_)
#define PARTICLE_HPP_

#include <opencv2/opencv.hpp>
#include "VIOMeasurements.hpp"

namespace locore{

class Particle{

    public:
        Particle() { ; }
    Particle(cv::Vec2d position, double cameraYaw, double motionYaw, double globalScaleCorrectionFactor) : position(position), cameraYaw(cameraYaw), course(motionYaw), globalScaleCorrectionFactor(globalScaleCorrectionFactor) { score = 1.; valid = true;}
        ~Particle(){;}
    
        inline cv::Vec2d getPositionVector() const      { return position;  }
        inline double getCameraYaw()              { return cameraYaw; }
        inline double getCourse()              { return course; }
        inline double getScore()                  { return score;     }
        inline double getPositionX() const         { return position[0]; } // position along map height
        inline double getPositionY() const          { return position[1]; } // position along map width
        inline cv::Point2d getPositionPoint() const { return cv::Point2d(position[0], position[1]);}

        bool valid;
        double score;
        double cameraYaw;
        double course; //heading of motion
        double globalScaleCorrectionFactor;
        cv::Vec2d position;
        double bestEstimatedDistance;
        double bestPredYaw;
        double bestDetYaw;
        std::map<std::string, std::vector<double>> candidateScores;
        std::map<double, double> estimatedDistances;
        std::map<double, double> predYaws;
        std::map<double, double> detYaws;
};

} // ::locore

#endif // PARTICLE_HPP_
