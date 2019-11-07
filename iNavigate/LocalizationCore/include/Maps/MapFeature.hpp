#if !defined(MAPFEATURE_HPP_)
#define MAPFEATURE_HPP_

#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace maps{

    enum FeatureType {ARUCO, EXIT_SIGN};
    
    
struct MapFeature{

    int id;
    FeatureType type;
    cv::Point2d position;
    cv::Vec2d normal;
    float height;
    
    cv::Vec2d getPositionVector() { return cv::Vec2d(position.x, position.y);}

    //MapFeature();
    
};

} // ::map

#endif // MAPFEATURE_HPP_
