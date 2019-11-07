#ifndef CVDETECTOR_HPP_
#define CVDETECTOR_HPP_
   
#include "CVObservation.hpp"
#include <vector>

namespace compvis{
// interface for Computer Vision based object detectors

class CVDetector{

    public:
        CVDetector(){;}
        virtual ~CVDetector(){;}
        virtual std::vector<CVObservation> detect(const cv::Mat &image) throw (std::runtime_error) = 0;
        bool feedRotatedImage;
    
};

}   // ::compvis
    
#endif /*CVDETECTOR_HPP_ */

