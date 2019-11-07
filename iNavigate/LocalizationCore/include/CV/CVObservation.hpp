#ifndef CVOBSERVATION_HPP_
#define CVOBSERVATION_HPP_

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>

namespace compvis{

class CVObservation{

    public:
        CVObservation(cv::Rect roi, std::string tag, float score, cv::Size imgSize = cv::Size(640,480)) : _roi(roi), _tag(tag), _score(score), _imageSize(imgSize)
                        { observationType = "CV";};
        CVObservation(cv::Rect roi, std::string tag, std::string id, std::vector<cv::Vec3d> tvec, std::vector<cv::Vec3d> rvec, cv::Size imgSize = cv::Size(640,480)) : _roi(roi),
                                                                _tag(tag), _score(1.), _id(id), _tvec(tvec), _rvec(rvec), _imageSize(imgSize) { observationType = "CV";};
    
        CVObservation(std::string tag, std::string id, std::vector<cv::Point2f> corners,  cv::Size imgSize = cv::Size(640,480)) : _tag(tag), _score(1.),
                                                               _id(id), _corners(corners), _imageSize(imgSize) { observationType = "CV";};
    
        inline cv::Rect getROI() const { return _roi; }
        inline cv::Point getROICenter() const { cv::Point center( _roi.x+(_roi.width)/2, _roi.y+(_roi.height/2 ) ); return center; }
    
        inline float getScore() const { return _score; }
    
        std::vector<cv::Vec3d> getRotationVector(){
            assert(_rvec.size() >0);
            return _rvec;
        };
    
        std::vector<cv::Vec3d> getTranslationVector(){
            assert(_tvec.size() >0);
            return _tvec;
        };
    
        std::vector<cv::Point2f> getCorners(){
            assert(_corners.size()>0);
            return _corners;
        }
    
        inline std::string detectionID() const { return _id; }
        inline std::string tag() const { return _tag; }
    
        inline cv::Size getImageSize() const    { return _imageSize; }
        std::string observationType;

      private:
        cv::Rect _roi;
        std::string _tag;
        double _score;
        std::string _id;
        std::vector<cv::Vec3d> _rvec;
        std::vector<cv::Vec3d> _tvec;
        std::vector<cv::Point2f> _corners;
        cv::Size _imageSize;
        
};

} // ::compvis

#endif // CVOBSERVATION_HPP_
