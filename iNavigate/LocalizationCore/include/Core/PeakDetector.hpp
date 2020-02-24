//
//  PeakDetector.hpp
//  iNavigate
//
//  Created by Giovanni Fusco on 5/6/19.
//  Copyright © 2019 Smith-Kettlewell Eye Research Institute. All rights reserved.
//

#ifndef PeakDetector_h
#define PeakDetector_h

#include "Particle.hpp"
#include "../Maps/MapManager.hpp"
#include "../Utils/MatUtils.hpp"

namespace locore{

class PeakDetector{
    
public:
    
    struct peak_t {
         cv::Point2i pxCoord;
         cv::Point2d uvCoord;
         bool valid;
       };
    
    PeakDetector(std::shared_ptr<maps::MapManager> mapManager) { _mapManager = mapManager; }
        
    void computePeaks(std::vector<Particle>& particles){
        std::vector<cv::Point2i> peaks;
        _mask = cv::Mat::zeros(_mapManager->getWallsImage().size(), CV_64F);
        for (Particle& p : particles){
            cv::Point pt = _mapManager->uv2pixels(p.getPositionPoint());
            if (pt.x < _mask.rows && pt.y < _mask.cols && pt.x >= 0 && pt.y >= 0)
                _mask.at<double>(pt.x, pt.y) += p.score;
        }
        
        _mask /= _mask.total();
        cv::GaussianBlur(_mask, _mask,cv::Size(21,21), 15);
        _mask.copyTo(_heatMap);
        
        double entropy = 0;
        _hardPeak = false;
        _uncertainPeak = false;
        std::multimap<double, int> peaksMap = _prepareMapForPeaksDetection(_mask, entropy);
        std::cout << "Entropy: " << entropy << "\n";
        
        if (entropy < 7.){
            _findNpeaks(peaksMap, _mask.rows, _mask.cols * _mask.channels(), 1, 50, 25, .0001);
//            _findNpeaks(peaksMap, _mask.rows, _mask.cols * _mask.channels(), 1, 50, 25, .00005);
            //_filterPeaks(2.5);
        }
        
        
//        else if(_peaksLocation.size() == 0 && !_uncertainPeak){
////            _uncertainPeak = true;
//            _currentPeak = cv::Point2i(-1,-1);
//        }
//
//        if (_previousPeak.x > -1 && !_uncertainPeak){
//            double res = cv::norm(_currentPeak-_previousPeak);
//            if (res > 50)
//                _uncertainPeak = true;
//            else
//                _uncertainPeak = false;
//        }
        
        int sub[2];
        int size[2];
        size[0] = _mask.rows;
        size[1] = _mask.cols * _mask.channels();
        cv::Point2i maxpt = cv::Point2i(-1, -1);
        if (peaksMap.size() > 0){
            matutils::ind2sub(size, 2, peaksMap.rbegin()->second, sub);
            maxpt = cv::Point2i(sub[1], sub[0]);
        }
        
        _visualizeKDE(maxpt);
        
        if (_peaksLocation.size() > 0){
            _previousPeak = _currentPeak;
            _currentPeak.pxCoord = cv::Point2i(_peaksLocation[0].x, _peaksLocation[0].y);
            _currentPeak.uvCoord = _mapManager->pixels2uv(_currentPeak.pxCoord);
            _currentPeak.valid = true;
            //std::cerr << "Peak px: " << _currentPeak << "\n";
            if (_peaksLocation.size() == 1)
                _hardPeak = true;
        }
        else{
            _currentPeak.pxCoord = cv::Point2i(-1, -1);
            _currentPeak.uvCoord = cv::Point2f(-1, -1);
            _currentPeak.valid = false;
        }
        
    }
    
    
    inline cv::Mat getHeatMap()   { return _heatMap; }
    inline bool isPeakUncertain() { return _uncertainPeak; }
    inline peak_t getPeak()  { return _currentPeak; }
    inline bool isPeakHard()      { return _hardPeak; }
    
private:
    
    std::shared_ptr<maps::MapManager> _mapManager;
    
    peak_t _currentPeak;
    peak_t _previousPeak;
    
    bool _hardPeak;
    bool _uncertainPeak;
    cv::Mat _mask; //map used for peak detection
    cv::Mat _heatMap;
    std::vector<cv::Point2i> _peaksLocation;
    
    void _visualizeKDE(cv::Point2i topPeaksPos){
        double minV, maxV;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(_mask, &minV, &maxV, &minLoc, &maxLoc);
        _heatMap = (_mask - minV) * ( 255 / (maxV - minV) );
        _heatMap.convertTo(_heatMap, CV_8UC1);
        cv::Mat colorizedHeatMap;
        cv::applyColorMap(_heatMap, colorizedHeatMap, cv::COLORMAP_RAINBOW);
        colorizedHeatMap.copyTo(_heatMap);
        if (_peaksLocation.size() > 0){
            cv::Point2i visPeak = cv::Point2i(_peaksLocation[0].y, _peaksLocation[0].x);
            cv::circle(_heatMap, visPeak, 8, cv::Scalar(255,255,255));
            cv::circle(_heatMap, visPeak, 6, cv::Scalar(255,255,255));
            cv::circle(_heatMap, visPeak, 4, cv::Scalar(255,255,255));
            cv::drawMarker(_heatMap, visPeak,  cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 11, 2);
        }
        _heatMap = _mapManager->showFeatures() + _heatMap;
    }
    
    void _filterPeaks(float ratioThreshold){
        if (_peaksLocation.size() > 1){
            cv::Point2i ptop = cv::Point2i(_peaksLocation[0].y, _peaksLocation[0].x);
            double vtop = _mask.at<double>(ptop);
            cv::Point2i p2 = cv::Point2i(_peaksLocation[1].y, _peaksLocation[1].x);
            double v2 = _mask.at<double>(p2);
            if (vtop/v2 < ratioThreshold)
                _peaksLocation.clear();
        }
    }
    
    void _findNpeaks(std::multimap<double, int>& peaksMap, int nRows, int nCols, int numPeaks, float minDist, int maxNumTests, float threshold){
        //std::vector<cv::Point2i> peaksLocation;
        this->_peaksLocation.clear();
        
        int sub[2];
        int size[2];
        size[0] = nRows;
        size[1] = nCols;
        bool peakOK = true;
        int numTests = 0;
        for (auto rit=peaksMap.rbegin(); rit!=peaksMap.rend(); ++rit){
//            std::cout << rit->first << "\n";
            if (_peaksLocation.size() == numPeaks)
                break;
            
            else if (_peaksLocation.size() == 0 && rit->first > threshold){
//                std::cout << rit->first << " => " << rit->second << '\n';
                matutils::ind2sub(size, 2, rit->second, sub);
                _peaksLocation.push_back(cv::Point2i(sub[0], sub[1]));
            }
            else if (rit->first > threshold){
                //check spatial distance from new peak
                matutils::ind2sub(size, 2, rit->second, sub);
                cv::Point2i candidatePeak = cv::Point2i(sub[0], sub[1]);
                for (int i = 0 ; i < _peaksLocation.size(); i++){
                    double res = cv::norm(_peaksLocation[i]-candidatePeak);
                    if (res <= minDist){
                        peakOK = false;
                        numTests++;
                        break;
                    }
                }
                if (peakOK){
                    _peaksLocation.push_back(candidatePeak);
//                    std::cout << rit->first << " => " << rit->second << '\n';
                }
                if (numTests == maxNumTests)
                    break;
            }
        }
        
    }
    
    
    std::multimap<double, int> _prepareMapForPeaksDetection(cv::Mat& I, double& entropy)
    {
        std::multimap<double, int> peaksMap;
        
        int channels = I.channels();
        
        int nRows = I.rows;
        int nCols = I.cols * channels;
        
        //        if (I.isContinuous())
        //        {
        //            nCols *= nRows;
        //            nRows = 1;
        //        }
        int i,j;
        double* p, *p0, *p1;
        entropy = 0;
        for( i = 1; i < nRows-1; ++i)
        {
            p = I.ptr<double>(i);
            p0 = I.ptr<double>(i-1);
            p1 = I.ptr<double>(i+1);
            for ( j = 1; j < nCols-1; ++j)
            {
                if ((p[j] > 0) && (p[j] >= p[j-1] && p[j] >= p[j+1] && p[j] >= p0[j] && p[j] >= p0[j-1] && p[j] >= p0[j+1]
                                   && p[j] >= p1[j] && p[j] >= p1[j-1] && p[j] >= p1[j+1]) &&
                    ((p[j] > p[j-1] || p[j] > p[j+1] || p[j] > p0[j] || p[j] > p0[j-1] || p[j] > p0[j+1]
                      || p[j] > p1[j] || p[j] > p1[j-1] || p[j] > p1[j+1])) ){
                    peaksMap.insert(std::pair<double, int>(p[j], i*nCols+j));
                    
                }
                if (p[j] > 0)
                    entropy += p[j] * log(p[j]);
                
            }
        }
        entropy *= -1;
        return peaksMap;
    }
};
    
} // end namespace locore
#endif /* PeakDetector_h */
