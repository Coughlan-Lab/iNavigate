#if !defined(ANNOTATEDMAP_HPP_)
#define ANNOTATEDMAP_HPP_

#include <opencv2/opencv.hpp>

#include "MapFeature.hpp"
#include "../Utils/ParseUtils.hpp"

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <map>
#include <sys/stat.h>

namespace maps{
    
    const std::string _PARSER_FEATURE_TAG = "map_feature";
    const std::string _PARSER_TYPE_TAG = "type";
    const std::string _PARSER_POSITION_TAG = "position";
    const std::string _PARSER_NORMAL_TAG = "normal";
    const std::string _PARSER_FID_TAG = "id";
    const std::string _PARSER_HEIGHT_TAG = "height";
    const std::string _PARSER_DELIMITER = ":";
    const int         _PARSER_FLOOR_BLOCK_LEN = 6;
    
    class AnnotatedMap{
        
    public:
        
        AnnotatedMap(std::string wallsImageFile, std::string walkableImageFile, std::string mapLandmarksFile, std::string roisImageFile, std::string roisDictionaryFile, float scale, std::string mapFolder, cv::Point2d offsetVU, float angleToNorth, int floorNumber){
            _mapFolder = mapFolder;
            _landmarksFile = mapFolder + '/' + mapLandmarksFile;
            _wallsImageFile = mapFolder + '/' + wallsImageFile;
            _walkableImageFile = mapFolder + '/' + walkableImageFile;
            _roisImageFile = mapFolder + '/' + roisImageFile;
            _roisDictionaryFile = mapFolder + '/' + roisDictionaryFile;
            _scale = scale;
            _angleToNorth = angleToNorth;
            _floorNumber = floorNumber;
            _offsetVU = offsetVU;
            _loadImageData();
            _loadFeatures();
            _loadRoisDictionary();
            _loadVisibilityMap();
        }
        
        AnnotatedMap(const AnnotatedMap& annotatedMap) = default;
        
        inline cv::Mat getWalkableMask() { return _walkMask; }
        inline cv::Mat getWallsImage()   { return _wallsImage; }
        
        // Convert u,v position in map to a pixel location in the map image
        cv::Point2i uv2pixels(cv::Point2d pt){ //assumption: All points in the system are stored row majow, i.e. x denotes rows
            pt += _offsetVU;
            cv::Point2i px;
            px.y = _scale * pt.y;
            px.x = _size.height - _scale * pt.x;
            return px;
        }
        
        inline cv::Size getMapSizePixels()      { return _size; }
        inline cv::Size getMapSizeMeters()      { return cv::Size(_size.width/_scale, _size.height/_scale); }
        inline bool isWalkable(cv::Point2i pt)  { if (pt.x < 0 || pt.x >= _size.height || pt.y < 0 || pt.y >= _size.width)
                                                    return false;
                                                  bool out;
                                                  _walkMask.at<unsigned char>(pt.x , pt.y) > 0 ? out = true : out = false; return out;
        }
        
        inline float getAngleToNorth() { return _angleToNorth; } // returns the angle of the map wrt true north
        
        bool isPathCrossingWalls(cv::Point2i startPt, cv::Point2i endPt){
            // iterate over pixels from startPt to endPT to check if walls are in the way of the path
            int dr = abs(startPt.x - endPt.x);
            int dc = abs(startPt.y - endPt.y);
            
            int span = std::max(dr, dc);  // if more rows than columns then loop over rows; else loop over columns
            float multiplier;
            if (span == 0)  //special case: (r1,c1) equals (r2,c2)
                multiplier = 0.;
            else
                multiplier = 1. / span;
            
            for ( int k=0; k <= span; k++){ // k goes from 0 through span; e.g., a span of 2 implies there are 2+1=3 pixels to reach in loop
                float frac = k * multiplier;
                int r = std::trunc(startPt.x + frac * (endPt.x - startPt.x));
                int c = std::trunc(startPt.y + frac * (endPt.y - startPt.y));
                if (isWallAt(cv::Point(r, c))>0)
                    return true;
            }
            return false;
        }
        
        
        inline bool isWallAt(cv::Point2i pt){
            if (pt.x < 0 || pt.x >= _size.height || pt.y < 0 || pt.y >= _size.width) return true;
            bool out; _wallsImage.at<uchar>(pt.x, pt.y) > 0 ? out = true : out = false; return out;
        }
        
        inline std::multimap<FeatureType, MapFeature> getLandmarksList() const { return _landmarks; }
        
        inline std::string getRoiLabel(int idx) {
            auto it = _roisDictionary.find(idx);
            if (it != _roisDictionary.end())
                return it->second;
            else return "";
        }
        
        cv::Mat showFeatures(){
            cv::Mat map;
            _wallsImage.copyTo(map);
            cv::cvtColor(map, map, cv::COLOR_GRAY2RGB);
            auto lmarks = _landmarks.find(maps::FeatureType::EXIT_SIGN);
            
            for(auto mf = lmarks; mf != _landmarks.end(); mf++){
                cv::Point2d pt = cv::Point2d(mf->second.position.x, mf->second.position.y);
                cv::Point2i px = uv2pixels(pt);
                cv::circle(map, cv::Point(px.y,px.x)  , 3, cv::Scalar(0,150,255));
                cv::circle(map, cv::Point(px.y,px.x)  , 1, cv::Scalar(0,150,255));
                std::string posstr = std::to_string(mf->second.position.x) + ", " + std::to_string(mf->second.position.y);
                cv::putText(map, posstr, cv::Point(px.y,px.x), cv::FONT_HERSHEY_PLAIN, .65, cv::Scalar(0,255,0));
            }
            //cv::imshow("Features", map);
            return map;
        }
        
        cv::Point2d pixels2uv(cv::Point2i pt){
            cv::Point2d pt_mt;
            pt_mt.y = (pt.y) / _scale;
            pt_mt.x = (_size.height - pt.x) / _scale;
            return pt_mt - _offsetVU;
        }
        
        cv::Point2d pixels2uvDouble(cv::Point2d pt){
            cv::Point2d pt_mt;
            pt_mt.y = (pt.y) / _scale;
            pt_mt.x = (_size.height - pt.x) / _scale;
            return pt_mt - _offsetVU;
        }
        
        inline double getScale() { return _scale; }
        
        cv::Mat getWallsImageRGB() { return _wallsImageRGB; }
        
        inline int getRoiAt(cv::Point2i pt) { return (int) _roisImage.at<unsigned char>(pt.x, pt.y); }
        
          // warning: x and y are swapped
        bool isSignVisibleFrom(cv::Point2i pt, int signId){
            int idx = pt.y + (pt.x*_wallsImage.cols);
            return _visMap[idx][signId];
        }
        
//        std::string getClosestPOI(cv::Point2i pt){
//            cv::Point2d pt_mt = pixels2uv(pt);
//            cv::Point2d featPt;
//            double minDist = 1e6;
//            double thrDist = 1.5;
//            std::string label = " ";
//            for (auto it = _landmarks.begin(); it != _landmarks.end(); ++it){
//                if (it->first != EXIT_SIGN){
//                    double res = cv::norm(pt_mt-it->second.position);
//                    if (res < minDist && res <= thrDist){
//                        minDist = res;
//                        label = it->second.id;
//                    }
//                }
//            }
//            
//            if (minDist <= thrDist)
//                return "Near " + label;
//            else return " ";
//        }
        
        cv::Point2d getOffsetVU() { return _offsetVU; }
        
    private:
        
        double _scale;
        float _angleToNorth;
        
        std::string _landmarksFile;
        std::string _wallsImageFile;
        std::string _walkableImageFile;
        std::string _roisImageFile;
        std::string _roisDictionaryFile;
        std::string _mapFolder;
        
        cv::Mat _wallsImage;
        cv::Mat _walkMask;
        cv::Mat _wallsImageRGB;
        cv::Mat _roisImage;
        std::vector<std::vector<bool>> _visMap;
        
        cv::Size _size;
        cv::Point2d _offsetVU;
        int _floorNumber;
        
        std::map<int, std::string> _roisDictionary;
        
        std::multimap<FeatureType, MapFeature> _landmarks;
        
        
        
        void _loadImageData(){
            _wallsImage = cv::imread(_wallsImageFile, cv::IMREAD_GRAYSCALE);
            cv::cvtColor(_wallsImage, _wallsImageRGB, cv::COLOR_GRAY2RGB);
            _walkMask   = cv::imread(_walkableImageFile, cv::IMREAD_GRAYSCALE);
            _size    = cv::Size(_wallsImage.cols, _wallsImage.rows);
            _roisImage = cv::imread(_roisImageFile, cv::IMREAD_GRAYSCALE);
            //cv::threshold(_wallsImage, _wallsImage, 0, 255, cv::THRESH_BINARY);
            //cv::threshold(_walkMask, _walkMask, 0, 255, cv::THRESH_BINARY);
            //cv::imshow("WALK", _walkMask);
        }
        
        void _loadRoisDictionary() {
            std::ifstream inFile(_roisDictionaryFile, std::ifstream::in);
            std::string strLine;
            while (inFile.good()){
                getline(inFile, strLine);
                parseutils::IndexValuePair indexValPair = parseutils::splitIndexValue(strLine, ",");
                _roisDictionary.insert(indexValPair);
            }
        }
        
        void _loadFeatures(){ _parseYamlFeaturesFile();}
        
        MapFeature _initMapFeature(std::ifstream& inFile){
            int lineCnt = 0;
            std::string strLine;
            MapFeature mapFeature;
            
            while (lineCnt < _PARSER_FLOOR_BLOCK_LEN){
                getline(inFile, strLine);
                 if (strLine.size() > 0){
                    parseutils::KeyValuePair keyValPair = parseutils::splitKeyValue(strLine, _PARSER_DELIMITER);
                    
                    if (keyValPair.first == _PARSER_TYPE_TAG){
                        std::size_t found = keyValPair.second.find("aruco");
                        if (found!=std::string::npos)
                            mapFeature.type = ARUCO;
                        else if (keyValPair.second == "exit_sign")
                            mapFeature.type = EXIT_SIGN;
                    }
                    
                    else if (keyValPair.first == _PARSER_FID_TAG)
                        mapFeature.id = std::stoi(keyValPair.second);
                    
                    else if (keyValPair.first == _PARSER_POSITION_TAG){
                        std::vector<double> v = parseutils::parseCSVArray<double>(keyValPair.second);
                        mapFeature.position = cv::Point2d(v[1], v[0]) - _offsetVU; // the features file has the coord swapped
                    }
                    
                    else if(keyValPair.first == _PARSER_NORMAL_TAG){
                        std::vector<int> v = parseutils::parseCSVArray<int>(keyValPair.second);
                        mapFeature.normal = cv::Vec2i(v[0], v[1]);
                    }
                    else if (keyValPair.first == _PARSER_HEIGHT_TAG)
                        mapFeature.height = std::stof(keyValPair.second);
                    lineCnt++;
                 }
            }
            return mapFeature;
        }
        
        // TODO: implement parser to populate _landmarks
        void _parseYamlFeaturesFile(){
            
            std::ifstream inFile(_landmarksFile, std::ifstream::in);
            std::string strLine;
            
            while (inFile.good()){
                getline(inFile, strLine);
                 if (strLine.size() > 0){
                    parseutils::KeyValuePair keyValPair = parseutils::splitKeyValue(strLine, _PARSER_DELIMITER);
                    
                    // parse map features
                    if (keyValPair.first == _PARSER_FEATURE_TAG){
                        MapFeature mf = _initMapFeature(inFile);
                        _landmarks.insert(std::make_pair(mf.type, mf));
                    }
                 }
                //std::cout << strLine << std::endl;
            }
            
            inFile.close();
        }
        
        
        void _loadVisibilityMap(){
            // TODO: iterate over _landmarks keys
            auto itr_start = _landmarks.find(maps::FeatureType::EXIT_SIGN);
            size_t numFeatures = _landmarks.count(maps::FeatureType::EXIT_SIGN);
            std::string mapFile = "exit_signs_visibility_" + std::to_string(_floorNumber) + ".vis";
            struct stat buffer;
            if (stat(mapFile.c_str(), &buffer) != 0){
                // create visibility map
                int width = _wallsImage.cols;
                _visMap = std::vector<std::vector<bool>> (_wallsImage.rows*_wallsImage.cols, std::vector<bool>(numFeatures, false));
                for (int i =0; i < _wallsImage.rows*_wallsImage.cols; i++){
                    cv::Point2i pt = cv::Point2i((i%width), i/width);
                    for (auto itr = itr_start; itr != _landmarks.end(); itr++){
                        cv::Point2i signPos = uv2pixels(itr->second.position);
                        float dist = cv::norm(cv::Vec2d(pt.x, pt.y) - itr->second.getPositionVector());
                        if (dist > 0.5 && dist < 15.0){
                            _visMap[i][itr->second.id] = !isPathCrossingWalls(pt, signPos);
                        }
                    }
                }
                
            }
        }
        
    };
    
} // ::map

#endif // ANNOTATEDMAP_HPP_
