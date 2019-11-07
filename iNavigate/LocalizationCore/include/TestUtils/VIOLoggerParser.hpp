//
//  VIOLoggerParser.hpp
//  LocalizationCore
//
//  Created by Giovanni Fusco on 11/6/18.
//  Copyright © 2018 SKERI. All rights reserved.
//

#ifndef VIOLoggerParser_h
#define VIOLoggerParser_h

#include "opencv2/opencv.hpp"
#include <iostream>

#include "ParseUtils.hpp"
#include "Core/VIOMeasurements.hpp"

namespace testutils{
    
    struct VIOData{
        double timestamp;
        double barometer_kPa;
        std::string trackerStatus;
        cv::Vec3d cameraRotation;
        cv::Vec3d cameraPosition;
        std::string imageFilename;
        cv::Mat rawFrame;
        
    };

// parses the text files created by VIOLogger app
class VIOLoggerParser{

    public:
        VIOLoggerParser(std::string dataFolder, bool withBarometer = true) : _dataFolder(dataFolder), _withBarometer(withBarometer){ _init();}
    
        //parse next sample from VIO log file
        locore::VIOMeasurements next(){
            VIOData data;
            
            std::string strLine;
            
            // parse timestamp
            getline(_inFile, strLine);
            if (strLine.size() > 0){
                data.timestamp = std::stod(parseutils::trim_copy(strLine));
                
                //parse image filename
                data.imageFilename = "/" + strLine + ".jpg";
                
                //load the image
                data.rawFrame = cv::imread(_dataFolder + data.imageFilename);
                cv::resize(data.rawFrame, data.rawFrame, cv::Size(480, 640));
                
                //parse tracker status
                getline(_inFile, strLine);
                data.trackerStatus = parseutils::trim_copy(strLine);
                
                // parse camera matrix -- we skip it for now
                getline(_inFile, strLine);
                
                // camera position
                getline(_inFile, strLine);
                std::vector<double> tmp = parseutils::parseCSVList<double>(strLine);
                data.cameraPosition = cv::Vec3d(tmp[0], tmp[1], tmp[2]);
                
                //parse camera rotation
                getline(_inFile, strLine);
                tmp = parseutils::parseCSVListWithType<double>(strLine);
                data.cameraRotation = cv::Vec3d(tmp[0], tmp[1], tmp[2]);
                
                
                // barometer
                if (_withBarometer == true){
                    getline(_inFile, strLine);
                    data.barometer_kPa = std::stod(parseutils::trim_copy(strLine));
                }
                else
                    data.barometer_kPa = 0.;
                
                locore::VIOMeasurements vioMeasurement(data.trackerStatus, data.timestamp, data.cameraPosition, data.cameraRotation, data.barometer_kPa, data.rawFrame);
            
                return vioMeasurement;
            }
            return locore::VIOMeasurements();
        }
    
    
    private:
        std::string _dataFolder;
        std::ifstream _inFile;
        bool _withBarometer;
    
        void _init(){
            _inFile = std::ifstream((_dataFolder + "/VIO.txt"), std::ifstream::in);
            std::string strLine;
            //skip first line, it's just a header
            if (_inFile.good())
                getline(_inFile, strLine);
        }
    
   
};
    
} //::testutils


#endif /* VIOLoggerParser_h */
