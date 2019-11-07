/*
Copyright 2015 The Smith-Kettlewell Eye Research Institute
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Author:    Giovanni Fusco & Ender Tekin

*/

#ifndef DETECTION_PARAMS_H
#define DETECTION_PARAMS_H


#include <exception>
#include <string>
//#include <iostream>
#include "opencv2/opencv.hpp"

namespace compvis{

/** \class DetectionParams
*  \brief container of the parameters used by ObjDetector.
*  \details This class parses the YAML configuration file and
*	initializes the parameters  used by the object detector.
*/

class DetectionParams
{
public:
	DetectionParams();
    DetectionParams(const std::string& yamlConfigFile, const std::string& classifiersFolder=std::string()) throw (std::runtime_error);
	~DetectionParams();

    void loadFromFile(const std::string& yamlConfigFile, const std::string& classifiersFolder=std::string()) throw(std::runtime_error);
    
    inline bool isInit() { return init_; }  ///< @return true if parameters are properly initialized.

    std::string classifiersFolder;  ///< base directory containing the Adaboost and the SVM classifier
    std::string configFileName;     ///< full path to the configuration file
    std::string cascadeFile;        ///< Adaboost cascade classifier filename
    std::string svmModelFile;       ///< SVM model filename for second stage
	std::string svmModelFile2;      ///< SVM model filename for third stage

    cv::Size hogWinSize;            ///< windows size for HOG descriptor
    cv::Size cascadeMinWin;         ///< min window size for multi-scale detection
    cv::Size cascadeMaxWin;         ///< max window size for multi-scale detection

    //float cascadeMaxWinFactor;  ///< the scale of the maximum cascade window size (when cascadeMinWin is of scale 1)
	float croppingFactors[2];	///< (0: width, 1: height)
	float scalingFactor;		///< image rescaling factor (0., +inf)
	float cascadeScaleFactor;	///< multiscale detection scaling factor
	float SVMThreshold;			///< theshold for rejection of candidate ROIs

    int maxAgePreConfirmation;  ///< max number of frames object can be missed before being confirmed.
    int maxAgePostConfirmation; ///< max number of frames object a confirmed object can be missed without declaring lost.
    int nHangOverFrames;        ///< number of hangover frames during which detection must be confirmed
	
	inline bool useThreeStages() { return use3Stages_; }
	std::vector < std::string > labels;

private:
	bool init_;					///< specifies if the parameters have been initialized or not
	bool use3Stages_;			///< specifies whether to use an additional verification step 
	void fixPathString(std::string& instring); ///< fix path strings according to the OS
	
	
	
	inline char separator(){
		#ifdef _WIN32
			return '\\';
		#else
			return '/';
		#endif
	}

};

} // ::compvis
#endif
