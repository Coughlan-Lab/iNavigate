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

Author: Giovanni Fusco - giofusco@ski.org

*/



#include "DetectionParams.h"

using namespace compvis;

/*!
* Basic constructor. The state will not be valid,
* unless loadFromFile is successively called.
*/
DetectionParams::DetectionParams() :
init_(false)
{
}


/*!
* Constructor with initialization
* @param[in] yamlConfigFile the full path to the configuration file to parse
* @param[in] classifiersFolder the full path to the classifiers folder if not defined at runtime
* @exception std::runtime_error error accessing or parsing the configuration file
*/
DetectionParams::DetectionParams(const std::string& yamlConfigFile, const std::string& classifiersFolder) throw(std::runtime_error) :
DetectionParams()
{
	loadFromFile(yamlConfigFile, classifiersFolder);
}


/*!
* Destructor
*/
DetectionParams::~DetectionParams() = default;


/*!
* Initialize the parameters using a configuration file
* @param[in] yamlConfigFile the full path to the configuration file to parse
* @param[in] classFolder the full path to the classifiers folder if not defined at runtime
* @exception std::runtime_error error accessing or parsing the configuration file
*/
void DetectionParams::loadFromFile(const std::string& yamlConfigFile, const std::string& classFolder) throw(std::runtime_error){

	try{
		cv::FileStorage fs(classFolder + "/" + yamlConfigFile, cv::FileStorage::READ);
		configFileName = yamlConfigFile;

		if (!fs.isOpened())
		{
			throw std::runtime_error("CONFIG PARSER ERROR :: Couldn't load configuration file: " + yamlConfigFile + "\n");
		}
		//if the folder containing the classifiers is not specified read it from the config file
		if (classFolder.empty()){
			classifiersFolder = (std::string)fs["ClassifiersFolder"];
			if (classifiersFolder.empty())
				throw (std::runtime_error("CONFIG PARSER ERROR :: Classifiers Folder not specified. \n"));
		}
		else classifiersFolder = classFolder;

		//replace separators with the right ones
		fixPathString(classifiersFolder);

		cascadeFile = (std::string)fs["CascadeFile"];
		if (cascadeFile.empty())
			throw (std::runtime_error("CONFIG PARSER ERROR :: Cascade Classifier not specified. \n"));
		cascadeFile = classifiersFolder + cascadeFile; //full filename

		svmModelFile = (std::string)fs["SVMFile"];
		if (svmModelFile.empty())
			throw (std::runtime_error("CONFIG PARSER ERROR :: SVM Classifier not specified. \n"));
		svmModelFile = classifiersFolder + svmModelFile; //full filename

		svmModelFile2 = (std::string)fs["SVMFile2"];
		if (svmModelFile2.empty()){
			//throw (std::runtime_error("CONFIG PARSER ERROR :: SVM Classifier not specified. \n"));
			use3Stages_ = false;
            std::string tmpLbl = (std::string)fs["Label"];
            labels.push_back(tmpLbl);
		}
		else{
			svmModelFile2 = classifiersFolder + svmModelFile2; //full filename
			std::string tmpNeg = (std::string)fs["negLabel"];
			std::string tmpPos = (std::string)fs["posLabel"];
			if (tmpNeg.empty() || tmpPos.empty())
				throw (std::runtime_error("CONFIG PARSER ERROR :: Labels for 3rd stage are not specified. \n"));
			else{
				labels.push_back(tmpNeg);
				labels.push_back(tmpPos);
			}
			use3Stages_ = true;
		}

		cv::FileNode n, n2;
		n = fs["minWinSize"];
		if (n.empty())
		{
			throw std::runtime_error("Parser Error :: Cascade Minimum Window Size not specified.\n");
		}
		n2 = n["width"];
		if (n2.empty())
		{
			throw std::runtime_error("Parser Error :: Cascade Minimum Window Size width not specified.\n");
		}
		cascadeMinWin.width = (int)n2;
		n2 = n["height"];
		if (n2.empty())
		{
			throw std::runtime_error("Parser Error :: Cascade Minimum Window Size height not specified.\n");
		}
		cascadeMinWin.height = (int)n2;

		n = fs["HOG_winSize"];
		if (n.empty())
		{
			throw(std::runtime_error("Parser Error :: HOG Window Size not specified.\n"));
		}
		n2 = n["width"];
		if (n2.empty())
		{
			throw std::runtime_error("Parser Error :: HOG Window Size width not specified.\n");
		}
		hogWinSize.width = (int)n2;
		n2 = n["height"];
		if (n2.empty())
		{
			throw std::runtime_error("Parser Error :: HOG Window Size height not specified.\n");
		}
		hogWinSize.height = (int)n2;


		n = fs["maxWinSizeFactor"];
		//const float cascadeMaxWinFactor = (n.empty() ? 8.f : (float)n[0]);
        const float cascadeMaxWinFactor = 5.;
		cascadeMaxWin.width = cascadeMinWin.width * cascadeMaxWinFactor;
		cascadeMaxWin.height = cascadeMinWin.height * cascadeMaxWinFactor;

		n = fs["CroppingFactors"];
		if (n.empty())
		{
			croppingFactors[0] = 1.f;
			croppingFactors[1] = 1.f;
		}
		else
		{
			n2 = n["width"];
			croppingFactors[0] = (n2.empty() ? 1.f : (float)n2);

			n2 = n["height"];
			croppingFactors[1] = (n2.empty() ? 1.f : (float)n2);
		}

		n = fs["ScaleFactor"];
		scalingFactor = (n.empty() ? 1. : (float)n);

		n = fs["CascadeScaleFactor"];
		cascadeScaleFactor = (n.empty() ? 1.1 : (float)n);

		n = fs["SVMThreshold"];
		SVMThreshold = (n.empty() ? .5 : (float)n);

		n = fs["maxAgePreConfirmation"];
		maxAgePreConfirmation = (n.empty() ? 5 : (int)n);

		n = fs["maxAgePostConfirmation"];
		maxAgePostConfirmation = (n.empty() ? 15 : (int)n);

		n = fs["nHangOverFrames"];
		nHangOverFrames = (n.empty() ? 3 : (int)n);

		init_ = true;
	}

	catch (std::exception& e)
	{
		throw(e);
	}

}

void DetectionParams::fixPathString(std::string& instring){

	char  sep = separator();

#ifdef _WIN32
	std::replace(instring.begin(), instring.end(), '/', '\\'); // replace all '/' to '\'
#else
	std::replace(instring.begin(), instring.end(), '\\', '/'); // replace all '\' to '/'
#endif
	if (instring.back() != sep)
		instring.push_back(sep);

}


