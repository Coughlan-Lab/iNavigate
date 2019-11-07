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
*/

#ifndef OBJ_DETECTOR_H
#define OBJ_DETECTOR_H


    
#include "opencv2/core/core.hpp"
#include "svm.h"

#include <time.h>
#include <vector>
#include <string>
#include <memory>

#include "DetectionParams.h"
#include "CVDetector.h"
#include "CVObservation.hpp"
#include "MedianFlowTracker.hpp"

namespace compvis{
    
/** @class ObjDetector
*   @brief Two-stages object detector.
*   @details This class defines a two stage classifier for object detection.
*    The first stage consists in a multiscale Adaboost Cascade + LBP descriptor to generate candidate ROIs.
*    The second stage is an SVM trained using HOG desciptor. It confirms or rejects candidate ROIs detected in the first stage.
*/
class ObjDetector : public CVDetector
{
    
    
    /// @class ObjDetector::CascadeDetector
    /// cascade detector using lbp features, used as first stage detector
    class CascadeDetector
    {
    public:
        
        /// Ctor
        /// @param[in] cascadeFileName name of file to load the cascade from
        /// @param[in] minWinSize minimum size of the scanning window
        /// @param[in] maxWinSize maximum size of the scanning window
        /// @param[in] scaleFactor scale factor to use for multi-scale detection
        /// @throw std::runtime_error if unable to allocate memory of read the cascade file
        CascadeDetector(const std::string& cascadeFileName, const cv::Size& minWinSize, const cv::Size& maxWinSize, float scaleFactor) throw (std::runtime_error) :
        minSz_(minWinSize),
        maxSz_(maxWinSize),
        scaleFactor_(scaleFactor),
        cascade_(cascadeFileName)
        {
            // check that cascade detector was loaded successfully
            if (cascade_.empty())
            {
                throw std::runtime_error("CascadeDetector :: Unable to load cascade detector from file " + cascadeFileName);
            }
        }
        
        /// Dtor
        ~CascadeDetector() = default;
        
        /*!
         * First stage of cascade classifier
         * @param[in] frame frame to process
         * @return a vector of detections candidates
         */
        std::vector<cv::Rect> detect(const cv::Mat& frame) const
        {
            std::vector<cv::Rect> rois;
            cascade_.detectMultiScale(frame, rois, scaleFactor_, 0, 0, minSz_, maxSz_);
            groupRectangles(rois, 1);
            return rois;
        }
        
        /*!
         * First stage of cascade classifier without grouping, overrides the min & max windows size
         * @param[in] frame frame to process
         * @return a vector of detections candidates
         */
        std::vector<cv::Rect> detectNoGrouping(const cv::Mat& frame, float scaleFactor, cv::Size minSize, cv::Size maxSize) const
        {
            std::vector<cv::Rect> rois;
            cascade_.detectMultiScale(frame, rois, scaleFactor, 0, 0, minSize, maxSize);
            return rois;
        }
        
        /*!
         * First stage of cascade classifier
         * @param[in] frame frame to process
         * @param[in] scaleFactor for multiscale detection
         * @return a vector of detections candidates
         */
        std::vector<cv::Rect> detect(const cv::Mat& frame, float scaleFactor) const
        {
            std::vector<cv::Rect> rois;
            cascade_.detectMultiScale(frame, rois, scaleFactor, 0, 0, minSz_, maxSz_);
            groupRectangles(rois, 1);
            return rois;
        }
        
        /*!
         * First stage of cascade classifier, overrides the default min & max search windows sizes
         * @param[in] frame frame to process
         * @param[in] scaleFactor for multiscale detection
         * @return a vector of detections candidates
         */
        std::vector<cv::Rect> detect(const cv::Mat& frame, float scaleFactor, cv::Size minSize, cv::Size maxSize) const
        {
            std::vector<cv::Rect> rois;
            cascade_.detectMultiScale(frame, rois, scaleFactor, 0, 0, minSize, maxSize);
            groupRectangles(rois, 1);
            return rois;
        }
    private:
        const cv::Size minSz_;  ///< min win size
        const cv::Size maxSz_;  ///< max win size
        const float scaleFactor_;     ///< scale factor
        mutable cv::CascadeClassifier cascade_;        ///< cascade classifier, mutable since cv::CascadeClassifier::detectMultiScale is not const, but does not change internal state of ObjDetector
    };  // ObjDetector::CascadeDetector
    
    
    /// @class ObjDetector::SVMClassifier
    /// svm detector using HoG features, used as second stage classifier
    class SVMClassifier
    {
    public:
        /// Ctor
        /// @param[in] svmModelFileName name of file to load the svm model from
        /// @param[in] hogWinSize size of the window to calculate HoG
        /// @throw std::runtime_error if unable to allocate memory of read the cascade file
        SVMClassifier(const std::string& svmModelFileName, const cv::Size& hogWinSize) throw (std::runtime_error) :
        hogWinSz_(hogWinSize),
        pModel_(svm_load_model(svmModelFileName.c_str())),
        hog_(hogWinSize,            //winSize
             cv::Size(16, 16),       //blockSize
             cv::Size(4, 4),         //blockStride
             cv::Size(8, 8),         //cellSize
             9,                     //nbins
             1,                     //derivAperture
             -1,                    //winSigma,
             cv::HOGDescriptor::L2Hys,  //histogramNormType,
             .2,                    //L2HysThreshold,
             true,                  //gammaCorrection
             1                      //nLevels
        )
        {
            //check that svm model was loaded successfully
            if (!pModel_)
            {
                throw std::runtime_error("SVMDetector :: Unable to load svm model from file " + svmModelFileName);
            }
        }
        
        /// Dtor
        ~SVMClassifier() = default;
        
        /*!
         * Verifies the ROIs detected in the first stage using SVM + HOG
         * @param[in] patch patch to classify
         * @return a pair of values indicating the estimated class and confidence of the patch.
         * @throw runtime error if unable to allocate memory for this stage
         */
        std::pair<int, double> classify(const cv::Mat& patch) const
        {
            std::vector<ObjDetector::DetectionInfo> result;
            double prob_est[2];
            
            //TODO: The index is set from scratch for each patch. If descriptor sizes are the same for each window, the next two can be optimized by setting it once as member variables
            std::vector<float> desc;
            std::vector<svm_node> x;
            
            cv::Mat resized(hogWinSz_, CV_32FC1);
            
            //use svm to classify the patch
            cv::resize(patch, resized, hogWinSz_);
            hog_.compute(resized, desc);
            x.resize(desc.size() + 1);
            
            for (int d = 0; d < desc.size(); d++){
                x[d].index = d + 1;  // Index starts from 1; Pre-computed kernel starts from 0
                x[d].value = desc[d];
            }
            
            x[desc.size()].index = -1;
            desc.clear();
            
            int label = round(svm_predict_probability(pModel_.get(), x.data(), prob_est));
            return std::make_pair(label, prob_est[label < 0]);
        }
    private:
        const cv::Size hogWinSz_;                   ///< min win size
        const std::unique_ptr<svm_model> pModel_;   ///< svm model
        const cv::HOGDescriptor hog_;                ///< hog feature extractor
    };  // ObjDetector::SVMDetector

    
    
public:
    /// Structure containing detection information.
    struct DetectionInfo
    {
        cv::Rect roi;           ///< detection location
        double confidence;      ///< detection confidence as estimated by the SVM
        int iLabel;                ///< label (-1, 1 if 3 stages, 0 otherwise) associated to the ROI
        std::string sLabel;        ///< string associated to the label
    };

    /// Default constructor. The parameters are not initialized.
    ObjDetector();
    
    /// constructor. The parameters are inizialized using the file passed in input and the specified classifiers folder.
    /// @param[in] yamlConfigFile config file used to load parameters from
    /// @param[in] classifiersFolder location of the classifier files (if not available in yamlConfigFile. if classifierFolder is empty, then the locations must be provided in the yamlConfigFile
    /// @throw runtime_error if there is any problem reading either the config file or the classifier files.
    ObjDetector(const std::string& yamlConfigFile, const std::string& classifiersFolder=std::string()) throw(std::runtime_error);
    
    /// Dtor
    ~ObjDetector();
    
    bool feedRotatedImage = true;
    
    /// detects SIGNs using the detectors initialized via the vonfiguraion file
    /// @param[in] frame input image
    /// @param[in] doTrack whether to use tracking (default mode). When no tracking is used each frame is considered individually. This is generally less efficient, and may lead to more false alarms.
    /// @return a set of detections
    /// TODO: even if doTrack is false, we can use tracking to see which detections may correspond to which previous detections
    /// @throw std::runtime_error if the classifiers have not been successfully initialized
    std::vector<DetectionInfo> detect(const cv::Mat& frame, bool doTrack=true) throw (std::runtime_error);
    
    
    // === INTERFACE IMPLEMENTATION OF CVDETECTOR ===
     std::vector<CVObservation> detect(const cv::Mat& image) throw (std::runtime_error);

    
    /// detects SIGNs using the detectors initialized via the vonfiguraion file
    /// @param[in] frame input image
    /// @param[out] FPS upon return containst the long-term average frames/second used for profiling
    /// @param[in] doTrack whether to use tracking (default mode). When no tracking is used each frame is considered individually. This is generally less efficient, and may lead to more false alarms.
    /// @return a set of detections
    /// TODO: even if doTrack is false, we can use tracking to see which detections may correspond to which previous detections
    /// @throw std::runtime_error if the classifiers have not been successfully initialized
    std::vector<DetectionInfo> detect(const cv::Mat& frame, double& FPS, bool doTrack=true) throw (std::runtime_error);
    
    /// initializes the parameters using the file in input.
    /// @param[in] yamlConfigFile config file used to load parameters from
    /// @param[in] classifiersFolder location of the classifier files (if not available in yamlConfigFile. if classifierFolder is empty, then the locations must be provided in the yamlConfigFile
    /// @throw runtime_error if there is any problem reading either the config file or the classifier files.
    void init(const std::string& yamlConfigFile, const std::string& classifiersFolder=std::string()) throw (std::runtime_error);
    
    /// For debugging only
    /// @return the outputs of the first stage (cascade) classifier
    inline std::vector<cv::Rect> getStage1Rois() const { return rois_;}
    
    /// For debugging only
    /// @return the outputs of the second stage (svm) classifier
    std::vector<DetectionInfo> getStage2Rois() const;
    
    /// saves ROIs coming from the first stage to disk
    /// @param[in] prefix prefix of the file names to use when saving first stage results.
    void dumpStage1(std::string prefix);
    
    /// saves ROIs coming from the second stage to disk
    /// @param[in] prefix prefix of the file names to use hen saving second stage results.G
    void dumpStage2(std::string prefix);
    
    cv::Mat currFrame; ///< last processed frame
    std::vector<DetectionInfo> rawRois;       ///< raw detections (used for debugging of refinement)

private:

    ObjDetector(const ObjDetector& that) = delete; //disable copy constructor

    void init() throw (std::runtime_error);    ///< initializes the classifiers

    std::vector<DetectionInfo> refineDetections(std::vector<DetectionInfo> rois, float scale);
    DetectionInfo refineDetection(cv::Rect roi, float scale);

    bool init_;

    //class CascadeDetector;  ///< first stage detector, LBP + Adaboost cascade
    //class SVMClassifier;     ///< second stage detector, HoG + SVM
    
    CascadeDetector* pCascadeDetector;
    SVMClassifier* pSVMClassifier;
    SVMClassifier* pSVMClassifier2;
    //std::unique_ptr<CascadeDetector> pCascadeDetector;  ///< ptr to first stage detector
    //std::unique_ptr<SVMClassifier> pSVMClassifier;      ///< ptr to second stage detector
    //std::unique_ptr<SVMClassifier> pSVMClassifier2;     ///< ptr to third stage detector
    
    DetectionParams params_;
    cv::Mat cropped_;
    
    struct TrackingInfo
    {
        cv::Rect roi;
        double confidence;
        int age;
        int nTimesSeen;
    };
    
    cv::Mat prevFrame_;
    
    
    std::vector<cv::Rect> rois_;        ///< first stage outputs
    std::vector<TrackingInfo> secondStageOutputs_;  ///< second stage outputs, objects that are potentially being tracked

    time_t start_;
    int counter_;
};

} // ::compvis
    
#endif
