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

Author: Giovanni Fusco - giofusco@ski.org & Ender Tekin

*/



#include "ObjDetector.h"
#include "MedianFlowTracker.hpp"
#include "svm.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cstdint>
using namespace std::chrono;

using namespace compvis;


//===========================
//
// OBJDETECTOR
//
//===========================

ObjDetector::ObjDetector() :
init_(false),
params_()
{
}

ObjDetector::ObjDetector(const std::string& yamlConfigFile, const std::string& classifiersFolder) throw(std::runtime_error) :
init_(false),
pCascadeDetector(nullptr),
pSVMClassifier(nullptr),
params_(yamlConfigFile, classifiersFolder)
{
    init();
}

void ObjDetector::init(const std::string& yamlConfigFile, const std::string& classifiersFolder) throw (std::runtime_error)
{
    params_.loadFromFile(yamlConfigFile, classifiersFolder);
    init();
};

//ObjDetector::~ObjDetector() = default;
ObjDetector::~ObjDetector() {;}

/*!
* initializes the classifiers and the HOG feature extractor.
* @exception std::runtime_error error loading one of the classfiers
*/
void ObjDetector::init() throw(std::runtime_error)
{
    counter_ = 0;
    feedRotatedImage = true;
    try
    {
        //pCascadeDetector = std::unique_ptr<CascadeDetector>(new CascadeDetector(params_.cascadeFile, params_.cascadeMinWin, params_.cascadeMaxWin, params_.cascadeScaleFactor));
        pCascadeDetector = new CascadeDetector(params_.cascadeFile, params_.cascadeMinWin, params_.cascadeMaxWin, params_.cascadeScaleFactor);
        //pSVMClassifier = std::unique_ptr<SVMClassifier>(new SVMClassifier(params_.svmModelFile, params_.hogWinSize));
        pSVMClassifier = new SVMClassifier(params_.svmModelFile, params_.hogWinSize);
        
        if (params_.useThreeStages()){
            //pSVMClassifier2 = std::unique_ptr<SVMClassifier>(new SVMClassifier(params_.svmModelFile2, params_.hogWinSize));
            pSVMClassifier2 = new SVMClassifier(params_.svmModelFile2, params_.hogWinSize);
        }

    }
    catch (std::exception& err)
    {
        throw std::runtime_error(std::string("OBJDETECTOR ERROR :: ") + err.what());
    }

    if (pCascadeDetector && pSVMClassifier) //this should be an assertion since we previosuly catch errors
        init_ = true;
}

/*!
* Use 2-Stages object detector on the input frame.
* @param[in] frame current frame to detect objects in
* @param[out] FPS frames per second
* @param[in] doTrack if true, use tracking, otherwise detection is independent between frames.
* @return a vector of DetectionInfo containing information about the detections and the frame rate
* @exception runtime_error if the detector is not properly initialized
*/
std::vector<ObjDetector::DetectionInfo> ObjDetector::detect(const cv::Mat& frame, double& FPS, bool doTrack) throw (std::runtime_error)
{
    //measure delta_T
    time_t end;
    if (counter_ == 0)
        time(&start_);
    auto result = detect(frame, doTrack);
    time(&end);
    counter_++;
    double sec = difftime(end, start_);
    FPS = counter_ / sec;
    return result;
}


/*!
* Use 2-Stages object detector on the input frame.
* @param[in] frame
* @return a vector of DetectionInfo containing information about the detections
* @exception runtime_error if the detector is not properly initialized
* @TODO: bring out the refinement step parameter
*/
std::vector<ObjDetector::DetectionInfo> ObjDetector::detect(const cv::Mat& frame, bool doTrack) throw (std::runtime_error)
{
    if (!params_.isInit())
    {
        throw std::runtime_error("OBJDETECTOR :: Parameters not initialized");
    }
    if (!init_)
    {
        throw std::runtime_error("OBJDETECTOR :: Detector not initialized");
    }
    assert(pCascadeDetector && pSVMClassifier);

    if (params_.scalingFactor != 1 && params_.scalingFactor > 0)
        resize(frame, frame, cv::Size(), params_.scalingFactor, params_.scalingFactor);
    
    frame.copyTo(currFrame);
    //cropping
    cropped_ = frame(cv::Rect(0, 0, frame.size().width * params_.croppingFactors[0], frame.size().height*params_.croppingFactors[1]));

    std::vector<DetectionInfo> result;

    //with or without tracking
    if (doTrack)    //with tracking
    {
        cv::Mat_<std::uint8_t> grayFrame;
        // track all objects that were previously detected
        if (!secondStageOutputs_.empty()) //objects being tracked
        {
            
            //cv::cvtColor(cropped_, grayFrame, cv::COLOR_BGR2GRAY);
            grayFrame = cropped_.clone();
            for (auto it = secondStageOutputs_.begin(); it != secondStageOutputs_.end();)
            {
                it->roi = trackMedianFlow(it->roi, prevFrame_, grayFrame);
                if (0 == it->roi.area())    //tracker lost object
                {
                    it = secondStageOutputs_.erase(it);
                    continue;
                }
                // attempt to confirm detections via svm.
                auto res = pSVMClassifier->classify(cropped_(it->roi));  //TODO: If SVM is using grayscale, we should just pass it the grayscale image to reduce computation
                it->confidence = res.second;
                if ((1 == res.first) && (res.second > params_.SVMThreshold)) //svm confirms detection
                {
                    
                    it->age = 0;
                }
                else    //svm did not classify patch as foreground
                {
                    ++it->age;   //increase age
                }
                ++it;
            }
        }

        // Run cascade detector
        rois_ = pCascadeDetector->detect(cropped_);
        std::vector<DetectionInfo> newDetections;
        for (const auto& det : rois_)
        {
            auto res = pSVMClassifier->classify(cropped_(det));
            if ((1 == res.first) && (res.second > params_.SVMThreshold)) //svm confirms detection
            {
                newDetections.push_back({ det, res.second });
            }
        }
        
        // Combine detections
        auto overlaps = [](const cv::Rect& r1, const cv::Rect& r2){return ((r1 & r2).area() > .5 * std::min(r1.area(), r2.area())); };   //two rectangles overlap if their intersection is greater than half the smaller
        
        for (auto& obj : secondStageOutputs_)
        {
            for (auto itDet = newDetections.begin(); itDet != newDetections.end();)
            {
                if (overlaps(obj.roi, itDet->roi))
                {
                    obj.age = 0;
                    if (itDet->confidence > obj.confidence)
                    {
                        obj.confidence = itDet->confidence;
                        obj.roi = itDet->roi;
                    }
                    itDet = newDetections.erase(itDet);
                    continue;
                }
                ++itDet;
            }
        }
        //std::cerr << "combine\n";

        // prune old detections, update the number oftimes new detections have been seen
        for (auto it = secondStageOutputs_.begin(); it != secondStageOutputs_.end();)
        {
            if (0 == it->age)
            {
                ++(it->nTimesSeen);
            }
            else
            {
                int maxAge = (it->nTimesSeen < params_.nHangOverFrames ? params_.maxAgePreConfirmation : params_.maxAgePostConfirmation);
                if (it->age > maxAge)
                {
                    it = secondStageOutputs_.erase(it);
                    continue;
                }
            }
            ++it;
        }

        //get confirmed detections
        for (const auto& obj : secondStageOutputs_)
        {
            if (obj.nTimesSeen > params_.nHangOverFrames)
            {
                result.push_back({ obj.roi, obj.confidence, 0, params_.labels.at(0) });
            }
        }
        //std::cerr << "confirmed\n";

        // add unmatched new detections
        for (const auto& det : newDetections)
        {
            secondStageOutputs_.push_back({ det.roi, det.confidence, 0, 1 });
        }

        //sort results in order of decreasing confidence (most confident first)
        std::sort(result.begin(), result.end(), [](const DetectionInfo& res1, const DetectionInfo& res2){return res1.confidence > res2.confidence; });

        if (!secondStageOutputs_.empty())   //we are tracking some objects, so save the grayscale image for next time
        {
            if (grayFrame.empty())  //no objects were tracked before
            {
                //cv::cvtColor(cropped_, prevFrame_, cv::COLOR_BGR2GRAY);
                prevFrame_ = cropped_.clone();
            }
            else
            {
                prevFrame_ = grayFrame.clone();
            }
        }
    }
    else    //no tracking
    {
        // Run cascade detector
        rois_ = pCascadeDetector->detect(cropped_);
        for (const auto& det : rois_)
        {
            auto res = pSVMClassifier->classify(cropped_(det));
            if ((1 == res.first) && (res.second > params_.SVMThreshold)) //svm confirms detection
            {
                result.push_back({ det, res.second, 0, params_.labels.at(0)});
            }
        }
    }

    
//    std::cerr << "size of filtered results: " << result.size() << std::endl;

    //if has a 3rd stage, classify the ROIs
    if (params_.useThreeStages()){
        std::vector<DetectionInfo> result2;
        for (const auto& det : result){
            auto res = pSVMClassifier2->classify(cropped_(det.roi));
            if ((1 == res.first) && (res.second > params_.SVMThreshold)) //svm labeled +1
                result2.push_back({ det.roi, res.second, 1, params_.labels.at(1)});
            else
                result2.push_back({ det.roi, res.second, -1, params_.labels.at(0)});
        }
        return result2;
    }

    else
        return result;
}



std::vector<CVObservation> ObjDetector::detect(const cv::Mat &image) throw (std::runtime_error)
{
    double fps;
    bool doTrack = false;
    std::vector<CVObservation> obs;
    
    //auto start = high_resolution_clock::now();
    
    std::vector<ObjDetector::DetectionInfo> res = this->detect(image, fps, doTrack);
    
    //auto stop = high_resolution_clock::now();
    
    //auto duration = duration_cast<microseconds>(stop - start);
    
    //std::cerr << "Time taken by ObjDetector: "
    // << duration.count() << " microseconds" << std::endl;
    
    // cv::Mat lFrame;
   // image.copyTo(lFrame);

    for ( DetectionInfo& info : res){
        obs.push_back(CVObservation(info.roi, info.sLabel, info.confidence));
        //cv::rectangle(lFrame, info.roi, cv::Scalar(255,255,0));
    }
    //cv::imshow("DET", lFrame);
    return obs;
}

std::vector<ObjDetector::DetectionInfo> ObjDetector::getStage2Rois() const
{
    std::vector<DetectionInfo> result;
    for (const auto& obj : secondStageOutputs_)
    {
        result.push_back({ obj.roi, obj.confidence });
    }
    return result;
}

void ObjDetector::dumpStage1(std::string prefix){
    int cnt = 0;
    for (const auto& r : rois_){
        cnt++;
        cv::Mat p = currFrame(r);
        std::string fname = prefix + "_" + std::to_string(counter_) + "_" + std::to_string(cnt) + ".png";
        cv::imwrite(fname, p);
    }
}

void ObjDetector::dumpStage2(std::string prefix){
    int cnt = 0;
    for (const auto& r : secondStageOutputs_){
        cnt++;
        cv::Mat p = currFrame(r.roi);
        std::string fname = prefix + "_" + std::to_string(counter_) + "_" + std::to_string(cnt) + "_" + std::to_string(r.confidence) + ".png";
        cv::imwrite(fname, p);
    }
}

std::vector<ObjDetector::DetectionInfo> ObjDetector::refineDetections(std::vector<DetectionInfo> rois, float scale){
    
    //std::cerr << "Frame # " << this->counter_ << std::endl;

    cv::Mat tmp;
    cropped_.copyTo(tmp);

    std::vector<DetectionInfo> refined_rois;
    for (const auto& r : rois){
    
        //std::cerr << r.roi << std::endl;
        
        int horSpan = floor(r.roi.width / 2 * scale);
        int vertSpan = floor(r.roi.height / 2 * scale);
        int new_x = r.roi.x - horSpan;
        int new_y = r.roi.y - vertSpan;
        if (new_x < 0)
            new_x = 0;
        if (new_y < 0)
            new_y = 0;
        
        int new_width = r.roi.width + 2*horSpan;
        int new_height = r.roi.height + 2*vertSpan;

        if (new_x + new_width > cropped_.size().width){
            new_width = cropped_.size().width - new_x;
        }
        if (new_y + new_height > cropped_.size().height){
            new_height = cropped_.size().height - new_y;
        }
        
        cv::Mat patch = cropped_(cv::Rect(new_x,new_y,new_width, new_height));
        //imshow("Patch", patch);
        //std::cerr << "# " << counter_ << std::endl;
        //std::vector<cv::Rect> det = pCascadeDetector->detectNoGrouping(patch, 1.01, params_.cascadeMinWin, patch.size());
        std::vector<cv::Rect> det = pCascadeDetector->detect(patch, 1.01, params_.cascadeMinWin, patch.size());

        //std::cerr << "Size of det: " << det.size() << std::endl;

        std::vector<DetectionInfo> result;

        for (const auto& d : det){
            auto res = pSVMClassifier->classify(patch(d));  //TODO: If SVM is using grayscale, we should just pass it the grayscale image to reduce computation
            
            if ((1 == res.first) && (res.second > params_.SVMThreshold)){ //svm confirms detection
                //cv::Rect tmp_roi(d.x + new_x, d.y + new_y, d.width, d.height);
                cv::Rect tmp_roi(d.x + new_x, d.y + new_y, d.width, d.height);
                result.push_back({ tmp_roi, res.second, 0 });
            }
        }

        //find roi with max confidence
        float max_conf = -1;
        std::vector<ObjDetector::DetectionInfo>::iterator best_it;
        std::vector<ObjDetector::DetectionInfo>::iterator it = result.begin();
        for (; it != result.end(); ++it){
            if (it->confidence > max_conf){
                max_conf = it->confidence;
                best_it = it;
            }
        }

        //debug
        
        cv::rectangle(tmp, r.roi, cv::Scalar(255, 0, 0), 2);

        if (max_conf > 0){
            cv::rectangle(tmp, best_it->roi, cv::Scalar(0, 255, 0), 2);
            refined_rois.push_back(*best_it);
        }
        
    
        //refined_rois = result; //DEBUG
    }
    //std::cerr << "size of ref. " << refined_rois.size() << std::endl;
    cv::imshow("Refinement", tmp);
    
    

    return refined_rois;

}


ObjDetector::DetectionInfo ObjDetector::refineDetection(cv::Rect roi , float scale){

    //std::cerr << "Frame # " << this->counter_ << std::endl;

    DetectionInfo refined_roi;

    cv::Mat tmp;
    cropped_.copyTo(tmp);
    cv::rectangle(tmp, roi, cv::Scalar(255, 0, 0), 2);
        //std::cerr << r.roi << std::endl;

        int horSpan = floor(roi.width / 2 * scale);
        int vertSpan = floor(roi.height / 2 * scale);
        int new_x = roi.x - horSpan;
        int new_y = roi.y - vertSpan;
        if (new_x < 0)
            new_x = 0;
        if (new_y < 0)
            new_y = 0;

        int new_width = roi.width + 2 * horSpan;
        int new_height = roi.height + 2 * vertSpan;

        if (new_x + new_width > cropped_.size().width){
            new_width = cropped_.size().width - new_x;
        }
        if (new_y + new_height > cropped_.size().height){
            new_height = cropped_.size().height - new_y;
        }

        cv::Mat patch = cropped_(cv::Rect(new_x, new_y, new_width, new_height));
        //cv::imshow("Patch", patch);
        //std::cerr << "# " << counter_ << std::endl;
        //std::vector<cv::Rect> det = pCascadeDetector->detectNoGrouping(patch, 1.01, params_.cascadeMinWin, patch.size());
        std::vector<cv::Rect> det = pCascadeDetector->detect(patch, 1.01, params_.cascadeMinWin, patch.size());

        //std::cerr << "Size of det: " << det.size() << std::endl;

        std::vector<DetectionInfo> result;

        for (const auto& d : det){
            auto res = pSVMClassifier->classify(patch(d));  //TODO: If SVM is using grayscale, we should just pass it the grayscale image to reduce computation

            if ((1 == res.first) && (res.second > params_.SVMThreshold)){ //svm confirms detection
                //cv::Rect tmp_roi(d.x + new_x, d.y + new_y, d.width, d.height);
                cv::Rect tmp_roi(d.x + new_x, d.y + new_y, d.width, d.height);
                result.push_back({ tmp_roi, res.second, 0 });
            }
        }

        //find roi with max confidence
        float max_conf = -1;
        std::vector<ObjDetector::DetectionInfo>::iterator best_it;
        std::vector<ObjDetector::DetectionInfo>::iterator it = result.begin();
        for (; it != result.end(); ++it){
            if (it->confidence > max_conf){
                max_conf = it->confidence;
                best_it = it;
            }
        }
        if (max_conf > 0){
            refined_roi = { best_it->roi, max_conf, 0 };
            cv::rectangle(tmp, best_it->roi, cv::Scalar(0, 255, 0), 2);
        }
        else
            refined_roi = { roi, -1, 0 };
        //refined_rois = result; //DEBUG
        cv::imshow("Single ref", tmp);
    //std::cerr << "size of ref. " << refined_rois.size() << std::endl;
    return refined_roi;

}


