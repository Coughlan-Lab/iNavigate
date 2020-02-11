#if !defined(PARTICLEFILTER_HPP_)
#define PARTICLEFILTER_HPP_

#include <opencv2/opencv.hpp>

#include "Particle.hpp"
#include "PeakDetector.hpp"
#include "../InitPolicies/InitPolicy.h"
#include "../InitPolicies/UniformInitPolicy.hpp"
#include "../InitPolicies/UniformWithYawInitPolicy.hpp"
#include "../InitPolicies/KnownLocationInitPolicy.hpp"
#include "../InitPolicies/AllKnownInitPolicy.hpp" 
#include "../LogTools/Visualizer.hpp"
#include "../CV/CVObservation.hpp"


#include <stdio.h>
#include <random>
#include <math.h>

#include <chrono>
using namespace std::chrono;

namespace locore{
    
    enum InitMode{UNIFORM, UNIFORM_WITH_YAW, AREA_WITH_YAW, LOCATION};
    
    //using PeakLocation = cv::Point2i;
    
    class ParticleFilter{
        
    public:
        struct PFParameters{
            PFParameters() { motionYawNoise = 0.; positionNoiseMajor = 1.1; positionNoiseMinor = .0;
                cameraYawNoise = 0.000001; resamplingMotionthreshold = 0.5; minSignScore = 0.1;
                signYawDifferenceThreshold = -.7; signDetectionScoringCoeff = 0.01;
                cameraHeight = 1.2; globalScaleCorrectionMin = 1.;
                globalScaleCorrectionMax = 1.2; arkitAngleCorrection = - CV_PI/2;
            }
            
            float motionYawNoise;
            float cameraYawNoise;
            float positionNoiseMinor;
            float positionNoiseMajor;
            float resamplingMotionthreshold; // how much motion (in meters) before triggering resampling
            float minSignScore; // min score a particle gets if it is incompatible with all signs
            float signYawDifferenceThreshold; // used when checking the orientation of the particle w.r.t. a candidate sign when scoring
            float signDetectionScoringCoeff;
            float cameraHeight;
            float globalScaleCorrectionMax;
            float globalScaleCorrectionMin;
            float arkitAngleCorrection;
        };
        
            
        struct ParticlesStats{
            ParticlesStats() { gsc_min =0.; gsc_max = 0.; gsc_mean = 0.; gsc_median = 0; }
            float gsc_min;
            float gsc_max;
            float gsc_mean;
            float gsc_median;
        };
        
        PFParameters pfParameters;
        ParticlesStats partStats;
        std::ostringstream particlesLog;
        
        ParticleFilter(int numParticles, InitMode initMode, std::shared_ptr<maps::MapManager> mapManager, float initMotionYaw = 0,
                       float initYawNoise = 0, float radius_mt = 2., cv::Point2d initLocMt = cv::Point2d(0,0)) : _initMode(initMode),_numParticles(numParticles){
            _mapManager = mapManager;
            _peakDetector = std::unique_ptr<locore::PeakDetector>(new locore::PeakDetector(mapManager));
            _init(initMotionYaw, initYawNoise, radius_mt, initLocMt);
        }
        
        ParticleFilter() { ; }
        ParticleFilter(const ParticleFilter& particleFilter);
        ~ParticleFilter() { ; }
        
        inline void attachVisualizer(logtools::Visualizer& vis) { _visualizer = vis; }
        
        inline bool particlesInitialized() { return _initializedParticles; }
        
        inline bool init(const locore::VIOMeasurements& vioData, bool verbose = true){
            _particles.clear();
            bool initOK = _initPolicy->initializeParticles(_numParticles, vioData, _particles, _mapManager,
                                                           pfParameters.globalScaleCorrectionMin, pfParameters.globalScaleCorrectionMax);
            _initializedParticles = initOK;
            _totalMotion = 0;
            estimatedDistanceToSign = 0;
            return _initializedParticles;
        }
        
        // TODO: extract into camera container
        inline void setCameraAPP(double app, double fx) { _app = app; _fx = fx;}
        
        float estimatedDistanceToSign;
        
        // Peak methods
        inline bool isPeakUncertain() { return _peakDetector->isPeakUncertain(); }
        inline PeakDetector::peak_t getPeak(){ return _peakDetector->getPeak(); }
        inline cv::Mat getHeatMap(){ return _peakDetector->getHeatMap(); }
        inline bool isPeakHard(){ return _peakDetector->isPeakHard(); }
        
        void step(VIOMeasurements vioData, const std::vector<compvis::CVObservation>& detections, bool logParticles){
            //auto start = high_resolution_clock::now();
            nanoseconds deltaT = nanoseconds(0);
            this->logParticles = logParticles;
            assert(_initializedParticles);
            _badFramesCounter = 0;
            _prevVIOData = vioData;
            _updateParticles(vioData, detections);
            _totalMotion += cv::norm(cv::Vec2f(vioData.getDeltaX(), vioData.getDeltaZ()),  cv::NORM_L2);
          
            if (detections.size() > 0){
                _scoreParticles(vioData, detections);
////                _resampleParticles();
           }

            if ( (_totalMotion >= pfParameters.resamplingMotionthreshold) ||
                 ( float(_numParticles - _nonValidParticles) / float(_numParticles) < 0.5) ){
                _resampleParticles();
                _totalMotion = 0;
                _nonValidParticles = 0;
                
            }
            auto start = high_resolution_clock::now();
            _peakDetector->computePeaks(_particles);
            auto stop = high_resolution_clock::now();
            deltaT = duration_cast<nanoseconds>(stop-start);
            std::cerr << "computePeaks time : " << deltaT.count() << "\n";
        }
        
        cv::Mat plotParticlesYaw(cv::Mat& image, int arrowLen){
            for (auto p : _particles){
                double yaw = p.getCameraYaw();// + pfParameters.arkitAngleCorrection;
                
                float v = arrowLen * sin(yaw);
                float u = arrowLen * cos(yaw);
                cv::Point2d p2 = cv::Point2d(v, u);
                cv::Point2i p1px = _mapManager->uv2pixels(p.getPositionPoint());
                cv::Point2i p2px = _mapManager->uv2pixels(p.getPositionPoint()+p2);
                cv::arrowedLine(image, cv::Point2i(p1px.y, p1px.x), cv::Point2i(p2px.y, p2px.x), cv::Scalar(0,255,0));
                cv::circle(image, cv::Point2i(p1px.y, p1px.x), 3, cv::Scalar(255,0,0));            }
            return image;
        }
        
        
        
        inline const std::vector<Particle> getParticles() { return _particles; }
        
    private:
        
        std::unique_ptr<initpol::InitPolicy> _initPolicy;
        std::vector<Particle> _particles;
        std::shared_ptr<maps::MapManager> _mapManager;
        std::unique_ptr<locore::PeakDetector> _peakDetector;
        logtools::Visualizer _visualizer;
        std::default_random_engine _generator;
        std::multimap<maps::FeatureType, maps::MapFeature> features;
        std::multimap<maps::FeatureType, maps::MapFeature>::iterator exitSignsFeatureList;
        
        
        int _currentFloor;
        double  _fx; //camera focal length
        
        double _totalMotion;
        double _app; // angle per pixel - using in sign detection scoring
        int _nonValidParticles;
        
        InitMode _initMode;
        int _numParticles;
        bool _initializedParticles = false;
        bool logParticles = false;
        
        VIOMeasurements _prevVIOData;
        int _badFramesCounter;
        
        void _scoreParticles(const locore::VIOMeasurements& vioData, const std::vector<compvis::CVObservation>& detections){
            for (const compvis::CVObservation& det : detections){
//                std::cout << "DetectionID: " << det.tag() << "\n";
                if (det.tag() == "exit_sign")
                    _scoreDistanceToExitSign(vioData, det);
            }
            // this handles the possibility of multiple bounding boxes: we take the product of the scores estimated over all the different sign detections
//            for (Particle& p : _particles){
//                if (p.valid){
//                    p.score = 1;
//                    for (auto it = p.candidateScores.begin(); it != p.candidateScores.end(); ++it){
//                        p.score *= (*std::max_element(it->second.begin(), it->second.end()));
//                    }
//                }
//                p.candidateScores.clear();
//            }
            
        }
        
       

        void _scoreDistanceToExitSign(const locore::VIOMeasurements& vioData, const compvis::CVObservation& det){

            std::vector<double> yawScores;

            //estimate sign distance
            double gamma = vioData.getPitch();
            std::cerr << "Gamma: " << gamma << "\n";

            // NOTE: image width and height are swapped
            float centerRow = vioData.getFrame().size().width/2;
            float detRow = det.getROICenter().y;
            double delta = atan((centerRow - detRow)/_fx);
//            std::cerr << "delta: " << delta << "\n";
//            double z = y / tan(gamma+delta);
//            this->estimatedDistanceToSign = z;
            int columnDetection = det.getROICenter().x;
            int deltaV = det.getROI().height;
            std::cerr << "BBox height: " << deltaV << "\n";
            std::cerr << "BBox width: " << det.getROI().width << "\n";
            float h0 = 0.2032; // exit sign height in meters
//            this->estimatedDistanceToSign = (_fx/deltaV)*h0*cos(gamma);
            this->estimatedDistanceToSign = 3;

            // NOTE: image width and height are swapped (because of how ARKit handles image on the sensor)
            double fov_h = atan(vioData.getFrame().size().height/2*_fx);
            double fov_v = atan(vioData.getFrame().size().width/2*_fx);

            auto start = high_resolution_clock::now();
            nanoseconds deltaT = nanoseconds(0);
            for(auto particle = _particles.begin(); particle != _particles.end(); ++particle ){
                if (particle->valid){
                    cv::Point2i startPoint = _mapManager->uv2pixels(particle->position);

                    yawScores.clear();
                    // loop over exit signs
                    for (auto itr = exitSignsFeatureList; itr != features.end(); itr++){
//                        double y = itr->second.height - pfParameters.cameraHeight;
//                        double z = y / tan(gamma+delta);
//                        this->estimatedDistanceToSign = z;
                        float dist = cv::norm(particle->getPositionVector() - itr->second.getPositionVector());

                        double score = pfParameters.minSignScore;
                        if (_mapManager->isSignVisibleFrom(startPoint, itr->second.id)){

                            //cv::Point2i signPos = _mapManager->uv2pixels(itr->second.position);
                            // check that particle orientation is compatible with sign orientation
//                            double yaw_diff = itr->second.normal.dot( cv::Vec2d( cos( particle->cameraYaw ),
//                                                                                sin( particle->cameraYaw ) ) );
                            double yaw_diff = itr->second.normal.dot( cv::Vec2d( cos( 3.14/2 ),
                            sin( 3.14/2 ) ) );
//                            std::cerr << "yaw diff: " << yaw_diff << "\n";
//                            if (yaw_diff < pfParameters.signYawDifferenceThreshold){
                            if (yaw_diff < 0){

                                // warning: x and y are swapped
                                double thetaPred = atan2(itr->second.position.x - particle->getPositionX(), itr->second.position.y - particle->getPositionY());
                                double thetaDetection = particle->getCameraYaw() + (det.getImageSize().width/2 - columnDetection) * _app;
//                                double distScore = (this->estimatedDistanceToSign/dist - 0.7) <= (1.3 - 0.7) ? 1. : pfParameters.minSignScore;
                                double x = pow(sin((thetaPred - thetaDetection)/2),2);
//                                double azimuthScore = x < 0.03 ? 1. : pfParameters.minSignScore;

                                //does the sign fall within the horizontal and vertical field of view?
                                double azimuthScore = abs(sin(thetaDetection - thetaPred)) < sin(fov_h/2) ? 1 :pfParameters.minSignScore;
                                double betaPred = acos(itr->second.height / dist);
                                double elevationScore = abs(sin(gamma - betaPred)) < sin(fov_v/2) ? 1 : pfParameters.minSignScore;

//                                score = distScore * azimuthScore;
//                                double distErrorFraction = abs(this->estimatedDistanceToSign - dist) / this->estimatedDistanceToSign;
                                double distErrorFraction = abs(this->estimatedDistanceToSign - dist) / dist;
                                score = pfParameters.signDetectionScoringCoeff/(pfParameters.signDetectionScoringCoeff+x*x);
                                score *= 1. / (1. + 1. * distErrorFraction);
                                score *= elevationScore * azimuthScore;
                                if (score < pfParameters.minSignScore)
                                    score = pfParameters.minSignScore;
                            }
                        }

                        yawScores.push_back(score);

                    }
                    // get max score and assign it to the particle

                    particle->score = (*std::max_element(yawScores.begin(), yawScores.end()));
//                    double score = (*std::max_element(yawScores.begin(), yawScores.end()));
//                    auto it = particle->candidateScores.find(det.tag());
//                    if (it != particle->candidateScores.end())
//                        particle->candidateScores[det.tag()].push_back(score);
//                    else{
//                        std::vector<double> tmpv;
//                        tmpv.push_back(score);
//                        particle->candidateScores.insert(std::make_pair(det.tag(), tmpv));
//                    }
                    
                }
            }
            auto stop = high_resolution_clock::now();
            deltaT = duration_cast<nanoseconds>(stop-start);
            //std::cerr << "sign scoring time : " << deltaT.count() << "\n";
        }
        
        
        void _updateParticles(const VIOMeasurements& vioData, const std::vector<compvis::CVObservation>& detections){
            _nonValidParticles = 0;
            std::normal_distribution<double> distMotionYaw(0.0, pfParameters.motionYawNoise);
            std::normal_distribution<double> distCameraYaw(0.0, pfParameters.cameraYawNoise);
            //std::cerr << "*Curr yaw: " << std::to_string( (_particles[0].cameraYaw + vioData.getDeltaYaw()) * 180/3.14) << "\n";
            std::vector<float> globCorrFactors;
            float mean_gcf = 0;
            globCorrFactors.reserve(_numParticles);
//            particlesLog.clear();
            particlesLog.str(std::string());
            for(auto particle = _particles.begin(); particle != _particles.end(); ++particle ){
                if (particle->valid){
                    // refactor into a matrix-vector mult.
                    cv::Vec2d rotatedDeltaPosition = cv::Vec2d(vioData.getDeltaX() * cos(particle->course + pfParameters.arkitAngleCorrection) + vioData.getDeltaZ() * sin(particle->course + pfParameters.arkitAngleCorrection),
                        vioData.getDeltaX() * sin(particle->course + pfParameters.arkitAngleCorrection) - vioData.getDeltaZ() * cos(particle->course + pfParameters.arkitAngleCorrection));
                    
                    cv::Vec2d n = rotatedDeltaPosition / (cv::norm(rotatedDeltaPosition, cv::NORM_L2) + 1e-6);
                    cv::Vec2d n_hat = cv::Vec2d(n[1], -n[0]);
                    double deltaRotPosMagnitude = cv::norm(rotatedDeltaPosition, cv::NORM_L2);
                    
                    
                    std::uniform_real_distribution<double> posNoiseMaj( -deltaRotPosMagnitude * pfParameters.positionNoiseMajor,
                                                                       deltaRotPosMagnitude * pfParameters.positionNoiseMajor);
                    std::uniform_real_distribution<double> posNoiseMin(-deltaRotPosMagnitude * pfParameters.positionNoiseMinor,
                                                                       deltaRotPosMagnitude * pfParameters.positionNoiseMinor);
                    double eps1 = posNoiseMaj(_generator);
                    double eps2 = posNoiseMin(_generator);
                    
                    cv::Vec2d noise = cv::Vec2d( eps1 * n[0], eps1 * n[1] ) + cv::Vec2d(eps2 * n_hat[0], eps2 * n_hat[1]);
                    
                    cv::Point2i startPoint = _mapManager->uv2pixels(particle->position);
                    
                    // swapping coordinates system between VIO and map ref. system
                    particle->position[1] += rotatedDeltaPosition[0] * particle->globalScaleCorrectionFactor + noise[0];
                    particle->position[0] += rotatedDeltaPosition[1] * particle->globalScaleCorrectionFactor + noise[1];
                    
                    particle->cameraYaw += vioData.getDeltaYaw() + distCameraYaw(_generator);
                    cv::Point2i endPoint = _mapManager->uv2pixelsVec(particle->position);
                    
                    if (_mapManager->isPathCrossingWalls(startPoint, endPoint)){
                        particle->valid = false; //mark particle as invalid
                        _nonValidParticles++;
                        particle->score = 0;
                    }
                    else{
                        globCorrFactors.push_back(particle->globalScaleCorrectionFactor);
                        mean_gcf += particle->globalScaleCorrectionFactor;
                    }
                    
                    if (logParticles){
                        cv::Point2i pt = _mapManager->uv2pixels(particle->getPositionPoint());
                        particlesLog << std::to_string(pt.y) << ", " << std::to_string(pt.x) << ", " << std::to_string(particle->getScore()) << ", " << std::to_string(particle->getCameraYaw()) << ", " << std::to_string(particle->getCourse()) << ", " << std::to_string(detections.size() > 0) << ", " << std::to_string(particle->bestEstimatedDistance) << ", " <<
                        std::to_string(particle->bestDetYaw) << ", " << std::to_string(particle->bestPredYaw) << "\n";
                    }
                    
                    
                }
            }
            
            partStats.gsc_mean = mean_gcf / (_numParticles - _nonValidParticles);
            std::vector<float>::iterator result = std::min_element(globCorrFactors.begin(), globCorrFactors.end());
            partStats.gsc_min = *result;
            result = std::max_element(globCorrFactors.begin(), globCorrFactors.end());
            partStats.gsc_max = *result;
        }
        
        
        void _init(float initMotionYaw = 0, float initYawNoise = 0, float radius_mt = 2., cv::Point2d initLocMt = cv::Point2d(0,0)){;
            _badFramesCounter = 0;
            switch (_initMode) {
                case locore::InitMode::UNIFORM:
                    _initPolicy = std::unique_ptr<initpol::InitPolicy>(new initpol::UniformInitPolicy());
                    break;
                case locore::InitMode::UNIFORM_WITH_YAW:
                    _initPolicy = std::unique_ptr<initpol::InitPolicy>(new initpol::UniformWithYawInitPolicy(initMotionYaw, initYawNoise));
//                    _initPolicy->setInitialMotionYaw(initMotionYaw, yawNoise);
                    break;
                case locore::InitMode::LOCATION:
                    _initPolicy = std::unique_ptr<initpol::InitPolicy>(new initpol::KnownLocationInitPolicy());
                    _initPolicy->setRadius(radius_mt);
                    _initPolicy->setInitialPosition(initLocMt, _mapManager);
                    break;
                case locore::InitMode::AREA_WITH_YAW:
                    _initPolicy = std::unique_ptr<initpol::AllKnownInitPolicy>(new initpol::AllKnownInitPolicy());
                    _initPolicy->setRadius(radius_mt);
                    _initPolicy->setInitialPosition(initLocMt, _mapManager);
                    _initPolicy->setInitialMotionYaw(initMotionYaw, initYawNoise);
                    break;
                default:
                    _initPolicy = std::unique_ptr<initpol::InitPolicy>(new initpol::UniformInitPolicy());
                    break;
            }
            _currentFloor = _mapManager->currentFloor;
            this->features = _mapManager->getLandmarksList();
            exitSignsFeatureList = features.find(maps::FeatureType::EXIT_SIGN);
        }
        
        void _resampleParticles(){
            std::vector<double> scores;
            std::vector<Particle> tmpParticles;
            tmpParticles.reserve(_particles.size());
            for(Particle& particle : _particles){
                scores.push_back(particle.score);
            }
            std::discrete_distribution<int> distribution(scores.begin(), scores.end());
            int cnt = 0, attempts = 0;
            
            // todo: preallocate list of particles
            while (cnt < _numParticles){
                int p = distribution(_generator);
                    if (_particles[p].valid){
                    _particles[p].score = 1;
                    _particles[p].valid = true;
                    tmpParticles.push_back(_particles[p]);
                    cnt++;
                }
                attempts++;
                if (attempts > _numParticles)
                    break;
            }
            _particles.clear();
            _particles.shrink_to_fit();
            _particles = tmpParticles;
        }
    };
    
} // ::locore


#endif // PARTICLEFILTER_HPP_
