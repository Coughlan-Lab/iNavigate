#include "../Core/VIOMeasurements.hpp"

namespace sensors {
    


class FloorChangeDetector{

    public:
        FloorChangeDetector(int currentFloorNumber){
            _currentFloorNumber = currentFloorNumber;
            hPaThreshold = 0.3;
            _pRef = 0;
            minDeltaVal = 0.005;
            _prevValue = 0;
            _lastTstamp = 0.;
            _reset = true;
        };
    
        FloorChangeDetector() { FloorChangeDetector(4) ; }
        ~FloorChangeDetector() { ; }

        // threshold above which we declare floor changed (approx. difference in pressure between stories)
        float hPaThreshold;
        
        // if the pressure differential goes above the threshold, we declare an event when local pressure delta is below this value
        float minDeltaVal;
    
    int update(float kPa, double tStamp){
        float numStories = 0.0;
        if (tStamp > _lastTstamp){
            _lastTstamp = tStamp;
            float hPa = kPa * 10;
            
            //float deltaP = 0;
            
//            if (_reset){
//                _pRef = hPa;
//                _reset = false;
//            }
            if (_pRef == 0 && hPa != 0){
                _pRef = hPa;
                std::cout << "+ pRef: " << _pRef;
            }
            
            else{
                float deltaP = (hPa - _pRef);
                std::cout  << "+ DeltaP " << abs(deltaP) << "\n";
                if (abs(deltaP) > hPaThreshold){
                    numStories = -(deltaP / abs(deltaP)) * floor(abs(deltaP) / hPaThreshold);
                    _currentFloorNumber += numStories;
                    _pRef = hPa;
                    std::cout  << "+ _currentFloorNumber " << _currentFloorNumber << "\n ++++ \n";
                }
                
//                std::cout  << "DeltaP " << deltaP << "\n";
//                float local_delta = 0;
//                if (_prevValue != 0)
//                    local_delta = abs(hPa - _prevValue);
//
//                if (abs(deltaP) > hPaThreshold){
//                    if (local_delta < minDeltaVal){
//                        _reset = true;
//                        numStories = -(deltaP / abs(deltaP)) * floor(abs(deltaP) / hPaThreshold);
//                        _currentFloorNumber += numStories;
//                    }
//                }
//                _prevValue = hPa;
//            }
            //std::cout << "FloorDet: " << numStories << "\n";
            
            }
        }
        return numStories;
    }

//        int update(float kPa, double tStamp){
//            float numStories = 0.0;
//            if (tStamp != _lastTstamp){
//                _lastTstamp = tStamp;
//                float hPa = kPa * 10;
//
//                float deltaP = 0;
//
//                if (_reset){
//                    _pRef = hPa;
//                    _reset = false;
//                }
//                else if (_pRef == 0)
//                    _pRef = hPa;
//
//                else{
//                    float deltaP = (hPa - _pRef);
//                    std::cout  << "DeltaP " << deltaP << "\n";
//                    float local_delta = 0;
//                    if (_prevValue != 0)
//                        local_delta = abs(hPa - _prevValue);
//
//                    if (abs(deltaP) > hPaThreshold){
//                        if (local_delta < minDeltaVal){
//                            _reset = true;
//                            numStories = -(deltaP / abs(deltaP)) * floor(abs(deltaP) / hPaThreshold);
//                            _currentFloorNumber += numStories;
//                        }
//                    }
//                    _prevValue = hPa;
//                }
//                //std::cout << "FloorDet: " << numStories << "\n";
//
//            }
//            return numStories;
//        }

    private:

    int _currentFloorNumber;
    
    float _previousHpa;
    
    float _pRef;
    float _prevValue;
    bool _reset;
    double _lastTstamp;
};

} // ::sensors
