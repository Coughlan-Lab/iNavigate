#if !defined(VIOMEASUREMENTS_HPP_)
#define VIOMEASUREMENTS_HPP_

#include<opencv2/opencv.hpp>

// Ref. System - thank me later
        //    ^ y
        //    | 
        //    . ---> x
        //   / 
        //  v z

namespace locore{

class VIOMeasurements{

    public:
    
        VIOMeasurements() { _init = false; }

        VIOMeasurements(std::string trackerStatus, double timestamp, double x, double y, double z, double rx, double ry, double rz, double kPa, double barometerTstamp, cv::Mat frame){
            _trackerStatus = trackerStatus;
            _x = x; _y = y; _z = z;
            _rx = rx; _ry = ry; _rz = rz;
            _kPa = kPa;
            _frame = frame;  // NOTE: image width and height are swapped
            _timestamp = timestamp;
            _init = true;
             _barometerTstamp = barometerTstamp;
        }
    
        VIOMeasurements(std::string trackerStatus, double timestamp, cv::Vec3d cameraPos, cv::Vec3d rotation, float kPa, double barometerTstamp, cv::Mat frame){
            _trackerStatus = trackerStatus;
            _x = cameraPos[0]; _y = cameraPos[1]; _z = cameraPos[2];
            _rx = rotation[0]; _ry = rotation[1]; _rz = rotation[2];
            _kPa = kPa;
            _frame = frame;  // NOTE: image width and height are swapped
            _timestamp = timestamp;
            _init = true;
            _barometerTstamp = barometerTstamp;
        }
    
        void update(std::string trackerStatus, double timestamp, cv::Vec3d pos, cv::Vec3d rot, double kPa, double barometerTstamp, cv::Mat frame){
            this->update(trackerStatus, timestamp, pos[0], pos[1], pos[2], rot[0], rot[1], rot[2], kPa, barometerTstamp, frame);
        }
    
        void update(const VIOMeasurements& newData){
            _trackerStatus = newData.getTrackerStatus();
            _timestamp = newData.getTimestamp();
            _deltaX = newData.getX() - _x;   _deltaY = newData.getY() - _y;   _deltaZ = newData.getZ() - _z;
            _x = newData.getX();             _y = newData.getY();             _z = newData.getZ();
            
            _deltaRx = newData.getPitch() - _rx;    _deltaRy = newData.getYaw() - _ry;    _deltaRz = newData.getRoll() - _rz;
            _rx = newData.getPitch();               _ry = newData.getYaw();               _rz = newData.getRoll();
            
            _kPa = newData.getPressure();
            _frame = newData.getFrame();  // NOTE: image width and height are swapped
            _barometerTstamp = newData.getBarometerTimestamp();
        }

        void update(std::string trackerStatus, double timestamp, double x, double y, double z, double rx, double ry, double rz, double kPa, double barometerTstamp, cv::Mat& frame){
            _trackerStatus = trackerStatus;
            _timestamp = timestamp;
            _deltaX = x - _x;   _deltaY = y - _y;   _deltaZ = z - _z;
            _x = x;             _y = y;             _z = z;

            _deltaRx = rx - _rx;    _deltaRy = ry - _ry;    _deltaRz = rz - _rz;
            _rx = rx;               _ry = ry;               _rz = rz;

            _kPa = kPa;
            _frame = frame;
            _barometerTstamp = barometerTstamp;
        }
        
        inline double getX() const { return _x; }
        inline double getY() const { return _y; }
        inline double getZ() const { return _z; }
        
        inline double getDeltaX() const { return _deltaX; }
        inline double getDeltaY() const { return _deltaY; }
        inline double getDeltaZ() const { return _deltaZ; }

        inline double getPitch() const { return _rx; }
        inline double getYaw(  ) const { return _ry; }
        inline double getRoll( ) const { return _rz; }
        
        inline double getDeltaPitch() const { return _deltaRx; }
        inline double getDeltaYaw(  ) const { return _deltaRy; }
        inline double getDeltaRoll( ) const { return _deltaRz; }

        inline std::string getTrackerStatus() const { return _trackerStatus; }
        inline double      getTimestamp()     const { return _timestamp; }
        inline double      getBarometerTimestamp()     const { return _barometerTstamp; }
        inline cv::Mat     getFrame()         const { return _frame; }  // NOTE: image width and height are swapped
        inline cv::Mat     getFrameNonConst()               { return _frame; }
        inline double      getPressure()      const { return _kPa; }
        inline bool        isValid()          const { return _init; }
    
        

    private:

        double _timestamp; // unused for now
        bool _init;
        
        // current frame from ARKit,  NOTE: image width and height are swapped
        cv::Mat _frame;
    
        std::string _trackerStatus;

        // VIO position coordinates      
        double _x;
        double _y;
        double _z;
        
        // VIO Rotation angles x, y, z
        double _rx;  // pitch
        double _ry;  // yaw
        double _rz; // roll
        
        // delta VIO position w.r.t. to last sample
        double _deltaX;
        double _deltaY;
        double _deltaZ;

        // delta VIO Rotations w.r.t. to last sample
        double _deltaRx;
        double _deltaRy;
        double _deltaRz;

        // latest barometer reading
        double _kPa;
        double _barometerTstamp;
};
    
} // ::locore

#endif // VIOMEASUREMENTS_HPP_
