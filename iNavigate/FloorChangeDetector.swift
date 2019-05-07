//
//  FloorChangeDetector.swift
//  iNavigate
//
//  Created by Giovanni Fusco on 4/23/19.
//  Copyright © 2019 Smith-Kettlewell Eye Research Institute. All rights reserved.
//

import Foundation
import CoreMotion

class FloorChangeDetector{
    
    var altimeter : CMAltimeter = CMAltimeter()
    
    var currentPressure_kPa : Double = 0.0
    var barometerTstamp : Double = 0
    var prevBarometerTstamp : Double = 0
    var prevKpa : Double = 0
    
    let barometerQueue: OperationQueue = {
        let barometerQueue = OperationQueue()
        barometerQueue.name = "org.ski.BarometerLogger.barometerQueue"
        barometerQueue.qualityOfService = .userInteractive
        return barometerQueue
    }()
    
    let min_delta = 0.00025
    let min_delta2 = 0.0002
    let floor_threshold = 0.03
    var pRef : Double = 0
    var pStart : Double = 0
    var pEnd : Double = 0
    private var _floorChanging : Int = 0
    private var _floorsDelta : Int = 0
    
    // Concurrent synchronization queue
    private let queue = DispatchQueue(label: "ThreadSafeCollection.queue", attributes: .concurrent)
    
    init() {
        NSLog("Floor detector init")
        setupBarometer()
//        let timerFloorDetector : Timer =
        Timer.scheduledTimer(timeInterval: 0.5, target: self, selector: #selector(detectFloorChange), userInfo: nil, repeats: true)
    }
    
    func setupBarometer(){
        if CMAltimeter.isRelativeAltitudeAvailable() {
            self.altimeter.startRelativeAltitudeUpdates(to: barometerQueue, withHandler: { (altitudeData:CMAltitudeData?, error:Error?) in
                
                // Pressure in kilopascals
                self.currentPressure_kPa = altitudeData!.pressure.doubleValue
                self.prevBarometerTstamp = self.barometerTstamp
                self.barometerTstamp = altitudeData!.timestamp
            })
        }
    }
    
    func getFloorsDelta() -> Int {
        var delta : Int = 0
        queue.sync { // Read
            delta = _floorsDelta
            _floorsDelta = 0
        }
        return delta
    }
    
    func getFloorChanging() -> Int {
        var changing : Int = 0
        queue.sync { // Read
            changing = _floorChanging
        }
        return changing
    }
    
    
    @objc func detectFloorChange(){
        
        if prevBarometerTstamp == -1 && self.barometerTstamp > 0{
            pRef = self.currentPressure_kPa
            prevBarometerTstamp = self.barometerTstamp
        }
        else if prevBarometerTstamp > 0 && (self.barometerTstamp - prevBarometerTstamp) > 0{
            prevBarometerTstamp = self.barometerTstamp
            let dp = (self.currentPressure_kPa - pRef)
            print(prevBarometerTstamp, dp)
            if (abs(dp) > min_delta){
                _floorChanging += 1
                if (pStart == 0){
                    pStart = self.currentPressure_kPa
                }
                else{
                    pEnd = self.currentPressure_kPa
                }
            }
            else if (pStart > 0 && pEnd > 0){
                _floorChanging = 0
                let dPTot = pEnd - pStart
                if (abs(dPTot) >= floor_threshold){
                    let numStories = -trunc(dPTot / floor_threshold)
                    queue.async(flags: .barrier) {
                        self._floorsDelta += Int(numStories)
                    }
                }
                pStart = 0
                pEnd = 0
            }
            
            pRef = self.currentPressure_kPa
        }
    }
    
    
}
