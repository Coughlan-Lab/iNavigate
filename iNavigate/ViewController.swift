//
//  ViewController.swift
//  iNavigate
//
//  Created by Giovanni Fusco on 4/2/19.
//  Copyright © 2019 Smith-Kettlewell Eye Research Institute. All rights reserved.
//

import UIKit
import SpriteKit
import ARKit
import CoreLocation
import CoreMotion
import AVFoundation
import Vision


public extension UIImage {
    convenience init?(color: UIColor, size: CGSize = CGSize(width: 1, height: 1)) {
        let rect = CGRect(origin: .zero, size: size)
        UIGraphicsBeginImageContextWithOptions(rect.size, false, 0.0)
        color.setFill()
        UIRectFill(rect)
        let image = UIGraphicsGetImageFromCurrentImageContext()
        UIGraphicsEndImageContext()
        
        guard let cgImage = image?.cgImage else { return nil }
        self.init(cgImage: cgImage)
    }
}


class AudioFeedback{
    var lastPOIAnnouncedTime : Date
    var lastAnnouncement : String
    var player: AVAudioPlayer?
    var synthesizer : AVSpeechSynthesizer
    
    init(){
        self.lastPOIAnnouncedTime = Date()
        synthesizer = AVSpeechSynthesizer()
        lastAnnouncement = ""
        guard let url = Bundle.main.url(forResource: "white_noise_filt", withExtension: "wav") else { return }
        
        do {
            try AVAudioSession.sharedInstance().setCategory(.playback, mode: .default)
            try AVAudioSession.sharedInstance().setActive(true)
            
            player = try AVAudioPlayer(contentsOf: url, fileTypeHint: AVFileType.wav.rawValue)
            //guard let player = player else { return }
            
            
        } catch let error {
            print(error.localizedDescription)
        }
    }
    
    func stopUtterance(){
        if (synthesizer.isSpeaking){
            synthesizer.stopSpeaking(at: AVSpeechBoundary.word)
        }
    }
    
    func announce(string: String, delay: Double){
        //if string != " "{
        if abs(lastPOIAnnouncedTime.timeIntervalSinceNow) > delay{ //|| lastAnnouncement != string{
            lastAnnouncement = string
            let utterance = AVSpeechUtterance(string: string)
            let psynthesizer = AVSpeechSynthesizer()
            utterance.rate = 0.68
            
            stopUtterance()
            psynthesizer.speak(utterance)
            lastPOIAnnouncedTime = Date();
            
            //  }
        }
    }
    
    func announceNow(string: String){
        let utterance = AVSpeechUtterance(string: string)
        
        utterance.rate = 0.62
        
        //utterance.voice = AVSpeechSynthesisVoice.
        // if repeating same message, wait for previous utterance to finish
        if (lastAnnouncement == string){
            synthesizer.speak(utterance)
        }
            // if it is a new message, interrupt previous message, if any
        else{
            if (synthesizer.isSpeaking){
                synthesizer.stopSpeaking(at: AVSpeechBoundary.immediate)
            }
            print(string)
            synthesizer.speak(utterance)
            //lastPOIAnnouncedTime = Date();
        }
        lastAnnouncement = string
    }
    
    func playStaticSound() {
        player?.play()
    }
}


class ViewController: UIViewController, ARSessionDelegate, CLLocationManagerDelegate{
    
    @IBOutlet weak var sceneView: ARSCNView!
    
    var audioFeedback : AudioFeedback = AudioFeedback()
    
    @IBOutlet weak var dumpSwitch: UISwitch!
    
    // ** the navigation system (wrapper)
    let navigationCore : NavigationCoreWrapper = NavigationCoreWrapper()
    
    // ** default values
    var mapsURL : URL = Bundle.main.resourceURL!.appendingPathComponent("res/maps")
    var resURL : URL = Bundle.main.resourceURL!.appendingPathComponent("res")
    
    // ** user selected location (set by previous activity)
    var locationURL : URL = Bundle.main.resourceURL!.appendingPathComponent("res/maps")
    
    @IBOutlet weak var yawLabel: UILabel!
    @IBOutlet weak var cvImage: UIImageView!
    @IBOutlet weak var navImage: UIImageView!
    // ** user selected destination
    var destination : String = ""
    var destID : Int = -1
    var start : String = ""
    var startID : Int = -1
    var startFloor : Int = -1
    
    var trackerInitialized : Bool = false
    var locSysInit : Bool = false
    var dumpParticles : Bool = false
    
    var floorChangeDetector : FloorChangeDetector = FloorChangeDetector()
    
    // ** QR code reader
    var qrRequests = [VNRequest]()
    var detectedDataAnchor: ARAnchor?
    
    var frameCounter : UInt64 = 0
    var lastProcessedFrameTime: TimeInterval = TimeInterval()
    let numParticles = 50000
    
    // set user height (retrieve last value from memory)
    var userHeight:String {
        get {
            // Get the standard UserDefaults as "defaults"
            let defaults = UserDefaults.standard
            // Makes the "userHeight" variable whatever the saved value for "userHeight" is
            return defaults.string(forKey: "userHeight") ?? "51"
        }
        set (newValue) {
            // Get the standard UserDefaults as "defaults"
            let defaults = UserDefaults.standard
            // Saves what the "userHeight" variable was just set to as the saved value for "welcome_string"
            defaults.set(newValue, forKey: "userHeight")
        }
    }
    
    
    // ** compass heading
    var compassHeading : Double = 0.0
    var compassAccuracy : Double = 180.0
    let locationManager: CLLocationManager = {
        $0.requestWhenInUseAuthorization()
        $0.startUpdatingHeading()
        $0.startUpdatingLocation()
        return $0
    }(CLLocationManager())
    
    @IBAction func dumpSwitchTouched(_ sender: UISwitch) {
        self.dumpParticles = sender.isOn
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Show statistics such as fps and node count
        self.sceneView.showsStatistics = true
        self.sceneView.debugOptions = [ARSCNDebugOptions.showFeaturePoints, ARSCNDebugOptions.showWorldOrigin]
        
        let configuration = ARWorldTrackingConfiguration()
        configuration.worldAlignment = .gravity
        
        // Run the view's session
        sceneView.session.run(configuration)
        sceneView.session.delegate = self
        
//        setupBarometer()
        startQrCodeDetection()
        locationManager.headingOrientation = .portrait
        locationManager.delegate = self;
        audioFeedback.announce(string: "Initializing Tracker", delay:0)
        navigationCore.initNavigationSystem(locationURL.relativePath, currentFloor: Int32(startFloor))
        navigationCore.setDestinationID(Int32(destID))
    }
    
    
    func initLocCore(frame: ARFrame){
        locationManager.stopUpdatingHeading()
        locationManager.stopUpdatingLocation()
        let theta_star =  90 - (compassHeading - 9.1)
        let phi_star = Double(frame.camera.eulerAngles[1] * 180 / .pi)
        let yaw = (theta_star - phi_star) * .pi/180
//        let yaw = 0.0
//        print("Initial yaw: " + String(yaw*180/3.14))
        yawLabel.text = String(yaw)
        if startID > 0{
            let pos = navigationCore.getNodeUVPosition(Int32(startID))
            navigationCore.initializeLocalizationSystem(resURL.relativePath, numParticles: Int32(numParticles), posU: pos[0] as! Double, posV: pos[1] as! Double, initYaw: Double(yaw), initYawNoise: 2 * compassAccuracy * .pi / 180)
        }
        else{
            navigationCore.initializeLocalizationSystemUnknownLocation(resURL.relativePath, numParticles: Int32(numParticles), initYaw: Double(yaw), initYawNoise: 2 * compassAccuracy * .pi / 180)
        }
        
        locSysInit = true
        navigationCore.setCameraHeight(Float(userHeight)!)
    }
    
    
    // process new ARKit frame
    func processARKitFrame(frame: ARFrame) -> UIImage{
        
        if (!locSysInit){
            audioFeedback.announce(string: "heading accuracy, \(compassAccuracy)", delay:0)
            if (compassAccuracy <= 95){
              initLocCore(frame : frame)
            }
            else{
                audioFeedback.announce(string: "insufficient heading accuracy, \(compassAccuracy)", delay:3.0)
            }
        }
        else if (locSysInit) && (frame.timestamp-lastProcessedFrameTime) > 0.1 {
            let deltaFloors = floorChangeDetector.getFloorsDelta()
            let trackerState = "\(frame.camera.trackingState)"
            let outimg = navigationCore.step(trackerState, timestamp: frame.timestamp, camera: frame.camera, deltaFloors: Int32(deltaFloors), frame: pixelBufferToUIImage(pixelBuffer: frame.capturedImage))
            
            cvImage.image = navigationCore.getCVDetectorOutputFrame();
//            let yaw = navigationCore.getParticlesYaw()*180/3.14;
//            yawLabel.text = String(yaw)
            if dumpParticles{
                navigationCore.dumpParticles();
            }
            lastProcessedFrameTime = frame.timestamp
            return outimg;
        }
        if (navImage.image == nil){
            navImage.image = UIImage(color: .black)
            cvImage.image = UIImage(color: .black);
        }
        return navImage.image!
    }
    
    //MARK: ARSessionDelegate
    func session(_ session: ARSession, didUpdate frame: ARFrame){
        
        trackerStatusFeedback(frame: frame)
        if self.trackerInitialized{
            self.frameCounter+=1
            
            // feed frame to localization core
            //  if (frameCounter % 10 == 0){
            self.navImage.image = self.processARKitFrame(frame : frame)
            // }
            
            //check system status and phone pitch and play bkg sound
            //sendPhonePitchFeedback(cameraPitch: frame.camera.eulerAngles.x)
            
//            if (locSysInit){
//                sendLocalizationConfidenceFeedback(hardPeak: localizationCore.isPeakHard())
//                announceROI(roiLabel: localizationCore.getROILabel())
//            }
        }
        
        //check if needs to update camera height
        if (frameCounter % 150 == 0 && locSysInit){
            checkQRCode(image: frame.capturedImage)
        }
    }
    
    
    func trackerStatusFeedback(frame: ARFrame){
        
        var status : String = ""
        switch frame.camera.trackingState {
        case .normal:
            if !trackerInitialized{
                trackerInitialized = true
                status = "Tracker ready! Camera height \(userHeight)"
                audioFeedback.announce(string: status, delay: 0)
                status = ""
            }
        case .notAvailable:
            status = "Wait"
        case .limited(.excessiveMotion):
            status = "Slow down"
        case .limited(.insufficientFeatures):
            status = "No features"
        case .limited(.initializing):
            status = "Initializing"
        case .limited(.relocalizing):
            status = "Relocalizing"
        case .limited(_):
            status = "Limited unknown"
        }
        
        let lightEstimate = frame.lightEstimate?.ambientIntensity
        if (lightEstimate! < 200){
            status = "Uncover Lens"
        }
        
        if (status != ""){
            audioFeedback.announce(string: status, delay: 2)
        }
        else if(!trackerInitialized){
            audioFeedback.announce(string: "Initializing Tracker", delay:0.0)
        }
    }
    
    
    
    func locationManager(_ manager: CLLocationManager, didUpdateHeading newHeading: CLHeading) {
        self.compassHeading = newHeading.trueHeading
        self.compassAccuracy = newHeading.headingAccuracy
    }
    
    
    func startQrCodeDetection() {
        // Create a Barcode Detection Request
        let request = VNDetectBarcodesRequest(completionHandler: self.requestHandler)
        // Set it to recognize QR code only
        request.symbologies = [.QR]
        self.qrRequests = [request]
    }
    
    func checkQRCode(image : CVPixelBuffer){
        let imageRequestHandler = VNImageRequestHandler(cvPixelBuffer: image,
                                                        options: [:])
        do{
            try imageRequestHandler.perform(self.qrRequests)
        }
        catch {  }
    }
    
    
    
    func requestHandler(request: VNRequest, error: Error?) {
        // Get the first result out of the results, if there are any
        if (self.trackerInitialized){
            if let results = request.results, let result = results.first as? VNBarcodeObservation {
                guard let payload = result.payloadStringValue else {return}
                userHeight = payload
                //audioFeedback.announceNow(string: "Camera height set to \(userHeight)")
                navigationCore.setCameraHeight(Float(userHeight)!)
            }
        }
    }
    
    
    func pixelBufferToUIImage(pixelBuffer: CVPixelBuffer) -> UIImage {
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        let context = CIContext(options: nil)
        let cgImage = context.createCGImage(ciImage, from: ciImage.extent)
        let uiImage = UIImage(cgImage: cgImage!)
        return uiImage
    }
    
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Create a session configuration
        let configuration = ARWorldTrackingConfiguration()

        // Run the view's session
        sceneView.session.run(configuration)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        // Pause the view's session
        sceneView.session.pause()
    }
    
    // MARK: - ARSKViewDelegate
    
    func session(_ session: ARSession, didFailWithError error: Error) {
        // Present an error message to the user
        
    }
    
    func sessionWasInterrupted(_ session: ARSession) {
        // Inform the user that the session has been interrupted, for example, by presenting an overlay
        
    }
    
    func sessionInterruptionEnded(_ session: ARSession) {
        // Reset tracking and/or remove existing anchors if consistent tracking is required
        
    }
}
