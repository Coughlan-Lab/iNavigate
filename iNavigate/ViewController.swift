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


class ViewController: UIViewController, ARSessionDelegate, CLLocationManagerDelegate{
    
    @IBOutlet weak var angleDiffLabel: UILabel!
    @IBOutlet weak var routeHeadingLabel: UILabel!
    @IBOutlet weak var userHeadingLabel: UILabel!
    @IBOutlet weak var sceneView: ARSCNView!
        
    var audioFeedback : AudioFeedback = AudioFeedback()
    
    // ** the navigation system (wrapper)
    let navigationCore : NavigationCoreWrapper = NavigationCoreWrapper()
    
    // ** default values
    var mapsURL : URL = Bundle.main.resourceURL!.appendingPathComponent("res/maps")
    var resURL : URL = Bundle.main.resourceURL!.appendingPathComponent("res")
    
    // ** user selected location (set by previous activity)
    var locationURL : URL = Bundle.main.resourceURL!.appendingPathComponent("res/maps")
    
    @IBOutlet weak var errorAngleImage: UIImageView!
    @IBOutlet weak var refAngleImage: UIImageView!
    @IBOutlet weak var headingImage: UIImageView!
    @IBOutlet weak var cvImage: UIImageView!
    @IBOutlet weak var navImage: UIImageView!
    // ** user selected destination
    var destination : String = ""
    var destID : Int = -1
    var start : String = ""
    var startID : Int = -1
    var startFloor : Int = -1
    var exploreMode : Bool = false
    var useSonifiedInterface: Bool = false
    var useVoiceInterface : Bool = false
    var useCenterBeacon: Bool = false
    
    var saveFrame : Bool = false
    var frameCnt : UInt64 = 0
    
    var trackerInitialized : Bool = false
    var locSysInit : Bool = false
    var dumpParticles : Bool = false
    
    var floorChangeDetector : FloorChangeDetector = FloorChangeDetector()
    
    // ** QR code reader
    var qrRequests = [VNRequest]()
    var detectedDataAnchor: ARAnchor?
    
    var frameCounter : UInt64 = 0
    var lastProcessedFrameTime: TimeInterval = TimeInterval()
    var lastVibrationFeedbackTime = Date()
    let numParticles = 10000
    
    var environment = AVAudioEnvironmentNode()
    let engine = AVAudioEngine()
    var isSpatialSoundPlaying : Bool = false
    var isCenterBeaconSoundPlaying : Bool = false
    let beaconNode = AVAudioPlayerNode()
    let centerBeaconNode = AVAudioPlayerNode()
    
    let notificationFeedbackGenerator = UINotificationFeedbackGenerator()
    
    var logger : Logger = Logger()
    
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
    
    
    // compass heading
    var compassHeading : Double = 0.0
    var compassAccuracy : Double = 180.0
    let locationManager: CLLocationManager = {
        $0.requestWhenInUseAuthorization()
        $0.startUpdatingHeading()
        $0.startUpdatingLocation()
        return $0
    }(CLLocationManager())
    
//    @IBAction func dumpSwitchTouched(_ sender: UISwitch) {
//        self.dumpParticles = sender.isOn
//    }
    
    
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
        
        // Initializing the spatial sound
        initSpatializedSound()
        
        notificationFeedbackGenerator.prepare()
        
//        setupBarometer()
        startQrCodeDetection()
        locationManager.headingOrientation = .portrait
        locationManager.delegate = self;
        setupBeaconSound(file: "drum_mono", atPosition: AVAudio3DPoint(x: 0, y: 0, z: -2))
        setupCenterBeaconSound(file: "point-blank", withExtension: String("mp3"), atPosition: AVAudio3DPoint(x: 0, y: 0, z: -2))
        audioFeedback.pushMessage(message: message_t.init(text: "Initializing tracker", highPriority: true))
        navigationCore.initNavigationSystem(locationURL.relativePath, currentFloor: Int32(startFloor), exploreMode: exploreMode)
        navigationCore.setDestinationID(Int32(destID))
        lastVibrationFeedbackTime = Date()
        logger.startSession(sessionID : "log")
    }
       
    
    @IBAction func snapshotPressed(_ sender: Any) {
        saveFrame = true
    }
    
    // Spatial Sound Initialization
   func setupBeaconSound(file:String, withExtension ext:String = "mp3", atPosition position:AVAudio3DPoint) {
           
       beaconNode.position = position
       beaconNode.reverbBlend = 0.0001
       beaconNode.renderingAlgorithm = .HRTF
       beaconNode.position = AVAudio3DPoint(x: 0, y: 0, z: 0)
       let url = Bundle.main.url(forResource: file, withExtension: ext)!
       let file = try! AVAudioFile(forReading: url)
       let buffer = AVAudioPCMBuffer(pcmFormat: file.processingFormat, frameCapacity: AVAudioFrameCount(file.length))
       try! file.read(into: buffer!)
       engine.attach(beaconNode)
       engine.connect(beaconNode, to: environment, format: buffer?.format)
       beaconNode.scheduleBuffer(buffer!, at: nil, options: .loops, completionHandler: nil)

   }
    
    func setupCenterBeaconSound(file:String, withExtension ext:String = "mp3", atPosition position:AVAudio3DPoint) {
      centerBeaconNode.position = position
      centerBeaconNode.reverbBlend = 0.0001
      centerBeaconNode.renderingAlgorithm = .HRTF
      centerBeaconNode.position = AVAudio3DPoint(x: 0, y: 0, z: 0)
      let url = Bundle.main.url(forResource: file, withExtension: ext)!
      let file = try! AVAudioFile(forReading: url)
      let buffer = AVAudioPCMBuffer(pcmFormat: file.processingFormat, frameCapacity: AVAudioFrameCount(file.length))
      try! file.read(into: buffer!)
      engine.attach(centerBeaconNode)
      engine.connect(centerBeaconNode, to: environment, format: buffer?.format)
      centerBeaconNode.scheduleBuffer(buffer!, at: nil, options: .loops, completionHandler: nil)

      }
    
    
    func initSpatializedSound(){
        environment.listenerPosition = AVAudio3DPoint(x: 0, y: 0, z: 0)
        engine.attach(environment)
        engine.connect(environment, to: engine.mainMixerNode, format: nil)
        engine.prepare()
        do {
            try engine.start()
        } catch let e as NSError {
            print("Couldn't start sound engine", e)
        }
    }
    
    
    func initLocCore(frame: ARFrame){
        locationManager.stopUpdatingHeading()
        locationManager.stopUpdatingLocation()
        let alpha = navigationCore.getNorthCorrectionAngle()
        let theta_star =  90 - (compassHeading - Double(alpha))
        let phi_star = Double(frame.camera.eulerAngles[1] * 180 / .pi)
        let yaw = (theta_star - phi_star) * .pi/180

        if startID >= 0{
            let pos = navigationCore.getNodeUVPosition(Int32(startID))
//            navigationCore.initializeLocalizationSystemUnknownLocation(resURL.relativePath, numParticles: Int32(numParticles), initYaw: Double(0), initYawNoise: 0)
//            navigationCore.initializeLocalizationSystemLocation(resURL.relativePath, numParticles: Int32(numParticles), posU: pos[0] as! Double, posV: pos[1] as! Double, initYaw: Double(yaw), initYawNoise: 2 * compassAccuracy * .pi / 180)
            navigationCore.initializeLocalizationSystem(resURL.relativePath, numParticles: Int32(numParticles), posU: pos[0] as! Double, posV: pos[1] as! Double, initYaw: Double(0), initYawNoise: 0)
//            navigationCore.initializeLocalizationSystem(resURL.relativePath, numParticles: Int32(numParticles), posU: pos[0] as! Double, posV: pos[1] as! Double, initYaw: Double(0), initYawNoise: 0)
        }
        else{
            //navigationCore.initializeLocalizationSystemUnknownLocation(resURL.relativePath, numParticles: Int32(numParticles), initYaw: Double(yaw), initYawNoise: 2 * compassAccuracy * .pi / 180)
            navigationCore.initializeLocalizationSystemUniform(resURL.relativePath, numParticles: Int32(numParticles), initYaw: Double(yaw), initYawNoise: 2 * compassAccuracy * .pi / 180)
        }
        
        locSysInit = true
        navigationCore.setCameraHeight(Float(userHeight)!)
//        navigationCore.setInitialCourse(Float(yaw))
    }
    
    
    // process new ARKit frame
    func processARKitFrame(frame: ARFrame) -> Dictionary<String,Any>{

        if (!locSysInit){
            var res : Dictionary<String, Any> = [:]
            res["cvDetectorImage"] = UIImage(color: .black)
            res["outputImage"] = UIImage(color: .black)
            res["distance"] = -1
            res["nodeLabel"] = ""
            //audioFeedback.announce(string: "please start walking", delay:1)
//            if (compassAccuracy <= 95){
              initLocCore(frame : frame)
//            }
//            else{
//                audioFeedback.announce(string: "insufficient heading accuracy, please calibrate compass", delay:3.0)
//            }
            return res;
        }
        else if (locSysInit) && (frame.timestamp-lastProcessedFrameTime) > 0.1 {
            let deltaFloors = floorChangeDetector.getFloorsDelta()
            let trackerState = "\(frame.camera.trackingState)"
            var res = navigationCore.step(trackerState, timestamp: frame.timestamp, camera: frame.camera, deltaFloors: Int32(deltaFloors), frame: pixelBufferToUIImage(pixelBuffer: frame.capturedImage), logParticles: dumpParticles) as! Dictionary<String,Any>
            res["cvDetectorImage"] = navigationCore.getCVDetectorOutputFrame()

            if dumpParticles{
                navigationCore.dumpParticles();
            }
            lastProcessedFrameTime = frame.timestamp
            return res;
        }
        else{
            var res : Dictionary<String, Any> = [:]
            res["cvDetectorImage"] = navigationCore.getCVDetectorOutputFrame()
            res["outputImage"] = navImage.image
            res["distance"] = -1
            res["nodeLabel"] = ""
            return res
        }
    }
    
    //MARK: ARSessionDelegate
    func session(_ session: ARSession, didUpdate frame: ARFrame){
        
        if (saveFrame){
            frameCnt+=1
            saveFrame = false
            logger.saveImage(imageToSave: frame.capturedImage, counter: frameCnt)
        }
        
        trackerStatusFeedback(frame: frame)
        if self.trackerInitialized{
            self.frameCounter+=1

            let res : Dictionary<String, Any> = self.processARKitFrame(frame : frame)
            let pStats = navigationCore.getParticlesStats()

            self.navImage.image = res["outputImage"] as? UIImage
            self.cvImage.image = res["cvDetectorImage"] as? UIImage

           
             
            if res["heading"] != nil{
                if useVoiceInterface{
                    dispatchInstruction(data: res) // TTS feedback
                }
                sendVibrationFeedback(data: res)
                if useSonifiedInterface || useCenterBeacon{
                    dispatchSonifiedInstruction(data: res, useNextTurn: false)
                }
                updateArrowsUI(data: res)
            }
                
        }
        
        //check if needs to update camera height
        if (frameCounter % 150 == 0 && locSysInit){
            checkQRCode(image: frame.capturedImage)
        }
    }
    
    func sendVibrationFeedback(data: Dictionary<String, Any> ){
        if data["validNavData"] != nil{
            if data["validNavData"] as! Bool == true{
                let direction : TurnDirection = TurnDirection(rawValue: data["instructions"] as! Int)!
                if direction == TurnDirection.TurnAround{
                    print(lastVibrationFeedbackTime.timeIntervalSinceNow)
                    if abs(lastVibrationFeedbackTime.timeIntervalSinceNow) > 5{
                        notificationFeedbackGenerator.notificationOccurred(.error)
                        lastVibrationFeedbackTime = Date()
                    }
                }
            }
        }
    }
    
    func updateArrowsUI(data : Dictionary<String, Any>){
        // Note: UIImage rotates clockwise (i.e. 90 degrees points down in a unit circle)
        var angle = data["heading"] as! Double
        
        if angle <= 360{
            headingImage.isHidden = false
            let head_rad = angle * (Double.pi/180.0)
            userHeadingLabel.text = "\(angle)"
            headingImage.transform = CGAffineTransform(rotationAngle: CGFloat(-head_rad));
        }
        else{
            headingImage.isHidden = true
        }
            angle = data["refAngle"] as! Double
            let yawVariance = data["yawVariance"] as! Double
//                    var vioYaw = frame.camera.eulerAngles[1]
        let head_rad = angle * (Double.pi/180.0)
            routeHeadingLabel.text = "Var:  \(yawVariance)"
            refAngleImage.transform = CGAffineTransform(rotationAngle: CGFloat(-head_rad));
    }
    
    
    func dispatchSonifiedInstruction(data : Dictionary<String, Any>, useNextTurn: Bool){
            if data["validNavData"] != nil{
                if data["validNavData"] as! Bool == true{
                    
                    if !isSpatialSoundPlaying && useSonifiedInterface{
                        beaconNode.play()
                        isSpatialSoundPlaying = true
                    }
                    
                    let validTurnNode = data["validTurnNode"] as! Bool
                    let throughDoor = data["destThroughDoor"] as! Bool
                    let direction : TurnDirection = TurnDirection(rawValue: data["instructions"] as! Int)!
                    switch direction {
                        case TurnDirection.LeftToDest:
                            if throughDoor{
                                audioFeedback.pushMessage(message: message_t.init(text: "your destination is through a door on the left", highPriority: true))
                            }
                            else{
                                audioFeedback.pushMessage(message: message_t.init(text: "your destination is on the left", highPriority: true))
                            }
                        case TurnDirection.RightToDest:
                            if throughDoor{
                                audioFeedback.pushMessage(message: message_t.init(text: "your destination is through a door on the right", highPriority: true))
                            }
                            else{
                                audioFeedback.pushMessage(message: message_t.init(text: "your destination is on the right", highPriority: true))
                            }
                        default:
                            break;
                    }
                    
//                    let label = data["nodeLabel"] as! String
                    
//                    let _ : NodeType = NodeType(rawValue: data["nodeType"] as! Int)!
                    var nodeU, nodeV: Float
                    if (validTurnNode && useNextTurn){ // sonifies only the nodes that come before a turn
                        nodeU = data["turnNodePositionU"] as! Float
                        nodeV = data["turnNodePositionV"] as! Float
                    }
                    else{
                        nodeU = data["nodePositionU"] as! Float
                        nodeV = data["nodePositionV"] as! Float
                    }
                    
                    let course = data["heading"] as! Float
                    
                    _ = data["refAngle"] as! Float
//                    let distToNode = data["distanceToApproachingNode"] as! Float
                    let userU = data["userPositionU"] as! Float
                    let userV = data["userPositionV"] as! Float
                    var angleToNode = atan2(nodeU - userU, nodeV - userV)*180/(.pi); // did I swap coordinates on purpose?
                    angleToNode = fmod(angleToNode, 360);
                    if (angleToNode < 0){
                        angleToNode += 360;
                    }
                    let diffAngle = angleToNode - course
 
                    angleDiffLabel.text = "\(diffAngle)"
                    let diffAngle_rad = diffAngle  * (.pi/180.0)
                    errorAngleImage.transform = CGAffineTransform(rotationAngle: CGFloat(diffAngle_rad));
                    environment.listenerAngularOrientation = AVAudioMake3DAngularOrientation(Float(diffAngle) , 0, 0)
                    
                    if (useCenterBeacon){
                        if (abs(diffAngle) < 20){
                            if (!isCenterBeaconSoundPlaying){
                                centerBeaconNode.play()
                                isCenterBeaconSoundPlaying = true
                            }
                        }
                        else{
                            if (isCenterBeaconSoundPlaying){
                                centerBeaconNode.pause()
                                isCenterBeaconSoundPlaying = false
                            }
                        }
                    }
                    
                }
                else{
                    if isSpatialSoundPlaying && useSonifiedInterface{
                        beaconNode.pause()
                        isSpatialSoundPlaying = false
                    }
                }
            }
        }
    
    
    func dispatchInstruction(data : Dictionary<String, Any>){
        if data["validNavData"] != nil{
            if data["validNavData"] as! Bool == true{
                let throughDoor = data["destThroughDoor"] as! Bool
                let nodeType : NodeType = NodeType(rawValue: data["nodeType"] as! Int)!
                //if nodeType == NodeType.Control{
                    let direction : TurnDirection = TurnDirection(rawValue: data["instructions"] as! Int)!
                    switch direction {
                    case TurnDirection.EasyLeft:
                        audioFeedback.pushMessage(message: message_t.init(text: "make an easy left", highPriority: true))
                    case TurnDirection.Left:
                        audioFeedback.pushMessage(message: message_t.init(text: "please turn left", highPriority: true))
                    case TurnDirection.EasyRight:
                        audioFeedback.pushMessage(message: message_t.init(text: "make an easy right", highPriority: true))
                    case TurnDirection.Right:
                        audioFeedback.pushMessage(message: message_t.init(text: "please turn right", highPriority: true))
                    case TurnDirection.TurnAround:
                        audioFeedback.pushMessage(message: message_t.init(text: "turn around", highPriority: false))
                    case TurnDirection.Forward:
                        audioFeedback.pushMessage(message: message_t.init(text: "please go straight", highPriority: true))
                    case TurnDirection.Arrived:
                        let label = data["nodeLabel"] as! String
                        audioFeedback.pushMessage(message: message_t.init(text: "you have arrived at " + label, highPriority: true))
                    case TurnDirection.LeftToDest:
                        if throughDoor{
                            audioFeedback.pushMessage(message: message_t.init(text: "your destination is through a door on the left", highPriority: true))
                        }
                        else{
                            audioFeedback.pushMessage(message: message_t.init(text: "your destination is on the left", highPriority: true))
                        }
                    case TurnDirection.RightToDest:
                        if throughDoor{
                            audioFeedback.pushMessage(message: message_t.init(text: "your destination is through a door on the right", highPriority: true))
                        }
                        else{
                            audioFeedback.pushMessage(message: message_t.init(text: "your destination is on the right", highPriority: true))
                        }
                    case TurnDirection.ForwardToDest:
                        if throughDoor{
                            audioFeedback.pushMessage(message: message_t.init(text: "your destination is through a door in front of you", highPriority: true))
                        }
                        else{
                            audioFeedback.pushMessage(message: message_t.init(text: "your destination is in front of you", highPriority: true))
                        }
                    default:
                        break;
                //    }
                }
            }
        }
    }
    
    
    func trackerStatusFeedback(frame: ARFrame){
        
        var status : String = ""
        switch frame.camera.trackingState {
        case .normal:
            if !trackerInitialized{
                trackerInitialized = true
                status = "Tracker ready! Camera height \(userHeight) inches"
                audioFeedback.pushMessage(message: message_t.init(text: status, highPriority: true))
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
            audioFeedback.pushMessage(message: message_t.init(text: status))
        }
        else if(!trackerInitialized){
            audioFeedback.pushMessage(message: message_t.init(text: "Initializing tracker", highPriority: true))
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
                navigationCore.setCameraHeight(Float(userHeight)!*Float(2.54/100)) //convert inches to mt
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
