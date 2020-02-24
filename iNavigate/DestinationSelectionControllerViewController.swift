//
//  DestinationSelectionControllerViewController.swift
//  iNavigate
//
//  Created by Giovanni Fusco on 4/2/19.
//  Copyright © 2019 Smith-Kettlewell Eye Research Institute. All rights reserved.
//

import UIKit

extension URL {
    var isDirectory: Bool {
        return (try? resourceValues(forKeys: [.isDirectoryKey]))?.isDirectory == true
    }
    var subDirectories: [URL] {
        guard isDirectory else { return [] }
        return (try? FileManager.default.contentsOfDirectory(at: self, includingPropertiesForKeys: nil, options: [.skipsHiddenFiles]).filter{ $0.isDirectory }) ?? []
    }
}

class DestinationSelectionControllerViewController: UIViewController , UIPickerViewDelegate, UIPickerViewDataSource{

    var mapsURL : URL = Bundle.main.resourceURL!.appendingPathComponent("res/maps")
    var resURL : URL = Bundle.main.resourceURL!.appendingPathComponent("res")
    var locationURL : URL = Bundle.main.resourceURL!.appendingPathComponent("res/maps")
    var locationPickerData: [String] = [String]()
    var destinationPickerData: [String] = [String]()
    var destinationIDs : [Int] = [Int]()
    var destinationsFloor : [Int] = [Int]()
    
    var startPickerData: [String] = [String]()
    var startIDs : [Int] = [Int]()
    var startFloors : [Int] = [Int]()
    
    var destination : String = ""
    var destID : Int = -1
    var startID: Int = -1;
    var startPoint : String = ""
    var startFloor : Int = -1
    @IBOutlet weak var spatializedSound: UISwitch!
    @IBOutlet weak var centerBacon: UISwitch!
    @IBOutlet weak var voiceFeedback: UISwitch!
    
    @IBOutlet weak var logOnEventSwitch: UISwitch!
    @IBOutlet weak var locationPicker: UIPickerView!
    @IBOutlet weak var destinationPicker: UIPickerView!
    @IBOutlet weak var startPicker: UIPickerView!
    
    @IBOutlet weak var useYawSwitch: UISwitch!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        initPickers()
        locationURL = mapsURL.appendingPathComponent(locationPickerData[0], isDirectory: true)
        listBuildings()
    }
    
    struct Node {
        var comments : String
        var edges : Int
        var floor : String
        var isDoor : Int
        var id : Int
        var label : String
        var position : String
        var type : String
        init(_ dictionary: [String: Any]) {
            self.comments = dictionary["comments"] as? String ?? ""
            self.id = dictionary["id"] as? Int ?? 0
            self.floor = dictionary["floor"] as? String ?? ""
            self.edges = dictionary["edges"] as? Int ?? 0
            self.isDoor = dictionary["isDoor"] as? Int ?? 0
            self.label = dictionary["label"] as? String ?? ""
            self.position = dictionary["position"] as? String ?? ""
            self.type = dictionary["type"] as? String ?? ""
        }
    }
    
    
    func initPickers(){
        self.locationPicker.delegate = self
        self.locationPicker.dataSource = self
        self.destinationPicker.delegate = self;
        self.destinationPicker.dataSource = self;
        self.startPicker.delegate = self;
        self.startPicker.dataSource = self;
        
        let buildingsFolders = mapsURL.subDirectories
        for d in buildingsFolders{
            locationPickerData.append(d.lastPathComponent)
        }
    }
    
    // Number of columns of data
    func numberOfComponents(in pickerView: UIPickerView) -> Int {
        return 1
    }
    
    // The number of rows of data
    func pickerView(_ pickerView: UIPickerView, numberOfRowsInComponent component: Int) -> Int {
        if pickerView == locationPicker{
            return locationPickerData.count
        }
        else if pickerView == destinationPicker{
            return destinationPickerData.count
        }
        else if pickerView == startPicker{
            return startPickerData.count
        }
        return 0
    }
    
    func pickerView(_ pickerView: UIPickerView, titleForRow row: Int, forComponent component: Int) -> String? {
        if pickerView == locationPicker{
            return locationPickerData[row]
        }
        else if pickerView == destinationPicker{
            return destinationPickerData[row]
        }
        else if pickerView == startPicker{
            return startPickerData[row]
        }
        return ""
    }
    
    // Capture the picker view selection
    func pickerView(_ pickerView: UIPickerView, didSelectRow row: Int, inComponent component: Int) {
        if pickerView == locationPicker{
            locationURL = mapsURL.appendingPathComponent(locationPickerData[row], isDirectory: true)
            listBuildings()
            destinationPicker.reloadAllComponents()
            startPicker.reloadAllComponents()
        }
        else if pickerView == destinationPicker{
            destination = destinationPickerData[row]
            destID = destinationIDs[row]
        }
        else if pickerView == startPicker{
            startPoint = startPickerData[row]
            startID = startIDs[row]
            startFloor = startFloors[row]
        }
    }
    
  
    func listBuildings(){
        
        //set default location URL (first value in list)
        //locationURL = mapsURL.appendingPathComponent(locationPickerData[0], isDirectory: true)
        let path = locationURL.appendingPathComponent("navgraph.json", isDirectory: false)
            do {
                let data = try Data(contentsOf: path, options: .mappedIfSafe)
                let jsonResult = try JSONSerialization.jsonObject(with: data, options: .mutableLeaves)
                let jsonResult2 = jsonResult as? Dictionary<String, AnyObject>
                let nodes = (jsonResult2!["nodes"] as? Dictionary<String, AnyObject>)!
                destinationPickerData.removeAll()
                destinationIDs.removeAll()
                destinationsFloor.removeAll()
                                
                for (_, value) in nodes{
                    let n = Node(value as! [String : Any])
                    if (n.type == "destination" || n.type == "link"){
                        destinationPickerData.append(n.label)
                        destinationIDs.append(n.id)
                        destinationsFloor.append(Int(n.floor) ?? -1)
                    }
                }
                
                startPickerData.removeAll()
                startFloors.removeAll()
                startIDs.removeAll()
                
                startPickerData = destinationPickerData
                startFloors = destinationsFloor
                startIDs = destinationIDs
                
                startPickerData.append("2nd Floor")
                startFloors.append(2)
                startIDs.append(0)
                startPickerData.append("3rd Floor")
                startFloors.append(3)
                startIDs.append(0)
                startPickerData.append("4th Floor")
                startFloors.append(4)
                startIDs.append(0)
                startPickerData.append("10th Floor")
                startFloors.append(10)
                startIDs.append(0)
                
                destinationPickerData.append("No destination")
                destinationIDs.append(0)
                destinationsFloor.append(0)
                
            } catch {
                // handle error
            }
    }

    @IBAction func onStartNavigationButton(_ sender: Any) {
        //if destID > 0 && startID > 0{
            performSegue(withIdentifier: "startNavigation", sender: self)
        //}
    }
    
    // pass parameters to next view
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        
        let destVC : ViewController = segue.destination as! ViewController
        
        destVC.resURL = resURL
        destVC.mapsURL = mapsURL
        destVC.locationURL = locationURL
        destVC.destination = destination
        destVC.destID = destID - 1
        destVC.start = startPoint
        destVC.startID = startID - 1
        destVC.startFloor = startFloor
        destVC.useSonifiedInterface = spatializedSound.isOn
        destVC.useCenterBeacon = centerBacon.isOn
        destVC.useVoiceInterface = voiceFeedback.isOn
        destVC.dumpParticles = logOnEventSwitch.isOn
        destVC.useYaw = useYawSwitch.isOn
        //destVC.exploreMode = exploreMode.isOn
        
    }
    
}
