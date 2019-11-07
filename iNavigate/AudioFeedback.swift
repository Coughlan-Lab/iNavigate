//
//  AudioFeedback.swift
//  iNavigate
//
//  Created by Giovanni Fusco on 10/2/19.
//  Copyright © 2019 Smith-Kettlewell Eye Research Institute. All rights reserved.
//

import Foundation

class AudioFeedback: AVSpeechSynthesizer{
    var lastPOIAnnouncedTime : Date
    var lastAnnouncedTime : Date
    var lastSoundTime : Date
    var lastAnnouncement : String
    var player: AVAudioPlayer?
    var synthesizer : AVSpeechSynthesizer
    var messageQueue : [message_t] = []
    
    struct message_t {
        var timestamp: Date
        var message: String
        var highPriority: Bool
    }
    
    override init(){
        self.lastAnnouncedTime = Date()
        self.lastSoundTime = Date()
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
    
    
    func isSpeaking() -> Bool{
        return synthesizer.isSpeaking
    }
    
    func stopUtterance(){
        if (synthesizer.isSpeaking){
            synthesizer.stopSpeaking(at: AVSpeechBoundary.word)
        }
    }
    
    func announce(message: String, delay: Double){
        if abs(lastAnnouncedTime.timeIntervalSinceNow) > delay || lastAnnouncement != message{
            stopUtterance()
            lastAnnouncement = message
            let utterance = AVSpeechUtterance(string: message)
            let psynthesizer = AVSpeechSynthesizer()
            utterance.rate = 0.6
            psynthesizer.speak(utterance)
            lastAnnouncedTime = Date();
        }
    }
    
    func announceNow(string: String){
        let utterance = AVSpeechUtterance(string: string)
        utterance.rate = 0.6
        print(string)
        if (!synthesizer.isSpeaking){
            synthesizer.speak(utterance)
            lastAnnouncement = string
            lastAnnouncedTime = Date();
        }
    }
    
    func playStaticSound() {
        player?.play()
    }
}
