//
//  AudioFeedback.swift
//  iNavigate
//
//  Created by Giovanni Fusco on 10/2/19.
//  Copyright © 2019 Smith-Kettlewell Eye Research Institute. All rights reserved.
//

import Foundation

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
            synthesizer.stopSpeaking(at: AVSpeechBoundary.immediate)
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
