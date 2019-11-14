//
//  AudioFeedback.swift
//  iNavigate
//
//  Created by Giovanni Fusco on 10/2/19.
//  Copyright © 2019 Smith-Kettlewell Eye Research Institute. All rights reserved.
//

import Foundation

public struct message_t {
    var timestamp: Date
    var text: String
    var highPriority: Bool
    var discardable: Bool
    
    init(text: String, discardable: Bool = true, highPriority: Bool = false){
        self.timestamp = Date()
        self.text = text
        self.highPriority = highPriority
        self.discardable = discardable
    }
}

class SpeechSynthThread: Thread, AVSpeechSynthesizerDelegate{
    let synthesizer : AVSpeechSynthesizer = AVSpeechSynthesizer()
    var messageQueue = Queue<message_t>()
    var run: Bool = true
    let staleThreshold : Double = 2 //secs
    var utteranceRate : Float = 0.62
    var lastSpeechTime = Date()
    var lastMessageSpoken : String = ""
    var lastPushedMessageText: String = ""
    var lastPushTime = Date()
    var isPlaying : Bool
    
    override init(){
        isPlaying = false
        super.init()
        self.synthesizer.delegate = self
        
    }
    
    override func main() {
        var utterance : AVSpeechUtterance
        var message : message_t
        var deltaT : Double
        while run{

            if (!messageQueue.isEmpty) && !isPlaying{
                //pop head of queue
                message = messageQueue.dequeue()!
                deltaT = abs(message.timestamp.timeIntervalSinceNow)
                if (!(message.discardable) || deltaT < staleThreshold){
                    utterance = AVSpeechUtterance(string: message.text)
                    utterance.rate = utteranceRate
                    synthesizer.speak(utterance)
                }
            }
    //    }
        }
    }
    
    func pushMessage(message: message_t){
        print(message)
        if message.highPriority{
            if (message.text != lastPushedMessageText || (lastPushTime.timeIntervalSinceNow) > 5){
                self.lastPushedMessageText = message.text
                self.lastPushTime = Date()
                let utterance = AVSpeechUtterance(string: message.text)
                stopSpeech()
                utterance.rate = utteranceRate
                synthesizer.speak(utterance)
            }
        }
        else{
            if (abs(lastPushTime.timeIntervalSinceNow) > 5){
                self.lastPushedMessageText = message.text
                self.lastPushTime = Date()
                messageQueue.enqueue(message)
            }
        }
    }
    
    func stopSpeech(){
        if (synthesizer.isSpeaking){
            synthesizer.stopSpeaking(at: AVSpeechBoundary.immediate)
        }
    }
    
    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didStart utterance: AVSpeechUtterance) {
      print("AVSpeechSynthesizerDelegate: didStart")
        isPlaying = true
    }

    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didFinish utterance: AVSpeechUtterance) {
      print("AVSpeechSynthesizerDelegate: didFinish")
        isPlaying = false
    }

    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didPause utterance: AVSpeechUtterance) {
      print("AVSpeechSynthesizerDelegate: didPause")
    }

    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didContinue utterance: AVSpeechUtterance) {
      print("AVSpeechSynthesizerDelegate: didContinue")
    }

    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didCancel utterance: AVSpeechUtterance) {
      print("AVSpeechSynthesizerDelegate: didCancel")
    }
    
}

class SpeechFeedback: NSObject{
    
    private var speechThread : SpeechSynthThread

    
    override init(){
//        self.lastSpeechTime = Date()
        speechThread = SpeechSynthThread()
//        lastMessageSpoken = ""
        speechThread.start()
        super.init()
    }
    
    func pushMessage(message: message_t){
            speechThread.pushMessage(message: message)
    }
    
}
