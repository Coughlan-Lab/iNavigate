//
//  AudioFeedback.swift
//  iNavigate
//
//  Created by Giovanni Fusco on 10/2/19.
//  Copyright © 2019 Smith-Kettlewell Eye Research Institute. All rights reserved.
//

import Foundation

public struct Queue<T> {
  fileprivate var array = [T?]()
  fileprivate var head = 0
  
  public var isEmpty: Bool {
    return count == 0
  }

  public var count: Int {
    return array.count - head
  }
  
  public mutating func enqueue(_ element: T) {
    array.append(element)
  }
  
  public mutating func dequeue() -> T? {
    guard head < array.count, let element = array[head] else { return nil }

    array[head] = nil
    head += 1

    let percentage = Double(head)/Double(array.count)
    if array.count > 50 && percentage > 0.25 {
      array.removeFirst(head)
      head = 0
    }
    
    return element
  }
  
  public var front: T? {
    if isEmpty {
      return nil
    } else {
      return array[head]
    }
  }
}

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

class SpeechSynthThread: Thread{
    var synthesizer : AVSpeechSynthesizer
    var messageQueue = Queue<message_t>()
    var run: Bool = true
    let staleThreshold : Double = 2 //secs
    var utteranceRate : Float = 0.62
    var lastSpeechTime = Date()
    var lastMessageSpoken : String = ""
    var lastPushedMessageText: String = ""
    var lastPushTime = Date()
    
    override init(){
        synthesizer = AVSpeechSynthesizer()
        super.init()
    }
    
    override func main() {
        while run{
            if !synthesizer.isSpeaking{
                if (messageQueue.count > 0){
                    //pop head of queue
                    let message = messageQueue.dequeue()
                    let deltaT = abs(message!.timestamp.timeIntervalSinceNow)
                    if (!(message!.discardable) || deltaT < staleThreshold){
                        let utterance = AVSpeechUtterance(string: message!.text)
                        utterance.rate = utteranceRate
                        synthesizer.speak(utterance)
                    }
                }
            }
        }
    }
    
    func pushMessage(message: message_t){
        print(message)
        if message.highPriority{
            if (message.text != lastPushedMessageText || abs(lastPushTime.timeIntervalSinceNow) > 1.5){
                self.lastPushedMessageText = message.text
                self.lastPushTime = Date()
                let utterance = AVSpeechUtterance(string: message.text)
                stopSpeech()
                utterance.rate = utteranceRate
                synthesizer.speak(utterance)
            }
        }
        else{
            if (message.text != lastPushedMessageText || abs(lastPushTime.timeIntervalSinceNow) > 5){
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

//extension SpeechFeedback: AVSpeechSynthesizerDelegate{
//    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didStart utterance: AVSpeechUtterance) {
//      print("AVSpeechSynthesizerDelegate: didStart")
//    }
//
//    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didFinish utterance: AVSpeechUtterance) {
//      print("AVSpeechSynthesizerDelegate: didFinish")
//    }
//
//    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didPause utterance: AVSpeechUtterance) {
//      print("AVSpeechSynthesizerDelegate: didPause")
//    }
//
//    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didContinue utterance: AVSpeechUtterance) {
//      print("AVSpeechSynthesizerDelegate: didContinue")
//    }
//
//    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didCancel utterance: AVSpeechUtterance) {
//      print("AVSpeechSynthesizerDelegate: didCancel")
//    }
//}

