//
//  NavigationCoreWrapper.m
//  iNavigate
//
//  Created by Giovanni Fusco on 4/2/19.
//  Copyright © 2019 Smith-Kettlewell Eye Research Institute. All rights reserved.
//

#import "LocalizationSystem.hpp"
#import "NavigationCore/NavigationSystem.hpp"
#include <vector>
#import "NavigationCoreWrapper.h"

#import <opencv2/imgcodecs/ios.h>
//#import <Foundation/Foundation.h>

@interface NavigationCoreWrapper() {
    navgraph::NavigationSystem* navsys;
    bool logFolderCreated;
    NSString *logFolder;
    long unsigned int frameCnt;
}
@end

@implementation NavigationCoreWrapper

- (void) initNavigationSystem:(NSString *)mapFolder currentFloor:(int)currentFloor{
    navsys = new navgraph::NavigationSystem(std::string([mapFolder cStringUsingEncoding:NSUTF8StringEncoding]), currentFloor);
    logFolderCreated = false;
}

- (void) initializeLocalizationSystem:(NSString *)resPath numParticles:(int) numParticles posU:(double) posU posV:(double) posV initYaw:(double) initYaw initYawNoise:(double) initYawNoise{
    navsys->initLocalizationSystem(std::string([resPath cStringUsingEncoding:NSUTF8StringEncoding]), numParticles, locore::InitMode::AREA_WITH_YAW, posU, posV, initYaw, initYawNoise);

}
    
- (void) initializeLocalizationSystemUnknownLocation:(NSString *)resPath numParticles:(int) numParticles initYaw:(double) initYaw initYawNoise:(double) initYawNoise{
    navsys->initLocalizationSystem(std::string([resPath cStringUsingEncoding:NSUTF8StringEncoding]), numParticles, locore::InitMode::UNIFORM_WITH_YAW, -1, -1, initYaw, initYawNoise);
        
    }

- (float) getParticlesYaw{
    return navsys->getParticlesYaw();
}

-(NSArray*) getNodeUVPosition : (int)nodeId{
    std::vector<float> pos = navsys->getNodeUVPosition(nodeId);
    NSArray *outpos = [[NSArray alloc] initWithObjects:
    [NSNumber numberWithDouble:pos[0]],
    [NSNumber numberWithDouble:pos[1]],
    nil];
    return outpos;
}

- (void) setDestinationID: (int) destId{
    navsys->destinationId = destId;
}

- (UIImage*) step : (NSString*) trackerStatus timestamp:(double)timestamp x:(double) x y:(double) y z:(double) z  rx:(double) rx ry:(double) ry rz:(double) rz deltaFloors:(int)deltaFloors frame:(UIImage*) frame{
    
    cv::Mat image, image_vga,image_vga_gray;
    UIImageToMat(frame, image);
    std::string tStatus([trackerStatus cStringUsingEncoding:NSUTF8StringEncoding]);
    cv::resize(image, image_vga, cv::Size(image.cols/3,image.rows/3), 0, 0, cv::INTER_NEAREST);
    cv::cvtColor(image_vga, image_vga_gray, cv::COLOR_RGB2GRAY);
    locore::VIOMeasurements vioData(tStatus, timestamp, x, y, z, rx, ry, rz, 0, 0, image_vga_gray);
    cv::Mat kde = navsys->step(vioData, deltaFloors);
    return MatToUIImage(kde);
}

- (UIImage*) step : (NSString*) trackerStatus timestamp:(double)timestamp camera:(ARCamera*) camera deltaFloors:(int)deltaFloors frame:(UIImage*) frame{
    double x = camera.transform.columns[3].x;
    double y = camera.transform.columns[3].y;
    double z = camera.transform.columns[3].z;
    
    double rx = camera.eulerAngles[0];
    double ry = camera.eulerAngles[1];
    double rz = camera.eulerAngles[2];
    
    return [self step:trackerStatus timestamp:timestamp x:x y:y z:z rx:rx ry:ry rz:rz deltaFloors:deltaFloors frame:frame];
}

- (void) setCameraHeight : (float) height{
    navsys->setCameraHeight(height);
}

- (UIImage*) getCVDetectorOutputFrame {
    cv::Mat frame = navsys->getCVDetectorOutputFrame();
    return MatToUIImage(frame);
}

- (void) dumpParticles{
    if (!logFolderCreated){
        frameCnt = 0;
       
        NSTimeInterval timeStamp = [[NSDate date] timeIntervalSince1970];
        NSInteger ts = timeStamp;
        // NSTimeInterval is defined as double
        NSString *tstampString = [NSString stringWithFormat:@"/%ld", (long)ts];
        
        NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        logFolder = [paths objectAtIndex:0];
        logFolder = [logFolder stringByAppendingString:tstampString];
        NSFileManager *manager = [NSFileManager defaultManager];
        NSError* error;
        [manager createDirectoryAtPath:logFolder withIntermediateDirectories:false attributes:nil error:&error];
        logFolderCreated = true;
    }
    frameCnt += 1;
    navsys->dumpParticles(std::string([logFolder cStringUsingEncoding:NSUTF8StringEncoding]), frameCnt);
}

@end
