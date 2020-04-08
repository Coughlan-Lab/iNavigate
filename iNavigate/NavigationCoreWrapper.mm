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

- (void) initNavigationSystem:(NSString *)mapFolder currentFloor:(int)currentFloor exploreMode:(bool)exploreMode courseDistThr:(float)courseDistThr{
    navsys = new navgraph::NavigationSystem(std::string([mapFolder cStringUsingEncoding:NSUTF8StringEncoding]), currentFloor, exploreMode, courseDistThr);
    logFolderCreated = false;
}

- (void) initializeLocalizationSystem:(NSString *)resPath numParticles:(int) numParticles posU:(double) posU posV:(double) posV initYaw:(double) initYaw initYawNoise:(double) initYawNoise{
    navsys->initLocalizationSystem(std::string([resPath cStringUsingEncoding:NSUTF8StringEncoding]), numParticles, locore::InitMode::AREA_WITH_YAW, posU, posV, initYaw, initYawNoise);

}

- (void) initializeLocalizationSystemLocation:(NSString *)resPath numParticles:(int) numParticles posU:(double) posU posV:(double) posV initYaw:(double) initYaw initYawNoise:(double) initYawNoise{
    navsys->initLocalizationSystem(std::string([resPath cStringUsingEncoding:NSUTF8StringEncoding]), numParticles, locore::InitMode::LOCATION, posU, posV, initYaw, initYawNoise);

}
    
- (void) initializeLocalizationSystemUnknownLocation:(NSString *)resPath numParticles:(int) numParticles initYaw:(double) initYaw initYawNoise:(double) initYawNoise{
    navsys->initLocalizationSystem(std::string([resPath cStringUsingEncoding:NSUTF8StringEncoding]), numParticles, locore::InitMode::UNIFORM_WITH_YAW, -1, -1, initYaw, initYawNoise);
        
    }

- (void) initializeLocalizationSystemUniform:(NSString *)resPath numParticles:(int) numParticles initYaw:(double) initYaw initYawNoise:(double) initYawNoise{
    navsys->initLocalizationSystem(std::string([resPath cStringUsingEncoding:NSUTF8StringEncoding]), numParticles, locore::InitMode::UNIFORM, -1, -1, initYaw, initYawNoise);
}

- (float) getNorthCorrectionAngle{
    assert (navsys != nullptr);
    return navsys->getMapManager()->getAngleToNorth();
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

- (void) setDestination: (int) destId destFloor:(int)destFloor linkID:(int)linkID{
    navsys->destinationId = destId;
    navsys->destFloor = destFloor;
    navsys->linkId = linkID;
}

- (NSDictionary*) step : (NSString*) trackerStatus timestamp:(double)timestamp x:(double) x y:(double) y z:(double) z  rx:(double) rx ry:(double) ry rz:(double) rz deltaFloors:(int)deltaFloors frame:(UIImage*) frame useYaw:(bool) useYaw logParticles:(bool) logParticles{

    cv::Mat image, image_vga,image_vga_gray;
    UIImageToMat(frame, image);
    std::string tStatus([trackerStatus cStringUsingEncoding:NSUTF8StringEncoding]);
//    cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
    cv::resize(image, image_vga, cv::Size(image.cols/3, image.rows/3), 0, 0, cv::INTER_CUBIC);
    cv::cvtColor(image_vga, image_vga_gray, cv::COLOR_RGB2GRAY);
    locore::VIOMeasurements vioData(tStatus, timestamp, x, y, z, rx, ry, rz, 0, 0, image_vga_gray);
//    locore::VIOMeasurements vioData(tStatus, timestamp, x, y, z, rx, ry, rz, 0, 0, image);
    navgraph::NavigationSystem::navigation_t navData = navsys->step(vioData, deltaFloors, useYaw, logParticles);

    NSDictionary *dict = @{ @"outputImage" : MatToUIImage(navsys->getNavigationGraph()),
                            @"heading": [NSNumber numberWithFloat:(navData.course)],
                            @"refAngle": [NSNumber numberWithFloat:(navData.refAngle)],
                            @"instructions": [NSNumber numberWithInt:(navData.instruction)],
                            @"distanceToApproachingNode" : [NSNumber numberWithFloat:(navData.distanceToApproachingNode)],
                            @"validNavData":[NSNumber numberWithBool:(navData.valid)],
                            @"nodeType":[NSNumber numberWithInt:(navData.approachingNodeType)],
                            @"nodeLabel":[NSString stringWithUTF8String:(navData.nodeLabel.c_str())],
                            @"angleError":[NSNumber numberWithFloat:(navData.angleError)],
                            @"destThroughDoor":[NSNumber numberWithBool:(navData.destThroughDoor)],
                            @"nodePositionU":[NSNumber numberWithFloat:(navData.nodeUVPos[0])],
                            @"nodePositionV":[NSNumber numberWithFloat:(navData.nodeUVPos[1])],
                            @"userPositionU":[NSNumber numberWithFloat:(navData.userUVPos[0])],
                            @"userPositionV":[NSNumber numberWithFloat:(navData.userUVPos[1])],
                            @"turnNodePositionU":[NSNumber numberWithFloat:(navData.turnNodeUVPos[0])],
                            @"turnNodePositionV":[NSNumber numberWithFloat:(navData.turnNodeUVPos[1])],
                            @"yawVariance":[NSNumber numberWithFloat:(navData.yawVariance)],
                            @"validTurnNode":[NSNumber numberWithBool:(navData.validTurnNode)]
                        };
    
        return dict;
}

//-(NSArray*) getYawHistogram{
//    std::vector<int> hist = navsys->getYawHistogram(); 
//    id yawHist = [NSMutableArray new];
//    std::for_each(hist.begin(), hist.end(), ^(int cnt) {
//        id count = [NSNumber numberWithInt:(cnt)];
//        [yawHist addObject:count];
//    });
//    return yawHist;
//}


- (NSDictionary*) getParticlesStats{
    locore::ParticleFilter::ParticlesStats pstats = navsys->getParticlesStats();
    NSDictionary *dict = @{ @"gcf_min" : [NSNumber numberWithFloat: (pstats.gsc_min)],
                            @"gcf_max": [NSNumber numberWithFloat:(pstats.gsc_max)],
                            @"gcf_mean": [NSNumber numberWithFloat:(pstats.gsc_mean)]
                        };
    
        return dict;
    
}

- (NSDictionary*) step : (NSString*) trackerStatus timestamp:(double)timestamp camera:(ARCamera*) camera deltaFloors:(int)deltaFloors frame:(UIImage*) frame useYaw:(bool) useYaw logParticles:(bool) logParticles{
    double x = camera.transform.columns[3].x;
    double y = camera.transform.columns[3].y;
    double z = camera.transform.columns[3].z;
    
    double rx = camera.eulerAngles[0];
    double ry = camera.eulerAngles[1];
    double rz = camera.eulerAngles[2];
    
    return [self step:trackerStatus timestamp:timestamp x:x y:y z:z rx:rx ry:ry rz:rz deltaFloors:deltaFloors frame:frame useYaw:useYaw logParticles:logParticles];
}

- (void) setCameraHeight : (float) height{
    navsys->setCameraHeight(height);
}

//- (void) setInitialCourse : (float) yaw{
//    navsys->setInitialCourse(yaw);
//}

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
