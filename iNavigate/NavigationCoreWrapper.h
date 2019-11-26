//
//  NavigationCoreWrapper.h
//  iNavigate
//
//  Created by Giovanni Fusco on 4/2/19.
//  Copyright © 2019 Smith-Kettlewell Eye Research Institute. All rights reserved.
//

#ifndef NavigationCoreWrapper_h
#define NavigationCoreWrapper_h

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#import <ARKit/ARKit.h>
NS_ASSUME_NONNULL_BEGIN

@interface NavigationCoreWrapper : NSObject

typedef NS_ENUM(NSInteger,TurnDirection) { Forward = 0, TurnAround = -1, Left = 1, Right = 2, EasyLeft = 3, EasyRight = 4, None = -2, Arrived = 100, LeftToDest = 101, RightToDest = 102, ForwardToDest = 103 };
typedef NS_ENUM(NSInteger, NodeType) { Control = 0, Destination = 1, Link = 2, Undef = -1 };

- (void) initNavigationSystem : (NSString*) mapFolder currentFloor:(int)currentFloor exploreMode:(bool)exploreMode;
- (void) initializeLocalizationSystem:(NSString *)resPath numParticles:(int) numParticles posU:(double) posU posV:(double) posV initYaw:(double) initYaw initYawNoise:(double) initYawNoise;
- (void) initializeLocalizationSystemLocation:(NSString *)resPath numParticles:(int) numParticles posU:(double) posU posV:(double) posV initYaw:(double) initYaw initYawNoise:(double) initYawNoise;
- (void) initializeLocalizationSystemUnknownLocation:(NSString *)resPath numParticles:(int) numParticles initYaw:(double) initYaw initYawNoise:(double) initYawNoise;
- (void) initializeLocalizationSystemUniform:(NSString *)resPath numParticles:(int) numParticles initYaw:(double) initYaw initYawNoise:(double) initYawNoise;
  
- (NSDictionary*) step : (NSString*) trackerStatus timestamp:(double)timestamp x:(double) x y:(double) y z:(double) z  rx:(double) rx ry:(double) ry rz:(double) rz deltaFloors:(int)deltaFloors frame:(UIImage*) frame;
- (NSDictionary*) step: (NSString*) trackerStatus timestamp:(double)timestamp camera:(ARCamera*) camera deltaFloors:(int)deltaFloors frame:(UIImage*) frame;
    
- (float) getParticlesYaw;
- (float) getNorthCorrectionAngle;

- (void) setDestinationID: (int) destId;
- (NSArray*) getNodeUVPosition : (int)nodeId;
- (void) setCameraHeight : (float) height;
//- (void) setInitialCourse : (float) yaw;
- (UIImage*) getCVDetectorOutputFrame;
- (void) dumpParticles;
//- (NSArray*) getYawHistogram;
@end


NS_ASSUME_NONNULL_END
#endif /* NavigationCoreWrapper_h */
