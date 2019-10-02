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

- (void) initNavigationSystem : (NSString*) mapFolder currentFloor:(int)currentFloor exploreMode:(bool)exploreMode;
- (void) initializeLocalizationSystem:(NSString *)resPath numParticles:(int) numParticles posU:(double) posU posV:(double) posV initYaw:(double) initYaw initYawNoise:(double) initYawNoise;
- (void) initializeLocalizationSystemUnknownLocation:(NSString *)resPath numParticles:(int) numParticles initYaw:(double) initYaw initYawNoise:(double) initYawNoise;
- (void) initializeLocalizationSystemUniform:(NSString *)resPath numParticles:(int) numParticles initYaw:(double) initYaw initYawNoise:(double) initYawNoise;

//- (UIImage*) step : (NSString*) trackerStatus timestamp:(double)timestamp x:(double) x y:(double) y z:(double) z  rx:(double) rx ry:(double) ry rz:(double) rz kPa:(double)kPa barometerTS:(double) barometerTS frame:(UIImage*) frame;
//- (UIImage*) step: (NSString*) trackerStatus timestamp:(double)timestamp camera:(ARCamera*) camera kPa:(double)kPa barometerTS:(double) barometerTS frame:(UIImage*) frame;
    
- (NSDictionary*) step : (NSString*) trackerStatus timestamp:(double)timestamp x:(double) x y:(double) y z:(double) z  rx:(double) rx ry:(double) ry rz:(double) rz deltaFloors:(int)deltaFloors frame:(UIImage*) frame;
- (NSDictionary*) step: (NSString*) trackerStatus timestamp:(double)timestamp camera:(ARCamera*) camera deltaFloors:(int)deltaFloors frame:(UIImage*) frame;
    
- (float) getParticlesYaw;
- (float) getNorthCorrectionAngle;

- (void) setDestinationID: (int) destId;
- (NSArray*) getNodeUVPosition : (int)nodeId;
- (void) setCameraHeight : (float) height;

- (UIImage*) getCVDetectorOutputFrame;
- (void) dumpParticles;

@end


NS_ASSUME_NONNULL_END
#endif /* NavigationCoreWrapper_h */
