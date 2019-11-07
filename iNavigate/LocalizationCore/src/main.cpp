///*

//
//  main.cpp
//  LocalizationCore
//
//  Created by Giovanni Fusco on 11/1/18.
//  Copyright © 2018 SKERI. All rights reserved.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Core/LocalizationSystem.hpp"
#include "TestUtils/VIOLoggerParser.hpp"


int main(int argc, const char * argv[]) {
    
    testutils::VIOLoggerParser vlp("/Users/gio/workspace/LocalizationCore/LocalizationCore/data/134J", false);
    locore::VIOMeasurements vio = vlp.next();
    //vio = vlp.next();
    int floor = 4;
    locore::LocalizationSystem lsy(50000, locore::InitMode::UNIFORM, floor);
    
    lsy.step(vio);
    while(1){
         vio = vlp.next();
        if (vio.isValid())
            lsy.step(vio);
        else
            break;
    }
    
    
    return 0;
}
