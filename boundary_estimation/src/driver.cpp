//
//  driver.cpp
//  Object boundary estimation
//
//  Created by Chaudhary Krishneel on 11/11/14.
//  Copyright (c) 2014 Chaudhary Krishneel. All rights reserved.
//

#include "../include/constants.h"
#include "../include/Point_Cloud_Processing.h"

/********************************************************************
  Main Program
*******************************************************************/
int main(int argc, char** argv)
 {
    ros::init(argc, argv, "PointCloud");
    PointCloudProcessing pcp;
    ros::spin();

    return 0;
 }
