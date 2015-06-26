//
//  Object Boundary.cpp
//  Handheld Object Tracking
//
//  Created by Chaudhary Krishneel on 3/11/14.
//  Copyright (c) 2014 Chaudhary Krishneel. All rights reserved.
//

#include "../include/mouse_event.h"

/**
 * Allows the user to interactively select the initial object region
 *
 * @param frame  The frame of video in which objects are to be selected
 * @param region A pointer to an array to be filled with rectangles
 */
Rect ObjectBoundary::cvGetobjectBoundary(const Mat &image){
    
   this->mouseParam = new cvMouseParam();
   Mat frame = image.clone();
    this->mouseParam->winName = "Select the Target Object Region";
    this->mouseParam->image = frame;
    
    cvNamedWindow(this->mouseParam->winName, 1);
    imshow(this->mouseParam->winName, frame);
    cvSetMouseCallback(this->mouseParam->winName, this->cvMouseCallback, this->mouseParam);
    waitKey(0);
    cvDestroyWindow(this->mouseParam->winName);
    
    Rect region;
    region.x = MIN(this->mouseParam->x.x, this->mouseParam->y.x);
    region.y = MIN(this->mouseParam->x.y, this->mouseParam->y.y);
    region.width = MAX(this->mouseParam->x.x, this->mouseParam->y.x) - region.x + sizeof(char);
    region.height = MAX(this->mouseParam->x.y, this->mouseParam->y.y) - region.y + sizeof(char);
    
    return region;
}


/**
 * Function to allow use interaction with the image for object
 * selection using single mouse left click. Function returns the image
 * space clicked registered position
 */

vector<Point> ObjectBoundary::cvGetobjectPosition(const Mat &image)
{
   this->mouseParam = new cvMouseParam();
   Mat frame = image.clone();
   this->mouseParam->winName = "Left click on Object Location";
   this->mouseParam->image = frame;

   cvNamedWindow(this->mouseParam->winName, 1);
   imshow(this->mouseParam->winName, frame);
   cvSetMouseCallback(this->mouseParam->winName, this->cvMousePositionCallback, this->mouseParam);
   waitKey();
   cvDestroyWindow(this->mouseParam->winName);

   return this->mouseParam->obj_position;
}
