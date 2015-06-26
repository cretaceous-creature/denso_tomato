//
//  Object Boundary.h
//  Handheld Object Tracking
//
//  Created by Chaudhary Krishneel on 3/11/14.
//  Copyright (c) 2014 Chaudhary Krishneel. All rights reserved.
//

#ifndef __Handheld_Object_Tracking__Object_Boundary__
#define __Handheld_Object_Tracking__Object_Boundary__

#include "../include/constants.h"


//! structure for region selection using mouse
typedef struct cvMouseParam{    
    Point x;
    Point y;
    const char* winName;
    Mat image;

   vector<Point> obj_position;
}cvMouseParam;


/**
 * Class of mouse interaction for object selection either as a region
 * or as point
 */
class ObjectBoundary {

private:
    /**
     * Mouse callback function that allows user to specify the
     * initial object region.
     * Parameters are as specified in OpenCV documentation.
     */
    static void cvMouseCallback(int event, int x, int y, int flags, void* param){
        
        cvMouseParam *p = (cvMouseParam*)param;
        Mat clone;
        static int pressed = false;
        
        if (!p->image.data) {
           ROS_ERROR("No Image Found in MouseCallback");
           return;
        }
    
        if (event == CV_EVENT_LBUTTONDOWN) {
            p->x.x = x;
            p->x.y = y;
            pressed = true;
        }
        //! on left button press, remember first corner of rectangle around object
        else if (event == CV_EVENT_LBUTTONUP){
            p->y.x = x;
            p->y.y = y;
            clone = p->image.clone();
            rectangle(clone, p->x, p->y, Scalar(0,255,0), 2);
            imshow(p->winName, clone);
//            cvDestroyWindow(p->winName);
            pressed = false;
        }
        //! on mouse move with left button down, draw rectangle
        else if (event == CV_EVENT_MOUSEMOVE && flags & CV_EVENT_FLAG_LBUTTON){
            clone = p->image.clone();
            rectangle(clone, p->x, p->y, Scalar(0,255,0),2);
            imshow(p->winName, clone);
//            cvDestroyWindow(p->winName);
        }
    }

   /**
    * Mouse left click callback function
    */
   static void cvMousePositionCallback(int event, int x, int y, int flags, void* param)
   {
      cvMouseParam *p = (cvMouseParam*)param;
      Mat clone = p->image.clone();

      if (p->image.empty())
      {
         ROS_ERROR("No image found for object point localization selection");
         return;
      }
      
      if ( event == CV_EVENT_LBUTTONDOWN )
      {
         p->obj_position.push_back(Point(x,y));
         for (size_t i = 0; i < p->obj_position.size(); i++) {
            cvCross(clone, p->obj_position[i], 5, 2);
         }
         imshow(p->winName, clone);
      }
   }
   
   cvMouseParam *mouseParam;
    
public:
   Rect cvGetobjectBoundary(const Mat &);
   vector<Point> cvGetobjectPosition(const Mat &);
};



#endif /* defined(__Handheld_Object_Tracking__Object_Boundary__) */
