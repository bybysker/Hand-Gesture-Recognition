//
//  functions.cpp
//  depthsensing
//
//  Created by Elbby AGWU Dev on 08/07/2019.
//  Copyright © 2019 Elbby AGWU Dev. All rights reserved.

#include "functions.hpp"


//default frame width and height
const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=5;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/4;




Mat processing(Mat src)
{
    
    Mat dst, thresh;
    
    
    char window_name[] = "Processed";
    int lowerb = 0; int upperb = 6 ;
    
    
    double min, max;
    minMaxLoc(src, &min, &max);
    cout << max << endl;
    
    cvtColor(src, src, COLOR_RGB2GRAY);
    // Denoising
    GaussianBlur( src, dst, Size( 7, 7 ), 0, 0 );
    
    // Contrast Enhancement
    equalizeHist( dst, dst );
    
    // Extract hand(s) assuming they are the shallower part of the image
    inRange(dst, lowerb, upperb, dst);
    
    //morphological opening (remove small objects from the foreground)
    erode(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    
    

    imshow( window_name, dst);
    return dst;
}

// Draw a point at each detected hand's position
void drawing(vector<Point> detected_hand,Mat &frame)
{
    
    for(int i =0; i<detected_hand.size(); i++)
    {
        circle(frame,detected_hand.at(i),1,Scalar(0,255,0),3);
    }
}



void hand_detector(double &x, double &y, Mat thresholded, Mat &cameraFeed)
{
    
    vector<Point> detected_hand;
    deque<Point> pts; // Vector to store points
    
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(thresholded,contours,hierarchy,RETR_CCOMP,CHAIN_APPROX_SIMPLE );
    //use moments method to find our filtered object
    bool objectFound = false;
    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
            for (int index = 0; index >= 0; index = hierarchy[index][0])
            {
                
                Moments moment = moments((Mat)contours[index]);
                double area = moment.m00;
                
                //if the area is less than MIN_OBJECT_AREA then it is probably just noise
                //if the area is more than 1/4 of the image size, probably the preprocessing is poor or it the person is in front of the camera
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA)
                {
                    x = moment.m10/area;
                    y = moment.m01/area;
                    
                    detected_hand.push_back(Point(x,y));
                    
                    objectFound = true;
                }else objectFound = false;
                
                
            }
           //draw object location on screen
            if(objectFound ==true)
            {
                drawing(detected_hand,cameraFeed);
            }
            
        }
    }
}



void folder2video_clipped (String img_path)
{
    double x, y; // Position of the center of tracked object
    deque<Point> pts; // Vector to store points for DRAWING
    Path2D path; // Vector to store points for the SHAPE IDENTIFICATION
    
    GeometricRecognizer G;
    G.loadTemplates(); // Loading the templates
    
    int counter = 64; // counter to keep the size of pts constant
    
    // Sort all the files of the folder in order to generate the stream
    vector<String> filenames;
    glob(img_path,filenames,true); // recurse
    sort(filenames.begin(), filenames.end());
    
    Mat src, clipped;
    
    for (size_t k=0; k<filenames.size(); ++k)
    {
        
        src = imread(filenames[k]);
        src.copyTo(clipped);
        
        // The most tricky part of the preprocessing :  CLIPPING the maximum value of the image pixels
        threshold(clipped,clipped, 20 , 0, THRESH_TRUNC);
        
        // Binary image of the image processing
        Mat dst = processing(clipped);
        
        hand_detector(x, y, dst, clipped);
        
        pts.push_back (Point(x,y));
        path.push_back(Point2D(x,y));
        
        // Limit the deques to #counter points
        while (pts.size() >= counter)
        {
            pts.pop_front();
            path.pop_front();
        }
        
        // Draw the trajectory starting from the 2nd frame
        if(k>2)
        {
            for ( int i=1; i<pts.size(); i++)
            {
                line(src, pts.at(i-1), pts.at(i), Scalar(0, 255, 0), 2);
            }
            
            // Predict the shape and print it whent it is beyond a limit
            RecognitionResult d = G.GeometricRecognizer::recognize(path);
            if(d.score>=0.75)
            {
                cout << d.name << ":" << d.score << endl;
                putText(src,d.name,Point(0,50),1,2,Scalar(0,255,0),2);
            }
            
        }
        
        // check if we succeeded
        if (src.empty()) continue;
        // show live
        imshow("Live", src);
        if (waitKey(5) >= 0)
            break;
        
        usleep(30000); // For approximately 15 fps
    }
    
}
