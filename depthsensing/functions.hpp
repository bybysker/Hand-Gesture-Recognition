//
//  functions.hpp
//  depthsensing
//
//  Created by Elbby AGWU Dev on 08/07/2019.
//  Copyright © 2019 Elbby AGWU Dev. All rights reserved.
//

#ifndef functions_hpp
#define functions_hpp


//OpenCV
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
//C
#include <stdio.h>
//C++
#include <iostream>
#include <deque>
#include <unistd.h>

// $1

#include "GeometricRecognizer.h"
#include "GeometricRecognizerTypes.h"
#include "GestureTemplate.h"
#include "PathWriter.h"
#include "SampleGestures.h"


using namespace cv;
using namespace std;
using namespace DollarRecognizer;

Mat processing(Mat src);
void drawing(vector<Point> hands,Mat &frame);
void hand_detector(double &x, double &y, Mat threshold, Mat &cameraFeed);
void folder2video_clipped (String img_path);



#endif /* functions_hpp */
