#ifndef PROCESSFUNC_H
#define PROCESSFUNC_H



// INCLUDES
    #include <opencv/cv.h>
    #include <opencv/highgui.h>
    #include "../Message/Message.h"
    #include <iostream>                                     // for standard I/O
    #include <string>                                       // for string handling functions
    #include <fstream>                                      // for file handling functions
    #include <vector>                                       // for vector handling functions
    #include <math.h>                                       // for math functions
    #include <sstream>                                      // for string streams


// NAMESPACES
    using namespace cv;
    using namespace std;

// DEFINES
    #define _BLACK_BACKGROUND               "black_back.jpg"
    #define _REGIONS_THRESHOLD_MAX_VALUE        255
    #define _CONTOURS_THRESHOLD_MAX_VALUE       255
    #define _HOUGH_GAP_MAX_VALUE                100
    #define _HOUGH_POINTS_MAX_VALUE             100
    #define _HOUGH_VOTES_MAX_VALUE              100
    #define _HOUGH_THETA_MAX_VALUE              180
    #define _PORT                              "4445"


// FUNCTIONS
    extern bool imageProcessing( Mat , vector<vector<float> >, string address);

#endif // PROCESSFUNC_H
