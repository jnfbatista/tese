
#include "processfunc.h"

// GLOBAL VARIABLES
    // VARIABLES TO LOAD FROM ".INI" FILE
        int regionsThreshold = 0;
        int contoursThreshold = 0;
        int houghPoints = 0;
        int houghGap = 0;
        int houghVotes = 1;
        int houghTheta = 1;
        double calibAngle = 0;

// Load from ".INI" file
    bool loadFromFile( void ) {
        const char* fileName = "configs.ini";
        string varName;

        ifstream configFile;
        configFile.open( fileName, ios::in );

        if( configFile.is_open() ) {
            configFile >> varName; configFile >> regionsThreshold;
            configFile >> varName; configFile >> contoursThreshold;
            configFile >> varName; configFile >> houghGap;
            configFile >> varName; configFile >> houghPoints;
            configFile >> varName; configFile >> houghTheta;
            configFile >> varName; configFile >> houghVotes;
            configFile >> varName; configFile >> calibAngle;
            configFile.close();
            return true;
        }
        else
            return false;
    }


// STRING CONVERSION FUNCTIONS
    // Template for string to integer conversion
        double convert_Str2Double( string s ) {
            stringstream ss( s );
            double number;
            ss >> number;
            return number;
        }

    // Integer to String Conversion
        string convert_Double2Str( double number ) {
            stringstream ss;                                                                // create a stringstream
            ss << number;                                                                   // add number to the stream
            return ss.str();                                                                // return a string with the contents of the stream
        }

// CALCULATING INFORMATION
    // Extract orientation of and distance to lines
        void getInfo( vector<Vec4i>* array_lines, Mat result, vector< vector<float> > kinectZdist ) {
            // Declarations
                float Z = 0;
                double angle = 0, distance = 0, Zproj = 0;
                Vec4i l;
                Message m;
                int count = 0;

            // Check for valid array
                if( !array_lines->empty() ) {
                    for( size_t i=0; i<array_lines->size(); i++) {
                        l = array_lines->at(i);
                        double x = l[2]-l[0], y = (l[3]-l[1]);
                        // Get angle from "good" lines
                        if( l[1]>=result.size().height-10 || l[3]>=result.size().height-10 ) {
                                count++;
                                if( x == 0 )
                                    angle = angle - CV_PI/2;
                                else
                                    angle = angle - atan( y/x ) - calibAngle;
                                Z = kinectZdist[l[0]][l[1]];
                                Zproj = Z*cos( asin(0.55/Z));
                                distance = distance + Zproj*sin( atan(y/x)-calibAngle );
                            }
                    }
                    angle = angle/count;
                    distance = distance/count;
                }

            // Send Message
                m.add_param( "distance", convert_Double2Str(distance) );
                m.add_param( "angle", convert_Double2Str(angle) );
                m.send_message( _IP, _PORT );
        }

// PROCESSING
    /* Processing Image
    **
    ** */
    bool imageProcessing( Mat input, vector< vector<float> > kinectXdist ) {
        // Declarations
            vector<Vec4i> lines;
            Vec4i l;
            Mat inputImage, outputImage;

        // Check for valid input
            if( !input.data )
                return false;

        // LOAD configurations
            if( !loadFromFile() )
                return false;

        // FILTERING
            // Smoothes an image and downsamples it
                pyrDown(input, inputImage);
            // Convert image from one color space to another (here to GrayScale)
                cvtColor( inputImage, inputImage, CV_RGB2GRAY );

        // BINARIZATION
            // Apply a fixed-level threshold to each array element (here threshold type is BINARY)
                threshold( inputImage, inputImage, regionsThreshold, _REGIONS_THRESHOLD_MAX_VALUE, THRESH_BINARY );

        // SKELETIZATION
            // Calculate the precise distance from every binary image pixel to the nearest zero pixel
                distanceTransform( inputImage, inputImage, CV_DIST_L2, CV_DIST_MASK_PRECISE );
            // Convert one array to another with optional linear transformation (here not used)
                inputImage.convertTo( outputImage, CV_8UC1 );
            // Apply adaptive threshold to an array
                adaptiveThreshold( outputImage, outputImage, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 5, 0 );

        // HOUGH TRANSFORM
            // Finding line segments in a binary image using probabilistic Hough transform
                HoughLinesP( outputImage, lines, 1, houghTheta*CV_PI/180, houghVotes, houghPoints, houghGap );
            // Load background
                outputImage = imread( _BLACK_BACKGROUND );
            // Check for valid input
                if( !outputImage.data )
                    return false;
            // Paint lines in background
                for( size_t i = 0; i < lines.size(); i++ ) {
                    l = lines[i];
                    line( outputImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,255), 1, CV_AA );
                }

        // Send information
            getInfo( &lines, outputImage, kinectXdist );

        // EXIT
            return true;
    }
