/*
 * The PoC program for Human Detection (HD)
 *
 * @author Dan-Marius Dobrea
 * Program developed for: HoverGames Challenge 1: Fight Fire with Flyers
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <time.h>

using namespace std;
using namespace cv;

enum Mode { Default, Daimler } m;

string modeName(void) ;
string modeName(void) 
{ return (m == Default ? "Default" : "Daimler"); }

int main() 
{
    HOGDescriptor hog, hog_d(Size(48, 96), Size(16, 16), Size(8, 8), Size(8, 8), 9);
    
    m = Default; 
    
    VideoCapture capture(0);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    if(!capture.isOpened())
        {
        cout << "Failed to connect to the camera." << endl;
        }
        
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());   
    hog_d.setSVMDetector(HOGDescriptor::getDaimlerPeopleDetector()); 
    
    Mat img;
    
    for (;;)
        {
        capture >> img;
        if(img.empty())
            {
            cout << "Failed to capture an image" << endl;
            return -1;
            }
   
        int64 t = getTickCount();
        // Run the detector with default parameters. to get a higher hit-rate
        // (and more false alarms, respectively), decrease the hitThreshold and
        // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).    
        vector<Rect> found;
        if (m == Default)
            hog.detectMultiScale(img, found, 0, Size(8,8), Size(), 1.05, 2, false);
        else if (m == Daimler)
            hog_d.detectMultiScale(img, found, 0, Size(8,8), Size(), 1.05, 2, true);
        t = getTickCount() - t;
        
        // show the window
        {
            ostringstream buf;
            buf << "Mode: " << modeName() << " ||| "
                << "FPS: "  << (getTickFrequency() / (double)t);
            putText(img, buf.str(), Point(10, 30), FONT_HERSHEY_PLAIN, 2.0, Scalar(0, 0, 255), 2, false);
        }        
        
        for (vector<Rect>::iterator i = found.begin(); i != found.end(); ++i)
            {
            Rect &r = *i;
            // The HOG detector returns slightly larger rectangles than the real objects,
            // so we slightly shrink the rectangles to get a nicer output.
            r.x     += cvRound(r.width*0.1);
            r.width  = cvRound(r.width*0.8);
            r.y     += cvRound(r.height*0.07);
            r.height = cvRound(r.height*0.8);            
            
            rectangle(img, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
            }
        imshow("People detector", img);                      
                    
        // interact with user
        const char key = (char)waitKey(1);
        if (key == 27 || key == 'q') // ESC
        {
            cout << "Exit requested" << endl;
            break;
        }
        else if (key == ' ')
            {m = (m == Default ? Daimler : Default);}
        }    
    
    //cout << "It took " << difference << " seconds to process " << frames << " frames" << endl;
    //cout << "Capturing and processing " << frames/difference << " frames per second " << endl;
    cout << "Capturing and processing " << difference << " seconds " << endl;

    return 0;
}
