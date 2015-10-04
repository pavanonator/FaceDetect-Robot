#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/contrib/detection_based_tracker.hpp>
#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>

#include <stdlib.h>

#include <signal.h>
#include "xboxctrl.h"
#include "coffeebot.h"
#include <armadillo>
#include <pthread.h>

using namespace cv;
using namespace std;

vector<Rect> faces;


static int stopsig;
using namespace arma;

static xboxctrl_t ctrl;
static pthread_t thread;
void *thread_ctrl_update(void *args) {
  while (!stopsig) {
    xboxctrl_update(&ctrl);
  }
  return NULL;
}

void stop(int signo) {
  printf("yo\n");
  stopsig = 1;
}


int main(int argc, char *argv[]) {

  // The header files inside opencv2/contrib are for programs that have been contributed or are experimental

  // Declare the parameters that are needed for the tracker to run;
  // All the parameters are mostly the same that you use for haar cascades.
  signal(SIGINT, stop);
  //xboxctrl_connect(&ctrl);
  CoffeeBot bot;

  //pthread_create(&thread, NULL, thread_ctrl_update, NULL);
  DetectionBasedTracker::Parameters param;
  param.maxObjectSize = 400;
  param.maxTrackLifetime = 20;
  param.minDetectionPeriod = 7;
  param.minNeighbors = 3;
  param.minObjectSize = 20;
  param.scaleFactor = 1.1;

  //The constructer is called with the cascade of choice and the Parameter structure

  DetectionBasedTracker tracker = DetectionBasedTracker("haarcascade_frontalface_alt.xml",param);
  tracker.run();
  VideoCapture cap(0);
  cv::Mat img,gray;
  cv::Rect_<int> face_i;
  cv::namedWindow("Detection Based Tracker",cv::WINDOW_NORMAL);
  cv::setWindowProperty("Detection Based Tracker", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
  char box_text_buf[256];
  for (;;) {
    cap>>img;
    cv::cvtColor(img,gray,CV_RGB2GRAY);
    // The class instance is run on a gray image.
    tracker.process(gray);
    // The results are a vector of Rect that enclose the item being tracked
    tracker.getObjects(faces);
    // if(faces.size() == 0) tracker.resetTracking();
    int index = -1;
    int largest_area = 0;
    int largest_width = 0;
    int largest_height = 0;
    int midx = 0;
    int midy = 0;
	
    	double frontleft=0; 
	double frontright=0;
	double	backleft=0;
	double backright=0;      
    int pos_x, pos_y;
    for (int i = 0; i < faces.size(); i++) {
	frontleft=0; 
	frontright=0;
	backleft=0;
	backright=0;      
	face_i = faces[i];
      // Make a rectangle around the detected item
      int tl_x = std::max(face_i.tl().x, 0);
      int tl_y = std::max(face_i.tl().y, 0);
      int br_x = std::max(face_i.br().x, 0);
      int br_y = std::max(face_i.br().y, 0);
      int width = br_x - tl_x;
      int height = br_y - tl_y;
//      string box_text = format("Tracked Area");
      // And now put it into the image:
      if (index == -1 || width * height > largest_area) {
        index = i;
        largest_area = width * height;
        largest_width = width;
        largest_height = height;
        midx = (br_x + tl_x)/2;
        midy = (br_y + tl_y)/2;
        pos_x = std::max(face_i.tl().x - 10, 0);
        pos_y = std::max(face_i.tl().y - 10, 0);
		    
        if(midx<230){
			    frontleft=0.15;
			    frontright=-0.15;
		    	backleft=0.15;
		    	backright=-0.15;
		    }
		    else if(midx>410){
		    	frontleft=-0.15;
		    	frontright=0.15;
		    	backleft=-0.15;
		    	backright=0.15;
		    }else if(midx>230 && midx<410){
		    	frontleft=-0.2;
		    	frontright=-0.2;
		    	backleft=-0.2;
		    	backright=-0.2;
			  }
      }
    }
    if (index != -1) {
      //slows down as it approaches target
      rectangle(img, faces[index], CV_RGB(0, 255,0), 3);
      sprintf(box_text_buf, "[%f %f %f %f]", frontleft , frontright, backleft, backright);
      putText(img, box_text_buf, Point(pos_x, pos_y), FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(0,255,0), 2.0);
	    if(largest_area<5000){
        bot.send(vec({2.5*frontleft, 2.5*frontright, 2.5*backleft, 2.5*backright,0}));
      }if(largest_area<15000 && largest_area>5000){
        bot.send(vec({1.75*frontleft, 1.75*frontright, 1.75*backleft, 1.75*backright,0}));
      }if(largest_area<28000){
		    bot.send(vec({frontleft, frontright, backleft, backright,0.2}));
	    }else{
		    bot.send(vec({0,0,0,0,0}));	
      }
    }else{
	    bot.send(vec({0,0,0,0,0}));
    }
    cv::imshow("Detection Based Tracker",img);      // Show the results.
    if(cv::waitKey(33) == 27 || stopsig) break;
  }
  tracker.stop();
  //pthread_join(thread, NULL);

  //xboxctrl_disconnect(&ctrl);
  return 0;
  return 0;
}
