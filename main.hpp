#ifndef MAIN_HPP
#define MAIN_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

struct lineRegionData {

  int upperX;       // upperPoint (upperX, upperY)
  int upperY;       //  _________x________
                    //           |          } Line-
  int lowerX;       //  _________|________  } region
  int lowerY;       //           x
                    // lower Point (lowerX,lowerY)


  // Gradient for the upper Point in each direction

  int upperHorizontalGradient;          // Horizontal (Y)  -------->
  int upperVerticalGradient;            // Vertical (X)   |
                                        //                |
                                        //               \ /
                                        //                *

  // Gradient for the upper Point in each direction
  int lowerHorizontalGradient;
  int lowerVerticalGradient;

  // The direction of the upper gradient ( atan(x/y) )
  double upperGradientAngle;

  // The direction of the lower gradient ( atan(x/y) )
  double lowerGradientAngle;




};

//cv::Vec3b green;// = {0,255,0};
//green[0] = 0;
//green[1] = 255;
//green[2] = 0;
//cv::Vec3b white;
//white[0] = 255;
//white[1] = 255;
//white[2] = 255;
//cv::Vec3b red;
//red[0] = 255;
//red[1] = 0;
//red[2] = 0;

#endif // MAIN_HPP

