#ifndef MAIN_HPP
#define MAIN_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

struct coordinate {
  int x;
  int y;
};

struct region {
  //x and y coordinate of the region start point
  int startX;
  int startY;
  //x and y coordinate of the region end point
  int endX;
  int endY;
};

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

  // Linyness
  double linyness;
};

#endif // MAIN_HPP

