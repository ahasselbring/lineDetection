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
  struct coordinate startPoint;
  //x and y coordinate of the region end point
  struct coordinate endPoint;
};

struct lineRegionData {


  struct coordinate upperPoint;   // upperPoint (upperX, upperY)
                                  //  _________x________
                                  //           |          } Line-
                                  //  _________|________  } region
                                  //           x
  struct coordinate lowerPoint;  // lower Point (lowerX,lowerY)


  // Gradient for the upper Point in each direction

  int upperHorizontalGradient = 0;      // Horizontal (Y)  -------->
  int upperVerticalGradient = 0;        // Vertical (X)   |
                                        //                |
                                        //               \ /
                                        //                *

  // Gradient for the upper Point in each direction
  int lowerHorizontalGradient = 0;
  int lowerVerticalGradient = 0;

  // The direction of the upper gradient ( atan(x/y) )
  double upperGradientAngle;

  // The direction of the lower gradient ( atan(x/y) )
  double lowerGradientAngle;

  // Linyness
  double linyness;
};

#endif // MAIN_HPP

