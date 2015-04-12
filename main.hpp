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

  struct coordinate middlePoint;
};

struct lineData {
  struct coordinate startPoint;
  struct coordinate endPoint;
};

struct coordinate subtractVector(struct coordinate point1, struct coordinate point2);


struct coordinate addVector(struct coordinate point1, struct coordinate point2);
#endif // MAIN_HPP

