#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define T_EDGE 50;
#define DEBUG

using namespace std;

void edgeDetection(int k, cv::Mat *image){

  cv::Vec3b color;
  color[0] = 0;
  color[1] = 0;
  color[2] = 255;

  std::vector<char> edge;

  int SCANLINE_SIZE = image->size().width;
  int scanline[SCANLINE_SIZE];

  int t_edge = 2 * T_EDGE;
  int g_max = -t_edge;
  int g_min = t_edge;
  int x_peak = 0;
  int fx_last = scanline[0];
  int fx = 0;
  int g = 0;

  for (int i=0; i<SCANLINE_SIZE; i++) {
    scanline[i] = (image->at<cv::Vec3b>(k,i))[0];
  }

  for(int x = 2; x < SCANLINE_SIZE; x=x+2) {
    fx = scanline[x];
    g = fx - fx_last;
    if(g > g_max) {
      if(g_min < (-t_edge)) {
        edge.push_back(x_peak);
        image->at<cv::Vec3b>(k,x_peak) = color;
      }
      g_max = g;
      g_min = t_edge;
      x_peak = x - 1;
    }

    if(g < g_min) {
      if(g_max > t_edge) {
        edge.push_back(x_peak);
        image->at<cv::Vec3b>(k,x_peak) = color;
      }
      g_min = g;
      g_max = (-t_edge);
      x_peak = x - 1;
    }
    fx_last = fx;
  }
}


int main()
{
  cv::Mat image;
  image = cv::imread("bottom0007.png", CV_LOAD_IMAGE_COLOR);

  for (int k=0;k<image.size().height; k++){
    edgeDetection(k, &image);
  }

  cv::namedWindow( "image", CV_WINDOW_AUTOSIZE );
  cv::imshow( "image", image);
  cv::waitKey(0);

  return 0;
}
