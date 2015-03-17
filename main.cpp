#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define T_EDGE 50;
#define DEBUG

using namespace std;

int main()
{
  cv::Mat image;
  image = cv::imread("bottom0007.png", CV_LOAD_IMAGE_COLOR);
  cv::Mat row = image.row(100);

  int SCANLINE_SIZE = row.size().width;

  char scanline[SCANLINE_SIZE];
  for (int i=0; i<SCANLINE_SIZE; i++) {
    //TODO: Something wrong in the following
    scanline[i] = (row.at<cv::Vec3b>(0,i))[0];
  }

  std::vector<char> edge;
  //char edge[SCANLINE_SIZE] = {};
  char t_edge = 2 * T_EDGE;
  char g_max = -t_edge;
  char g_min = t_edge;
  char x_peak = 0;
  char fx_last = scanline[0];
  char fx = 0;
  char g = 0;


  printf("\r\n**************SCANLINE*******************\r\n");
  /* Scanline mit random Werten setzen */
  for(int i = 0; i < SCANLINE_SIZE; i++) {
    scanline[i] = (char) rand() % 255;
#ifdef DEBUG

    printf("%d \r\n ",scanline[i]);

#endif
  }

   printf("***************************************** \r\n");

  for(int x = 2; x < SCANLINE_SIZE; x+=2) {

    fx = scanline[x];
    g = fx - fx_last;

    if(g > g_max) {

      if(g_min < -t_edge) {
        edge.push_back(x_peak);
      }
      g_max = g;
      g_min = t_edge;
      x_peak = x - 1;
    }

    if(g < g_min) {

      if(g_max > t_edge) {
        edge.push_back(x_peak);
      }
      g_min = g;
      g_max = -t_edge;
      x_peak = x - 1;
    }
    fx_last = fx;

  }

#ifdef DEBUG
  printf("\r\n**************EDGE POINTS*******************\r\n");
  for(int i = 0;i < edge.size(); i++) {
    printf("%d \r\n", edge[i]);
  }
   printf("***************************************** \r\n");
#endif





  return 0;
}
