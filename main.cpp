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
  cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE);
  cv::imshow( "Display window", image);

  cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(0,0));
  color[0] = 0;
  color[1] = 0;
  color[2] = 255;

  int SCANLINE_SIZE = image.size().width;
  char scanline[SCANLINE_SIZE];

  std::vector<char> edge;

  char t_edge = 2 * T_EDGE;
  char g_max = -t_edge;
  char g_min = t_edge;
  char x_peak = 0;
  char fx_last = scanline[0];
  char fx = 0;
  char g = 0;

  printf("%d",image.size().height);

  for (int k=0;k<image.size().height; k++){
    cv::Mat row = image.row(k);

    printf("\r\nSCANLINE %d\r\n",k);
    //  for(int i = 0; i < SCANLINE_SIZE; i++) {
    //    scanline[i] = (char) rand() % 255;

    for (int i=0; i<SCANLINE_SIZE; i++) {
      scanline[i] = (row.at<cv::Vec3b>(0,i))[0];
    //printf("%d \r\n ",scanline[i]);
    }

    for(int x = 2; x < SCANLINE_SIZE; x+=2) {

      fx = scanline[x];
      g = fx - fx_last;

      if(g > g_max) {

        if(g_min < -t_edge) {
          edge.push_back(x_peak);
           //Pixel färben
          image.at<cv::Vec3b>(cv::Point(k,x_peak)) = color;
          printf("colored");
        }
        g_max = g;
        g_min = t_edge;
        x_peak = x - 1;
      }

      if(g < g_min) {

        if(g_max > t_edge) {
          edge.push_back(x_peak);
          //Pixel färben
          image.at<cv::Vec3b>(cv::Point(k,x_peak)) = color;
          printf("colored");
        }
        g_min = g;
        g_max = -t_edge;
        x_peak = x - 1;
      }
      fx_last = fx;

    }

    printf("\r\nEDGE POINTS %d\r\n", k);
    for(int i = 0;i < edge.size(); i++) {
        printf("%d \r\n", edge[i]);
    }
  }

  cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
  // Show our image inside it.
  cv::imshow( "Display window", image);
  cv::waitKey(0);

  return 0;
}
