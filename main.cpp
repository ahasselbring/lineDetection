#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <chrono>
#include <ctime>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define DEBUG

using namespace std;

void edgeDetectionOnScanline(int column, cv::Mat *image,cv::Mat *imageEdges, int t_edge){

  cv::Vec3b color;
  color[0] = 255;
  color[1] = 255;
  color[2] = 255;

  std::vector<char> edge;

  int SCANLINE_SIZE = image->size().height;
  int scanline[SCANLINE_SIZE];

  int g_max = -t_edge;
  int g_min = t_edge;
  int x_peak = 0;
  int fx_last = scanline[0];
  int fx = 0;
  int g = 0;

  for (int x=0; x<SCANLINE_SIZE; x++) {
    scanline[x] = (image->at<cv::Vec3b>(x,column))[0];
  }

  for(int x = 2; x < SCANLINE_SIZE; x=x+2) {
    fx = scanline[x];
    g = fx - fx_last;
    if(g > g_max) {
      if(g_min < (-t_edge)) {
        edge.push_back(x_peak);
        imageEdges->at<cv::Vec3b>(x_peak,column) = color;
      }
      g_max = g;
      g_min = t_edge;
      x_peak = x - 1;
    }

    if(g < g_min) {
      if(g_max > t_edge) {
        edge.push_back(x_peak);
        imageEdges->at<cv::Vec3b>(x_peak,column) = color;
      }
      g_min = g;
      g_max = (-t_edge);
      x_peak = x - 1;
    }
    fx_last = fx;
  }
}

void edgeDetection(cv::Mat *image,cv::Mat *imageEdges, int t_edge) {
  for (int column=0;column<image->size().width; column+=2){
    edgeDetectionOnScanline(column, image, imageEdges, t_edge);
  }
}

int main()
{
  cv::Mat image;
  image = cv::imread("bottom0007.png", CV_LOAD_IMAGE_COLOR);
  cv::Mat imageEdges(image.size().height, image.size().width, CV_8UC3);

  int t_edge;
  t_edge = 40;
  std::chrono::duration<float, std::ratio<1, 1000>> delta;

  chrono::time_point<std::chrono::system_clock> startTime = chrono::system_clock::now();
  edgeDetection(&image, &imageEdges, t_edge);
  delta = chrono::system_clock::now() - startTime;

  printf("%f",delta.count());
  cv::imwrite ("40.png",imageEdges, vector<int>());


  return 0;
}
