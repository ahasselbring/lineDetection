#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
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
  for (int column=0;column<image->size().width; column++){
    edgeDetectionOnScanline(column, image, imageEdges, t_edge);
  }
}

int main()
{
  cv::Mat image;
  image = cv::imread("bottom0007.png", CV_LOAD_IMAGE_COLOR);
  cv::Mat imageEdges_0(image.size().height, image.size().width, CV_8UC3);
  cv::Mat imageEdges_40(image.size().height, image.size().width, CV_8UC3);
  cv::Mat imageEdges_80(image.size().height, image.size().width, CV_8UC3);
  cv::Mat imageEdges_120(image.size().height, image.size().width, CV_8UC3);
  cv::Mat imageEdges_160(image.size().height, image.size().width, CV_8UC3);

  int t_edge;
  t_edge = 0;

  edgeDetection(&image, &imageEdges_0, t_edge);
  cv::imwrite ("0.png",imageEdges_0, vector<int>());
  t_edge = 40;
  edgeDetection(&image, &imageEdges_40, t_edge);
  cv::imwrite ("40.png", imageEdges_40, vector<int>());
  t_edge = 80;
  edgeDetection(&image, &imageEdges_80, t_edge);
  cv::imwrite ("80.png", imageEdges_80, vector<int>());
  t_edge = 120;
  edgeDetection(&image, &imageEdges_120, t_edge);
  cv::imwrite ("120.png", imageEdges_120, vector<int>());
  t_edge = 160;
  edgeDetection(&image, &imageEdges_160, t_edge);
  cv::imwrite ("160.png", imageEdges_160, vector<int>());

  //cv::imshow("40", imageEdges_40);

  return 0;
}
