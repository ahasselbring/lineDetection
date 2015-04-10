#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <chrono>
#include <ctime>
#include <main.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define DEBUG
#define T_EDGE 8
#define FIELD_Y 126
#define FIELD_CB 114
#define FIELD_CR 74
#define T_Y 62
#define T_CB 12
#define T_CR 30 //19
#define Y_THRESHOLD 0 //30  //TODO: to be optimized
#define PI 3.14159265

using namespace std;

int medianOfFive(int a, int b, int c, int d, int e){
  return b < a ? d < c ? b < d ? a < e ? a < d ? e < d ? e : d
                                                       : c < a ? c : a
                                                               : e < d ? a < d ? a : d
                                                                               : c < e ? c : e
                                                                                       :c < e ? b < c ? a < c ? a : c
                                                                                                              : e < b ? e : b
                                                                                                                      : b < e ? a < e ? a : e
                                                                                                                                      : c < b ? c : b
                                                                                                                                              : b < c ? a < e ? a < c ? e < c ? e : c
                                                                                                                                                                              : d < a ? d : a
                                                                                                                                                                                      : e < c ? a < c ? a : c
                                                                                                                                                                                                      : d < e ? d : e
                                                                                                                                                                                                              : d < e ? b < d ? a < d ? a : d
                                                                                                                                                                                                                                      : e < b ? e : b
                                                                                                                                                                                                                                              : b < e ? a < e ? a : e
                                                                                                                                                                                                                                                              : d < b ? d : b
                                                                                                                                                                                                                                                                      : d < c ? a < d ? b < e ? b < d ? e < d ? e : d
                                                                                                                                                                                                                                                                                                              : c < b ? c : b
                                                                                                                                                                                                                                                                                                                      : e < d ? b < d ? b : d
                                                                                                                                                                                                                                                                                                                                      : c < e ? c : e
                                                                                                                                                                                                                                                                                                                                              : c < e ? a < c ? b < c ? b : c
                                                                                                                                                                                                                                                                                                                                                                      : e < a ? e : a
                                                                                                                                                                                                                                                                                                                                                                              : a < e ? b < e ? b : e
                                                                                                                                                                                                                                                                                                                                                                                              : c < a ? c : a
                                                                                                                                                                                                                                                                                                                                                                                                      : a < c ? b < e ? b < c ? e < c ? e : c
                                                                                                                                                                                                                                                                                                                                                                                                                                      : d < b ? d : b
                                                                                                                                                                                                                                                                                                                                                                                                                                              : e < c ? b < c ? b : c
                                                                                                                                                                                                                                                                                                                                                                                                                                                              : d < e ? d : e
                                                                                                                                                                                                                                                                                                                                                                                                                                                                      : d < e ? a < d ? b < d ? b : d
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              : e < a ? e : a
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      : a < e ? b < e ? b : e
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      : d < a ? d : a;
}

bool fieldCheck(int Y, int Cb, int Cr) {
  if((Cr-FIELD_CR)<T_CR){
    if((Cb-FIELD_CB)<T_CB){
      if((Y-FIELD_Y)<T_Y){
        return true;
      }
    }
  }
  return false;
}

void edgeDetectionOnScanline(int column, cv::Mat &image,cv::Mat &imageEdges, int t_edge, std::vector<cv::Vec2i> &edges){

  cv::Vec3b color;
  color[0] = 255;
  color[1] = 255;
  color[2] = 255;

  int SCANLINE_SIZE = image.size().height;
  int scanline[SCANLINE_SIZE];

  int g_max = -t_edge;
  int g_min = t_edge;
  int x_peak = 0;
  int fx_last = scanline[0];
  int fx = 0;
  int g = 0;

  std::chrono::duration<float, std::ratio<1, 1000>> delta;
  chrono::time_point<std::chrono::system_clock> startTime = chrono::system_clock::now();
  for (int x=0; x<SCANLINE_SIZE; x++) {
    scanline[x] = (image.at<cv::Vec3b>(x,column))[0];
  }
  delta = chrono::system_clock::now() - startTime;
  cout << "These ms are to subtract from sum: " << delta.count() << endl;

  for(int x = 2; x < SCANLINE_SIZE; x=x+2) {
    fx = scanline[x];
    g = fx - fx_last;
    if(g > g_max) {
      if(g_min < (-t_edge)) {
        edges.push_back(cv::Vec2i(x_peak,column));
      }
      g_max = g;
      g_min = t_edge;
      x_peak = x - 1;
    }

    if(g < g_min) {
      if(g_max > t_edge) {
        edges.push_back(cv::Vec2i(x_peak,column));
      }
      g_min = g;
      g_max = (-t_edge);
      x_peak = x - 1;
    }
    fx_last = fx;
  }
}

void edgeDetection(cv::Mat &image, cv::Mat &imageEdges, int t_edge, std::vector<cv::Vec2i> &edges) {
  //+2 cause of downsampling/subsampling
  for (int column=0;column<image.size().width; column+=1){
    edges.push_back(cv::Vec2i(0,column));
    edgeDetectionOnScanline(column, image, imageEdges, t_edge, edges);
    edges.push_back(cv::Vec2i(image.size().height,column));
  }
}

void classifyRegions(cv::Mat &image, std::vector<cv::Vec2i> &edges, std::vector<cv::Vec4i> &fieldRegions, std::vector<cv::Vec4i> &lineRegions, std::vector<cv::Vec4i> &unknownRegions){
  int currentColumn = 0;
  int nextColumn = 0;
  int currentX = 0;
  int nextX = 0;
  int diff = 0;
  int gap = 0;
  int median_Y = 0;
  int median_Cb = 0;
  int median_Cr = 0;
  for (unsigned int i=0; i<edges.size()-1; i++){
    currentX = edges[i][0];
    nextX = edges[i+1][0];
    currentColumn = edges[i][1];
    nextColumn = edges[i+1][1];
    if((nextColumn-currentColumn)==0){
      diff = nextX-currentX;
      gap = diff/6; //TODO: magic number
      median_Y = medianOfFive(image.at<cv::Vec3b>(currentX+1*gap,currentColumn)[0],image.at<cv::Vec3b>(currentX+2*gap,currentColumn)[0],image.at<cv::Vec3b>(currentX+3*gap,currentColumn)[0],image.at<cv::Vec3b>(currentX+4*gap,currentColumn)[0],image.at<cv::Vec3b>(currentX+5*gap,currentColumn)[0]);
      median_Cb = medianOfFive(image.at<cv::Vec3b>(currentX+1*gap,currentColumn)[1],image.at<cv::Vec3b>(currentX+2*gap,currentColumn)[1],image.at<cv::Vec3b>(currentX+3*gap,currentColumn)[1],image.at<cv::Vec3b>(currentX+4*gap,currentColumn)[1],image.at<cv::Vec3b>(currentX+5*gap,currentColumn)[1]);
      median_Cr = medianOfFive(image.at<cv::Vec3b>(currentX+1*gap,currentColumn)[2],image.at<cv::Vec3b>(currentX+2*gap,currentColumn)[2],image.at<cv::Vec3b>(currentX+3*gap,currentColumn)[2],image.at<cv::Vec3b>(currentX+4*gap,currentColumn)[2],image.at<cv::Vec3b>(currentX+5*gap,currentColumn)[2]);
      if(fieldCheck(median_Y,median_Cb,median_Cr)){
        //update the last vector to connect contiguous regions to hold the vector small
        if(fieldRegions.size()>0){
          if(fieldRegions[fieldRegions.size()-1][2] == currentX && fieldRegions[fieldRegions.size()-1][3] == currentColumn) {
            fieldRegions[fieldRegions.size()-1][2] = nextX;
          }
          else {
            fieldRegions.push_back(cv::Vec4i(currentX,currentColumn, nextX, currentColumn));
          }
        }
        else{
          //first entry of current column
          fieldRegions.push_back(cv::Vec4i(currentX,currentColumn, nextX, currentColumn));
        }
      }
      else{
        if(unknownRegions.size()>0){
          if(unknownRegions[unknownRegions.size()-1][2] == currentX && unknownRegions[unknownRegions.size()-1][3] == currentColumn) {
            unknownRegions[unknownRegions.size()-1][2] = nextX;
          }
          else {
            unknownRegions.push_back(cv::Vec4i(currentX,currentColumn, nextX, currentColumn));
          }
        }
        else{
          //first entry of current column
          unknownRegions.push_back(cv::Vec4i(currentX,currentColumn, nextX, currentColumn));
        }
      }
    }
  }

  //line regions:
  for(unsigned int i = 0; i < unknownRegions.size(); i++) {

    if(unknownRegions[i][0] > 0 && unknownRegions[i][2] < image.size().height ) {
      if( (image.at<cv::Vec3b>(unknownRegions[i][0]-1,unknownRegions[i][1])[0] + Y_THRESHOLD) < image.at<cv::Vec3b>( unknownRegions[i][0],unknownRegions[i][1])[0] ) {
        if( (image.at<cv::Vec3b>(unknownRegions[i][2]+1,unknownRegions[i][1])[0] + Y_THRESHOLD) < image.at<cv::Vec3b>( unknownRegions[i][2],unknownRegions[i][1])[0]) {
          lineRegions.push_back(unknownRegions[i]);
        }
      }
    }
  }
}

void drawResults(cv::Mat &imageRegions, vector<cv::Vec4i> &fieldRegions, vector<cv::Vec4i> &lineRegions, vector<cv::Vec4i> &unknownRegions, vector<cv::Vec8i> &gradientVector, vector<cv::Vec2i> &edges){
  cv::Vec3b green;
  green[0] = 0; //B
  green[1] = 255; //G
  green[2] = 0; //R
  cv::Vec3b white;
  white[0] = 255;
  white[1] = 255;
  white[2] = 255;
  cv::Vec3b red;
  red[0] = 0;
  red[1] = 0;
  red[2] = 255;
  cv::Vec3b blue;
  blue[0] = 255;
  blue[1] = 0;
  blue[2] = 0;
  cv::Vec3b black;
  black[0] = 0;
  black[1] = 0;
  black[2] = 0;
  cv::Vec3b yellow;
  yellow[0]=0;
  yellow[1]=255;
  yellow[2]=255;

  for(unsigned int i=0; i<unknownRegions.size(); i++){
    int startX = unknownRegions[i][0];
    int endX = unknownRegions[i][2];
    int column = unknownRegions[i][1];
    for(int k=startX; k<=endX; k++){
      imageRegions.at<cv::Vec3b>(k,column) = red;
    }
  }
  for(unsigned int i=0; i<fieldRegions.size(); i++){
    int startX = fieldRegions[i][0];
    int endX = fieldRegions[i][2];
    int column = fieldRegions[i][1];
    for(int k=startX; k<=endX; k++){
      imageRegions.at<cv::Vec3b>(k,column) = green;
    }
  }
  for(unsigned int i=0; i<lineRegions.size(); i++){
    int startX = lineRegions[i][0];
    int endX = lineRegions[i][2];
    int column = lineRegions[i][1];
    for(int k=startX; k<=endX; k++){
      imageRegions.at<cv::Vec3b>(k,column) = white;
    }
  }
  for(unsigned int i=0; i<edges.size(); i++){
    imageRegions.at<cv::Vec3b>(edges[i][0],edges[i][1]) = black;
  }

  for(unsigned int i=0; i<gradientVector.size(); i++){
    //cv::arrowedLine(imageRegions, cv::Point(gradientVector[i][1],gradientVector[i][0]), cv::Point(gradientVector[i][5]+gradientVector[i][1],gradientVector[i][4]+gradientVector[i][0]),cv::Scalar(yellow),1,8);
    //upperX,upperY,lowerX,lowerY,upperVerticalGradient,upperHorizontalGradient,lowerVerticalGradient,lowerHorizontalGradient
  }
}

double calculateGradientAngle(int x, int y) {
  double angle = 0;

  if(y != 0) { //usefull :D
    angle = atan(x/y)*180/PI;
  }
  //cout << "Gradient Angle: " << angle << endl;
  return angle;
}

/**
 * @brief calculateGradient
 * This method uses a sobel operator to smooth and differentiate the picture in x direction
 * @param image
 * The image which should be processed
 * @param lineRegions
 * The candidate points from the edgeDetection
 */
void calculateLineGradients(const cv::Mat &image, const vector<cv::Vec4i> &lineRegions, vector<cv::Vec8i> &gradientVector) {

  /*Sobel operators:
   *
   * 1 0 -1
   * 2 0 -2
   * 1 0 -1
   *
   * 1 2 1
   * 0 0 0
   * -1 -2 -1
   */

  //gradient of the upper point
  signed int upperVerticalGradient = 0;
  signed int upperHorizontalGradient = 0;

  //gradient of the lower point
  signed int lowerVerticalGradient = 0;
  signed int lowerHorizontalGradient = 0;


  int upperX = 0;
  int upperY = 0;

  int lowerX = 0;
  int lowerY = 0;

  for(unsigned int i=0;i< lineRegions.size();i++) {

    upperX = lineRegions[i][0];
    upperY = lineRegions[i][1];

    lowerX = lineRegions[i][2];
    lowerY = lineRegions[i][3];
    if(upperX > 0 && upperY > 0 && upperX < image.size().width && upperY < image.size().height) {  // TODO: we should use a bool function for that

      upperVerticalGradient = 1 * image.at<cv::Vec3b>(upperX-1,upperY-1)[0] + 2 * image.at<cv::Vec3b>(upperX-1,upperY)[0] + 1 * image.at<cv::Vec3b>(upperX-1,upperY+1)[0] - 1 * image.at<cv::Vec3b>(upperX+1,upperY-1)[0] - 2 * image.at<cv::Vec3b>(upperX+1,upperY)[0] - 1 * image.at<cv::Vec3b>(upperX+1,upperY+1)[0];
      upperHorizontalGradient = 1 * image.at<cv::Vec3b>(upperX-1,upperY-1)[0] + 2 * image.at<cv::Vec3b>(upperX,upperY-1)[0] + 1 * image.at<cv::Vec3b>(upperX+1,upperY-1)[0] - 1 * image.at<cv::Vec3b>(upperX-1,upperY+1)[0] - 2 * image.at<cv::Vec3b>(upperX,upperY+1)[0] - 1 * image.at<cv::Vec3b>(upperY+1,upperY+1)[0];
      //cout << "Gradient Angle: " << calculateGradientAngle(upperVerticalGradient, upperHorizontalGradient) << endl;
      //cout << " Upper Y Gradient : " << upperHorizontalGradient << "Upper X Gradient: " << upperVerticalGradient << endl;
    }
    if(lowerX > 0 && lowerY > 0 && lowerX < image.size().width && lowerY < image.size().height) {  // we should use a bool function for that
      lowerVerticalGradient = 1 * image.at<cv::Vec3b>(lowerX-1,lowerY-1)[0] + 2 * image.at<cv::Vec3b>(lowerX-1,lowerY)[0] + 1 * image.at<cv::Vec3b>(lowerX-1,lowerY+1)[0] - 1 * image.at<cv::Vec3b>(lowerX+1,lowerY+1)[0] - 2 * image.at<cv::Vec3b>(lowerX+1,lowerY)[0] - 1 * image.at<cv::Vec3b>(lowerX+1,lowerY+1)[0];
      lowerHorizontalGradient = 1 * image.at<cv::Vec3b>(lowerX-1,lowerY-1)[0] + 2 * image.at<cv::Vec3b>(lowerX,lowerY-1)[0] + 1 * image.at<cv::Vec3b>(lowerX+1,lowerY-1)[0] - 1 * image.at<cv::Vec3b>(lowerX-1,lowerY+1)[0] - 2 * image.at<cv::Vec3b>(lowerX,lowerY+1)[0] - 1 * image.at<cv::Vec3b>(lowerY+1,lowerY+1)[0];
    }
    gradientVector.push_back(cv::Vec8i(upperX,upperY,lowerX,lowerY,upperVerticalGradient,upperHorizontalGradient,lowerVerticalGradient,lowerHorizontalGradient)) ;
  }
}

double evaluateAdjacentPoints(const vector<cv::Vec6i> &gradientVector) {
  return 0;
}

int main() {
  cv::Mat image;
  image = cv::imread("bottom0007.png", CV_LOAD_IMAGE_COLOR);
  cv::Mat imageEdges(image.size().height, image.size().width, CV_8UC3);
  //cv::Mat imageRegions(image.size().height, image.size().width, CV_8UC3);
  cv::Mat imageRegions;
  imageRegions = cv::imread("bottom0007.png", CV_LOAD_IMAGE_COLOR);
  cv::Mat imageLines(image.size().height, image.size().width, CV_8UC3);

  vector<cv::Vec2i> edges;
  vector<cv::Vec4i> fieldRegions; //startX startY endX endY
  vector<cv::Vec4i> lineRegions; //startX startY endX endY
  vector<cv::Vec4i> unknownRegions; //startX startY endX endY
  vector<cv::Vec8i> gradientVector;

  std::chrono::duration<float, std::ratio<1, 1000>> delta1;
  std::chrono::duration<float, std::ratio<1, 1000>> delta2;
  std::chrono::duration<float, std::ratio<1, 1000>> delta3;

  chrono::time_point<std::chrono::system_clock> startTime = chrono::system_clock::now();
  edgeDetection(image, imageEdges, T_EDGE, edges);
  delta1 = chrono::system_clock::now() - startTime;
  classifyRegions(image, edges, fieldRegions, lineRegions, unknownRegions);
  delta2 = chrono::system_clock::now() - startTime - delta1;
  calculateLineGradients(image, lineRegions, gradientVector);
  delta3 = chrono::system_clock::now() - startTime - delta1 - delta2;
  drawResults(imageRegions, fieldRegions, lineRegions, unknownRegions, gradientVector, edges);

  cout << "fieldRegions: " << fieldRegions.size() << endl;
  cout << "unknownRegions: " << unknownRegions.size() << endl;

  printf("edgeDetection: %fms\n",delta1.count());
  printf("classifyRegions: %fms\n",delta2.count());
  printf("calculateLineGradients: %fms\n",delta3.count());
  printf("sum: %fms\n",delta1.count()+delta2.count()+delta3.count());
  cv::imwrite ("result_regions.png",imageRegions, vector<int>());
  cv::imwrite ("result_lines.png",imageLines, vector<int>());

  return 0;
}
