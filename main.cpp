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


void edgeDetectionOnScanline(int column, cv::Mat &image, int t_edge, std::vector<coordinate> &edges){

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
        struct coordinate peak;
        peak.x=x_peak;
        peak.y=column;
        edges.push_back(peak);
      }
      g_max = g;
      g_min = t_edge;
      x_peak = x - 1;
    }

    if(g < g_min) {
      if(g_max > t_edge) {
        struct coordinate peak;
        peak.x=x_peak;
        peak.y=column;
        edges.push_back(peak);
      }
      g_min = g;
      g_max = (-t_edge);
      x_peak = x - 1;
    }
    fx_last = fx;
  }
}

void edgeDetection(cv::Mat &image, int t_edge, std::vector<struct coordinate> &edges) {
  //+2 cause of downsampling/subsampling
  for (int column=0;column<image.size().width; column+=1){
    struct coordinate topPixel;
    struct coordinate bottomPixel;
    topPixel.x=0;
    topPixel.y=column;
    bottomPixel.x=image.size().height-1;
    bottomPixel.y=column;
    edges.push_back(topPixel);
    edgeDetectionOnScanline(column, image, t_edge, edges);
    edges.push_back(bottomPixel);
  }
}

void classifyRegions(cv::Mat &image, std::vector<struct coordinate> &edges, std::vector<struct region> &fieldRegions, std::vector<struct region> &lineRegions, std::vector<struct region> &unknownRegions){
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
    struct region currentRegion;
    currentRegion.startPoint.x=edges[i].x; //currentX
    currentRegion.startPoint.y=edges[i].y; //currentColumn
    currentRegion.endPoint.x=edges[i+1].x; //nextX
    currentRegion.endPoint.y=edges[i].y; //currentColumn
    nextColumn = edges[i+1].y;
    if((nextColumn-currentColumn)==0){
      diff = nextX-currentX;
      gap = diff/6; //TODO: magic number
      median_Y = medianOfFive(image.at<cv::Vec3b>(currentX+1*gap,currentColumn)[0],image.at<cv::Vec3b>(currentX+2*gap,currentColumn)[0],image.at<cv::Vec3b>(currentX+3*gap,currentColumn)[0],image.at<cv::Vec3b>(currentX+4*gap,currentColumn)[0],image.at<cv::Vec3b>(currentX+5*gap,currentColumn)[0]);
      median_Cb = medianOfFive(image.at<cv::Vec3b>(currentX+1*gap,currentColumn)[1],image.at<cv::Vec3b>(currentX+2*gap,currentColumn)[1],image.at<cv::Vec3b>(currentX+3*gap,currentColumn)[1],image.at<cv::Vec3b>(currentX+4*gap,currentColumn)[1],image.at<cv::Vec3b>(currentX+5*gap,currentColumn)[1]);
      median_Cr = medianOfFive(image.at<cv::Vec3b>(currentX+1*gap,currentColumn)[2],image.at<cv::Vec3b>(currentX+2*gap,currentColumn)[2],image.at<cv::Vec3b>(currentX+3*gap,currentColumn)[2],image.at<cv::Vec3b>(currentX+4*gap,currentColumn)[2],image.at<cv::Vec3b>(currentX+5*gap,currentColumn)[2]);
      if(fieldCheck(median_Y,median_Cb,median_Cr)){

        //update the last vector to connect contiguous regions to hold the vector small
        if(fieldRegions.size()>0){
          if(fieldRegions[fieldRegions.size()-1].endPoint.x == currentX && fieldRegions[fieldRegions.size()-1].endPoint.y == currentColumn) {
            fieldRegions[fieldRegions.size()-1].endPoint.x = nextX;
          }
          else {
            fieldRegions.push_back(currentRegion);
          }
        }
        else{
          //first entry of current column
          fieldRegions.push_back(currentRegion);
        }
      }
      else{
        if(unknownRegions.size()>0){
          if(unknownRegions[unknownRegions.size()-1].endPoint.x == currentX && unknownRegions[unknownRegions.size()-1].endPoint.y == currentColumn) {
            unknownRegions[unknownRegions.size()-1].endPoint.x = nextX;
          }
          else {
            unknownRegions.push_back(currentRegion);
          }
        }
        else{
          //first entry of current column
          unknownRegions.push_back(currentRegion);
        }
      }
    }
  }

  //line regions:
  for(unsigned int i = 0; i < unknownRegions.size(); i++) {
    //TODO switched x and y at at
    if(unknownRegions[i].startPoint.x > 0 && unknownRegions[i].endPoint.x < image.size().height ) { //TODO some regions are not processed
      if( (image.at<cv::Vec3b>(unknownRegions[i].startPoint.x-1,unknownRegions[i].startPoint.y)[0] + Y_THRESHOLD) < image.at<cv::Vec3b>( unknownRegions[i].startPoint.x,unknownRegions[i].startPoint.y)[0] ) {
        if( (image.at<cv::Vec3b>(unknownRegions[i].endPoint.x+1,unknownRegions[i].startPoint.y)[0] + Y_THRESHOLD) < image.at<cv::Vec3b>( unknownRegions[i].endPoint.x,unknownRegions[i].startPoint.y)[0]) {
          lineRegions.push_back(unknownRegions[i]);
        }
      }
    }
  }
}

void drawResults(cv::Mat &imageRegions, vector<struct region> &fieldRegions, vector<struct region> &lineRegions, vector<struct region> &unknownRegions, vector<struct lineRegionData> &gradientVector, vector<coordinate> &edges){
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
    int startX = unknownRegions[i].startPoint.x;
    int endX = unknownRegions[i].endPoint.x;
    int column = unknownRegions[i].startPoint.y;
    for(int k=startX; k<=endX; k++){
      imageRegions.at<cv::Vec3b>(k,column) = red;
    }
  }
  for(unsigned int i=0; i<fieldRegions.size(); i++){
    int startX = fieldRegions[i].startPoint.x;
    int endX = fieldRegions[i].endPoint.x;
    int column = fieldRegions[i].startPoint.y;
    for(int k=startX; k<=endX; k++){
      imageRegions.at<cv::Vec3b>(k,column) = green;
    }
  }
  for(unsigned int i=0; i<lineRegions.size(); i++){
    int startX = lineRegions[i].startPoint.x;
    int endX = lineRegions[i].endPoint.x;
    int column = lineRegions[i].startPoint.y;
    for(int k=startX; k<=endX; k++){
      imageRegions.at<cv::Vec3b>(k,column) = white;
    }
  }
  for(unsigned int i=0; i<edges.size(); i++){
    cout << edges[i].x << "," << edges[i].y << endl;
    imageRegions.at<cv::Vec3b>(edges[i].x,edges[i].y) = black;
  }

  for(unsigned int i=0; i<gradientVector.size(); i++){
    //cv::line(imageRegions, cv::Point(gradientVector[i][1],gradientVector[i][0]), cv::Point(gradientVector[i][5]+gradientVector[i][1],gradientVector[i][4]+gradientVector[i][0]),cv::Scalar(yellow),1,8);
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

bool boundaryCheck(const cv::Mat &image, const int &x, const int &y){
  if(x>0&&y>0&&x<image.size().height&&y<image.size().width){
    return true;
  }
  return false;
}

/**
 * @brief calculateGradient
 * This method uses a sobel operator to smooth and differentiate the picture in x direction
 * @param image
 * The image which should be processed
 * @param lineRegions
 * The candidate points from the edgeDetection
 */
void calculateLineGradients(const cv::Mat &image, const vector<struct region> &lineRegions, vector<struct lineRegionData> &gradientVector) {

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


  struct lineRegionData lineData;

  for(unsigned int i=0;i< lineRegions.size();i++) {


    lineData.upperPoint.x = lineRegions[i].startPoint.x;
    lineData.upperPoint.y = lineRegions[i].startPoint.y;
    lineData.lowerPoint.x = lineRegions[i].endPoint.x;
    lineData.lowerPoint.y = lineRegions[i].endPoint.y;

    if(boundaryCheck(image,lineData.upperPoint.x,lineData.upperPoint.y)) {  // TODO: we should use a bool function for that

      lineData.upperVerticalGradient = 1 * image.at<cv::Vec3b>(lineData.upperPoint.x-1,lineData.upperPoint.y-1)[0] + 2 * image.at<cv::Vec3b>(lineData.upperPoint.x-1,lineData.upperPoint.y)[0] + 1 * image.at<cv::Vec3b>(lineData.upperPoint.x-1,lineData.upperPoint.y+1)[0] - 1 * image.at<cv::Vec3b>(lineData.upperPoint.x+1,lineData.upperPoint.y-1)[0] - 2 * image.at<cv::Vec3b>(lineData.upperPoint.x+1,lineData.upperPoint.y)[0] - 1 * image.at<cv::Vec3b>(lineData.upperPoint.x+1,lineData.upperPoint.y+1)[0];
      lineData.upperHorizontalGradient = 1 * image.at<cv::Vec3b>(lineData.upperPoint.x-1,lineData.upperPoint.y-1)[0] + 2 * image.at<cv::Vec3b>(lineData.upperPoint.x,lineData.upperPoint.y-1)[0] + 1 * image.at<cv::Vec3b>(lineData.upperPoint.x+1,lineData.upperPoint.y-1)[0] - 1 * image.at<cv::Vec3b>(lineData.upperPoint.x-1,lineData.upperPoint.y+1)[0] - 2 * image.at<cv::Vec3b>(lineData.upperPoint.x,lineData.upperPoint.y+1)[0] - 1 * image.at<cv::Vec3b>(lineData.upperPoint.y+1,lineData.upperPoint.y+1)[0];
      //cout << "Gradient Angle: " << calculateGradientAngle(upperVerticalGradient, upperHorizontalGradient) << endl;
      //cout << " Upper Y Gradient : " << upperHorizontalGradient << "Upper X Gradient: " << upperVerticalGradient << endl;
    }
    if(boundaryCheck(image,lineData.lowerPoint.x,lineData.lowerPoint.y)) {  // we should use a bool function for that
      lineData.lowerVerticalGradient = 1 * image.at<cv::Vec3b>(lineData.lowerPoint.x-1,lineData.lowerPoint.y-1)[0] + 2 * image.at<cv::Vec3b>(lineData.lowerPoint.x-1,lineData.lowerPoint.y)[0] + 1 * image.at<cv::Vec3b>(lineData.lowerPoint.x-1,lineData.lowerPoint.y+1)[0] - 1 * image.at<cv::Vec3b>(lineData.lowerPoint.x+1,lineData.lowerPoint.y+1)[0] - 2 * image.at<cv::Vec3b>(lineData.lowerPoint.x+1,lineData.lowerPoint.y)[0] - 1 * image.at<cv::Vec3b>(lineData.lowerPoint.x+1,lineData.lowerPoint.y+1)[0];
      lineData.lowerHorizontalGradient = 1 * image.at<cv::Vec3b>(lineData.lowerPoint.x-1,lineData.lowerPoint.y-1)[0] + 2 * image.at<cv::Vec3b>(lineData.lowerPoint.x,lineData.lowerPoint.y-1)[0] + 1 * image.at<cv::Vec3b>(lineData.lowerPoint.x+1,lineData.lowerPoint.y-1)[0] - 1 * image.at<cv::Vec3b>(lineData.lowerPoint.x-1,lineData.lowerPoint.y+1)[0] - 2 * image.at<cv::Vec3b>(lineData.lowerPoint.x,lineData.lowerPoint.y+1)[0] - 1 * image.at<cv::Vec3b>(lineData.lowerPoint.x+1,lineData.lowerPoint.y+1)[0];
    }

    gradientVector.push_back(lineData);
  }
}

double evaluateAdjacentPoints(const vector<cv::Vec6i> &gradientVector) {
}

double scalarProduct(struct coordinate point1, struct coordinate point2) {
  return point1.x*point2.x+point1.y*point2.y;
}



double evaluateAdjacentPoints( vector<struct lineRegionData> &gradientVector) {
  for(unsigned int i = 0; i < gradientVector.size(); i++) {
    if(scalarProduct(gradientVector[i].upperPoint, gradientVector[i+1].upperPoint) > 0) {
      //gradientVector[i].linyness = 1 -
    }
  }
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

  vector<struct coordinate> edges;
  vector<struct region> fieldRegions; //startX startY endX endY
  vector<struct region> lineRegions; //startX startY endX endY
  vector<struct region> unknownRegions; //startX startY endX endY
  vector<struct lineRegionData> gradientVector;

  std::chrono::duration<float, std::ratio<1, 1000>> delta1;
  std::chrono::duration<float, std::ratio<1, 1000>> delta2;
  std::chrono::duration<float, std::ratio<1, 1000>> delta3;

  chrono::time_point<std::chrono::system_clock> startTime = chrono::system_clock::now();
  edgeDetection(image, T_EDGE, edges);
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
