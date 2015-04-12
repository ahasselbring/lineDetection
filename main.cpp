#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <cmath>
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
  if(abs(Cr-FIELD_CR)<T_CR){
    if(abs(Cb-FIELD_CB)<T_CB){
      if(abs(Y-FIELD_Y)<T_Y){
        return true;
      }
    }
  }
  return false;
}

//yep
void edgeDetectionOnScanline(int column, cv::Mat &image, int t_edge, std::vector<coordinate> &edges){

  int SCANLINE_SIZE = image.size().height;
  int scanline[SCANLINE_SIZE];

  int g_max = -t_edge;
  int g_min = t_edge;
  int yPeak = 0;
  int fYLast = 0;
  int fY = 0;
  int g = 0;

  for (int y=0; y<SCANLINE_SIZE; y++) {
    scanline[y] = (image.at<cv::Vec3b>(y,column))[0];
  }
  
  fYLast = scanline[0];

  for(int y = 2; y < SCANLINE_SIZE; y=y+2) {
    fY = scanline[y];
    g = fY - fYLast;
    if(g > g_max) {
      if(g_min < (-t_edge)) {
        struct coordinate peak;
        peak.y=yPeak;
        peak.x=column;
        edges.push_back(peak);
      }
      g_max = g;
      g_min = t_edge;
      yPeak = y - 1;
    }

    if(g < g_min) {
      if(g_max > t_edge) {
        struct coordinate peak;
        peak.y=yPeak;
        peak.x=column;
        edges.push_back(peak);
      }
      g_min = g;
      g_max = (-t_edge);
      yPeak = y - 1;
    }
    fYLast = fY;
  }
}

//yep
void edgeDetection(cv::Mat &image, int t_edge, std::vector<struct coordinate> &edges) {
  //+2 cause of downsampling/subsampling
  for (int column=0;column<image.size().width; column+=16){
    struct coordinate topPixel;
    struct coordinate bottomPixel;
    topPixel.y=0;
    topPixel.x=column;
    bottomPixel.y=image.size().height-1;
    bottomPixel.x=column;
    edges.push_back(topPixel);
    edgeDetectionOnScanline(column, image, t_edge, edges);
    edges.push_back(bottomPixel);
  }
}

//yep
void classifyRegions(cv::Mat &image, std::vector<struct coordinate> &edges, std::vector<struct region> &fieldRegions, std::vector<struct region> &lineRegions, std::vector<struct region> &unknownRegions){
  int nextColumn = 0;
  int diff = 0;
  int gap = 0;
  int median_Y = 0;
  int median_Cb = 0;
  int median_Cr = 0;
  for (unsigned int i=0; i<edges.size()-1; i++){
    struct region currentRegion;
    currentRegion.startPoint.y=edges[i].y; //currentX
    currentRegion.startPoint.x=edges[i].x; //currentColumn
    currentRegion.endPoint.y=edges[i+1].y; //nextX
    currentRegion.endPoint.x=edges[i].x; //currentColumn
    nextColumn = edges[i+1].x;
    if((nextColumn-currentRegion.startPoint.x)==0){
      diff = currentRegion.endPoint.y-currentRegion.startPoint.y;
      gap = diff/6; //TODO: magic number
      median_Y = medianOfFive(image.at<cv::Vec3b>(currentRegion.startPoint.y+1*gap,currentRegion.startPoint.x)[0],image.at<cv::Vec3b>(currentRegion.startPoint.y+2*gap,currentRegion.startPoint.x)[0],image.at<cv::Vec3b>(currentRegion.startPoint.y+3*gap,currentRegion.startPoint.x)[0],image.at<cv::Vec3b>(currentRegion.startPoint.y+4*gap,currentRegion.startPoint.x)[0],image.at<cv::Vec3b>(currentRegion.startPoint.y+5*gap,currentRegion.startPoint.x)[0]);
      median_Cb = medianOfFive(image.at<cv::Vec3b>(currentRegion.startPoint.y+1*gap,currentRegion.startPoint.x)[1],image.at<cv::Vec3b>(currentRegion.startPoint.y+2*gap,currentRegion.startPoint.x)[1],image.at<cv::Vec3b>(currentRegion.startPoint.y+3*gap,currentRegion.startPoint.x)[1],image.at<cv::Vec3b>(currentRegion.startPoint.y+4*gap,currentRegion.startPoint.x)[1],image.at<cv::Vec3b>(currentRegion.startPoint.y+5*gap,currentRegion.startPoint.x)[1]);
      median_Cr = medianOfFive(image.at<cv::Vec3b>(currentRegion.startPoint.y+1*gap,currentRegion.startPoint.x)[2],image.at<cv::Vec3b>(currentRegion.startPoint.y+2*gap,currentRegion.startPoint.x)[2],image.at<cv::Vec3b>(currentRegion.startPoint.y+3*gap,currentRegion.startPoint.x)[2],image.at<cv::Vec3b>(currentRegion.startPoint.y+4*gap,currentRegion.startPoint.x)[2],image.at<cv::Vec3b>(currentRegion.startPoint.y+5*gap,currentRegion.startPoint.x)[2]);
      if(fieldCheck(median_Y,median_Cb,median_Cr)){
        //update the last vector to connect contiguous regions to hold the vector small
        if(fieldRegions.size()>0){
          if(fieldRegions[fieldRegions.size()-1].endPoint.y == currentRegion.startPoint.y && fieldRegions[fieldRegions.size()-1].endPoint.x == currentRegion.startPoint.x) {
            fieldRegions[fieldRegions.size()-1].endPoint.y = currentRegion.endPoint.y;
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
          if(unknownRegions[unknownRegions.size()-1].endPoint.y == currentRegion.startPoint.y && unknownRegions[unknownRegions.size()-1].endPoint.x == currentRegion.startPoint.x) {
            unknownRegions[unknownRegions.size()-1].endPoint.y = currentRegion.endPoint.y;
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
    if(unknownRegions[i].startPoint.y > 0 && unknownRegions[i].endPoint.y < image.size().height ) { //TODO some regions are not processed
      if( (image.at<cv::Vec3b>(unknownRegions[i].startPoint.y-1,unknownRegions[i].startPoint.x)[0]) < image.at<cv::Vec3b>( unknownRegions[i].startPoint.y,unknownRegions[i].startPoint.x)[0] ) {
        if( (image.at<cv::Vec3b>(unknownRegions[i].endPoint.y+1,unknownRegions[i].startPoint.x)[0]) < image.at<cv::Vec3b>( unknownRegions[i].endPoint.y,unknownRegions[i].startPoint.x)[0]) {
          unknownRegions[i].middlePoint = addVector(unknownRegions[i].startPoint, unknownRegions[i].endPoint);
          unknownRegions[i].middlePoint.x /= 2;
          unknownRegions[i].middlePoint.y /= 2;
          lineRegions.push_back(unknownRegions[i]);
        }
      }
    }
  }
}


void drawResults(cv::Mat &imageRegions, vector<struct region> &fieldRegions, vector<struct region> &lineRegions, \
  vector<struct region> &unknownRegions, vector<coordinate> &edges, vector<struct lineData> &lineVector, vector<struct coordinate> &intersections)
{
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
  cv::Vec3b cyan;
  cyan[0] = 255;
  cyan[1] = 255;
  cyan[2] = 0;

//  for(unsigned int i=0; i<unknownRegions.size(); i++){
//    int startY = unknownRegions[i].startPoint.y;
//    int endY = unknownRegions[i].endPoint.y;
//    int column = unknownRegions[i].startPoint.x;
//    for(int y=startY; y<=endY; y++){
//      imageRegions.at<cv::Vec3b>(y,column) = red;
//    }
//  }
//  for(unsigned int i=0; i<fieldRegions.size(); i++){
//    int startY = fieldRegions[i].startPoint.y;
//    int endY = fieldRegions[i].endPoint.y;
//    int column = fieldRegions[i].startPoint.x;
//    for(int y=startY; y<=endY; y++){
//      imageRegions.at<cv::Vec3b>(y,column) = green;
//    }
//  }
  for(unsigned int i=0; i<lineRegions.size(); i++){
    int startY = lineRegions[i].middlePoint.y;
    int endY = lineRegions[i].middlePoint.y;
    int column = lineRegions[i].middlePoint.x;
    for(int y=startY; y<=endY; y++){
      imageRegions.at<cv::Vec3b>(y,column) = red;
    }
  }

//  for(unsigned int i=0; i<edges.size(); i++){
//    imageRegions.at<cv::Vec3b>(edges[i].y,edges[i].x) = black;
//  }

  for(unsigned int i = 0; i < lineVector.size(); i ++) {
    //for(unsigned int j = 0; j < lineVector[i].linePoints.size(); j+=2) {
      cv::line(imageRegions, cv::Point(lineVector[i].startPoint.x,lineVector[i].startPoint.y),cv::Point(lineVector[i].endPoint.x,lineVector[i].endPoint.y), cv::Scalar(blue) );
    //}
  }
  for (unsigned int i = 0; i < intersections.size(); i++) {
    if (intersections[i].x < imageRegions.cols && intersections[i].y < imageRegions.rows) {
      cv::line(imageRegions, cv::Point(intersections[i].x, intersections[i].y), cv::Point(0, 0), cv::Scalar(green));
    }
  }
}

/**
 * @brief subtractVector
 * subtracts point1 from point2
 * @param point1
 * @param point2
 * @return
 */
struct coordinate subtractVector(struct coordinate point1, struct coordinate point2) {
  struct coordinate result;
  result.x = point1.x - point2.x;
  result.y = point1.y - point2.y;
  return result;
}

/**
 * @brief addVector
 * Adds point1 to point2
 * @param point1
 * @param point2
 * @return
 */
struct coordinate addVector(struct coordinate point1, struct coordinate point2) {
  struct coordinate result;
  result.x = point1.x + point2.x;
  result.y = point1.y + point2.y;
  return result;
}


double absVector(struct coordinate vec) {
  return ( sqrt(pow(vec.x,2)+ pow(vec.y,2)));
}

double distanceToLine(struct coordinate point1, struct coordinate point2, struct coordinate poi) {
  return (double)abs(  (point2.y - point1.y) * poi.x - (point2.x - point1.x) * poi.y + point2.x * point1.y - point2.y * point1.x) / (absVector(subtractVector(point2,point1)));
}

void ransac(vector<struct region> &lineRegions, vector<lineData> &lineVector) {
  srand (time(NULL));
  struct lineData bestLine;
  vector<struct region> tempRemaining, bestRemaining;
  int score, maxScore = 0;

  int randomIndex2 = 0;
  int randomIndex1 = 0;

  for(int iterationen = 0; iterationen < 20; iterationen++) {

    randomIndex1 = rand() % lineRegions.size();
    randomIndex2 = rand() % lineRegions.size();

    struct coordinate point1 = lineRegions[randomIndex1].middlePoint;
    struct coordinate point2 = lineRegions[randomIndex2].middlePoint;
    tempRemaining.clear();
    score = 0;
    for(vector<struct region>::iterator it = lineRegions.begin(); it != lineRegions.end(); it++)  {
      double dist = distanceToLine(point1,point2,(*it).middlePoint);
      //cout << "distance " << i << "  :" << dist << endl;
      if(dist < 3) {
        score ++;
      } else {

        tempRemaining.push_back(*it);
        //score --;
      }

    }
    // cout << "score: " << score << " maxScore " << maxScore <<  endl;
    if(score > maxScore) {
      maxScore = score;
      bestLine.startPoint = point1;
      bestLine.endPoint = point2;
      bestRemaining = tempRemaining;
    }

  }
  if(bestRemaining.size() > 5) ransac(bestRemaining,lineVector);

  lineVector.push_back(bestLine);



}

struct coordinate getIntersection(const struct coordinate &p1, const struct coordinate &p2, const struct coordinate &p3, const struct coordinate &p4)
{
  struct coordinate result;
  int denominator = (p4.y - p3.y) * (p2.x - p1.x) - (p2.y - p1.y) * (p4.x - p3.x);
  if (!denominator) {
    result.x = -1;
    result.y = -1;
  } else {
    result.x = ((p4.x - p3.x) * (p2.x * p1.y - p1.x * p2.y) - (p2.x - p1.x) * (p4.x * p3.y - p3.x * p4.y)) / denominator;
    result.y = ((p1.y - p2.y) * (p4.x * p3.y - p3.x * p4.y) - (p3.y - p4.y) * (p2.x * p1.y - p1.x * p2.y)) / denominator;
  }
  return result;
}

void intersect(vector<lineData> &lineVector, vector<coordinate> &intersections)
{
  struct coordinate point;
  for (unsigned int i = 0; i < lineVector.size(); i++) {
    for (unsigned int j = 0; j < i; j++) {
      point = getIntersection(lineVector[i].startPoint, lineVector[i].endPoint, lineVector[j].startPoint, lineVector[j].endPoint);
      if (point.x < 0) {
        continue;
      }
      cout << "intersection: " << point.x << " " << point.y << endl;
      intersections.push_back(point);
    }
  }
}

int main() {
  cv::Mat image;
  image = cv::imread("bottom0007.png", CV_LOAD_IMAGE_COLOR);
  cv::Mat imageEdges(image.size().height, image.size().width, CV_8UC3);
  //cv::Mat imageRegions(image.size().height, image.size().width, CV_8UC3);
  cv::Mat imageRegions;
  imageRegions = cv::imread("bottom0007.png", CV_LOAD_IMAGE_COLOR);
  cv::Mat imageLines(image.size().height, image.size().width, CV_8UC3);
  cv::cvtColor(imageRegions,imageRegions,CV_YUV2RGB);
  vector<struct coordinate> edges;
  vector<struct region> fieldRegions; //startX startY endX endYr
  vector<struct region> lineRegions; //startX startY endX endY
  vector<struct region> unknownRegions; //startX startY endX endY
  vector<struct lineData> lineVector;
  vector<struct coordinate> intersections;

  std::chrono::duration<float, std::ratio<1, 1000>> delta1;
  std::chrono::duration<float, std::ratio<1, 1000>> delta2;
  std::chrono::duration<float, std::ratio<1, 1000>> delta3;
  std::chrono::duration<float, std::ratio<1, 1000>> delta4;

  chrono::time_point<std::chrono::system_clock> startTime = chrono::system_clock::now();
  edgeDetection(image, T_EDGE, edges);
  delta1 = chrono::system_clock::now() - startTime;
  classifyRegions(image, edges, fieldRegions, lineRegions, unknownRegions);
  delta2 = chrono::system_clock::now() - startTime - delta1;
  ransac(lineRegions,lineVector);
  delta3 = chrono::system_clock::now() - startTime - delta1 - delta2;
  intersect(lineVector, intersections);
  delta4 = chrono::system_clock::now() - startTime - delta1 - delta2 - delta3;
  drawResults(imageRegions, fieldRegions, lineRegions, unknownRegions, edges, lineVector, intersections);

  cout << "fieldRegions: " << fieldRegions.size() << endl;
  cout << "unknownRegions: " << unknownRegions.size() << endl;
  cout << "lineRegions: " << lineRegions.size() << endl;

  cout << "Number of Lines: " << lineVector.size() << endl;

  printf("edgeDetection: %fms\n",delta1.count());
  printf("classifyRegions: %fms\n",delta2.count());
  printf("ransac: %fms\n",delta3.count());
  printf("intersect: %fms\n",delta4.count());
  printf("sum: %fms\n",delta1.count()+delta2.count()+delta3.count()+delta4.count());
  cv::imwrite ("result_regions.png",imageRegions, vector<int>());
  cv::imwrite ("result_lines.png",imageLines, vector<int>());

  return 0;
}
