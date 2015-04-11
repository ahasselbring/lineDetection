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
#define PI 3.14159265
//TODO Determine these thresholds
#define D_MIN 0
#define D_MAX 21

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

//yep
void edgeDetectionOnScanline(int column, cv::Mat &image, int t_edge, std::vector<coordinate> &edges){

  int SCANLINE_SIZE = image.size().height;
  int scanline[SCANLINE_SIZE];

  int g_max = -t_edge;
  int g_min = t_edge;
  int yPeak = 0;
  int fYLast = scanline[0];
  int fY = 0;
  int g = 0;

  for (int y=0; y<SCANLINE_SIZE; y++) {
    scanline[y] = (image.at<cv::Vec3b>(y,column))[0];
  }

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
          lineRegions.push_back(unknownRegions[i]);
        }
      }
    }
  }
}

double calculateGradientAngle(double x, double y) {
  double angle = 0;
  if(y != 0) {
    angle = atan(x/y)*180/PI;
  }
  //cout << "Gradient Angle: " << angle << endl;
  return angle;
}


void drawResults(cv::Mat &imageRegions, vector<struct region> &fieldRegions, vector<struct region> &lineRegions, vector<struct region> &unknownRegions, vector<struct lineRegionData> &gradientVector, vector<coordinate> &edges, vector<struct edgePointPair> &edgePointPairs, vector<struct lineData> &lineVector){
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

  for(unsigned int i=0; i<unknownRegions.size(); i++){
    int startY = unknownRegions[i].startPoint.y;
    int endY = unknownRegions[i].endPoint.y;
    int column = unknownRegions[i].startPoint.x;
    for(int y=startY; y<=endY; y++){
      imageRegions.at<cv::Vec3b>(y,column) = red;
    }
  }
  for(unsigned int i=0; i<fieldRegions.size(); i++){
    int startY = fieldRegions[i].startPoint.y;
    int endY = fieldRegions[i].endPoint.y;
    int column = fieldRegions[i].startPoint.x;
    for(int y=startY; y<=endY; y++){
      imageRegions.at<cv::Vec3b>(y,column) = green;
    }
  }
  for(unsigned int i=0; i<lineRegions.size(); i++){
    int startY = lineRegions[i].startPoint.y;
    int endY = lineRegions[i].endPoint.y;
    int column = lineRegions[i].startPoint.x;
    for(int y=startY; y<=endY; y++){
      imageRegions.at<cv::Vec3b>(y,column) = cyan;
    }
  }

  /*for(unsigned int i=0; i<gradientVector.size(); i++){
    cv::line(imageRegions, cv::Point(gradientVector[i].upperPoint.x,gradientVector[i].upperPoint.y), cv::Point(gradientVector[i].upperGradient.horizontal / 10+ gradientVector[i].upperPoint.x,gradientVector[i].upperGradient.vertical / 10 +gradientVector[i].upperPoint.y),cv::Scalar(blue),1,8);
    cv::line(imageRegions, cv::Point(gradientVector[i].lowerPoint.x,gradientVector[i].lowerPoint.y), cv::Point(gradientVector[i].lowerGradient.horizontal / 10+ gradientVector[i].lowerPoint.x,gradientVector[i].lowerGradient.vertical / 10 +gradientVector[i].lowerPoint.y),cv::Scalar(yellow),1,8);
  }*/

  for(unsigned int i=0; i<edges.size(); i++){
    imageRegions.at<cv::Vec3b>(edges[i].y,edges[i].x) = black;
  }


  for(unsigned int i = 0; i < edgePointPairs.size(); i++) {
    cv::line(imageRegions, cv::Point(edgePointPairs[i].point1.x,edgePointPairs[i].point1.y), cv::Point(edgePointPairs[i].point2.x,edgePointPairs[i].point2.y), cv::Scalar(red));

  }

  for(unsigned int i = 0; i < lineVector.size(); i ++) {
    for(unsigned int j = 0; j < lineVector[i].linePoints.size(); j+=2) {
      cv::line(imageRegions, cv::Point(lineVector[i].linePoints[j].x,lineVector[i].linePoints[j].y),cv::Point(lineVector[i].linePoints[j+1].x,lineVector[i].linePoints[j+1].y), cv::Scalar(blue) );
    }
  }
}

//yep
bool boundaryCheck(const cv::Mat &image, const int &x, const int &y){
  if(x>0&&y>0&&y<image.size().height&&x<image.size().width){
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

//yep
void calculateLineGradients(const cv::Mat &image, const vector<struct region> &lineRegions, vector<struct lineRegionData> &gradientVector) {

  /*Sobel operators:
   *
   * -1 0 1
   * -2 0 2   Horizontal
   * -1 0 1
   *
   *  1  2  1
   *  0  0  0 Vertical
   * -1 -2 -1
   */


  struct lineRegionData lineData;

  for(unsigned int i=0;i< lineRegions.size();i++) {


    lineData.upperPoint.y = lineRegions[i].startPoint.y;
    lineData.upperPoint.x = lineRegions[i].startPoint.x;
    lineData.lowerPoint.y = lineRegions[i].endPoint.y;
    lineData.lowerPoint.x = lineRegions[i].endPoint.x;
    lineData.upperGradient.vertical = 0;
    lineData.upperGradient.horizontal = 0;
    lineData.lowerGradient.vertical = 0;
    lineData.lowerGradient.horizontal = 0;
    if(boundaryCheck(image,lineData.upperPoint.y,lineData.upperPoint.x)) {
      lineData.upperGradient.vertical = 1 * image.at<cv::Vec3b>(lineData.upperPoint.y-1,lineData.upperPoint.x-1)[0] + 2 * image.at<cv::Vec3b>(lineData.upperPoint.y-1,lineData.upperPoint.x)[0] + 1 * image.at<cv::Vec3b>(lineData.upperPoint.y-1,lineData.upperPoint.x+1)[0] - 1 * image.at<cv::Vec3b>(lineData.upperPoint.y+1,lineData.upperPoint.x-1)[0] - 2 * image.at<cv::Vec3b>(lineData.upperPoint.y+1,lineData.upperPoint.x)[0] + 1 * image.at<cv::Vec3b>(lineData.upperPoint.y+1,lineData.upperPoint.x+1)[0];
      lineData.upperGradient.horizontal = -1 * image.at<cv::Vec3b>(lineData.upperPoint.y-1,lineData.upperPoint.x-1)[0] - 2 * image.at<cv::Vec3b>(lineData.upperPoint.y,lineData.upperPoint.x-1)[0] - 1 * image.at<cv::Vec3b>(lineData.upperPoint.y+1,lineData.upperPoint.x-1)[0] + 1 * image.at<cv::Vec3b>(lineData.upperPoint.y-1,lineData.upperPoint.x+1)[0] + 2 * image.at<cv::Vec3b>(lineData.upperPoint.y,lineData.upperPoint.x+1)[0] + 1 * image.at<cv::Vec3b>(lineData.upperPoint.y+1,lineData.upperPoint.x+1)[0];
    }
    if(boundaryCheck(image,lineData.lowerPoint.y,lineData.lowerPoint.x)) {
      lineData.lowerGradient.vertical = -1 * (1 * image.at<cv::Vec3b>(lineData.lowerPoint.y-1,lineData.lowerPoint.x-1)[0] + 2 * image.at<cv::Vec3b>(lineData.lowerPoint.y-1,lineData.lowerPoint.x)[0] + 1 * image.at<cv::Vec3b>(lineData.lowerPoint.y-1,lineData.lowerPoint.x+1)[0] - 1 * image.at<cv::Vec3b>(lineData.lowerPoint.y+1,lineData.lowerPoint.x+1)[0] - 2 * image.at<cv::Vec3b>(lineData.lowerPoint.y+1,lineData.lowerPoint.x)[0] - 1 * image.at<cv::Vec3b>(lineData.lowerPoint.y+1,lineData.lowerPoint.x+1)[0]);
      lineData.lowerGradient.horizontal = -1 * (-1 * image.at<cv::Vec3b>(lineData.lowerPoint.y-1,lineData.lowerPoint.x-1)[0] - 2 * image.at<cv::Vec3b>(lineData.lowerPoint.y,lineData.lowerPoint.x-1)[0] - 1 * image.at<cv::Vec3b>(lineData.lowerPoint.y+1,lineData.lowerPoint.x-1)[0] + 1 * image.at<cv::Vec3b>(lineData.lowerPoint.y-1,lineData.lowerPoint.x+1)[0] + 2 * image.at<cv::Vec3b>(lineData.lowerPoint.y,lineData.lowerPoint.x+1)[0] + 1 * image.at<cv::Vec3b>(lineData.lowerPoint.y+1,lineData.lowerPoint.x+1)[0]);
    }
    gradientVector.push_back(lineData);
  }
}

double scalarProduct(struct coordinate point1, struct coordinate point2) {
  return point1.x*point2.x+point1.y*point2.y;
}

double scalarProduct(struct coordinate point1, struct gradient point2) {
  return point1.x*point2.vertical+point1.y*point2.horizontal;
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

double absVector(struct gradient vec) {
  return ( sqrt(pow(vec.vertical,2)+ pow(vec.horizontal,2)));
}

void createEdgePointPairs( vector<struct lineRegionData> &gradientVector, vector<struct edgePointPair> &edgePointPairs) {

  for(unsigned int i = 0; i < gradientVector.size(); i++) {
    double upperTempLineliness = 0;
    double lowerTempLineliness = 0;
    gradientVector[i].upperLineliness = 0;
    for(unsigned int j = 0; j < gradientVector.size(); j++) {


      struct coordinate upper_v_ij = subtractVector(gradientVector[i].upperPoint,gradientVector[j].upperPoint);
      struct coordinate lower_v_ij = subtractVector(gradientVector[i].lowerPoint,gradientVector[j].lowerPoint);

       if(absVector(upper_v_ij) >= D_MIN && absVector(upper_v_ij) <= D_MAX) {
         if(scalarProduct(gradientVector[i].upperPoint, gradientVector[j].upperPoint) > 0) {
           upperTempLineliness = 1 - (( (abs(scalarProduct(upper_v_ij,gradientVector[i].upperGradient)) / (absVector(upper_v_ij) * absVector(gradientVector[i].upperGradient))) + (abs(scalarProduct(upper_v_ij,gradientVector[j].upperGradient)) / (absVector(upper_v_ij) * absVector(gradientVector[j].upperGradient)))  ) / 2);
           if(upperTempLineliness >= gradientVector[i].upperLineliness) {
             struct edgePointPair upper_epp;
             gradientVector[i].upperLineliness = upperTempLineliness;
             upper_epp.point1 = gradientVector[i].upperPoint;
             upper_epp.point2 = gradientVector[j].upperPoint;
             edgePointPairs.push_back(upper_epp);

           }
         }
       }

       if(absVector(lower_v_ij) >= D_MIN && absVector(lower_v_ij) <= D_MAX) {
         if(scalarProduct(gradientVector[i].lowerPoint, gradientVector[j].lowerPoint) > 0) {
           lowerTempLineliness = 1 - (( (abs(scalarProduct(lower_v_ij,gradientVector[i].lowerGradient)) / (absVector(lower_v_ij) * absVector(gradientVector[i].lowerGradient))) + (abs(scalarProduct(lower_v_ij,gradientVector[j].lowerGradient)) / (absVector(lower_v_ij) * absVector(gradientVector[j].lowerGradient)))  ) / 2);
           if(lowerTempLineliness >= gradientVector[i].lowerLineliness) {
             struct edgePointPair lower_epp;
             gradientVector[i].lowerLineliness = lowerTempLineliness;
             lower_epp.point1 = gradientVector[i].lowerPoint;
             lower_epp.point2 = gradientVector[j].lowerPoint;
             edgePointPairs.push_back(lower_epp);

           }
         }
       }
    }
  }
}


double distanceFunction(struct coordinate A, struct coordinate B, struct coordinate C, struct coordinate D, double dist_AB) {
   double epsilon1,epsilon2 = 0.0;

   epsilon1 = ((C.x-A.x) * (A.y - B.y) + (C.y - A.y) * (B.x - A.x)) / ( dist_AB);
   epsilon2 = ((D.x-A.x) * (A.y - B.y) + (D.y - A.y) * (B.x - A.x)) / ( dist_AB);

   return (epsilon1 + epsilon2) / 2;



}

void groupLinePairs(vector<struct edgePointPair> &edgePointPairs, vector<struct lineData> &lineVector)  {

  double dist1_1,dist1_2,dist2_1,dist2_2 = 0;
  double distance = 0;

  int end_i = edgePointPairs.size();
  int end_j = edgePointPairs.size();
  int i = 0;
  int j = 0;
  //for(unsigned int i = 0; i < edgePointPairs.size(); i++) {
  while(i < end_i) {
    //for(unsigned int j = 0; j < edgePointPairs.size(); j++) {
    while(j < end_j) {
      dist1_1 = absVector(subtractVector(edgePointPairs[i].point1,edgePointPairs[j].point1));
      dist1_2 = absVector(subtractVector(edgePointPairs[i].point1,edgePointPairs[j].point2));
      dist2_1 = absVector(subtractVector(edgePointPairs[i].point2,edgePointPairs[j].point1));
      dist2_2 = absVector(subtractVector(edgePointPairs[i].point2,edgePointPairs[j].point2));
      //cout << "dist1_1: " << dist1_1 << " dist1_2: " << dist1_2 << " dist2_1: " << dist2_1 << " dist2_2: " << dist2_2 << endl;
      double temp = 0;
      int state = 0;

      if(dist1_1 > temp) {
        temp = dist1_1;
        state = 0;
      }
      if(dist1_2 > temp) {
        temp = dist1_2;
        state = 1;
      }
      if(dist2_1 > temp) {
        temp = dist2_1;
        state = 2;
      }
      if(dist2_2 > temp) {
        temp = dist2_2;
        state = 3;
      }

      //cout << "dist1_1: " << dist1_1 << " dist1_2: " << dist1_2 << " dist2_1: " << dist2_1 << " dist2_2: " << dist2_2 << " Max dist: " << temp << endl;

      switch(state) {
      case 0:
        distance = distanceFunction(edgePointPairs[i].point1,edgePointPairs[j].point1,edgePointPairs[i].point2,edgePointPairs[j].point2,dist1_1);
        break;
      case 1:
        distance = distanceFunction(edgePointPairs[i].point1,edgePointPairs[j].point2,edgePointPairs[i].point2,edgePointPairs[j].point1, dist1_2);
        break;
      case 2:
        distance = distanceFunction(edgePointPairs[i].point2,edgePointPairs[j].point1,edgePointPairs[i].point1,edgePointPairs[j].point2, dist2_1);
        break;
      case 3:
        distance = distanceFunction(edgePointPairs[i].point2,edgePointPairs[j].point2,edgePointPairs[i].point1,edgePointPairs[j].point1, dist2_2);
        break;
      }

      //cout << "Distance Measure: " << distance << endl;
      if( distance < 4) {
        struct lineData lineSegment;
        lineSegment.linePoints.push_back(edgePointPairs[i].point1);
        lineSegment.linePoints.push_back(edgePointPairs[i].point2);
        lineSegment.linePoints.push_back(edgePointPairs[j].point1);
        lineSegment.linePoints.push_back(edgePointPairs[j].point2);
        lineVector.push_back(lineSegment);
        edgePointPairs.erase(edgePointPairs.begin()+i);
        edgePointPairs.erase(edgePointPairs.begin()+j);
        end_i = edgePointPairs.size();
        end_j = edgePointPairs.size();
      }
      j++;
     }
    i++;
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
  vector<struct lineRegionData> gradientVector;
  vector<struct edgePointPair> edgePointPairs;
  vector<struct lineData> lineVector;

  std::chrono::duration<float, std::ratio<1, 1000>> delta1;
  std::chrono::duration<float, std::ratio<1, 1000>> delta2;
  std::chrono::duration<float, std::ratio<1, 1000>> delta3;
  std::chrono::duration<float, std::ratio<1, 1000>> delta4;
  std::chrono::duration<float, std::ratio<1, 1000>> delta5;

  chrono::time_point<std::chrono::system_clock> startTime = chrono::system_clock::now();
  edgeDetection(image, T_EDGE, edges);
  delta1 = chrono::system_clock::now() - startTime;
  classifyRegions(image, edges, fieldRegions, lineRegions, unknownRegions);
  delta2 = chrono::system_clock::now() - startTime - delta1;
  calculateLineGradients(image, lineRegions, gradientVector);
  delta3 = chrono::system_clock::now() - startTime - delta1 - delta2;
  createEdgePointPairs(gradientVector, edgePointPairs );
  delta4 = chrono::system_clock::now() - startTime - delta1 - delta2 - delta3;
  groupLinePairs(edgePointPairs,lineVector);
  delta5 = chrono::system_clock::now() - startTime - delta1 - delta2 - delta3 - delta4;
  drawResults(imageRegions, fieldRegions, lineRegions, unknownRegions, gradientVector, edges, edgePointPairs, lineVector);

  cout << "fieldRegions: " << fieldRegions.size() << endl;
  cout << "unknownRegions: " << unknownRegions.size() << endl;

  cout << "edgePointPairs: " << edgePointPairs.size() << endl;
  cout << "Number of Lines: " << lineVector.size() << endl;

  printf("edgeDetection: %fms\n",delta1.count());
  printf("classifyRegions: %fms\n",delta2.count());
  printf("calculateLineGradients: %fms\n",delta3.count());
  printf("createEdgePointPairs: %fms\n",delta4.count());
  printf("groupLinePairs: %fms\n",delta5.count());
  printf("sum: %fms\n",delta1.count()+delta2.count()+delta3.count()+delta4.count()+delta5.count());
  cv::imwrite ("result_regions.png",imageRegions, vector<int>());
  cv::imwrite ("result_lines.png",imageLines, vector<int>());

  return 0;
}
