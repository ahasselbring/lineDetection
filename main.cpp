#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <chrono>
#include <ctime>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define DEBUG
#define FIELD_Y 126
#define FIELD_CB 114
#define FIELD_CR 74
#define T_Y 62
#define T_CB 19
#define T_CR 12

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

//int median(cv::Mat *image, int x, int column, int channel){
//  int medianArray[5];
//  for(int i=0; i<5; i++){
//    //x-2, x-1, x, x+1, x+2
//    medianArray[i]=image->at<cv::Vec3b>((x+i)-2,column)[channel];
//  }
//  return medianOfFive(medianArray[0],medianArray[1],medianArray[2],medianArray[3],medianArray[4]);
//}

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

bool lineCheck(){

}

//void classifyEdges(cv::Mat *image, cv::Mat *imageLines, int x, int column){
//  if(checkForLine(median(image, x, column, 0),median(image, x, column, 1),median(image, x, column, 2))) {
//    //x,column ist eine weiße linie auf dem spielfeld.
//    //dies nun in ein neues bild malen

//    cv::Vec3b color;
//    color[0] = 255;
//    color[1] = 255;
//    color[2] = 255;

//    imageLines->at<cv::Vec3b>(x,column) = color;
//  }
//}

void edgeDetectionOnScanline(int column, cv::Mat &image,cv::Mat &imageEdges, int t_edge, std::vector<cv::Vec2i> &edgePointer){

  cv::Vec3b color;
  color[0] = 255;
  color[1] = 255;
  color[2] = 255;

  //std::vector<char> edge;

  //  vector<cv::Vec3b*> edgePointer;

  //  cv::Vec3b *p = &(image->at<cv::Vec3b>(0,0));
  //  edgePointer.push_back(p);
  //  cout << "Größe: " << edgePointer.size() << " " << *edgePointer[0] << endl;

  //  std::vector<vector<int>> edges(4, vector<int>(4));
  //  edges[1][2] = 10;
  //  cout << edges [3][3] << endl;

  //  std::vector<*std::vector<int>> edgePointers;
  //  edgePointers.push_back();

  //  std::vector<int[]> edges;  std::vector<int[]> edges;
  //  int test[] = {1,2};
  //  edges.push_back(test);
  //  int test[] = {1,2};
  //  edges.push_back(test);

  //  uchar* p;
  //  p=image->ptr(0);

  //  std::vector<int[]> edges;
  //  int test[0];
  //  edges.push_back(test);

  int SCANLINE_SIZE = image.size().height;
  int scanline[SCANLINE_SIZE];

  int g_max = -t_edge;
  int g_min = t_edge;
  int x_peak = 0;
  int fx_last = scanline[0];
  int fx = 0;
  int g = 0;

  for (int x=0; x<SCANLINE_SIZE; x++) {
    scanline[x] = (image.at<cv::Vec3b>(x,column))[0];
  }

  for(int x = 2; x < SCANLINE_SIZE; x=x+2) {
    fx = scanline[x];
    g = fx - fx_last;
    if(g > g_max) {
      if(g_min < (-t_edge)) {
        //edge.push_back(x_peak);
        //edges.push_back(imageEdges->at<cv::Vec3b>(x_peak,column));
        //edgePointer->push_back(&(imageEdges->at<cv::Vec3b>(x_peak,column)));
        //edgePointer->push_back(new cv::Vec2i(x_peak, column));
        edgePointer.push_back(cv::Vec2i(x_peak,column));
        //draw edges
        //imageEdges.at<cv::Vec3b>(x_peak,column) = color;
        //Methode von Erik und Pascal. Klappt aber nicht.
        //classifyEdges(image, imageLines, x_peak,column);
      }
      g_max = g;
      g_min = t_edge;
      x_peak = x - 1;
    }

    if(g < g_min) {
      if(g_max > t_edge) {
        //edge.push_back(x_peak);
        edgePointer.push_back(cv::Vec2i(x_peak,column));
        //draw edges
        //imageEdges.at<cv::Vec3b>(x_peak,column) = color;
        //edgePointer->push_back(&(imageEdges->at<cv::Vec3b>(x_peak,column)));
        //Methode von Erik und Pascal. Klappt aber nicht.
        //classifyEdges(image, imageLines, x_peak,column);
      }
      g_min = g;
      g_max = (-t_edge);
      x_peak = x - 1;
    }
    fx_last = fx;
  }
}

void edgeDetection(cv::Mat &image, cv::Mat &imageEdges, int t_edge, std::vector<cv::Vec2i> &edgePointer) {
  //+2 cause of downsampling
  for (int column=0;column<image.size().width; column+=1){ //SCANLINE SUBSAMPLING +=2
    edgePointer.push_back(cv::Vec2i(0,column));
    edgeDetectionOnScanline(column, image, imageEdges, t_edge, edgePointer);
    edgePointer.push_back(cv::Vec2i(image.size().height,column));
  }
}

//void setRegion(cv::Mat &imageRegions, const int startX, const int endX, const int column, const int regionType){
//  //regionType
//  //1-field
//  //2-line
//  //3-unknown

//  cv::Vec3b green;
//  green[0] = 0; //B
//  green[1] = 255; //G
//  green[2] = 0; //R
//  cv::Vec3b white;
//  white[0] = 255;
//  white[1] = 255;
//  white[2] = 255;
//  cv::Vec3b red;
//  red[0] = 0;
//  red[1] = 0;
//  red[2] = 255;

//  if(regionType==1){

//    for(int i=startX; i<endX; i++){
//      //out << *startX << endl;
//      //     cout << i << "," << column << endl;
//      //     cout << imageRegions->size().width << "," << imageRegions->size().height << endl;
//      imageRegions.at<cv::Vec3b>(i, column) = green;
//    }
//  }

//  //cout << "startx: " << startX << "\t" << "endx: " << endX << endl;
//  else if (regionType==2){
//    //cout << "no sir" << endl;
//    for(int i=startX; i<=endX; i++){
//      imageRegions.at<cv::Vec3b>(i,column) = white;
//    }
//  }
//  else if(regionType==3){
//    //cout << "no sir 2" << endl;
//    for(int i=startX; i<=endX; i++){
//      imageRegions.at<cv::Vec3b>(i,column) = red;
//    }
//  }
//  else {
//    cout << "Das sollte niemals vorkommen" << endl;
//  }
//}

void classifyRegions(cv::Mat &image, cv::Mat &imageRegions, std::vector<cv::Vec2i> &edgePointer, std::vector<cv::Vec4i> &fieldRegions, std::vector<cv::Vec4i> &lineRegions, std::vector<cv::Vec4i> &unknownRegions){
  int currentColumn = 0;
  int nextColumn = 0;
  int currentX = 0;
  int nextX = 0;
  int diff = 0;
  int gap = 0;
  int median_Y = 0;
  int median_Cb = 0;
  int median_Cr = 0;
  int currentRegion = 0; //0 - default, 1 - field, 2 - line, 3 - unknown
  int startRegion = 0;
  int endRegion = 0;
  //  std::vector<cv::Vec2i> test = *edgePointer;
  //  cv::Vec2i test2 = (*edgePointer)[0];
  //  int test3 = (*edgePointer)[0][1];
  //  cout << test3 << endl;
  for (unsigned int i=0; i<edgePointer.size()-1; i++){
    //later on this is to optimize
    //do not save current and previous x temporally
    //current_column = edgePointer[i][1];
    currentX = edgePointer[i][0];
    nextX = edgePointer[i+1][0];
    currentColumn = edgePointer[i][1];
    nextColumn = edgePointer[i+1][1];
    if((nextColumn-currentColumn)==0){
      diff = nextX-currentX;
      gap = diff/6; //TODO: magic number
      median_Y = medianOfFive(image.at<cv::Vec3b>(currentX+1*gap,currentColumn)[0],image.at<cv::Vec3b>(currentX+2*gap,currentColumn)[0],image.at<cv::Vec3b>(currentX+3*gap,currentColumn)[0],image.at<cv::Vec3b>(currentX+4*gap,currentColumn)[0],image.at<cv::Vec3b>(currentX+5*gap,currentColumn)[0]);
      median_Cb = medianOfFive(image.at<cv::Vec3b>(currentX+1*gap,currentColumn)[1],image.at<cv::Vec3b>(currentX+2*gap,currentColumn)[1],image.at<cv::Vec3b>(currentX+3*gap,currentColumn)[1],image.at<cv::Vec3b>(currentX+4*gap,currentColumn)[1],image.at<cv::Vec3b>(currentX+5*gap,currentColumn)[1]);
      median_Cr = medianOfFive(image.at<cv::Vec3b>(currentX+1*gap,currentColumn)[2],image.at<cv::Vec3b>(currentX+2*gap,currentColumn)[2],image.at<cv::Vec3b>(currentX+3*gap,currentColumn)[2],image.at<cv::Vec3b>(currentX+4*gap,currentColumn)[2],image.at<cv::Vec3b>(currentX+5*gap,currentColumn)[2]);
      //cout << median_Y << "," << median_Cb << "," << median_Cr << endl;
      if(fieldCheck(median_Y,median_Cb,median_Cr)){
        //cout << current_x << "," << next_x << "," << current_column << "," << next_column << endl;
        //        if(currentRegion == 0){
        //          currentRegion = 1;
        //        }
        //        else if(currentRegion == )
        //        if (startRegion>currentX){
        //          startRegion = currentX;
        //        }
        //        endRegion = nextX;
        //update the last vector to connect contiguous regions to hold the vector small
        if(fieldRegions.size()>0){
          if(fieldRegions[fieldRegions.size()-1][3] == currentX && fieldRegions[fieldRegions.size()-1][3]==currentColumn) {
            fieldRegions[fieldRegions.size()-1][3] = nextX;
            //cout << "updated field" << endl;
          }
          else {
            fieldRegions.push_back(cv::Vec4i(currentX,currentColumn, nextX, currentColumn));
          }
        }
        else{
          //first entry of current column
          fieldRegions.push_back(cv::Vec4i(currentX,currentColumn, nextX, currentColumn));
        }
        //paint
        //setRegion(imageRegions, currentX, nextX, currentColumn, 1); //TODO magic snort snort
      }
      //      else if(lineCheck()){
      //        setRegion(imageRegions, currentX, nextX, currentColumn, 2);
      //      }
      else{
        if(unknownRegions.size()>0){
          if(unknownRegions[unknownRegions.size()-1][3] == currentX && unknownRegions[unknownRegions.size()-1][3]==currentColumn) {
            unknownRegions[unknownRegions.size()-1][3] = nextX;
            //cout << "updated unknown" << endl;
          }
          else {
            unknownRegions.push_back(cv::Vec4i(currentX,currentColumn, nextX, currentColumn));
          }
        }
        else{
          //first entry of current column
          unknownRegions.push_back(cv::Vec4i(currentX,currentColumn, nextX, currentColumn));
        }
        //unknownRegions.push_back(cv::Vec4i(currentX,currentColumn, nextX, currentColumn));
        //paint
        //setRegion(imageRegions, currentX, nextX, currentColumn, 3);
      }
    }
  }
}

void drawResults(cv::Mat &imageRegions, vector<cv::Vec4i> &fieldRegions, vector<cv::Vec4i> &lineRegions, vector<cv::Vec4i> &unknownRegions){
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

  for(int i=0; i<unknownRegions.size(); i++){
    int startX = unknownRegions[i][0];
    int endX = unknownRegions[i][2];
    int column = unknownRegions[i][1];
    for(int k=startX; k<=endX; k++){
      imageRegions.at<cv::Vec3b>(k,column) = red;
    }
  }
  for(int i=0; i<fieldRegions.size(); i++){
    int startX = fieldRegions[i][0];
    int endX = fieldRegions[i][2];
    int column = fieldRegions[i][1];
    for(int k=startX; k<=endX; k++){
      imageRegions.at<cv::Vec3b>(k,column) = green;
    }
  }
  for(int i=0; i<lineRegions.size(); i++){
    int startX = lineRegions[i][0];
    int endX = lineRegions[i][2];
    int column = lineRegions[i][1];
    for(int k=startX; k<=endX; k++){
      imageRegions.at<cv::Vec3b>(k,column) = white;
    }
  }
}

int main()
{
  cv::Mat image;
  image = cv::imread("bottom0007.png", CV_LOAD_IMAGE_COLOR);
  cv::Mat imageEdges(image.size().height, image.size().width, CV_8UC3);
  cv::Mat imageRegions(image.size().height, image.size().width, CV_8UC3);
  cv::Mat imageLines(image.size().height, image.size().width, CV_8UC3);

  //vector<cv::Vec3b*> edgePointer;
  vector<cv::Vec2i> edgePointer;
  vector<cv::Vec4i> fieldRegions; //startX startY endX endY
  vector<cv::Vec4i> lineRegions; //startX startY endX endY
  vector<cv::Vec4i> unknownRegions; //startX startY endX endY
  //cv::Vec2i test(10,11);
  //edgePointer.push_back(test);
  //edgePointer.push_back(cv::Vec2i(8,9));

  //setRegion(&imageRegions, 13, 14, 5, 1);

  int t_edge;
  t_edge = 8;
  std::chrono::duration<float, std::ratio<1, 1000>> delta1;
  std::chrono::duration<float, std::ratio<1, 1000>> delta2;

  chrono::time_point<std::chrono::system_clock> startTime = chrono::system_clock::now();
  edgeDetection(image, imageEdges, t_edge, edgePointer);
  delta1 = chrono::system_clock::now() - startTime;
  classifyRegions(image, imageRegions, edgePointer, fieldRegions, lineRegions, unknownRegions);
  delta2 = chrono::system_clock::now() - startTime;
  drawResults(imageRegions, fieldRegions, lineRegions, unknownRegions);

  //cout << edgePointer.size() << endl;
  //  cout << "Size: " << edgePointer.size() << endl;
  //  cout << edgePointer[0][1] << endl;
  //  int test = edgePointer[0][1];
  //  cout << test << endl;

  //  for(int i=0; i<18; i++){
  //    cout << edgePointer[i] << endl;
  //  }
  //  cout << edgePointer.size() << endl;
  //  for (int i=0; i<edgePointer.size(); i++){
  //    cout << *edgePointer[i] << endl;
  //  }

  cout << "fieldRegions: " << fieldRegions.size() << endl;
  cout << "unknownRegions: " << unknownRegions.size() << endl;

  printf("edgeDetection: %fms\n",delta1.count());
  printf("classifyRegions: %fms\n",delta2.count());
  printf("sum: %fms\n",delta1.count()+delta2.count());
  //cv::imwrite ("result_edges.png",imageEdges, vector<int>());
  cv::imwrite ("result_regions.png",imageRegions, vector<int>());
  cv::imwrite ("result_lines.png",imageLines, vector<int>());

  return 0;
}
