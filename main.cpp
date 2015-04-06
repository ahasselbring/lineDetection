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

void edgeDetectionOnScanline(int column, cv::Mat *image,cv::Mat *imageEdges, cv::Mat *imageLines, int t_edge, std::vector<cv::Vec2i> *edgePointer){

  cv::Vec3b color;
  color[0] = 255;
  color[1] = 255;
  color[2] = 255;

  std::vector<char> edge;

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
        //edges.push_back(imageEdges->at<cv::Vec3b>(x_peak,column));
        //edgePointer->push_back(&(imageEdges->at<cv::Vec3b>(x_peak,column)));
        //edgePointer->push_back(new cv::Vec2i(x_peak, column));
        edgePointer->push_back(cv::Vec2i(x_peak,column));
        imageEdges->at<cv::Vec3b>(x_peak,column) = color;
        //Methode von Erik und Pascal. Klappt aber nicht.
        //classifyEdges(image, imageLines, x_peak,column);
      }
      g_max = g;
      g_min = t_edge;
      x_peak = x - 1;
    }

    if(g < g_min) {
      if(g_max > t_edge) {
        edge.push_back(x_peak);
        imageEdges->at<cv::Vec3b>(x_peak,column) = color;
        edgePointer->push_back(cv::Vec2i(x_peak,column));
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

void edgeDetection(cv::Mat *image, cv::Mat *imageEdges, cv::Mat *imageLines, int t_edge, std::vector<cv::Vec2i> *edgePointer) {
  //+2 cause of downsampling
  for (int column=0;column<image->size().width; column+=2){
    edgeDetectionOnScanline(column, image, imageEdges, imageLines, t_edge, edgePointer);
  }
}

void setRegion(cv::Mat *imageRegions, const int *startX, const int *endX, const int *column, int regionType){
  //regionType
  //1-field
  //2-line
  //3-unknown

  cv::Vec3b green;
  green[0] = 0;
  green[1] = 255;
  green[2] = 0;
  cv::Vec3b white;
  white[0] = 255;
  white[1] = 255;
  white[2] = 255;
  cv::Vec3b red;
  red[0] = 255;
  red[1] = 0;
  red[2] = 0;

  //if(regionType==1){
  //cout << "startx: " << bla << "\t" << "endx: " << bla2 << endl;
  for(int i=*startX; i<*endX; i++){
  //out << *startX << endl;
  //     cout << i << "," << column << endl;
  //     cout << imageRegions->size().width << "," << imageRegions->size().height << endl;
    imageRegions->at<cv::Vec3b>(i,*column) = green;
  }
  //}
  //  else if (regionType==2){
  //    cout << "no sir" << endl;
  //    for(int i=startX; i<=endX; i++){
  //      //imageRegions->at<cv::Vec3b>(i,column) = white;
  //    }
  //  }
  //  else if (regionType==3){
  //    cout << "no sir 2" << endl;
  //    for(int i=startX; i<=endX; i++){
  //      //imageRegions->at<cv::Vec3b>(i,column) = red;
  //    }
  //  }
}

void classifyRegions(cv::Mat *image, cv::Mat *imageRegions, std::vector<cv::Vec2i> *edgePointer){
  int current_column = 0;
  int next_column = 0;
  int current_x = 0;
  int next_x = 0;
  int diff = 0;
  int gap = 0;
  int median_Y = 0;
  int median_Cb = 0;
  int median_Cr = 0;
  //  std::vector<cv::Vec2i> test = *edgePointer;
  //  cv::Vec2i test2 = (*edgePointer)[0];
  //  int test3 = (*edgePointer)[0][1];
  //  cout << test3 << endl;
  for (int i=0; i<edgePointer->size()-1; i++){
    //later on this is to optimize
    //do not save current and previous x temporally
    //current_column = edgePointer[i][1];
    current_x = (*edgePointer)[i][0];
    next_x = (*edgePointer)[i+1][0];
    current_column = (*edgePointer)[i][1];
    next_column = (*edgePointer)[i+1][1];
    if((next_column-current_column)==0){
      diff = next_x-current_x;
      gap = diff/6; //TODO: magic number
      median_Y = medianOfFive(image->at<cv::Vec3b>(i+1*gap,current_column)[0],image->at<cv::Vec3b>(i+2*gap,current_column)[0],image->at<cv::Vec3b>(i+3*gap,current_column)[0],image->at<cv::Vec3b>(i+4*gap,current_column)[0],image->at<cv::Vec3b>(i+5*gap,current_column)[0]);
      median_Cb = medianOfFive(image->at<cv::Vec3b>(i+1*gap,current_column)[1],image->at<cv::Vec3b>(i+2*gap,current_column)[1],image->at<cv::Vec3b>(i+3*gap,current_column)[1],image->at<cv::Vec3b>(i+4*gap,current_column)[1],image->at<cv::Vec3b>(i+5*gap,current_column)[1]);
      median_Cr = medianOfFive(image->at<cv::Vec3b>(i+1*gap,current_column)[2],image->at<cv::Vec3b>(i+2*gap,current_column)[2],image->at<cv::Vec3b>(i+3*gap,current_column)[2],image->at<cv::Vec3b>(i+4*gap,current_column)[2],image->at<cv::Vec3b>(i+5*gap,current_column)[2]);
      //cout << median_Y << "," << median_Cb << "," << median_Cr << endl;
      if(fieldCheck(median_Y,median_Cb,median_Cr)){
        //cout << current_x << "," << next_x << "," << current_column << "," << next_column << endl;
        setRegion(imageRegions, &current_x, &next_x, &current_column, 1);
      }
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
  //cv::Vec2i test(10,11);
  //edgePointer.push_back(test);
  //edgePointer.push_back(cv::Vec2i(8,9));

  //setRegion(&imageRegions, 13, 14, 5, 1);

  int t_edge;
  t_edge = 8;
  std::chrono::duration<float, std::ratio<1, 1000>> delta;

  chrono::time_point<std::chrono::system_clock> startTime = chrono::system_clock::now();
  edgeDetection(&image, &imageEdges, &imageLines, t_edge, &edgePointer);
  delta = chrono::system_clock::now() - startTime;

  classifyRegions(&image, &imageRegions, &edgePointer);

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

  printf("%f",delta.count());
  cv::imwrite ("result_edges.png",imageEdges, vector<int>());
  cv::imwrite ("result_regions.png",imageRegions, vector<int>());
  cv::imwrite ("result_lines.png",imageLines, vector<int>());

  return 0;
}
