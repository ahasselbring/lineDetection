#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;


void writeHistogram(Mat& src_img, int * histArray, int sraster, int qbin) {

  int imgWidth = src_img.cols;
  int imgHeight = src_img.rows;

  for(int x = sraster/2; x < imgWidth; x+=sraster) {
    for(int y = sraster/2; y < imgHeight; y+=sraster) {

      Scalar intensity = src_img.at<uchar>(Point(x,y));
      int intensityVal = intensity.val[0];
      histArray[intensityVal / qbin]++;
    }
  }
}

/**
 * @brief getMax
 *
 *
 * @param data
 * @param qbin
 * @return
 */
int getMax(int * data, int qbin) {
  int result = 0;

  int max = 0;
  for(int i = 0; i < 256; i++) {
    if(data[i] > max) {
      max = data[i];
      result = (i * qbin) + (qbin / 2);
    }
  }

  return result;
}

bool isFieldColor(int cr, int cr_max, int threshold_cr) {
  if(abs(cr - cr_max) < threshold_cr) {
    return true;
  } else {
    return false;
  }
}

int weight(int i) {

  int weightNumber = 128 - i;

  if(weight <= 0) {
    return 0;
  } else {
    return weightNumber*weightNumber;
  }
}

void getWeightedHistogram(int * hist, int qbin) {
  for(int i = 0; i < 256; i++) {
    hist[i] = hist[i] * weight(i*qbin);
  }
}


int main( int argc, char** argv )
{

    int SRASTER = 16;
    int QBIN = 4;
    int THRESHOLD_CR = 19;
    int THRESHOLD_CB = 12;
    int THRESHOLD_Y = 65;

    if( argc != 2)
    {
     cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
     return -1;
    }

    Mat image;
    image = imread(argv[1]);   // Read the file

    if(! image.data )         // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    /**
    * Split source image into channels
    * Channel 0: Y
    * Channel 1: Cb
    * Channel 2: Cr
    */
    vector<Mat> channels;
    split(image, channels);

    int histRed[256] = {0};
    writeHistogram(channels[2], histRed, SRASTER, QBIN);
    getWeightedHistogram(histRed, QBIN);
    for(int i = 0; i < 256; ++i) {
      cout << histRed[i] << endl;
    }

    //Create a mat structure to contain the histogram plot
    //Paramters: height, width, colors
    Mat histPlot(755, 256/QBIN, CV_8UC3); //CV_8UC3: Three colors
    for(int i = 0; i < 256; i++)
    {
        int mag = histRed[i];
        line(histPlot,Point(i,histPlot.rows-1),Point(i,histPlot.rows-1-mag),Scalar(0,0,255));
    }

    // Display the histogram
    namedWindow("Histogram", CV_WINDOW_AUTOSIZE);
    imshow("Histogram", histPlot);

    Mat RGB_image;
    cvtColor(image, RGB_image, CV_YCrCb2RGB);
    namedWindow("Display", CV_WINDOW_AUTOSIZE);
    imshow("Display", RGB_image);

    int cr_max = getMax(histRed, QBIN);
    cout << "Cr max: " << cr_max << endl;

    int histBlue[256] = {0};
    int histY[256] = {0};

    for(int x = 0; x < image.cols; x++) {
      for(int y = 0; y < image.rows; y++) {
        Scalar intensity = channels[2].at<uchar>(Point(x,y));
        int intensityVal = intensity.val[0];

        if(isFieldColor(intensityVal, cr_max, THRESHOLD_CR)) {

          Scalar intensity_Cb = channels[1].at<uchar>(Point(x,y));
          int intensityVal_Cb = intensity_Cb.val[0];
          histBlue[intensityVal_Cb / QBIN]++;

          Scalar intensity_Y = channels[0].at<uchar>(Point(x,y));
          int intensityVal_Y = intensity_Y.val[0];
          histY[intensityVal_Y / QBIN]++;
        }
      }
    }

    int cb_max = getMax(histBlue, QBIN);
    int y_max = getMax(histY, QBIN);

    for(int x = 0; x < image.cols; x++) {
      for(int y = 0; y < image.rows; y++) {
        Scalar intensity_Cr = channels[2].at<uchar>(Point(x,y));
        int intensityVal_Cr = intensity_Cr.val[0];
        Scalar intensity_Cb = channels[1].at<uchar>(Point(x,y));
        int intensityVal_Cb = intensity_Cb.val[0];

        Scalar intensity_Y = channels[0].at<uchar>(Point(x,y));
        int intensityVal_Y = intensity_Y.val[0];

        if(isFieldColor(intensityVal_Cr, cr_max, THRESHOLD_CR) && isFieldColor(intensityVal_Cb, cb_max, THRESHOLD_CB) && isFieldColor(intensityVal_Y, y_max, THRESHOLD_Y)) {

          Vec3b color = RGB_image.at<Vec3b>(Point(x,y));
          color[0] = 0;
          color[1] = 0;
          color[2] = 255;
          RGB_image.at<Vec3b>(Point(x,y)) = color;

        }
      }
    }


    // Create a window for display.
    namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
    namedWindow( "Channel 0: Y", CV_WINDOW_AUTOSIZE );
    //namedWindow( "Channel 1: Cb", CV_WINDOW_AUTOSIZE );
    namedWindow( "Channel 2: Cr", CV_WINDOW_AUTOSIZE );

    // Show our image inside it.
    imshow( "Display window", RGB_image );
    imshow( "Channel 0: Y", channels[0]);
    //imshow( "Channel 1: Cb", channels[1]);
    imshow( "Channel 2: Cr", channels[2]);

    waitKey(0);              // Wait for a keystroke in the window
    return 0;
}
