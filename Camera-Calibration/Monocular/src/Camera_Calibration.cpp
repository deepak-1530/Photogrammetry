#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <sys/stat.h>

using namespace std;
using namespace cv;

vector< vector< Point3f > > object_points;
vector< vector< Point2f > > image_points;
vector< Point2f > corners;
vector< vector< Point2f > > left_img_points;

Mat img, gray;
Size im_size;

void setup_calibration(int board_width, int board_height, int num_imgs, 
                       float square_size,vector<Point2f> corners)
{
  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;

  for (int k = 1; k <= num_imgs; k = k+1) 
  {
    char img_file[100];

    sprintf(img_file, "/home/deepak/Photogrammetry/Camera Calibration/Data/%d.png", k);  // change path for chessboard images
  
    img = imread(img_file,0);

    cv::imshow("current image",img);

    bool found = false;
  
    found = cv::findChessboardCorners(img, board_size, corners,
                                      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    /******Determine sub-pixel accuracy*****/
    if (!found)
    { 
      cornerSubPix(img, corners, cv::Size(5, 5), cv::Size(-1, -1),
                   TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
    }
  
    drawChessboardCorners(img, board_size, corners, found);
  
    imshow("chess",img);

    waitKey(1);
    

     /***************************************** 
     * Collect Object Points.
     * Frame of reference is the top left corner 
     * of the chessboard
     * ****************************************/
    vector< Point3f > obj;

    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 9; j++)
        obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

    if (found) {
      cout << k << ". Found corners!" << endl;
      image_points.push_back(corners);
      object_points.push_back(obj);
    }
  }
}

/************************************************
 * Compute the reprojection error Optimum camera 
 * parameters are obtained by minimizing the 
 * reprojection error
************************************************/

double computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
                                 const vector< vector< Point2f > >& imagePoints,
                                 const vector< Mat >& rvecs, const vector< Mat >& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs) 
{
  vector< Point2f > imagePoints2;

  int i, totalPoints = 0;

  double totalErr = 0, err;

  vector< float > perViewErrors;

  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); ++i) 
  {
    projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints2);

    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

    int n = (int)objectPoints[i].size();

    perViewErrors[i] = (float) std::sqrt(err*err/n);

    totalErr += err*err;

    totalPoints += n;
  }
  return std::sqrt(totalErr/totalPoints);
}



int main()
{
  /***************************
   * Checker-board parameters
  ****************************/ 
  int board_width = 9, board_height = 6, num_imgs = 50;
  float square_size = 0.025;  
  
  
  char* out_file = "CamIntrinsic_params.yaml";
  
  setup_calibration(board_width, board_height, num_imgs, square_size,corners);
  
  printf("Starting Calibration\n");
  
  
  Mat K;
  Mat D;
  vector< Mat > rvecs, tvecs;

  int flag = 0;
  
  flag |=CV_CALIB_FIX_PRINCIPAL_POINT;  // Fix the prinicipal point location as the center point of the image.

  calibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs , flag);

  cout << "Calibration error: " << computeReprojectionErrors(object_points, image_points, rvecs, tvecs, K, D) << endl;

  FileStorage fs(out_file, FileStorage::WRITE);

  fs << "K" << K;
  fs << "D" << D;
  fs << "board_width" << board_width;
  fs << "board_height" << board_height;
  fs << "square_size" << square_size;
  printf("Done Calibration\n");
  return 0;
}
