#include <iostream>
#include <chrono>
#include <opencv2/highgui.hpp>
#include <opencv2/ximgproc.hpp>
#include <cv.hpp>
#include <pcl/io/pcd_io.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/media.h>

#include "StereoCamera.hpp"

void depthPrint(int event, int x, int y, int flags, void *userData);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr matToPoinXYZRGB(cv::Mat frameXYZ, cv::Mat frame);

int main() {
  const bool ALREADY_CAPTURED = true;
  const bool INDIVIDUAL_ALREADY_CAPTURED = true;
  const bool ALREADY_CALIBRATED = true;

  const int WIDTH = 640;
  const int HEIGHT = 360;
  const int FPS = 25;

  const float SQUARE_SIZE = 42.0F;
  const int BOARD_WIDTH = 5;
  const int BOARD_HEIGHT = 6;

  const int MAX_DISP = 160;
  const int W_SIZE = 3;

  const unsigned long LEFT = 0;
  const unsigned long RIGHT = 1;

  std::cout << "Init camera" << std::endl;
  std::pair<int, std::string> p1{1, "left"};
  std::pair<int, std::string> p2{0, "right"};
  StereoCamera camera{{p1, p2}, WIDTH, HEIGHT, FPS};
  camera.grabStart();

  if (ALREADY_CALIBRATED) {
    camera.loadCalibrationMaps("/tmp/3d_calibrations");
  } else {
    camera.initCalibrateBoard(BOARD_WIDTH, BOARD_HEIGHT, SQUARE_SIZE);
    camera.calibrateIndividualCameras(INDIVIDUAL_ALREADY_CAPTURED);
    camera.captureStereoCalibrationPoints(ALREADY_CAPTURED);
    camera.calibrateCamera();
    camera.saveCalibrationMaps("/tmp/3d_calibrations");
  }

  while (true) {
    cv::imshow("left_original", camera.getLRFrames().at(LEFT));
    cv::imshow("right_original", camera.getLRFrames().at(RIGHT));
    imshow("left_mapped", camera.getLRFramesRemapped().at(LEFT));
    imshow("right_mapped", camera.getLRFramesRemapped().at(RIGHT));

    int c = cv::waitKey(1000 / FPS);
    if (27 == char(c))
      break;
  }
  cv::destroyAllWindows();

//	cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(12, 192, 9, 205, 480, 1, 0, 10, 171, 1, true);
//	cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(0, 48, 5, 0, 0, 0, 0, 0, 0, 0, true);
//	cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(0, 32, 5);
//	cv::Ptr<cv::StereoBM> left_matcher = cv::StereoBM::create(160, 5);

  cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(0, MAX_DISP, W_SIZE);
  left_matcher->setP1(24 * W_SIZE * W_SIZE);
  left_matcher->setP2(96 * W_SIZE * W_SIZE);
  left_matcher->setPreFilterCap(63);
  left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);


  cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
  cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

  cv::Mat leftDisp(WIDTH, HEIGHT, CV_32F);
  cv::Mat rightDisp(WIDTH, HEIGHT, CV_32F);
  cv::Mat filteredDisp(WIDTH, HEIGHT, CV_32F);
  cv::Mat filteredDispVis;
  cv::Mat XYZ(WIDTH, HEIGHT, CV_32FC3);

  cv::namedWindow("filtered disparity");
  setMouseCallback("filtered disparity", depthPrint, &XYZ);

  std::cout << "Q " << camera.getDispToDepth() << std::endl;
  using Clock = std::chrono::high_resolution_clock;
  long t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;
  int total_ms = 0;
  std::vector<cv::Mat> frames;
  cv::Mat leftImg;
  cv::Mat rightImg;
  cv::Mat kernel = -1 * cv::Mat::ones(cv::Size{5, 5}, CV_32F);
  kernel.at<float>(1, 1) = 2;
  kernel.at<float>(1, 2) = 2;
  kernel.at<float>(1, 3) = 2;
  kernel.at<float>(2, 1) = 2;
  kernel.at<float>(2, 3) = 2;
  kernel.at<float>(3, 1) = 2;
  kernel.at<float>(3, 2) = 2;
  kernel.at<float>(3, 3) = 2;
  kernel.at<float>(2, 2) = 8;
  kernel /= 8;
  int i{0};
  while (true) {
    t1 = Clock::now().time_since_epoch().count();

    frames = camera.getLRFramesRemapped();
    t2 = Clock::now().time_since_epoch().count();
    std::cout << "Acquire: " << (t2 - t1) / (1000 * 1000);

		leftImg = cv::imread("/tmp/capture_left_calibration_image.jpg");
		rightImg = cv::imread("/tmp/capture_right_calibration_image.jpg");
//    leftImg = frames.at(LEFT);
//    rightImg = frames.at(RIGHT);
//    cv::filter2D(frames.at(LEFT), leftImg, -1, kernel);
//		cv::filter2D(frames.at(RIGHT), rightImg, -1, kernel);
//    cv::cvtColor(frames.at(LEFT), leftImg, CV_BGR2GRAY);
//    cv::cvtColor(frames.at(RIGHT), rightImg, CV_BGR2GRAY);


    t3 = Clock::now().time_since_epoch().count();
    std::cout << " Grayscale: " << (t3 - t2) / (1000 * 1000);

    left_matcher->compute(leftImg, rightImg, leftDisp);
    right_matcher->compute(rightImg, leftImg, rightDisp);

    t4 = Clock::now().time_since_epoch().count();
    std::cout << " Match: " << (t4 - t3) / (1000 * 1000);

    wls_filter->setLambda(8000);
    wls_filter->setSigmaColor(1.5);
    wls_filter->filter(leftDisp, leftImg, filteredDisp, rightDisp);

    t5 = Clock::now().time_since_epoch().count();
    std::cout << " WLS filter: " << (t5 - t4) / (1000 * 1000);

    cv::ximgproc::getDisparityVis(filteredDisp, filteredDispVis);
    imshow("filtered disparity", filteredDispVis);
    imshow("left image", frames.at(LEFT));
    imshow("right image", frames.at(RIGHT));

    reprojectImageTo3D(filteredDispVis, XYZ, camera.getDispToDepth(), true, CV_32F);

    total_ms = (int) std::round((Clock::now().time_since_epoch().count() - t1) / (1000 * 1000));

    std::cout << " Total: " << total_ms << std::endl;

/*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud = matToPoinXYZRGB(XYZ, frames.at(1));
    pcl::io::savePCDFileASCII<pcl::PointXYZRGB>("/tmp/test_pcd_" + std::to_string(i++) + ".pcd", *pointCloud.get());
*/

//    int c = cv::waitKey(std::max(1000 / FPS - total_ms, 1));
    int c = cv::waitKey(0);
    if ('\e' == char(c))
      break;
    else if ('s' == char(c)) {
      cv::imwrite("/tmp/capture_right_calibration_image.jpg", rightImg);
      cv::imwrite("/tmp/capture_left_calibration_image.jpg", leftImg);
    }
  }
  cv::destroyAllWindows();
  return 0;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr matToPoinXYZRGB(cv::Mat frameXYZ, cv::Mat frame) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  auto height = (uint32_t) frame.rows;
  auto width = (uint32_t) frame.cols;
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {

      pcl::PointXYZRGB point;
      cv::Point3f xyz = frameXYZ.at<cv::Point3f>(y, x);
      point.x = xyz.x;
      point.y = xyz.y;
      point.z = xyz.z;

      cv::Vec3b intensity = frame.at<cv::Vec3b>(y, x);
      uchar blue = intensity.val[0];
      uchar green = intensity.val[1];
      uchar red = intensity.val[2];
      uint32_t rgb = (
          static_cast<uint32_t>(red) << 16 |
              static_cast<uint32_t>(green) << 8 |
              static_cast<uint32_t>(blue));
      point.rgb = *reinterpret_cast<float *>(&rgb);

      point_cloud_ptr->points.push_back(point);

    }
  }
  point_cloud_ptr->width = width;
  point_cloud_ptr->height = height;

  return point_cloud_ptr;

}

void depthPrint(int event, int x, int y, int flags, void *userData) {
  auto *image3D = (cv::Mat *) userData;

  if (event == cv::EVENT_MOUSEMOVE) {
    cv::Point3f xyz = image3D->at<cv::Point3f>(y, x);
    std::cout << "Z: " << xyz.z << std::endl;
  }
}