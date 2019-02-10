//
// Created by conny on 26/03/17.
//

#ifndef _3DHEADTRACKER_CAMERA_HPP
#define _3DHEADTRACKER_CAMERA_HPP

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <thread>
#include <future>

#include "LockQueue.hpp"

class Camera {
	std::string name;
	cv::VideoCapture camera;
	cv::Mat frameData;
	cv::Mat undistortedFrameData;

	cv::Size captureSize;
	int captureFPS;

  	cv::Size boardSize;
	std::vector<cv::Point3f> realCorners;

	bool getFrames = true;
	LockQueue frameLock;
	std::promise<bool> isRunning;
	std::future<bool> promiseFuture;
	std::thread frameGrabberThread;

	std::vector<std::vector<cv::Point2f>> imageCornerVector;
	std::vector<std::vector<cv::Point3f>> realCornerVector;
	std::vector<cv::Mat> rvecs, tvecs;
	cv::Mat cameraMatrix{3, 3, CV_64FC1};
	cv::Mat distCoeffs;
	cv::Mat map1, map2;

	void findChessboard(std::vector<cv::Point2f> &imageCorners, cv::Mat &frame, bool showImage);
public:
	Camera(std::pair<int, std::string> id, int width, int height, int fps);
	~Camera();

	void readFrame();
	cv::Mat getFrame();
	cv::Mat getFrameRemapped();

	void initCalibrateBoard(int width, int height, float squareSize);
	void captureCalibrationPoints(bool alreadyCaptured);
	double calibrateCamera();
	void initDistortRectifyMap();
	void initDistortRectifyMap(cv::Mat R, cv::Mat P);

	void saveCalibrationMaps(std::string filename);
	void loadCalibrationMaps(std::string filename);

	void grabStart();

	const std::string &getName() const;
	const cv::Mat &getCameraMatrix() const;
	const cv::Mat &getDistCoeffs() const;
};

#endif //_3DHEADTRACKER_CAMERA_HPP
