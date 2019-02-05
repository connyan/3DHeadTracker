//
// Created by conny on 26/03/17.
//

#include <iostream>
#include <opencv/cv.hpp>

#include "Camera.hpp"

Camera::Camera(std::pair<int, std::string> id, int width, int height, int fps) : frameLock{2}, camera{cv::VideoCapture{id.first}}, name{id.second}, captureSize{width, height}, captureFPS{fps} {
	if (!camera.isOpened()) {
	  std::cout << "Camera problems" << std::endl;
		throw std::exception();
	}

	camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
	camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
	camera.set(cv::CAP_PROP_FPS, fps);
	std::cout << "Frame width " << camera.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
	std::cout << "Frame height " << camera.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
	std::cout << "Frame fps " << camera.get(cv::CAP_PROP_FPS) << std::endl;

	promiseFuture = isRunning.get_future();
}

void Camera::grabStart() {
	frameGrabberThread = std::thread(&Camera::readFrame, this);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

Camera::~Camera() {
	auto status = promiseFuture.wait_for(std::chrono::milliseconds(0));
	if (status == std::future_status::ready) {
		getFrames = false;
		frameGrabberThread.join();
	}
}

bool Camera::readFrame() {
	using Clock = std::chrono::steady_clock;
	isRunning.set_value(true);

	cv::Mat tmpFrameData;
	while(getFrames) {
		camera.grab();

		if (camera.retrieve(tmpFrameData)) {
			frameLock.lock(0);
			tmpFrameData.copyTo(frameData);
			flip(frameData, frameData, 1);
			frameLock.unlock();
		}
	}
}

cv::Mat Camera::getFrame() {
	frameLock.lock(1);
	cv::Mat tmp = frameData.clone();
	frameLock.unlock();
	return tmp;
}

void Camera::initCalibrateBoard(int width, int height, float squareSize) {
	boardSize.width = width;
	boardSize.height = height;
	for (int i=0; i<height; i++) {
		for (int j=0; j<width; j++) {
			realCorners.push_back(cv::Point3f(i*squareSize, j*squareSize, 0.0F));
		}
	}
}

void Camera::captureCalibrationPoints(bool alreadyCaptured) {
	int i=0;
	std::vector<cv::Point2f> imageCorners;

	if (alreadyCaptured) {
		int i{0};
		while (true) {
			std::cout << "camera: " << name << " image number: " << i << std::endl;
			cv::Mat frame;
			try {
				frame = cv::imread("/tmp/" + name + "_calibration_image_" + std::to_string(i++) + ".jpg");
				if (frame.data == NULL) {
					break;
				}
			} catch (std::exception &ex) {
				std::cout << ex.what() << std::endl;
				break;
			}
			findChessboard(imageCorners, frame, false);

			if (imageCorners.size() == boardSize.area()) {
				realCornerVector.push_back(realCorners);
				imageCornerVector.push_back(imageCorners);
			}
		}
	} else {
		while (true) {
			cv::Mat frame = getFrame();
			findChessboard(imageCorners, frame, true);

			int c = cv::waitKey(1000 / captureFPS);
			if (27 == char(c)) {
				break;
			} else if (' ' == char(c)) {
				if (imageCorners.size() == boardSize.area()) {
					realCornerVector.push_back(realCorners);
					imageCornerVector.push_back(imageCorners);

					imwrite("/tmp/" + name + "_calibration_image_" + std::to_string(i++) + ".jpg", frame);

					std::cout << "Image mapping saved" << std::endl;
				}
			}
		}
	}
	cv::destroyAllWindows();
}

void Camera::findChessboard(std::vector<cv::Point2f> &imageCorners, cv::Mat &frame, bool showImage) {
	cv::Mat gray;
	cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
	bool patternFound = findChessboardCorners(gray, boardSize, imageCorners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);
	std::cout << "Pattern frame found " << patternFound << std::endl;
	if(patternFound) {
		cornerSubPix(gray, imageCorners, cv::Size{11, 11}, cv::Size{-1, -1}, cv::TermCriteria{CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1});
	}
	if (showImage) {
		drawChessboardCorners(gray, boardSize, cv::Mat{imageCorners}, patternFound);
		imshow("camera_" + name, gray);
	}
}

double Camera::calibrateCamera() {
	double error = cv::calibrateCamera(realCornerVector, imageCornerVector, captureSize, cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_ZERO_TANGENT_DIST);
	 std::cout << "Calibration matrices calculated for " + name + " camera with error: " << error << std::endl;
	 std::cout << "CM " << cameraMatrix << std::endl;
	 std::cout << "D " << distCoeffs << std::endl;
	return error;
}

void Camera::initDistortRectifyMap() {
	initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cv::Mat(), captureSize, CV_32FC1, map1, map2);
}

void Camera::initDistortRectifyMap(cv::Mat R, cv::Mat P) {
	initUndistortRectifyMap(cameraMatrix, distCoeffs, R, P, captureSize, CV_32FC1, map1, map2);
}

cv::Mat Camera::getFrameRemapped() {
	remap(getFrame(), undistortedFrameData, map1, map2, CV_INTER_CUBIC, cv::BORDER_CONSTANT, 0);
	return undistortedFrameData;
}

void Camera::saveCalibrationMaps(std::string filename) {
	cv::FileStorage file = cv::FileStorage{filename, cv::FileStorage::WRITE};
	file << "rectify_map1" << map1;
	file << "rectify_map2" << map2;
	file.release();
}

void Camera::loadCalibrationMaps(std::string filename) {
	cv::FileStorage file = cv::FileStorage{filename, cv::FileStorage::READ};
	file["rectify_map1"] >> map1;
	file["rectify_map2"] >> map2;
	file.release();
}

const std::string &Camera::getName() const {
	return name;
}

const cv::Mat &Camera::getCameraMatrix() const {
	return cameraMatrix;
}

const cv::Mat &Camera::getDistCoeffs() const {
	return distCoeffs;
}