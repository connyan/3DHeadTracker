//
// Created by conny on 26/03/17.
//

#include "StereoCamera.hpp"

#include <iostream>
#include <future>
#include <opencv/cv.hpp>
#include <opencv2/core/mat.hpp>

StereoCamera::StereoCamera(std::vector<std::pair<int, std::string>> id, int width, int height, int fps) :
		leftCamera{id.at(0), width, height, fps},
		rightCamera{id.at(1), width, height, fps},
		captureSize{width, height},
		captureFPS{fps} {
}

void StereoCamera::grabStart() {
	leftCamera.grabStart();
	rightCamera.grabStart();
}

std::vector<cv::Mat> StereoCamera::getLRFrames() {
	cv::Mat leftFrame = leftCamera.getFrame();
	cv::Mat rightFrame = rightCamera.getFrame();
	return {leftFrame, rightFrame};
}

std::vector<cv::Mat> StereoCamera::getLRFramesRemapped() {
	auto leftFrameFuture = std::async(&Camera::getFrameRemapped, &leftCamera);
	auto rightFrameFuture = std::async(&Camera::getFrameRemapped, &rightCamera);
	cv::Mat leftFrame = leftFrameFuture.get();
	cv::Mat rightFrame = rightFrameFuture.get();
	return {leftFrame, rightFrame};
}

void StereoCamera::captureStereoCalibrationPoints(bool alreadyCaptured) {
	std::vector<cv::Point2f> leftImageCorners;
	std::vector<cv::Point2f> rightImageCorners;
	if (alreadyCaptured) {
		int i{0};
		while (true) {
			std::cout << "image number: " << i << std::endl;
			cv::Mat left, right;
			try {
				left = cv::imread("/tmp/stereo_" + leftCamera.getName() + "_calibration_image_" + std::to_string(i) + ".jpg");
				right = cv::imread("/tmp/stereo_" + rightCamera.getName() + "_calibration_image_" + std::to_string(i++) + ".jpg");
				if (left.data == nullptr || right.data == nullptr) {
					break;
				}
			} catch (std::exception &ex) {
				std::cout << ex.what() << std::endl;
				break;
			}
			findChessboard(leftImageCorners, left, leftCamera, false);
			findChessboard(rightImageCorners, right, rightCamera, false);

			if (leftImageCorners.size() == boardSize.area() &&
				rightImageCorners.size() == boardSize.area()) {
				realCornerVector.push_back(realCorners);
				leftImageCornerVector.push_back(leftImageCorners);
				rightImageCornerVector.push_back(rightImageCorners);
			}
		}
	} else {
		int i{0};
		while (true) {
			std::vector<cv::Mat> frames = getLRFrames();

			findChessboard(leftImageCorners, frames.at(0), leftCamera, true);
			findChessboard(rightImageCorners, frames.at(1), rightCamera, true);

			int c = cv::waitKey(1000 / captureFPS);
			if (27 == char(c)) {
				break;
			} else if (' ' == char(c)) {
				if (leftImageCorners.size() == boardSize.area() &&
					rightImageCorners.size() == boardSize.area()) {
					realCornerVector.push_back(realCorners);
					leftImageCornerVector.push_back(leftImageCorners);
					rightImageCornerVector.push_back(rightImageCorners);

					imwrite("/tmp/stereo_" + leftCamera.getName() + "_calibration_image_" + std::to_string(i) + ".jpg", frames.at(0));
					imwrite("/tmp/stereo_" + rightCamera.getName() + "_calibration_image_" + std::to_string(i++) + ".jpg", frames.at(1));

					std::cout << "Image mapping saved" << std::endl;
				}
			}
		}
	}
	if (!alreadyCaptured) {
		cv::destroyWindow("camera_" + leftCamera.getName());
	  cv::destroyWindow("camera_" + rightCamera.getName());
	}
}

void StereoCamera::findChessboard(std::vector<cv::Point2f> &imageCorners, cv::Mat &frame, Camera &camera, bool showImage) {
	cv::Mat gray;
	cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
	bool patternFound = findChessboardCorners(gray, boardSize, imageCorners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);
	std::cout << "Pattern frame found " << patternFound << std::endl;
	if(patternFound) {
			cornerSubPix(gray, imageCorners, cv::Size{11, 11}, cv::Size{-1, -1}, cv::TermCriteria{CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1});
		}
	if (showImage) {
		drawChessboardCorners(gray, boardSize, cv::Mat{imageCorners}, patternFound);
		imshow("camera_" + camera.getName(), gray);
	}
}

void StereoCamera::calibrateCamera() {
	std::cout << "left real corner std::vector size " << realCornerVector.size() << std::endl;
	std::cout << "left real corner std::vector front size " << realCornerVector.front().size() << std::endl;
	std::cout << "left image corner std::vector size " << leftImageCornerVector.size() << std::endl;
	std::cout << "left image corner std::vector front size " << leftImageCornerVector.front().size() << std::endl;

	double error;

	error = stereoCalibrate(
			realCornerVector,
			leftImageCornerVector, rightImageCornerVector,
			leftCamera.getCameraMatrix(), leftCamera.getDistCoeffs(),
			rightCamera.getCameraMatrix(), rightCamera.getDistCoeffs(),
			captureSize,
			rotationMat, translationMat, essentialMat, fundamentalMat,
			CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST,
			cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 100, 1e-5));

	std::cout << "Rotation and translation cv::Matrices calculated with error: " << error << std::endl;
	std::cout << "R " << rotationMat << std::endl;
	std::cout << "T " << translationMat << std::endl;

	stereoRectify(
			leftCamera.getCameraMatrix(), leftCamera.getDistCoeffs(),
			rightCamera.getCameraMatrix(), rightCamera.getDistCoeffs(),
			captureSize, rotationMat, translationMat,
			rotationMatLeft, rotationMatRight,
			projectionMatLeft, projectionMatRight,
			dispToDepth,
			cv::CALIB_ZERO_DISPARITY, 0, captureSize);

	std::cout << "Rotation and Projection cv::Matrices calculated" << std::endl;
	std::cout << "RLeft " << rotationMatLeft << std::endl;
	std::cout << "PLeft " << projectionMatLeft << std::endl;
	std::cout << "RRight " << rotationMatRight << std::endl;
	std::cout << "PRight " << projectionMatRight << std::endl;

	leftCamera.initDistortRectifyMap(rotationMatLeft, projectionMatLeft);
	rightCamera.initDistortRectifyMap(rotationMatRight, projectionMatRight);
}

void StereoCamera::initCalibrateBoard(int width, int height, float squareSize) {
	boardSize.width = width;
	boardSize.height = height;
	for (int i=0; i<height; i++) {
		for (int j=0; j<width; j++) {
			realCorners.push_back(cv::Point3f(i*squareSize, j*squareSize, 0.0F));
		}
	}
	leftCamera.initCalibrateBoard(width, height, squareSize);
	rightCamera.initCalibrateBoard(width, height, squareSize);
}

void StereoCamera::saveCalibrationMaps(const std::string filename) {
	leftCamera.saveCalibrationMaps(filename + "_left");
	rightCamera.saveCalibrationMaps(filename + "_right");
	cv::FileStorage file = cv::FileStorage{filename + "_disp", cv::FileStorage::WRITE};
	file << "dispToDepth" << dispToDepth;
	file.release();
}

void StereoCamera::loadCalibrationMaps(const std::string filename) {
	leftCamera.loadCalibrationMaps(filename + "_left");
	rightCamera.loadCalibrationMaps(filename + "_right");
	cv::FileStorage file = cv::FileStorage{filename + "_disp", cv::FileStorage::READ};
	file["dispToDepth"] >> dispToDepth;
	file.release();
}

const cv::Mat StereoCamera::getDispToDepth() {
	return dispToDepth;
}

void StereoCamera::calibrateIndividualCameras(bool alreadyCaptured) {
	leftCamera.captureCalibrationPoints(alreadyCaptured);
	leftCamera.calibrateCamera();
	rightCamera.captureCalibrationPoints(alreadyCaptured);
	rightCamera.calibrateCamera();
}
