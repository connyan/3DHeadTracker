//
// Created by conny on 26/03/17.
//

#ifndef _3DHEADTRACKER_STEREOCAMERA_HPP
#define _3DHEADTRACKER_STEREOCAMERA_HPP

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include "Camera.hpp"

class StereoCamera {
	Camera leftCamera;
	Camera rightCamera;

	cv::Size captureSize;
	int captureFPS;

	cv::Size boardSize;
	std::vector<cv::Point3f> realCorners;
	std::vector<std::vector<cv::Point2f>> leftImageCornerVector;
	std::vector<std::vector<cv::Point2f>> rightImageCornerVector;
	std::vector<std::vector<cv::Point3f>> realCornerVector;

	cv::Mat rotationMat, translationMat, essentialMat, fundamentalMat;
	cv::Mat rotationMatLeft, rotationMatRight, projectionMatLeft, projectionMatRight;
	cv::Mat dispToDepth;

	void findChessboard(std::vector<cv::Point2f> &imageCorners, cv::Mat &frame, Camera &camera, bool showImage);
public:
	StereoCamera(std::vector<std::pair<int, std::string>> id, int width, int height, int fps);

	std::vector<cv::Mat> getLRFrames();
	std::vector<cv::Mat> getLRFramesRemapped();

	void initCalibrateBoard(int width, int height, float squareSize);
	void calibrateIndividualCameras(bool alreadyCaptured);
	void captureStereoCalibrationPoints(bool takeNew);
	void calibrateCamera();

	void grabStart();

	void saveCalibrationMaps(const std::string filename);
	void loadCalibrationMaps(const std::string filename);

	const cv::Mat getDispToDepth();
};


#endif //_3DHEADTRACKER_STEREOCAMERA_HPP
