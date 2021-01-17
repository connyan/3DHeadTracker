//
// Created by conny on 1/22/19.
//

#include <iostream>
#include <opencv2/opencv.hpp>

#include "Camera.hpp"

int main() {
	Camera c1{std::pair<int, std::string>{1, "single"}, 640, 360, 5};
	c1.grabStart();
	cv::Mat img = c1.getFrame();
	cv::imshow("single", img);
	cv::waitKey();
}
