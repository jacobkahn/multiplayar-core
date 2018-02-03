#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <thread>
#include "crow.h"

int main() {
  /********** Sample opencv to make sure it links and builds **********/
  const cv::Mat input =
      cv::imread("image.jpg", 0); // Load as grayscale because yeah

  // Detect
  cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SiftFeatureDetector::create();
  std::vector<cv::KeyPoint> keypoints;
  f2d->detect(input, keypoints);

  // Add results to image and save.
  cv::Mat output;
  cv::drawKeypoints(input, keypoints, output);
  cv::imwrite("image-out.jpg", output);

  /********** Simple server **********/
  // Start server
  crow::SimpleApp app;

  CROW_ROUTE(app, "/")([]() { return "Hello world"; });

  app.port(18080).multithreaded().run();
}