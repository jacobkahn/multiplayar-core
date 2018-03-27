#pragma once

#include <opencv2/core/cvstd.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <memory>
#include <utility>
#include <vector>
#include "include/cv/SIFT.hpp"
#include "include/environment/Entity.hpp"

/**
 * A client interacting with the AR environment
 */
class Client : public Entity {
 public:
  Client(EntityID id);

  void processImage(std::string image);

  std::vector<cv::KeyPoint> getKeypoints() {
    return keypoints_;
  }

  cv::Mat getDescriptors() {
    return descriptors_;
  }

  int getRows() {
    return rows_;
  }

  int getCols() {
    return cols_;
  }

 private:
  // Keypoints associated with this client's last orientation
  std::vector<cv::KeyPoint> keypoints_;
  // Matrix of descriptors for this image
  cv::Mat descriptors_;
  // Row size of image
  int rows_;
  // Column size of image
  int cols_;
};