#pragma once

#include <opencv2/core/cvstd.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Forward delcaration
class Client;

// Point data for a given perspective transform derived from a homography
using PointList = std::vector<std::unordered_map<std::string, double>>;

class SIFTClient {
 public:
  /**
   * Runs a toy sift example with "queryImage.jpg" and "trainImage.jpg" located
   * in the executable directory, and writes "out.jpg" as the output image file
   */
  void runToySIFT();

  std::pair<std::vector<cv::KeyPoint>, cv::Mat>
  detectAndComputeKeypointsAndDescriptors(const cv::Mat& image);

  std::pair<PointList, PointList> computeHomographyTransformationFromClients(
      std::shared_ptr<Client> client1,
      std::shared_ptr<Client> client2);

 private:
  /**
   * Given two pieces of raw image data, run SIFT, find a homography, and
   * perform a perspective transform. Return the ordered point data in the order
   * of the iamge data (the first transform on the first image, and the second
   * transform on the second image)
   */
  std::pair<PointList, PointList> computeHomographyTransformation(
      // Keypoints to be extracted from image
      std::vector<cv::KeyPoint> queryKeypoints,
      std::vector<cv::KeyPoint> trainKeypoints,
      // Descriptors to be extracted from image
      cv::Mat queryDescriptors,
      cv::Mat trainDescriptors,
      // The dimensions of the query image
      int queryRows,
      int queryCols,
      // The dimensions of the train image
      int trainRows,
      int trainCols);
};