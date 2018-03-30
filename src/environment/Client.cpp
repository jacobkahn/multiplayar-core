#include "include/environment/Client.hpp"
#include <opencv2/core/cvstd.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <vector>
#include "include/cv/SIFT.hpp"

Client::Client(EntityID id) : Entity(std::move(id)) {}

void Client::processImage(std::string rawImage) {
  // Create a matrix out of
  std::vector<char> rawArr(rawImage.begin(), rawImage.end());
  cv::Mat image = cv::imdecode(cv::Mat(rawArr), cv::IMREAD_UNCHANGED);
  // Create a client and get results
  auto siftClient = std::make_unique<SIFTClient>();
  auto result = siftClient->detectAndComputeKeypointsAndDescriptors(image);

  keypoints_ = std::move(result.first);
  descriptors_ = std::move(result.second);
  rows_ = image.rows;
  cols_ = image.cols;
}