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

std::vector<cv::KeyPoint> Client::getKeypoints() {
  return keypoints_;
}

cv::Mat Client::getDescriptors() const {
  return descriptors_;
}

int Client::getRows() const {
  return rows_;
}

int Client::getCols() const {
  return cols_;
}

AnchorPoint2D Client::get2DAnchorPoint() const {
  return anchor2D_;
}

void Client::update2DAnchorPoints(AnchorPoint2D point) {
  if (!initializedAnchor_) {
    // TODO: log
    std::cout << "Updating anchor point for client with ID " << getID()
              << ". Anchor point is " << point << "\n";
    anchor2D_ = std::move(point);
    initializedAnchor_ = true;
  } else {
    // TODO: log
    std::cout << "Attempted to update anchor point for client which"
              << " already had anchor point\n";
  }
}

bool Client::hasInitializedAnchor() const {
  return initializedAnchor_;
}

void Client::addHomographyTransformResult(
    EntityID id,
    std::shared_ptr<HomographyTransformResult> homographyData) {
  otherClientHomographies_.emplace(std::make_pair(id, homographyData));
}

const HomographyMap& Client::getHomographyMap() const {
  return otherClientHomographies_;
}

std::vector<cv::Point2f> Client::getCandidatePoints() const {
  return candidatePoints_;
}

void Client::setCandidatePoints(std::vector<cv::Point2f> points) {
  candidatePoints_ = points;
}
