#pragma once

#include <opencv2/core/cvstd.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <memory>
#include <utility>
#include <vector>
#include "include/cv/SIFT.hpp"
#include "include/environment/Entity.hpp"

// The 2D representation of the anchor point of the user
using AnchorPoint2D = cv::Point2f;
// The 3D anchor point for a user
using AnchorPoint3D = cv::Point3f;
// A map of user ids to homography data
using HomographyMap =
    std::unordered_map<EntityID, std::shared_ptr<HomographyTransformResult>>;

/**
 * A client interacting with the AR environment
 */
class Client : public Entity {
 public:
  Client(EntityID id);

  void processImage(std::string image);

  std::vector<cv::KeyPoint> getKeypoints();

  cv::Mat getDescriptors() const;

  int getRows() const;

  int getCols() const;

  AnchorPoint2D get2DAnchorPoint() const;

  void update2DAnchorPoints(AnchorPoint2D point);

  bool hasInitializedAnchor() const;

  void addHomographyTransformResult(
      EntityID id,
      std::shared_ptr<HomographyTransformResult> homographyData);

  const HomographyMap& getHomographyMap() const;

 private:
  // Keypoints associated with this client's last orientation
  std::vector<cv::KeyPoint> keypoints_;
  // Matrix of descriptors for this image
  cv::Mat descriptors_;
  // Row size of image
  int rows_;
  // Column size of image
  int cols_;
  // Homography results mapped by client
  HomographyMap otherClientHomographies_;
  // Determines whether or not this client has a valid, initialized anchor point
  bool initializedAnchor_{false};
  // 2D anchor points for this client
  AnchorPoint2D anchor2D_;
};