#pragma once

#include <boost/functional/hash.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "include/environment/Entity.hpp"

// Forward delcaration
class Client;

// Point data for a given perspective transform derived from a homography
using StringyPoint = std::unordered_map<std::string, std::string>;
using PointList = std::vector<StringyPoint>;

// Data returned from a raw transform. A pair of 2D points for each client, and
// a pair of homographies originating from the first order
using RawHomographyData =
    std::pair<std::pair<PointList, PointList>, std::pair<cv::Mat, cv::Mat>>;

/**
 * Utility to hash OpenCV Point2fs in such a way that they can be used in an
 * unordered_map. Uses boost::hash_value and boost::hash_combine to generate a
 * good hash.
 */
struct PointHasher {
  size_t operator()(const cv::Point2f& point) const {
    size_t seed{0};
    boost::hash_combine(seed, boost::hash_value(point.x));
    boost::hash_combine(seed, boost::hash_value(point.y));
    return seed;
  }
};

// A piece of SIFT result data stored with a client
struct HomographyTransformResult {
  // Mapping of an entity ID to the points for this homography
  std::unordered_map<EntityID, PointList> pointMap;
  // A mapping of entity id to homographies. For some given ID, the homography
  // at that location is the homography from the other user to the user with
  // that ID, such that it can be used to convert an arbitrary point from the
  // other user in 2D space to a point in this user's space
  std::unordered_map<EntityID, cv::Mat> homographyMap;
  // For the client that sent AR point candidates and received AR points back in
  // its initial calibration, we retain the mapping from the SIFT-generated
  // points to the AR points. This is used to compute the transformation of the
  // chosen anchor point
  std::unordered_map<cv::Point2f, cv::Point2f, PointHasher>
      siftToARPointMapping;
};

/**
 * Utilities for manipulating points representations and formats.
 *
 * Transforms a point back and forth between its serialized network
 * representation (which is similar to JSON, but represented internally as an
 * std::unordered_map) and its internal OpenCV Point2f representation, which is
 * used in calculations, SIFT, etc.
 */
namespace PointRepresentationUtils {
const std::string kStringyPointXFieldName = "x";
const std::string kStringyPointYFieldName = "y";

StringyPoint cvPoint2fToStringyPoint(cv::Point2f point);

cv::Point2f stringyPointToPoint2f(StringyPoint stringyPointMap);

} // namespace PointRepresentationUtils

class SIFTClient {
 public:
  /**
   * Runs a toy sift example with "queryImage.jpg" and "trainImage.jpg" located
   * in the executable directory, and writes "out.jpg" as the output image file
   */
  void runToySIFT();

  std::pair<std::vector<cv::KeyPoint>, cv::Mat>
  detectAndComputeKeypointsAndDescriptors(const cv::Mat& image);

  std::shared_ptr<HomographyTransformResult>
  computeHomographyTransformationFromClients(
      std::shared_ptr<Client> client1,
      std::shared_ptr<Client> client2);

 private:
  /**
   * Given two pieces of raw image data, run SIFT, find a homography, and
   * perform a perspective transform. Return the ordered point data in the order
   * of the iamge data (the first transform on the first image, and the second
   * transform on the second image)
   */
  RawHomographyData computeHomographyTransformation(
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