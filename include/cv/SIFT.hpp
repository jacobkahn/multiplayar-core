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

/**
 * Utility to hash OpenCV DMatches in such a way that they can be used in an
 * unordered_map. Uses boost::hash_value and boost::hash_combine to generate a
 * good hash from the points
 */
struct MatchHasher {
  size_t operator()(const cv::DMatch& match) const {
    size_t seed{0};
    boost::hash_combine(seed, boost::hash_value(match.queryIdx));
    boost::hash_combine(seed, boost::hash_value(match.trainIdx));
    return seed;
  }
};

/**
 * Utility to compare OpenCV DMatches in such a way that they can be used in an
 * unordered_map. Simply compares train and query indices of the related
 * keypoints.
 */
struct MatchEquals {
  bool operator()(const cv::DMatch& match1, const cv::DMatch& match2) const {
    return (match1.queryIdx == match2.queryIdx) &&
        (match1.trainIdx == match2.trainIdx);
  }
};

// A piece of SIFT result data stored with a client
struct HomographyTransformResult {
  // Entities to which this result pertains
  std::pair<EntityID, EntityID> entities;
  // Mapping of an entity ID to the points for this homography
  std::unordered_map<EntityID, PointList> pointMap;
  // A mapping of entity id to homographies. For some given ID, the homography
  // at that location is the homography from the other user to the user with
  // that ID, such that it can be used to convert an arbitrary point from the
  // other user in 2D space to a point in this user's space
  std::unordered_map<EntityID, cv::Mat> homographyMap;
  // For the client that sent AR point candidates and received AR points back
  // in its initial calibration, we retain the mapping from the SIFT-generated
  // points to the AR points. This is used to compute the transformation of
  // the chosen anchor point
  std::unordered_map<cv::Point2f, cv::Point2f, PointHasher>
      siftToARPointMapping;
  // All matchings as generated by SIFT with a simple KD ration test
  std::vector<cv::DMatch> allMatches;
  // Matchings as filtered out by AR centrality score
  std::vector<cv::DMatch> bestMatches;
  // Returns the other entity
  EntityID getOtherEntity(const EntityID& id) {
    return entities.first == id ? entities.second : entities.first;
  }
};

/**
 * Utilities for manipulating points representations and formats.
 *
 * Transforms a point back and forth between its serialized network
 * representation (which is similar to JSON, but represented internally as an
 * std::unordered_map) and its internal OpenCV Point2f representation, which
 * is used in calculations, SIFT, etc.
 */
namespace PointRepresentationUtils {
const std::string kStringyPointXFieldName = "x";
const std::string kStringyPointYFieldName = "y";

/**
 * A stringy point is a representation of a point as a std::unordered_map with
 * standardized keys
 */
StringyPoint cvPoint2fToStringyPoint(cv::Point2f point);

/**
 * Converts a mapped point to a cv::Point2f
 */
cv::Point2f stringyPointToPoint2f(StringyPoint stringyPointMap);

/**
 * Stringifies a cv::Point2f in (x, y) format
 */
std::string cvPoint2fToString(cv::Point2f point);

} // namespace PointRepresentationUtils

class SIFTClient {
 public:
  /**
   * Run the preliminary image analysis for SIFT.
   *
   * Given an image, compute the keypoints and descriptors for the image. Return
   * a pair of entities, the first of which is a vector of the keypoints
   * (represented as cv::KeyPoint), along with a matrix containing all image
   * descriptors for the image.
   */
  std::pair<std::vector<cv::KeyPoint>, cv::Mat>
  detectAndComputeKeypointsAndDescriptors(const cv::Mat& image);

  /**
   * Copute a full homography given client data.
   *
   * Using the keypoints and desriptors, for an image, compute matchings across
   * that images, then use such matchings in conjunction with keypoints to
   * produce homographies. Also store matchings and mappings from SIFT-returned
   * points to candidate points in the resulting data structure.
   *
   * Wraps raw computation functions with client inputs.
   */
  std::shared_ptr<HomographyTransformResult>
  computeHomographyTransformationFromClients(
      std::shared_ptr<Client> client1,
      std::shared_ptr<Client> client2);

  /**
   * Runs a toy sift example with "queryImage.jpg" and "trainImage.jpg"
   * located in the executable directory, and writes "out.jpg" as the output
   * image file
   */
  void runToySIFT();

 private:
  // The good matchings as computed by this client
  std::vector<cv::DMatch> goodMatches_;
  /**
   * Compute point ranking given a point and some candidate points.
   *
   * The algorithm simply computes the sum of the distances from the query
   * point to each of the candidate points for the 5 closest points: that is,
   * determine the 5 closest points to the query point, and the sum over those
   * 5 closest distances.
   */
  double computeCentralityScoreForPoint(
      cv::Point2f queryPoint,
      const std::vector<cv::Point2f>& candidatePoints);

  /**
   * Given two points in a match, compute the centrality score for the match
   * (which is the pair of points).
   *
   * This is equal to the sum of query point and query candidate points'
   * centrality score, and the train point and the train candidate points'
   * centrality score.
   */
  double computeCentralityScore(
      cv::Point2f queryPoint,
      cv::Point2f trainPoint,
      const std::vector<cv::Point2f>& queryCandidatePoints,
      const std::vector<cv::Point2f>& trainCandidatePoints);

  /**
   * Given keypoints, descriptors, and dimensions, compute matchings
   * with multiple randomized KD trees and a norm-based test to assess accuracy.
   */
  std::vector<cv::DMatch> computeMatchings(
      // Descriptors to be extracted from image
      cv::Mat queryDescriptors,
      cv::Mat trainDescriptors);

  /**
   * Given two pieces of raw image data, run SIFT, find a homography, and
   * perform a perspective transform. Return the ordered point data in the
   * order of the iamge data (the first transform on the first image, and the
   * second transform on the second image)
   */
  RawHomographyData computeHomographyTransformation(
      std::vector<cv::KeyPoint> queryKeypoints,
      std::vector<cv::KeyPoint> trainKeypoints,
      const std::vector<cv::Point2f>& queryCandidatePoints,
      const std::vector<cv::Point2f>& trainCandidatePoints,
      std::vector<cv::DMatch> filteredMatches);
};