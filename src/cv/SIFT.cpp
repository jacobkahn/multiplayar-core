#include "include/cv/SIFT.hpp"
#include <opencv2/core/cvstd.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "include/environment/Client.hpp"

// Distance for KD matching
const double kMinDistanceKDMatching = 20;
// Percentile of total matches that we are taking as "good" matches
const double kDistancePercentKDMatch = 0.005;
// Minimum number of points we need for a homography threshold
const size_t kMinPointsHomographyThreshold = 6;
// Minimum number of points we need for a homography threshold
const size_t kMaxPointsHomographyHeap = 50;
// Constant for Lowe's ratio test across the best matches
const double kRatioTestConstant = 0.6;

std::pair<std::vector<cv::KeyPoint>, cv::Mat>
SIFTClient::detectAndComputeKeypointsAndDescriptors(const cv::Mat& image) {
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

  cv::Ptr<cv::xfeatures2d::SIFT> siftClient = cv::xfeatures2d::SIFT::create(
      //   0, // nFeatures
      //   3, // nOctaveLayers
      //   0.04, // contrastThreshold
      //   10, // edgeThreshold
      //   1.6 // sigma
  );
  siftClient->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
  return std::make_pair(keypoints, descriptors);
}

std::pair<PointList, PointList>
SIFTClient::computeHomographyTransformationFromClients(
    std::shared_ptr<Client> client1,
    std::shared_ptr<Client> client2) {
  return computeHomographyTransformation(
      client1->getKeypoints(),
      client2->getKeypoints(),
      client1->getDescriptors(),
      client2->getDescriptors(),
      client1->getRows(),
      client1->getCols(),
      client2->getRows(),
      client2->getCols());
}

/**
 * Given an image
 * Returns a tuple of images after a perspectice transform
 */
std::pair<PointList, PointList> SIFTClient::computeHomographyTransformation(
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
    int trainCols) {
  // Make a new FLANN KD dtree
  const cv::Ptr<cv::flann::IndexParams>& indexParams =
      new cv::flann::KDTreeIndexParams(5);
  const cv::Ptr<cv::flann::SearchParams>& searchParams =
      new cv::flann::SearchParams(64);

  /***** Match the Descriptors with FLANN KD Tree *****/
  cv::Mat indexMat;
  // FLANN KD tree matcher
  cv::FlannBasedMatcher matcher(indexParams, searchParams);
  // Matches we find from the compare
  std::vector<std::vector<cv::DMatch>> matches;
  matcher.knnMatch(queryDescriptors, trainDescriptors, matches, 2);
  // We use Lowe's Euclidean ratio test to measure the match distance to assess
  // how good them matches are y'all
  std::vector<cv::DMatch> filteredMatches;

  for (int k = 0; k < std::min(queryDescriptors.rows - 1, (int)matches.size());
       k++) {
    // Ratio test from the Lowe paper. Take the first result if distance is
    // smaller than a threshold (ignore the other descriptor) if the second
    // distance is very different
    if ((matches[k][0].distance <
         kRatioTestConstant * (matches[k][1].distance)) &&
        ((int)matches[k].size() <= 2 && (int)matches[k].size() > 0)) {
      filteredMatches.push_back(matches[k][0]);
    }
  }

  // Count the number of mathes where the distance is less than 2 * min_dist
  // Lambda comparator between distance fields of DMatches
  auto cmp = [](cv::DMatch left, cv::DMatch right) {
    return left.distance > right.distance;
  };
  std::priority_queue<cv::DMatch, std::vector<cv::DMatch>, decltype(cmp)>
      matchMaxHeap(cmp);
  for (size_t i = 0; i < filteredMatches.size(); i++) {
    // std::cout << "Match distances " << filteredMatches[i].distance << "\n";
    // Add to heap
    matchMaxHeap.push(filteredMatches[i]);
    // If we're at capacity, pop the top element
    if (matchMaxHeap.size() > kMaxPointsHomographyHeap) {
      matchMaxHeap.pop();
    }
  }
  // Remove everything, save to vector
  std::vector<cv::DMatch> goodMatches = std::vector<cv::DMatch>();
  while (matchMaxHeap.size() != 0) {
    goodMatches.push_back(matchMaxHeap.top());
    matchMaxHeap.pop();
  }

  // Make sure we have enough points?
  // TODO: stuff
  if (goodMatches.size() == 0 ||
      goodMatches.size() < kMinPointsHomographyThreshold) {
    std::cout << "Insufficient matches found for homography. Exiting.\n";
    return std::make_pair(PointList(), PointList());
  }

  // Get keypoints that we care about
  std::vector<cv::Point2f> finalQueryKeypoints;
  std::vector<cv::Point2f> finalTrainKeypoints;
  for (auto& match : goodMatches) {
    finalQueryKeypoints.push_back(queryKeypoints[match.queryIdx].pt);
    finalTrainKeypoints.push_back(trainKeypoints[match.trainIdx].pt);
  }
  /***** Find Homography *****/
  // TODO: is this the right way to use ransac?
  cv::Mat QThomography =
      cv::findHomography(finalQueryKeypoints, finalTrainKeypoints);
  cv::Mat TQhomography =
      cv::findHomography(finalTrainKeypoints, finalQueryKeypoints);

  /***** Perspective transform for query image *****/
  std::vector<cv::Point2f> queryImageCorners(4);
  // Compute dimensions of images
  queryImageCorners[0] = cv::Point2f(0, 0);
  queryImageCorners[1] = cv::Point2f(queryCols, 0);
  queryImageCorners[2] = cv::Point2f(queryCols, queryRows);
  queryImageCorners[3] = cv::Point2f(0, queryRows);

  /***** Perspective transform for train image *****/
  std::vector<cv::Point2f> trainImageCorners(4);
  // Compute dimensions of images
  trainImageCorners[0] = cv::Point2f(0, 0);
  trainImageCorners[1] = cv::Point2f(trainCols, 0);
  trainImageCorners[2] = cv::Point2f(trainCols, trainRows);
  trainImageCorners[3] = cv::Point2f(0, trainRows);

  std::vector<cv::Point2f> queryWorldCorners = std::vector<cv::Point2f>(4);
  std::vector<cv::Point2f> trainWorldCorners = std::vector<cv::Point2f>(4);

  /***** Perform perspective transform *****/
  // Find dimensions of both images so we can apply the homography to the
  // source images to compute 2D markers for anchor offsets Compute
  // transform. Save world corners for both in the client
  cv::perspectiveTransform(trainImageCorners, trainWorldCorners, TQhomography);
  cv::perspectiveTransform(queryImageCorners, queryWorldCorners, QThomography);

  std::cout << "Query world corners " << queryWorldCorners << "\n";
  std::cout << "Train world corners " << trainWorldCorners << "\n";

  // Format for res
  PointList queryPoints, trainPoints;
  for (size_t i = 0; i < queryWorldCorners.size(); i++) {
    queryPoints.push_back(
        {{"x", queryWorldCorners[i].x}, {"y", queryWorldCorners[i].y}});
  }
  for (size_t i = 0; i < trainWorldCorners.size(); i++) {
    trainPoints.push_back(
        {{"x", trainWorldCorners[i].x}, {"y", trainWorldCorners[i].y}});
  }

  auto result = std::make_pair(queryPoints, trainPoints);

  // Return a tuple of points
  return result;
}

void SIFTClient::runToySIFT() {
  // Read in an image from an interesting
  const cv::Mat queryImage =
      cv::imread("queryImage.jpg");
  // Read in
  const cv::Mat trainImage = cv::imread("trainImage.jpg");

  /***** Extract Keypoints and Descriptors from both images *****/
  // Keypoints to be extracted from image
  std::vector<cv::KeyPoint> queryKeypoints;
  std::vector<cv::KeyPoint> trainKeypoints;
  // Descriptors to be extracted from image
  cv::Mat queryDescriptors;
  cv::Mat trainDescriptors;

  // New SIFT
  cv::Ptr<cv::xfeatures2d::SIFT> siftClientQuery = cv::xfeatures2d::SIFT::create(
      //   0, // nFeatures
      //   3, // nOctaveLayers
      //   0.04, // contrastThreshold
      //   10, // edgeThreshold
      //   1.6 // sigma
  );
  siftClientQuery->detectAndCompute(queryImage, cv::noArray(), queryKeypoints, queryDescriptors);

  // New SIFT
  cv::Ptr<cv::xfeatures2d::SIFT> siftClientTrain = cv::xfeatures2d::SIFT::create(
      //   0, // nFeatures
      //   3, // nOctaveLayers
      //   0.04, // contrastThreshold
      //   10, // edgeThreshold
      //   1.6 // sigma
  );
  siftClientTrain->detectAndCompute(trainImage, cv::noArray(), trainKeypoints, trainDescriptors);

  // KD Tree parameters
  const cv::Ptr<cv::flann::IndexParams>& indexParams =
      new cv::flann::KDTreeIndexParams(5);
  const cv::Ptr<cv::flann::SearchParams>& searchParams =
      new cv::flann::SearchParams(64);

  /***** Match the Descriptors with FLANN KD Tree *****/
  cv::Mat indexMat;
  // FLANN KD tree matcher
  cv::FlannBasedMatcher matcher(indexParams, searchParams);
  // Matches we find from the compare
  std::vector<std::vector<cv::DMatch>> matches;
  matcher.knnMatch(queryDescriptors, trainDescriptors, matches, 2);
  // We use Lowe's Euclidean ratio test to measure the match distance to assess
  // how good them matches are y'all
  std::vector<cv::DMatch> filteredMatches;

  for (int k = 0; k < std::min(queryDescriptors.rows - 1, (int)matches.size());
       k++) {
    // Ratio test from the Lowe paper. Take the first result if distance is
    // smaller than a threshold (ignore the other descriptor) if the second
    // distance is very different
    if ((matches[k][0].distance <
         kRatioTestConstant * (matches[k][1].distance)) &&
        ((int)matches[k].size() <= 2 && (int)matches[k].size() > 0)) {
      filteredMatches.push_back(matches[k][0]);
    }
  }

  // Count the number of mathes where the distance is less than 2 * min_dist
  // Lambda comparator between distance fields of DMatches
  auto cmp = [](cv::DMatch left, cv::DMatch right) {
    return left.distance > right.distance;
  };
  std::priority_queue<cv::DMatch, std::vector<cv::DMatch>, decltype(cmp)>
      matchMaxHeap(cmp);
  for (size_t i = 0; i < filteredMatches.size(); i++) {
    // std::cout << "Match distances " << filteredMatches[i].distance << "\n";
    // Add to heap
    matchMaxHeap.push(filteredMatches[i]);
    // If we're at capacity, pop the top element
    if (matchMaxHeap.size() > kMaxPointsHomographyHeap) {
      matchMaxHeap.pop();
    }
  }
  // Remove everything, save to vector
  std::vector<cv::DMatch> goodMatches = std::vector<cv::DMatch>();
  while (matchMaxHeap.size() != 0) {
    goodMatches.push_back(matchMaxHeap.top());
    matchMaxHeap.pop();
  }

  // Make sure we have enough points, toy simulation only exits
  if (goodMatches.size() < kMinPointsHomographyThreshold) {
    std::cout << "Not enough points to compute homography. Exiting.\n";
    return;
  }

  // Get keypoints that we care about
  std::vector<cv::Point2f> finalQueryKeypoints;
  std::vector<cv::Point2f> finalTrainKeypoints;
  for (auto& match : goodMatches) {
    finalQueryKeypoints.push_back(queryKeypoints[match.queryIdx].pt);
    finalTrainKeypoints.push_back(trainKeypoints[match.trainIdx].pt);
  }
  /***** Find Homography *****/
  // TODO: is this the right way to use ransac?
  cv::Mat homography =
      cv::findHomography(finalQueryKeypoints, finalTrainKeypoints);
  std::cout << "Homography is " << homography << "\n";

  std::vector<cv::Point2f> queryWorldCorners(4);
  std::vector<cv::Point2f> trainWorldCorners;
  // Compute dimensions of images
  queryWorldCorners[0] = cv::Point2f(0, 0);
  queryWorldCorners[1] = cv::Point2f(queryImage.cols, 0);
  queryWorldCorners[2] = cv::Point2f(queryImage.cols, queryImage.rows);
  queryWorldCorners[3] = cv::Point2f(0, queryImage.rows);
  // Find dimensions of both images so we can apply the homography to the source
  // images to compute 2D markers for anchor offsets
  // Compute transform
  cv::perspectiveTransform(queryWorldCorners, trainWorldCorners, homography);

  cv::Mat imageDrawMatches;
  drawMatches(
      queryImage,
      queryKeypoints,
      trainImage,
      trainKeypoints,
      goodMatches,
      imageDrawMatches,
      cv::Scalar::all(-1),
      cv::Scalar::all(-1),
      std::vector<char>(),
      cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  cv::line(
      imageDrawMatches,
      trainWorldCorners[0] + cv::Point2f(queryImage.cols, 0),
      trainWorldCorners[1] + cv::Point2f(queryImage.cols, 0),
      cv::Scalar(0, 255, 0),
      4);
  cv::line(
      imageDrawMatches,
      trainWorldCorners[1] + cv::Point2f(queryImage.cols, 0),
      trainWorldCorners[2] + cv::Point2f(queryImage.cols, 0),
      cv::Scalar(0, 255, 0),
      4);
  cv::line(
      imageDrawMatches,
      trainWorldCorners[2] + cv::Point2f(queryImage.cols, 0),
      trainWorldCorners[3] + cv::Point2f(queryImage.cols, 0),
      cv::Scalar(0, 255, 0),
      4);
  cv::line(
      imageDrawMatches,
      trainWorldCorners[3] + cv::Point2f(queryImage.cols, 0),
      trainWorldCorners[0] + cv::Point2f(queryImage.cols, 0),
      cv::Scalar(0, 255, 0),
      4);

  cv::imwrite("out.png", imageDrawMatches);
}