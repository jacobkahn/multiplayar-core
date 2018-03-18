#include "include/cv/SIFT.hpp"
#include <opencv2/core/cvstd.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <memory>
#include <queue>
#include <vector>

// Distance for KD matching
const double kMinDistanceKDMatching = 20;
// Percentile of total matches that we are taking as "good" matches
const double kDistancePercentKDMatch = 0.02;
// Minimum number of points we need for a homography threshold
const size_t kMinPointsHomographyThreshold = 0;

/**
 * Given an image
 */
void runSift() {
  // Read in an image from an interesting
  const cv::Mat queryImage =
      cv::imread("queryImage.jpg", 0); // Load as grayscale because yeah
  // Read in
  const cv::Mat trainImage = cv::imread("trainImage.jpg");

  /***** Extract Keypoints and Descriptors from both images *****/
  // Keypoints to be extracted from image
  std::vector<cv::KeyPoint> queryKeypoints;
  std::vector<cv::KeyPoint> trainKeypoints;
  // Descriptors to be extracted from image
  cv::Mat queryDescriptors;
  cv::Mat trainDescriptors;

  // Feature detector
  cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> siftFeatureDetector =
      cv::xfeatures2d::SiftFeatureDetector::create(
          0, // nFeatures
          4, // nOctaveLayers
          0.04, // contrastThreshold
          10, // edgeThreshold
          1.6 // sigma
      );
  // Make descriptor extractor
  cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> siftDescriptorExtractor =
      cv::xfeatures2d::SiftDescriptorExtractor::create();

  // Extract keypoints from image 1
  siftFeatureDetector->detect(queryImage, queryKeypoints);
  // Extract features from image 1
  siftDescriptorExtractor->compute(
      queryImage, queryKeypoints, queryDescriptors);

  // Extract keypoints from image 2
  siftFeatureDetector->detect(trainImage, trainKeypoints);
  // Extract features from image 1
  siftDescriptorExtractor->compute(
      trainImage, trainKeypoints, trainDescriptors);

  // Make a new FLANN KD dtree
  const cv::Ptr<cv::flann::IndexParams>& indexParams =
      new cv::flann::KDTreeIndexParams(1);
  const cv::Ptr<cv::flann::SearchParams>& searchParams =
      new cv::flann::SearchParams(64);

  /***** Match the Descriptors with FLANN KD Tree *****/
  cv::Mat indexMat;
  // FLANN KD tree matcher
  cv::FlannBasedMatcher matcher(indexParams, searchParams);
  // Matches we find from the compare
  std::vector<cv::DMatch> matches;
  matcher.match(queryDescriptors, trainDescriptors, matches);
  // We use Lowe's Euclidean ratio test to measure the match distance to assess
  // how good them matches are y'all
  std::vector<cv::DMatch> goodMatches;
  // Count the number of mathes where the distance is less than 2 * min_dist
  // Lambda comparator between distance fields of DMatches
  auto cmp = [](cv::DMatch left, cv::DMatch right) {
    return left.distance > right.distance;
  };
  std::priority_queue<cv::DMatch, std::vector<cv::DMatch>, decltype(cmp)>
      matchMaxHeap(cmp);
  for (size_t i = 0; i < matches.size(); i++) {
    // Add to heap
    matchMaxHeap.push(matches[i]);
    // If we're at capacity, pop the top element
    std::cout << "Heap size " << matchMaxHeap.size() << "\n";
    if (matchMaxHeap.size() > kDistancePercentKDMatch * matches.size()) {
      matchMaxHeap.pop();
    }
  }
  // Remove everything, save to vector
  while (matchMaxHeap.size() != 0) {
    goodMatches.push_back(matchMaxHeap.top());
    matchMaxHeap.pop();
  }

  // Make sure we have enough points?
  // TODO: stuff
  // if (goodMatches.size() < kMinPointsHomographyThreshold) {

  // }

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

  std::cout << "Query world corners " << queryWorldCorners << "\n";
  std::cout << "Train world corners " << trainWorldCorners << "\n";

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