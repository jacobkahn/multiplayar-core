#include "include/cv/SIFT.hpp"
#include <opencv2/core/cvstd.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "include/environment/Client.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Distance for KD matching
const double kMinDistanceKDMatching = 20;
// Percentile of total matches that we are taking as "good" matches
const double kDistancePercentKDMatch = 0.005;
// Minimum number of points we need for a homography threshold
const size_t kMinPointsHomographyThreshold = 8;
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
    if ((matches[k][0].distance <
         kRatioTestConstant * (matches[k][1].distance)) &&
        ((int)matches[k].size() <= 2 && (int)matches[k].size() > 0)) {
      // take the first result only if its distance is smaller than
      // 0.6*second_best_dist that means this descriptor is ignored if the
      // second distance is bigger or of similar
      filteredMatches.push_back(matches[k][0]);
    }
  }

  std::cout << "Out of " << matches.size() << " matches, "
            << filteredMatches.size() << " good matches found.\n";

  // Count the number of mathes where the distance is less than 2 * min_dist
  // Lambda comparator between distance fields of DMatches
  auto cmp = [](cv::DMatch left, cv::DMatch right) {
    return left.distance > right.distance;
  };
  std::priority_queue<cv::DMatch, std::vector<cv::DMatch>, decltype(cmp)>
      matchMaxHeap(cmp);
  for (size_t i = 0; i < filteredMatches.size(); i++) {
    std::cout << "Match distances " << filteredMatches[i].distance << "\n";
    // Add to heap
    matchMaxHeap.push(filteredMatches[i]);
    // If we're at capacity, pop the top element
    if (matchMaxHeap.size() > kMinPointsHomographyThreshold) {
      matchMaxHeap.pop();
    }
  }
  // Remove everything, save to vector
  std::vector<cv::DMatch> goodMatches;
  while (matchMaxHeap.size() != 0) {
    goodMatches.push_back(matchMaxHeap.top());
    matchMaxHeap.pop();
  }

  // Make sure we have enough points?
  // TODO: stuff
  if (goodMatches.size() == 0 ||
      goodMatches.size() < kMinPointsHomographyThreshold) {
    std::cout << "Insufficient matches found for homography. Exiting.\n";
    // return std::make_pair();
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
      cv::findHomography(finalQueryKeypoints, finalTrainKeypoints, cv::RANSAC);
  cv::Mat TQhomography =
      cv::findHomography(finalTrainKeypoints, finalQueryKeypoints, cv::RANSAC);
  std::cout << "Query to train homography is " << QThomography << "\n";
  std::cout << "Train to query homography is " << TQhomography << "\n";

  /***** Perspective transform for query image *****/
  std::vector<cv::Point2f> queryImageCorners(4);
  // Compute dimensions of images
  queryImageCorners[0] = cv::Point2f(0, 0);
  queryImageCorners[1] = cv::Point2f(queryCols, 0);
  queryImageCorners[2] = cv::Point2f(queryCols, queryRows);
  queryImageCorners[3] = cv::Point2f(0, queryRows);
  std::vector<cv::Point2f> queryWorldCorners(4);
  /***** Perspective transform for train image *****/
  std::vector<cv::Point2f> trainImageCorners(4);
  // Compute dimensions of images
  trainImageCorners[0] = cv::Point2f(0, 0);
  trainImageCorners[1] = cv::Point2f(trainCols, 0);
  trainImageCorners[2] = cv::Point2f(trainCols, trainRows);
  trainImageCorners[3] = cv::Point2f(0, trainRows);
  std::vector<cv::Point2f> trainWorldCorners(4);

  /***** Perform perspective transform *****/
  // Find dimensions of both images so we can apply the homography to the
  // source images to compute 2D markers for anchor offsets Compute
  // transform
  cv::perspectiveTransform(trainImageCorners, trainWorldCorners, TQhomography);
  cv::perspectiveTransform(queryImageCorners, queryWorldCorners, QThomography);

  std::cout << "Query world corners " << queryWorldCorners << "\n";
  std::cout << "Train world corners " << trainWorldCorners << "\n";

  // Format for response
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
  // // Open and read raw image strings
  // std::ifstream queryStream("queryImage.jpg");
  // std::ifstream trainStream("trainImage.jpg");
  // queryStream >> std::noskipws;
  // trainStream >> std::noskipws;

  // std::string rawQueryString(
  //     (std::istreambuf_iterator<char>(queryStream)),
  //     std::istreambuf_iterator<char>());

  // std::string rawTrainString(
  //     (std::istreambuf_iterator<char>(trainStream)),
  //     std::istreambuf_iterator<char>());

  //     // Decode image data
  // std::vector<char> rawQueryArr(rawQueryData.begin(), rawQueryData.end());
  // cv::Mat queryImage = cv::imdecode(cv::Mat(rawQueryArr),
  // cv::IMREAD_UNCHANGED); std::vector<char> rawTrainArr(rawTrainData.begin(),
  // rawTrainData.end()); cv::Mat trainImage =
  // cv::imdecode(cv::Mat(rawTrainArr), cv::IMREAD_UNCHANGED);

  // /***** Extract Keypoints and Descriptors from both images *****/
  // // Keypoints to be extracted from image
  // std::vector<cv::KeyPoint> queryKeypoints;
  // std::vector<cv::KeyPoint> trainKeypoints;
  // // Descriptors to be extracted from image
  // cv::Mat queryDescriptors;
  // cv::Mat trainDescriptors;

  // // Feature detector
  // cv::Ptr<cv::xfeatures2d::SIFT> siftTrainClient =
  //     cv::xfeatures2d::SIFT::create(
  //         //   0, // nFeatures
  //         //   3, // nOctaveLayers
  //         //   0.04, // contrastThreshold
  //         //   10, // edgeThreshold
  //         //   1.6 // sigma
  //     );
  // // Make descriptor extractor
  // cv::Ptr<cv::xfeatures2d::SIFT> siftQueryClient =
  //     cv::xfeatures2d::SIFT::create(
  //         //   0, // nFeatures
  //         //   3, // nOctaveLayers
  //         //   0.04, // contrastThreshold
  //         //   10, // edgeThreshold
  //         //   1.6 // sigma
  //     );

  // // Extract keypoints and descriptors from image 1
  // siftTrainClient->detectAndCompute(
  //     queryImage, cv::noArray(), queryKeypoints, queryDescriptors);

  // // Extract keypoints and descriptors from image 2
  // siftTrainClient->detectAndCompute(
  //     trainImage, cv::noArray(), trainKeypoints, trainDescriptors);

  // computeHomographyTransformation(rawQueryString, rawTrainString, true);

  // // Draw the image if we want to
  // if (drawImageResult) {
  //   cv::Mat imageDrawMatches;
  //   drawMatches(
  //       queryImage,
  //       queryKeypoints,
  //       trainImage,
  //       trainKeypoints,
  //       goodMatches,
  //       imageDrawMatches,
  //       cv::Scalar::all(-1),
  //       cv::Scalar::all(-1),
  //       std::vector<char>(),
  //       cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  //   cv::line(
  //       imageDrawMatches,
  //       trainWorldCorners[0] + cv::Point2f(queryCols, 0),
  //       trainWorldCorners[1] + cv::Point2f(queryCols, 0),
  //       cv::Scalar(0, 255, 0),
  //       4);
  //   cv::line(
  //       imageDrawMatches,
  //       trainWorldCorners[1] + cv::Point2f(queryCols, 0),
  //       trainWorldCorners[2] + cv::Point2f(queryCols, 0),
  //       cv::Scalar(0, 255, 0),
  //       4);
  //   cv::line(
  //       imageDrawMatches,
  //       trainWorldCorners[2] + cv::Point2f(queryCols, 0),
  //       trainWorldCorners[3] + cv::Point2f(queryCols, 0),
  //       cv::Scalar(0, 255, 0),
  //       4);
  //   cv::line(
  //       imageDrawMatches,
  //       trainWorldCorners[3] + cv::Point2f(queryCols, 0),
  //       trainWorldCorners[0] + cv::Point2f(queryCols, 0),
  //       cv::Scalar(0, 255, 0),
  //       4);

  //   cv::imwrite("out.png", imageDrawMatches);
  // }
}