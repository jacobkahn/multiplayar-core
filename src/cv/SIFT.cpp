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
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include "include/cv/SIFTWriter.hpp"
#include "include/environment/Client.hpp"

StringyPoint PointRepresentationUtils::cvPoint2fToStringyPoint(
    cv::Point2f point) {
  return {{kStringyPointXFieldName, std::to_string(point.x)},
          {kStringyPointYFieldName, std::to_string(point.y)}};
}

cv::Point2f PointRepresentationUtils::stringyPointToPoint2f(
    StringyPoint stringyPointMap) {
  return cv::Point2f(
      std::stod(stringyPointMap[kStringyPointXFieldName]),
      std::stod(stringyPointMap[kStringyPointYFieldName]));
}

std::string PointRepresentationUtils::cvPoint2fToString(cv::Point2f point) {
  std::string s =
      "(" + std::to_string(point.x) + ", " + std::to_string(point.y) + ")";
  return s;
}

// Minimum number of points we need for a homography threshold
const size_t kMinPointsHomographyThreshold = 6;
// Minimum number of points we need for a homography threshold
const size_t kMaxPointsHomographyHeap = 75;
// Constant for Lowe's ratio test across the best matches
const double kRatioTestConstant = 2;
// Centrality score threshold
const size_t kCentralityScoreNumberOfPointComparisons = 5;
// Number of points to return from SIFT
const size_t kNumMatchesToReturn = 4;

double SIFTClient::computeCentralityScoreForPoint(
    cv::Point2f queryPoint,
    const std::vector<cv::Point2f>& candidatePoints) {
  // Given two other points, prefer the closest one
  auto cmp = [&](cv::Point2f left, cv::Point2f right) {
    auto leftDistance = cv::norm(cv::Mat(queryPoint), cv::Mat(left));
    auto rightDistance = cv::norm(cv::Mat(queryPoint), cv::Mat(right));
    return leftDistance > rightDistance;
  };
  std::priority_queue<cv::Point2f, std::vector<cv::Point2f>, decltype(cmp)>
      matchMaxHeap(cmp);
  for (size_t i = 0; i < candidatePoints.size(); i++) {
    // std::cout << "Match distances " << filteredMatches[i].distance << "\n";
    // Add to heap
    matchMaxHeap.push(candidatePoints[i]);
    // If we're at capacity, pop the top element
    if (matchMaxHeap.size() > kCentralityScoreNumberOfPointComparisons) {
      matchMaxHeap.pop();
    }
  }
  // Remove everything, save to vector
  double totalDistanceScore = 0.0;
  while (matchMaxHeap.size() != 0) {
    totalDistanceScore +=
        cv::norm(cv::Mat(queryPoint), cv::Mat(matchMaxHeap.top()));
    matchMaxHeap.pop();
  }
  return totalDistanceScore;
}

double SIFTClient::computeCentralityScore(
    cv::Point2f queryPoint,
    cv::Point2f trainPoint,
    const std::vector<cv::Point2f>& queryCandidatePoints,
    const std::vector<cv::Point2f>& trainCandidatePoints) {
  return computeCentralityScoreForPoint(queryPoint, queryCandidatePoints) +
      computeCentralityScoreForPoint(trainPoint, trainCandidatePoints);
}

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

std::shared_ptr<HomographyTransformResult>
SIFTClient::computeHomographyTransformationFromClients(
    std::shared_ptr<Client> client1,
    std::shared_ptr<Client> client2) {
  // Compute good matchings
  auto matchings =
      computeMatchings(client1->getDescriptors(), client2->getDescriptors());

  // Write matchings to file
  auto thread1 = std::thread([&]() {
    auto writer = std::make_shared<SIFTWriter>();
    auto filenames = writer->computeCompoundFilenamesForIDs(
        client1->getID(), client2->getID(), 3);
    writer->createImageWithMachings(
        client1->getID(),
        client2->getID(),
        goodMatches_,
        client1->getKeypoints(),
        client2->getKeypoints(),
        3);
    ;
  });
  thread1.detach();

  // Compute the homography with raw datas and return candidate points
  auto transformData = computeHomographyTransformation(
      client1->getKeypoints(),
      client2->getKeypoints(),
      client1->getCandidatePoints(),
      client2->getCandidatePoints(),
      matchings);

  // Write matchings to file
  auto thread2 = std::thread([&]() {
    auto writer = std::make_shared<SIFTWriter>();
    writer->createImageWithMachings(
        client1->getID(),
        client2->getID(),
        goodMatches_,
        client1->getKeypoints(),
        client2->getKeypoints(),
        4);
  });
  thread2.detach();

  // Make a new result
  auto result = std::make_shared<HomographyTransformResult>();
  // The first two fields in the result are point data for client 1 and client
  // 2, respectively
  result->pointMap.emplace(
      client1->getID(), std::move(transformData.first.first));
  result->pointMap.emplace(
      client2->getID(), std::move(transformData.first.second));
  // The first two fields in the result are point data for client 1 and client
  // 2, respectively
  result->homographyMap.emplace(
      client1->getID(), std::move(transformData.second.first));
  result->homographyMap.emplace(
      client2->getID(), std::move(transformData.second.second));
  return result;
}

/**
 * Given an image
 * Returns a tuple of images after a perspectice transform
 */
std::vector<cv::DMatch> SIFTClient::computeMatchings(
    // Descriptors to be extracted from image
    cv::Mat queryDescriptors,
    cv::Mat trainDescriptors) {
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

  /***** Lowe Ratio test *****/
  // We use Lowe's Euclidean ratio test (Frobenius) to measure the match
  // distance to assess how good them matches are y'all
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
  return filteredMatches;
}

RawHomographyData SIFTClient::computeHomographyTransformation(
    std::vector<cv::KeyPoint> queryKeypoints,
    std::vector<cv::KeyPoint> trainKeypoints,
    const std::vector<cv::Point2f>& queryCandidatePoints,
    const std::vector<cv::Point2f>& trainCandidatePoints,
    std::vector<cv::DMatch> filteredMatches) {
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
    RawHomographyData emptyData;
    return emptyData;
  }

  /***** Rank Matches based on centrality AR candidate point scores *****/
  // Pre-compute scores and place into a map
  std::unordered_map<cv::DMatch, double, MatchHasher, MatchEquals> matchScores;
  for (auto& match : goodMatches) {
    auto score = computeCentralityScore(
        queryKeypoints[match.queryIdx].pt,
        trainKeypoints[match.trainIdx].pt,
        queryCandidatePoints,
        trainCandidatePoints);
    matchScores.insert({match, score});
  }
  // Put everything in a heap
  auto dMatchCmp = [&](cv::DMatch left, cv::DMatch right) {
    return matchScores.at(left) > matchScores.at(right);
  };
  // Make a match queue
  std::priority_queue<cv::DMatch, std::vector<cv::DMatch>, decltype(dMatchCmp)>
      matchHeap(dMatchCmp);
  for (size_t i = 0; i < goodMatches.size(); i++) {
    // std::cout << "Match distances " << filteredMatches[i].distance << "\n";
    // Add to heap
    matchHeap.push(goodMatches[i]);
    // If we're at capacity, pop the top element
    if (matchHeap.size() > kNumMatchesToReturn) {
      matchHeap.pop();
    }
  }
  // Remove everything, save to vector
  std::vector<cv::DMatch> betterMatches;
  while (matchHeap.size() != 0) {
    betterMatches.push_back(matchHeap.top());
    matchHeap.pop();
  }
  // Save best matches
  goodMatches_ = betterMatches;

  // Get keypoints that we care about
  std::vector<cv::Point2f> finalQueryKeypoints;
  std::vector<cv::Point2f> finalTrainKeypoints;
  for (auto& match : betterMatches) {
    finalQueryKeypoints.push_back(queryKeypoints[match.queryIdx].pt);
    finalTrainKeypoints.push_back(trainKeypoints[match.trainIdx].pt);
  }

  PointList finalQueryPointList;
  for (auto& point : finalQueryKeypoints) {
    finalQueryPointList.push_back(
        PointRepresentationUtils::cvPoint2fToStringyPoint(point));
  }
  PointList finalTrainPointList;
  for (auto& point : finalTrainKeypoints) {
    finalTrainPointList.push_back(
        PointRepresentationUtils::cvPoint2fToStringyPoint(point));
  }

  /***** Find Homography *****/
  // TODO: is this the right way to use ransac?
  cv::Mat QThomography =
      cv::findHomography(finalQueryKeypoints, finalTrainKeypoints);
  cv::Mat TQhomography =
      cv::findHomography(finalTrainKeypoints, finalQueryKeypoints);

  // Return special data structure
  return std::make_pair(
      std::make_pair(finalQueryPointList, finalTrainPointList),
      std::make_pair(QThomography, TQhomography));
}

void SIFTClient::runToySIFT() {
  // Read in an image from an interesting
  const cv::Mat queryImage = cv::imread("queryImage.jpg");
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
  cv::Ptr<cv::xfeatures2d::SIFT> siftClientQuery =
      cv::xfeatures2d::SIFT::create(
          //   0, // nFeatures
          //   3, // nOctaveLayers
          //   0.04, // contrastThreshold
          //   10, // edgeThreshold
          //   1.6 // sigma
      );
  siftClientQuery->detectAndCompute(
      queryImage, cv::noArray(), queryKeypoints, queryDescriptors);

  // New SIFT
  cv::Ptr<cv::xfeatures2d::SIFT> siftClientTrain =
      cv::xfeatures2d::SIFT::create(
          //   0, // nFeatures
          //   3, // nOctaveLayers
          //   0.04, // contrastThreshold
          //   10, // edgeThreshold
          //   1.6 // sigma
      );
  siftClientTrain->detectAndCompute(
      trainImage, cv::noArray(), trainKeypoints, trainDescriptors);

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
  // We use Lowe's Euclidean ratio test to measure the match distance to
  // assess how good them matches are y'all
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
  // Find dimensions of both images so we can apply the homography to the
  // source images to compute 2D markers for anchor offsets Compute transform
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

  //   cv::line(
  //       imageDrawMatches,
  //       trainWorldCorners[0] + cv::Point2f(queryImage.cols, 0),
  //       trainWorldCorners[1] + cv::Point2f(queryImage.cols, 0),
  //       cv::Scalar(0, 255, 0),
  //       4);
  //   cv::line(
  //       imageDrawMatches,
  //       trainWorldCorners[1] + cv::Point2f(queryImage.cols, 0),
  //       trainWorldCorners[2] + cv::Point2f(queryImage.cols, 0),
  //       cv::Scalar(0, 255, 0),
  //       4);
  //   cv::line(
  //       imageDrawMatches,
  //       trainWorldCorners[2] + cv::Point2f(queryImage.cols, 0),
  //       trainWorldCorners[3] + cv::Point2f(queryImage.cols, 0),
  //       cv::Scalar(0, 255, 0),
  //       4);
  //   cv::line(
  //       imageDrawMatches,
  //       trainWorldCorners[3] + cv::Point2f(queryImage.cols, 0),
  //       trainWorldCorners[0] + cv::Point2f(queryImage.cols, 0),
  //       cv::Scalar(0, 255, 0),
  //       4);

  cv::imwrite("out.png", imageDrawMatches);
}