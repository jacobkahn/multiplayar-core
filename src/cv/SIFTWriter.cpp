#include "include/cv/SIFTWriter.hpp"
#include <boost/functional/hash.hpp>
#include <fstream>
#include <string>
#include <utility>
#include "include/environment/Entity.hpp"

SIFTWriter::SIFTWriter() {}

void SIFTWriter::writeImage(const std::string& filename, const cv::Mat& image) {
  cv::imwrite(filename, image);
}

size_t
SIFTWriter::computeHashForIDsAndStage(EntityID id1, EntityID id2, Stage stage) {
  size_t seed{stage};
  boost::hash_combine(seed, boost::hash_value(id1));
  boost::hash_combine(seed, boost::hash_value(id2));
  return seed;
}

std::string SIFTWriter::computeSingleFilenameForID(EntityID id1, Stage stage) {
  return std::string(id1) + std::to_string(stage);
}

std::pair<std::string, std::string> SIFTWriter::computeCompoundFilenamesForIDs(
    EntityID id1,
    EntityID id2,
    Stage stage) {
  return std::make_pair(
      std::to_string(computeHashForIDsAndStage(id1, id2, stage)),
      std::to_string(computeHashForIDsAndStage(id2, id1, stage)));
}

std::string SIFTWriter::getSingleImageDataForID(EntityID id, Stage stage) {
  auto filename = computeSingleFilenameForID(id, stage);
  std::ifstream stream(filename);
  std::string buffer;
  stream >> std::noskipws;
  stream >> buffer;
  return buffer;
}

std::string SIFTWriter::getCompoundImageDataForIDs(
    EntityID id1,
    EntityID id2,
    Stage stage) {
  // Compute the filename
  auto candidates = computeCompoundFilenamesForIDs(id1, id2, stage);

  // Check which filename exists (only one does)
  std::ifstream inStream;
  std::ifstream testStream(candidates.first);
  if (testStream.good()) {
    inStream = std::move(testStream);
  } else {
    inStream = std::ifstream(candidates.second);
  }

  // Now we have the file, so read its data and return it
  std::string buffer;
  inStream >> std::noskipws;
  inStream >> buffer;
  return buffer;
}

void SIFTWriter::createImageWithKeypointsAndARPoints(
    EntityID id,
    std::vector<cv::KeyPoint> keypoints,
    const std::vector<cv::Point2f>& candidatePoints) {
  // Draw keypoints
  cv::Mat imgKey1;

  auto image = cv::imread(computeSingleFilenameForID(id, 1));

  cv::drawKeypoints(
      image, keypoints, imgKey1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

  // Show detected (drawn) keypoints
  cv::imwrite(computeSingleFilenameForID(id, 2), imgKey1);
}

void SIFTWriter::createImageWithMachings(
    EntityID id1,
    EntityID id2,
    std::vector<cv::DMatch> matchings,
    std::vector<cv::KeyPoint> keypoints1,
    std::vector<cv::KeyPoint> keypoints2,
    Stage stage) {
  // Get original image files
  auto image1File = computeSingleFilenameForID(id1, 1);
  auto image2File = computeSingleFilenameForID(id2, 1);
  // Get the image data
  auto image1 = cv::imread(image1File);
  auto image2 = cv::imread(image2File);
  // Draw the matches
  cv::Mat imageDrawMatches;
  cv::drawMatches(
      image1,
      keypoints1,
      image2,
      keypoints1,
      matchings,
      imageDrawMatches,
      cv::Scalar::all(-1),
      cv::Scalar::all(-1),
      std::vector<char>(),
      cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  cv::imwrite(
      computeCompoundFilenamesForIDs(id1, id2, stage).first, imageDrawMatches);
}
