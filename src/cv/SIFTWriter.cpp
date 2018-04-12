#include "include/cv/SIFTWriter.hpp"
#include <boost/functional/hash.hpp>
#include <fstream>
#include <string>
#include <utility>
#include "include/environment/Entity.hpp"

const std::string kFileExtension = ".png";

SIFTWriter::SIFTWriter() {}

void SIFTWriter::writeImage(const std::string& filename, const cv::Mat& image) {
  std::vector<int> params;
  // params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  // params.push_back(9);
  cv::imwrite(filename + kFileExtension, image, params);
}

cv::Mat SIFTWriter::readImageAsMatrix(const std::string& fileID) {
  return cv::imread(fileID + kFileExtension, CV_LOAD_IMAGE_COLOR);
}

std::string SIFTWriter::readFileByFileID(std::string fileID) {
  std::ifstream myStream(fileID + kFileExtension, std::ios::binary);
  std::string outBuffer;
  std::ostringstream oStream;
  oStream << myStream.rdbuf();
  return oStream.str();
}

void SIFTWriter::writeFileByID(std::string fileID, std::string buffer) {
  std::ofstream outStream;
  outStream.open(
      fileID + kFileExtension, std::ios_base::out | std::ios_base::binary);
  outStream << buffer;
  outStream.close();
}

bool SIFTWriter::fileWithIDExists(std::string fileID) {
  std::ifstream testStream(fileID + kFileExtension);
  return testStream.good();
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
  auto fileId = computeSingleFilenameForID(id, stage);
  return readFileByFileID(fileId);
}

std::string SIFTWriter::getCompoundImageDataForIDs(
    EntityID id1,
    EntityID id2,
    Stage stage) {
  // Compute the filename
  auto candidates = computeCompoundFilenamesForIDs(id1, id2, stage);

  // Check which filename exists (only one does)
  std::string fileId;
  if (fileWithIDExists(candidates.first)) {
    fileId = std::move(candidates.first);
  } else {
    fileId = std::move(candidates.second);
  }

  // Now we have the correct id, so read its data and return it
  return readFileByFileID(fileId);
}

void SIFTWriter::createImageWithKeypointsAndARPoints(
    EntityID id,
    std::vector<cv::KeyPoint> keypoints,
    const std::vector<cv::Point2f>& candidatePoints) {
  auto image = readImageAsMatrix(computeSingleFilenameForID(id, 1));
  // Draw keypoints
  for (auto& keypoint : keypoints) {
    cv::circle(image, keypoint.pt, 6, cv::Scalar(255, 0, 0), 15);
  }
  // Draw candidate points
  for (auto& candPoint : candidatePoints) {
    cv::circle(image, candPoint, 6, cv::Scalar(0, 255, 255), 15);
  }
  writeImage(computeSingleFilenameForID(id, 2), image);
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
  auto image1 = readImageAsMatrix(image1File);
  auto image2 = readImageAsMatrix(image2File);

  // Draw the matches
  cv::Mat imageDrawMatches;
  cv::drawMatches(
      image1, keypoints1, image2, keypoints2, matchings, imageDrawMatches);
  writeImage(
      computeCompoundFilenamesForIDs(id1, id2, stage).first, imageDrawMatches);
}

std::string SIFTWriter::base64Encode(const std::string& in) {
  std::string out;

  int val = 0, valb = -6;
  for (unsigned char c : in) {
    val = (val << 8) + c;
    valb += 8;
    while (valb >= 0) {
      out.push_back(
          "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"
              [(val >> valb) & 0x3F]);
      valb -= 6;
    }
  }
  if (valb > -6)
    out.push_back(
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"
            [((val << 8) >> (valb + 8)) & 0x3F]);
  while (out.size() % 4)
    out.push_back('=');
  return out;
}