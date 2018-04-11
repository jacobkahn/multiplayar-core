#include <opencv2/opencv.hpp>
#include <string>
#include <utility>
#include "include/environment/Entity.hpp"

// Stage of image progression
using Stage = size_t;

class SIFTWriter {
 public:
  SIFTWriter();

  void writeImage(const std::string& filename, const cv::Mat& image);

  size_t computeHashForIDsAndStage(EntityID id1, EntityID id2, Stage stage);

  std::string getSingleImageDataForID(EntityID id, Stage stage);

  std::string computeSingleFilenameForID(EntityID id1, Stage stage);

  std::pair<std::string, std::string>
  computeCompoundFilenamesForIDs(EntityID id1, EntityID id2, Stage stage);

  std::string
  getCompoundImageDataForIDs(EntityID id1, EntityID id2, Stage stage);

  void createImageWithKeypointsAndARPoints(
      EntityID id,
      std::vector<cv::KeyPoint> keypoints,
      const std::vector<cv::Point2f>& candidatePoints);

  void createImageWithMachings(
      EntityID id1,
      EntityID id2,
      std::vector<cv::DMatch> matches,
      std::vector<cv::KeyPoint> keypoints1,
      std::vector<cv::KeyPoint> keypoints2,
      Stage stage);

 private:
};