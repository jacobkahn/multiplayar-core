#include "include/environment/Environment.hpp"
#include <boost/functional/hash.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include "include/cv/SIFTWriter.hpp"
#include "include/environment/Client.hpp"
#include "include/environment/Object.hpp"

std::shared_ptr<HomographyTransformResult> Environment::updateClient(
    EntityID id,
    std::string image,
    std::vector<cv::Point2f> candidatePointsRaw) {
  std::unordered_set<cv::Point2f, PointHasher> candidatePoints;
  for (auto& candidatePoint : candidatePointsRaw) {
    candidatePoints.insert(candidatePoint);
  }

  std::shared_ptr<Client> client;
  if (clientExists(id)) {
    client = clientsByID.find(id)->second;
    client->processImage(std::move(image));
  } else {
    client = std::make_shared<Client>(id);
    client->processImage(std::move(image));
    clients.push_back(client);
    clientsByID.insert(std::make_pair(id, client));
  }
  // Score candidate points for the client
  client->setCandidatePoints(candidatePointsRaw);
  // Attempt to compute homography with another client
  // Make sure we're not the only client
  if (clients.size() > 1) {
    // Get not you
    for (auto& otherClient : clients) {
      if (otherClient->getID() != client->getID()) {
        // We found another client that isn't us. Compute homography
        auto siftClient = std::make_unique<SIFTClient>();
        auto result = siftClient->computeHomographyTransformationFromClients(
            client, otherClient);
        // Store the result of the homography with the other user inside the
        // mapping for this user so we can retrieve it for anchor calibration
        // later
        client->addHomographyTransformResult(otherClient->getID(), result);
        
        /***** Choose the closest candidate points to our output points *****/
        // We perform a brute force algorithm over which the runtime is 4
        // times the total number of ARKit-detected points. This is a minor
        // compuation: we're simply computing an L2 metric over the point
        // set
        for (auto& point : result->pointMap[id]) {
          // Look through candidate points, track the lowest current point
          double minDistance = std::numeric_limits<float>::max();
          cv::Point2f bestPoint;
          // Check all AR points and choose the best one
          for (auto& candidatePoint : candidatePoints) {
            auto bestDistance = cv::norm(
                cv::Mat(PointRepresentationUtils::stringyPointToPoint2f(point)),
                cv::Mat(candidatePoint));
            // Check if this is the best candidate point
            if (bestDistance < minDistance) {
              minDistance = bestDistance;
              bestPoint = candidatePoint;
            }
          }
          // Add the matched point to the matched point under the aggregate
          // homography data
          result->siftToARPointMapping.emplace(
              bestPoint,
              PointRepresentationUtils::stringyPointToPoint2f(point));
          // Remove this point from consideration - make sure the AR points we
          // choose are distinct
          candidatePoints.erase(candidatePoints.find(bestPoint));
        }
        // We only care about the first point list - that's the calling user's
        return result;
      }
    }
  }
  return std::make_shared<HomographyTransformResult>();
}

bool Environment::clientExists(const EntityID& id) {
  return clientsByID.find(id) != clientsByID.end();
}

EntityID Environment::addObject() {
  // TODO: make this and all entities into actual UUID generation
  // Create new object with the next ID
  auto object = std::make_shared<Object>(std::to_string(objects.size() + 1));
  // Add to collections
  objects.push_back(object);
  objectsByID.insert(std::make_pair(object->getID(), object));
  return object->getID();
}

void Environment::updateObject(
    EntityID id,
    Location location,
    RotationScalar rotation) {
  auto iter = objectsByID.find(id);
  if (iter != objectsByID.end()) {
    iter->second->updateLocation(location, rotation);
  } else {
    // Invalid object ID: simply return and do nothing
  }
}

ObjectData Environment::getObjectRepresentation() {
  ObjectData data;
  for (auto& object : objects) {
    auto location = object->getLocation();
    auto rotation = object->getRotation();
    data.push_back({{"id", object->getID()},
                    {"x", std::to_string(location.x)},
                    {"y", std::to_string(location.y)},
                    {"z", std::to_string(location.z)},
                    {"rotation", std::to_string(rotation)}});
  }
  return data;
}

void Environment::update2DAnchorForClient(EntityID id, cv::Point2f point) {
  auto client = getClientByID(id);
  client->update2DAnchorPoints(point);

  // Now, compute updated homographies for each client with which we have a
  // homography, but only do this for clients that don't have an anchor yet
  // (because we're deciding their anchor by computing the homography)
  for (auto& entry : client->getHomographyMap()) {
    auto homographyData = entry.second;
    auto otherClient = getClientByID(entry.first);
    if (!otherClient->hasInitializedAnchor()) {
      auto homography = homographyData->homographyMap[id];
      // Look up which SIFT point this AR point was matched to so we can get
      // the corresponding point from the computed homography
      auto originalSIFTPoint = homographyData->siftToARPointMapping[point];
      // Look up the index of the point in the map to get the corresponding
      // one
      size_t pointIndex = 0;
      for (auto& siftPoint : homographyData->pointMap[client->getID()]) {
        if (PointRepresentationUtils::cvPoint2fToStringyPoint(
                originalSIFTPoint) == siftPoint) {
          break;
        }
        pointIndex++;
      }
      // The transformed point for the other client
      cv::Point2f transformedPoint;
      // Get the corresponding point in the other client's list. We need to
      // iterate through all entities and their related point lists (it's
      // simply this client and the other client)
      auto iter = homographyData->pointMap.begin();
      for (; iter != homographyData->pointMap.end(); iter++) {
        if (iter->first != client->getID()) {
          // This is the PointList we want, since it's not the client's; it's
          // the other client's
          transformedPoint = PointRepresentationUtils::stringyPointToPoint2f(
              iter->second[pointIndex]);
        }
      }
      // Compute the optimal candidate point - the AR point that is closest to
      // the inverse transformed point
      cv::Point2f bestPoint;
      double minDistance = std::numeric_limits<float>::max();
      for (auto& candPoint : otherClient->getCandidatePoints()) {
        auto dist = cv::norm(cv::Mat(candPoint), cv::Mat(transformedPoint));
        if (dist < minDistance) {
          bestPoint = candPoint;
        }
      }
      // Update the client's anchor with this new point
      otherClient->update2DAnchorPoints(bestPoint);
    }
  }
}

void Environment::clear() {
  // Clear clients list
  clients.clear();
  clientsByID.clear();
  // Clear objects
  objects.clear();
  objectsByID.clear();
}

std::vector<std::shared_ptr<Client>> Environment::getClientList() {
  return clients;
}
