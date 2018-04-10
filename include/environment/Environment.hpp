#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "include/environment/Client.hpp"
#include "include/environment/Entity.hpp"
#include "include/environment/Object.hpp"

// Object data
using ObjectData = std::vector<std::unordered_map<std::string, std::string>>;

class Environment {
 public:
  /**
   * Add a client to an environment
   */
  PointList updateClient(
      EntityID id,
      std::string image,
      std::vector<cv::Point2f> candidatePoints);

  bool clientExists(const EntityID& id);

  std::shared_ptr<Client> getClientByID(EntityID id) {
    if (!clientExists(id)) {
      return nullptr;
    }
    return clientsByID.at(id);
  }

  /**
   * Given a 2D anchor point, update it for the client that it pertains to,
   * but also use the previous homographies and update the anchor points for any
   * clients that don't have 2D anchor point yet.
   *
   * If a homography exists between the calling client and another client, we
   * get the nearest SIFT point for which there is an AR point, then use the
   * transformed point from the original homography to find an anchor point for
   * the other client.
   */
  void update2DAnchorForClient(EntityID id, cv::Point2f point);

  /**
   * Add an object to the environment
   */
  EntityID addObject();

  /**
   * Updates an object's location in the environment, including its rotation vector
   */
  void updateObject(EntityID id, Location location, double rotation);

  /**
   * Returns an object representation of
   *
   * Format is:
   * [
   *    {
   *      id: [id],
   *      x: [double],
   *      y: [double],
   *      z: [double]
   *    }
   * ]
   */
  ObjectData getObjectRepresentation();

  /**
   * Clear the environment of all objects and users: do a hard reset.
   */
  void clear();

 private:
  //  Collection of clients in this environment
  std::vector<std::shared_ptr<Client>> clients;
  // Mapped by IDs for quick lookup
  std::unordered_map<EntityID, std::shared_ptr<Client>> clientsByID;
  // Collection of objects in the environment
  std::vector<std::shared_ptr<Object>> objects;
  // Mapped by IDs for quick lookup
  std::unordered_map<EntityID, std::shared_ptr<Object>> objectsByID;
};