#pragma once

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

  /**
   * Add an object to the environment
   */
  EntityID addObject();

  /**
   * Updates an object's location in the environment
   */
  void updateObject(EntityID id, Location location);

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