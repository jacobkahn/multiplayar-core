#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "include/environment/Client.hpp"
#include "include/environment/Entity.hpp"
#include "include/environment/Object.hpp"

class Environment {
 public:
  Environment();

  /**
   * Add a client to an environment
   */
  PointList updateClient(EntityID id, const std::string& image);

  bool clientExists(const EntityID& id);

  /**
   * Add an object to the environment
   */
  void addObject(EntityID id);

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