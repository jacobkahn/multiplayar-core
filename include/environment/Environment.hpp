#pragma once

#include <boost/uuid/uuid.hpp>
#include <memory>
#include <unordered_map>
#include <vector>
#include "include/environment/Client.hpp"
#include "include/environment/Object.hpp"

class Environment {
 public:
  Environment();

  /**
   * Add a client to an environment
   */
  boost::uuids::uuid addClient(Location loc);

  /**
   * Add an object to the environment
   */
  boost::uuids::uuid addObject(Location loc);

 private:
  //  Collection of clients in this environment
  std::vector<std::shared_ptr<Client>> clients;
  // Mapped by IDs for quick lookup
  std::unordered_map<std::size_t, std::shared_ptr<Client>> clients_by_id;
  // Collection of objects in the environment
  std::vector<std::shared_ptr<Object>> objects;
  // Mapped by IDs for quick lookup
  std::unordered_map<std::size_t, std::shared_ptr<Object>> objects_by_id;
};