#include "include/environment/Environment.hpp"
#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>
#include <memory>
#include <utility>
#include <vector>
#include "include/environment/Client.hpp"
#include "include/environment/Object.hpp"

boost::uuids::uuid Environment::addClient(Location loc) {
  auto client = std::make_shared<Client>(loc);
  auto uid = client->getUUID();

  clients.push_back(client);

  boost::hash<boost::uuids::uuid> uuid_hasher;
  clients_by_id.insert(std::pair<std::size_t, std::shared_ptr<Client>>(
      uuid_hasher(uid), client));

  return uid;
}

boost::uuids::uuid Environment::addObject(Location loc) {
  auto object = std::make_shared<Object>(loc);
  auto uid = object->getUUID();

  objects.push_back(object);

  boost::hash<boost::uuids::uuid> uuid_hasher;
  objects_by_id.insert(std::pair<std::size_t, std::shared_ptr<Object>>(
      uuid_hasher(uid), object));

  return uid;
}