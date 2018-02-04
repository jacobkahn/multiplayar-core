#include "include/environment/Entity.hpp"
#include <boost/geometry.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

Entity::Entity(Location startLocation) : location_(startLocation) {
  // Generate uuid
  uuid = boost::uuids::random_generator()();
}

boost::uuids::uuid Entity::getUUID() const {
  return uuid;
}
