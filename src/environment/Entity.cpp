#include "include/environment/Entity.hpp"
#include <boost/geometry.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

Entity::Entity(EntityID id) : id_(id) {
}

EntityID Entity::getID() const {
  return id_;
}
