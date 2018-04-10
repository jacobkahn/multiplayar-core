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

Location Entity::getLocation() const {
  return location_;
}

void Entity::updateLocation(Location location, double rotation) {
  location_ = location;
  rotationScalar_ = rotation;
}

double Entity::getRotation() const {
  return rotationScalar_;
}
