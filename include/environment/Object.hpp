#pragma once

#include "include/environment/Entity.hpp"
#include <boost/uuid/uuid.hpp>

/**
 * A representation of an object in the AR environment
 */
class Object : public Entity {
 public:
  Object(EntityID id);
};
