#pragma once

#include <boost/uuid/uuid.hpp>
#include "include/environment/Entity.hpp"

/**
 * A representation of an object in the AR environment
 */
class Object : public Entity {
 public:
  Object(EntityID id);
};
