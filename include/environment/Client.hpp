#pragma once

#include "include/environment/Entity.hpp"

/**
 * A client interacting with the AR environment
 */
class Client : public Entity {
 public:
  Client(Location startLocation);
};