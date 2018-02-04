#pragma once

#include <boost/geometry.hpp>
#include <boost/uuid/uuid.hpp>

// Generalize to n-dimensional space for a point
const size_t kPointDimension = 3;

// Boost geometry
namespace bg = boost::geometry;

// A coordinate representation as a Cartesian Boost geometry point
using Location = bg::model::point<double, kPointDimension, bg::cs::cartesian>;

/**
 * A generic entity is something that exists in the environment. This could be a
 * user, an object, or something else. It has an ID and a location at a given
 * time, and other implementation-specific properties
 */
class Entity {
 public:
  /**
   * Requires a starting location on construction
   */
  Entity(Location startLocation);

  /**
   * Returns the UUID for this entity
   */
  boost::uuids::uuid getUUID() const;

 private:
  boost::uuids::uuid uuid;
  // The entity's current location
  Location location_{0.0, 0.0, 0.0};
};