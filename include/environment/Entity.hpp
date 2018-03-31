#pragma once

#include <boost/uuid/uuid.hpp>
#include <opencv2/opencv.hpp>

// A coordinate representation as a Cartesian Boost geometry point
using Location = cv::Point3d;

// Entity string
using EntityID = std::string;

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
  Entity(EntityID id);

  /**
   * Returns the UUID for this entity
   */
  EntityID getID() const;

  /**
   * Returns the location for this entity
   */
  Location getLocation() const;

  /**
   * Updates the location for this entity
   */
  void updateLocation(Location location);

 private:
  EntityID id_;
  Location location_;
};