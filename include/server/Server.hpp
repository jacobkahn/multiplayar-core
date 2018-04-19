#pragma once

#include <string>
#include <unordered_map>
#include "include/deps/crow.h"
#include "include/environment/Environment.hpp"

/**
 * Server implementation: thin wrapper around a Crow server
 */
class Server {
 public:
  Server(bool debugMode = false, bool writeImageMode = false);

  /**
   * Set up routes
   */
  void setup();

  /**
   * Run the server
   */
  void run(uint32_t port);

  /**
   * Converts an unordered map into a crow-json-serializable value
   */
  crow::json::wvalue mapToCrowWValue(
      std::unordered_map<std::string, std::string> map);

 private:
  bool debugMode_{false};
  // App client for
  crow::SimpleApp app;
  // The environment for this server instance
  Environment environment_;
  // Mode which writes progressive images as they're processed
  bool writeImageMode_{false};
};