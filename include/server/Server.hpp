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
  Server(bool debugMode);

  /**
   * Set up routes
   */
  void setup();

  /**
   * Run the server
   */
  void run(uint32_t port);

  /**
   * Converts an unordered map into a
   */
  crow::json::wvalue mapToCrowWValue(
      std::unordered_map<std::string, std::string> map);

 private:
  bool debugMode_{false};
  // App client for
  crow::SimpleApp app;
  // The environment for this server instance
  Environment environment_;
};