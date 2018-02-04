#pragma once

#include "include/deps/crow.h"

/**
 * Server implementation: thin wrapper around a Crow server
 */
class Server {
 public:
  Server();

  /**
   * Set up routes
   */
  void setup();

  /**
   * Run the server
   */
  void run(uint32_t port);

 private:
  // App client for
  crow::SimpleApp app;
};