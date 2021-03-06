#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <memory>
#include <string>
#include "include/cv/SIFT.hpp"
#include "include/server/Server.hpp"

// Default port number
const uint32_t kDefaultPort = 10000;
// Max port number
const uint32_t kMaxPortNumber = 65535;

int main(int argc, char* argv[]) {
  /***** Parse command line args *****/

  // Port number for server bind
  uint32_t port = kDefaultPort;
  bool debugMode = false;
  bool writeImageMode = false;

  // Read in arguments
  int c;
  while ((c = getopt(argc, argv, "p:avsi")) != -1) {
    switch (c) {
      case 'p':
        try {
          port = std::stoi(optarg);
        } catch (const std::exception&) {
          std::cerr << "Invalid port number specified.\n";
          return EXIT_FAILURE;
        }
        // Check port number in range
        if (port > kMaxPortNumber) {
          std::cerr << "Specified port out of range.\n";
          return EXIT_FAILURE;
        }
        break;
      case 'v':
        debugMode = true;
        break;
      case 's':
        // Run Toy SIFT
        std::make_unique<SIFTClient>()->runToySIFT();
        return 0;
      case 'i':
        writeImageMode = true;
        break;
      case '?':
        if (optopt == 'c') {
          fprintf(stderr, "Option -%c requires an argument.\n", optopt);
        } else if (isprint(optopt)) {
          fprintf(stderr, "Unknown option `-%c'.\n", optopt);
        } else {
          fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
        }
        return EXIT_FAILURE;
        break;
      default:
        std::cerr << "Invalid command line option specified.\n";
        return EXIT_FAILURE;
    }
  }

  /******* Start Server *******/
  auto server = std::make_unique<Server>(debugMode, writeImageMode);
  server->setup();
  server->run(port);
}